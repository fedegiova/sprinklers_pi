// core.cpp
// This file constitutes the core functions that run the scheduling for the Sprinkler system.
// Author: Richard Zimmerman
// Copyright (c) 2013 Richard Zimmerman
// 

#include "core.h"
#include "settings.h"

#if defined(WEATHER_WUNDERGROUND)
#include "Wunderground.h"
#elif defined(WEATHER_AERIS)
#include "Aeris.h"
#elif defined(WEATHER_DARKSKY)
#include "DarkSky.h"
#elif defined(WEATHER_OPENWEATHER)
#include "OpenWeather.h"
#elif defined(WEATHER_OPENMETEO)
#include "OpenMeteo.h"
#else
#include "Weather.h"
#endif

#include "web.h"
#include "Event.h"
#include "port.h"
#include <stdlib.h>
#ifdef ARDUINO
#include "tftp.h"
static tftp tftpServer;
#else
#include <wiringPi.h>
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <atomic>
#include <assert.h>
#include <time.h>
#include <thread>
#include <poll.h>
#include <fcntl.h>
#include <sys/types.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*a))

time_t getTimeMonotonic()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return ts.tv_sec;
}
int64_t getTimeMonotonicMs()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC,&ts);
    return ts.tv_sec*1000ll + ts.tv_nsec / 1000000;
}


#ifdef LOGGING
Logging logger;
#endif
static web webServer;
nntp nntpTimeServer;
runStateClass runState;

// A bitfield that defines which zones are currently on.
int ZoneState = 0;

runStateClass::runStateClass() : m_bSchedule(false), m_bManual(false), m_iSchedule(-1), m_zone(-1), m_endTime(0), m_eventTime(0)
{
}

void runStateClass::LogSchedule()
{
#ifdef LOGGING
	if ((m_eventTime > 0) && (m_zone >= 0))
		logger.LogZoneEvent(m_eventTime, m_zone, nntpTimeServer.LocalNow() - m_eventTime, m_bSchedule ? m_iSchedule+1:-1, m_adj.seasonal, m_adj.wunderground);
#endif
}

void runStateClass::SetSchedule(bool val, int8_t iSched, const runStateClass::DurationAdjustments * adj)
{
	LogSchedule();
	m_bSchedule = val;
	m_bManual = false;
	m_zone = -1;
	m_endTime = 0;
	m_iSchedule = val?iSched:-1;
	m_eventTime = nntpTimeServer.LocalNow();
	m_adj = adj?*adj:DurationAdjustments();
}

void runStateClass::ContinueSchedule(int8_t zone, short endTime)
{
	LogSchedule();
	m_bSchedule = true;
	m_bManual = false;
	m_zone = zone;
	m_endTime = endTime;
	m_eventTime = nntpTimeServer.LocalNow();
}

void runStateClass::SetManual(bool val, int8_t zone)
{
	LogSchedule();
	m_bSchedule = false;
	m_bManual = val;
	m_zone = zone;
	m_endTime = 0;
	m_iSchedule = -1;
	m_eventTime = nntpTimeServer.LocalNow();
	m_adj=DurationAdjustments();
}

#ifdef ARDUINO
uint8_t ZoneToIOMap[] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
#endif
#if defined(GREENIQ)
uint8_t ZoneToIOMap[] = {5, 7, 0, 1, 2, 3, 4};
#else
uint8_t ZoneToIOMap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
#define SR_CLK_PIN  7
#define SR_NOE_PIN  0
#define SR_DAT_PIN  2
#define SR_LAT_PIN  3
#endif

#define UTILITY_WATER_GPIO 10
#define FLOWMETER_CHECK_TIME 15
#define K_FLUX 0.0181 //lt / impulso

#define PUMP_STATES \
    PUMP_STATE(PS_STOPPED) \
    PUMP_STATE(PS_START_DELAY) \
    PUMP_STATE(PS_ACTIVE) \
    PUMP_STATE(PS_ERROR) \
    PUMP_STATE(PS_PRIMING) \
    PUMP_STATE(PS_UTILITY_WATER) \

enum PumpState {
#define PUMP_STATE(s) s,
PUMP_STATES
#undef PUMP_STATE
};

static const char * PUMP_STATES_STR[] = {
#define PUMP_STATE(s) #s,
PUMP_STATES
#undef PUMP_STATE
    0
};

static uint16_t outState;
static uint16_t prevOutState;
static std::atomic_ullong theFlowmeterPulseCount;
static std::thread        theFlowmeterThread;
static volatile bool      flowmeterShouldRun;
static float              theFlow;
static enum PumpState thePumpState;
static time_t             theStartupTime;


static void flowmeter_thread(void)
{
    system("echo 21 > /sys/class/gpio/export");
    system("echo in > /sys/class/gpio/gpio21/direction");
    system("echo rising > /sys/class/gpio/gpio21/edge");

    int gpioFd = open("/sys/class/gpio/gpio21/value", O_RDONLY);
    if(gpioFd == -1)
    {
        printf("Gpio open error\n");
        return;
    }

    while(flowmeterShouldRun)
    {
        struct pollfd pf[1] = {};
        pf[0].fd = gpioFd;
        pf[0].events = POLLPRI | POLLERR;

        int rv = poll(pf,1,100);
        if( rv != 1) continue;

        lseek(gpioFd,0,SEEK_SET);
        char v;
        read(gpioFd,&v,1);
        if( v == '1')
            theFlowmeterPulseCount++;
    }
    close(gpioFd);
}

float TotalLitres()
{
	return theFlowmeterPulseCount * K_FLUX;
}
float flow()
{
    return theFlow;
}
time_t uptime()
{
    //monotonic cloc starts from 0
    return getTimeMonotonic() - theStartupTime;
}
const char * pumpState()
{
    assert( thePumpState < ARRAY_SIZE ( PUMP_STATES_STR ) );
    return PUMP_STATES_STR[ thePumpState ];
}

static void io_latch()
{
	// check if things have changed
	if (outState == prevOutState)
		return;

	const EOT eot = GetOT();
	switch (eot)
	{
	case OT_NONE:
#ifndef ARDUINO
#ifdef EXTERNAL_SCRIPT
        struct stat buffer;
        char cmd[50];
        if (stat(EXTERNAL_SCRIPT, &buffer) == 0) {
            for (int i = 0; i <= NUM_ZONES; i++)
            {
                sprintf(cmd, "%s %i %i", EXTERNAL_SCRIPT, i, (outState&(0x01<<i))?1:0);
                system(cmd);
            }
        }
#endif
#endif
		break;
	case OT_DIRECT_POS:
	case OT_DIRECT_NEG:
		//@fede: start with 1 to prevent control of the pump here
		for (int i = 1; i <= NUM_ZONES; i++)
		{
			if (eot == OT_DIRECT_POS)
				digitalWrite(ZoneToIOMap[i], (outState&(0x01<<i))?1:0);
			else
				digitalWrite(ZoneToIOMap[i], (outState&(0x01<<i))?0:1);
		}
		break;

	case OT_OPEN_SPRINKLER:
#ifndef ARDUINO
#ifndef GREENIQ
		// turn off the latch pin
		digitalWrite(SR_LAT_PIN, 0);
		digitalWrite(SR_CLK_PIN, 0);

		for (uint8_t i = 0; i < 16; i++)
		{
			digitalWrite(SR_CLK_PIN, 0);
			digitalWrite(SR_DAT_PIN, outState&(0x01<<(15-i)));
			digitalWrite(SR_CLK_PIN, 1);
		}
		// latch the outputs
		digitalWrite(SR_LAT_PIN, 1);

		// Turn off the NOT enable pin (turns on outputs)
		digitalWrite(SR_NOE_PIN, 0);
#endif
#endif
		break;
	}

	// Now store the new output state so we know if things have changed
	prevOutState = outState;
}

void io_setup()
{
	const EOT eot = GetOT();
	if ((eot != OT_NONE))
	{

#ifndef ARDUINO
		if (geteuid() != 0)
		{
			trace("You need to be root to run this.  Setting output mode to NONE\n");
			SetOT(OT_NONE);
			return;
		}
		else if (wiringPiSetup() == -1)
		{
			trace("Failed to Setup Outputs\n");
		}
#endif
		if (eot == OT_OPEN_SPRINKLER)
		{
#ifndef GREENIQ
			pinMode(SR_CLK_PIN, OUTPUT);
			digitalWrite(SR_CLK_PIN, 0);
			pinMode(SR_NOE_PIN, OUTPUT);
			digitalWrite(SR_NOE_PIN, 0);
			pinMode(SR_DAT_PIN, OUTPUT);
			digitalWrite(SR_DAT_PIN, 0);
			pinMode(SR_LAT_PIN, OUTPUT);
			digitalWrite(SR_LAT_PIN, 0);
#endif
		}
		else
		{
			for (uint8_t i=0; i<sizeof(ZoneToIOMap); i++)
			{
				pinMode(ZoneToIOMap[i], OUTPUT);
				digitalWrite(ZoneToIOMap[i], (eot==OT_DIRECT_NEG)?1:0);
			}
			//setup the input for the flowmeter
			//wiringPiISR(29,INT_EDGE_FALLING,flowmeter_isr);
            flowmeterShouldRun = 1;
            theFlowmeterThread = std::thread(&flowmeter_thread);
			trace("Setup pin mode\n");
		}
	}
	outState = 0;
	prevOutState = 1;
	io_latch();
}

void io_latchNow()
{
	io_latch();
}

void TurnOffZones()
{
	trace(F("Turning Off All Zones\n"));
	outState = 0;
}

bool isZoneOn(int iNum)
{
	if ((iNum <= 0) || (iNum > NUM_ZONES))
		return false;
	return outState & (0x01 << iNum);
}

bool pumpDesiredState()
{
	return outState & (0x01);
}

static void pumpControl(bool val)
{
	if (val)
		outState |= 0x01;
	else
		outState &= ~0x01;
}

void TurnOnZone(int iValve)
{
	trace(F("Turning on Zone %d\n"), iValve);
	if ((iValve <= 0) || (iValve > NUM_ZONES))
		return;

	ShortZone zone;
	LoadShortZone(iValve - 1, &zone);
	outState = 0x01 << iValve;
	// Turn on the pump if necessary
	pumpControl(zone.bPump);
}

// Adjust the durations based on atmospheric conditions
static runStateClass::DurationAdjustments AdjustDurations(Schedule * sched)
{
	runStateClass::DurationAdjustments adj(100);
	if (sched->IsWAdj()) {
#if defined(WEATHER_WUNDERGROUND)
		Wunderground w;
#elif defined(WEATHER_AERIS)
		Aeris w;
#elif defined(WEATHER_DARKSKY)
		DarkSky w;
#elif defined(WEATHER_OPENWEATHER)
        OpenWeather w;
#elif defined(WEATHER_OPENMETEO)
        OpenMeteo w;		
#else
		// this is a dummy provider which will just result in 100
		Weather w;
#endif
		// get factor to adjust times by.  100 = 100% (i.e. no adjustment)
		adj.wunderground = w.GetScale();
	}
	adj.seasonal = GetSeasonalAdjust();
	long scale = ((long)adj.seasonal * (long)adj.wunderground) / 100;
	for (uint8_t k = 0; k < NUM_ZONES; k++)
		sched->zone_duration[k] = (uint8_t)spi_min(((long)sched->zone_duration[k] * scale + 50) / 100, 255);
	return adj;
}

// Load the on/off events for a specific schedule/time or the quick schedule
void LoadSchedTimeEvents(uint8_t sched_num, bool bQuickSchedule)
{
	Schedule sched;
	runStateClass::DurationAdjustments adj;
	if (!bQuickSchedule)
	{
		const uint8_t iNumSchedules = GetNumSchedules();
		if ((sched_num < 0) || (sched_num >= iNumSchedules))
			return;
		LoadSchedule(sched_num, &sched);
		adj=AdjustDurations(&sched);
	}
	else
		sched = quickSchedule;

	const time_t local_now = nntpTimeServer.LocalNow();
	short start_time = (local_now - previousMidnight(local_now)) / 60;

	for (uint8_t k = 0; k < NUM_ZONES; k++)
	{
		ShortZone zone;
		LoadShortZone(k, &zone);
		if (zone.bEnabled && (sched.zone_duration[k] > 0))
		{
			if (iNumEvents >= MAX_EVENTS - 1)
			{  // make sure we have room for the on && the off events.. hence the -1
				trace(F("ERROR: Too Many Events!\n"));
			}
			else
			{
				events[iNumEvents].time = start_time;
				events[iNumEvents].command = 0x01; // Turn on a zone
				events[iNumEvents].data[0] = k + 1; // Zone to turn on
				events[iNumEvents].data[1] = (start_time + sched.zone_duration[k]) >> 8;
				events[iNumEvents].data[2] = (start_time + sched.zone_duration[k]) & 0x00FF;
				iNumEvents++;
				start_time += sched.zone_duration[k];
			}
		}
	}
	// Load up the last turn off event.
	events[iNumEvents].time = start_time;
	events[iNumEvents].command = 0x02; // Turn off all zones
	events[iNumEvents].data[0] = 0;
	events[iNumEvents].data[1] = 0;
	events[iNumEvents].data[2] = 0;
	iNumEvents++;
	runState.SetSchedule(true, bQuickSchedule?99:sched_num, &adj);
}

void ClearEvents()
{
	iNumEvents = 0;
	runState.SetSchedule(false);
}

// TODO:  Schedules that go past midnight!
//  Pretty simple.  When we one-shot at midnight, check to see if any outstanding events are at time >1400.  If so, move them
//  to the top of the event stack and subtract 1440 (24*60) from their times.

// Loads the events for the current day
void ReloadEvents(bool bAllEvents)
{
	ClearEvents();
	TurnOffZones();

	// Make sure we're running now
	if (!GetRunSchedules())
		return;

	const time_t time_now = nntpTimeServer.LocalNow();
	const uint8_t iNumSchedules = GetNumSchedules();
	for (uint8_t i = 0; i < iNumSchedules; i++)
	{
		Schedule sched;
		LoadSchedule(i, &sched);
		if (sched.IsRunToday(time_now))
		{
			// now load up events for each of the start times.
			for (uint8_t j = 0; j <= 3; j++)
			{
				const short start_time = sched.time[j];
				if (start_time != -1)
				{
					if (!bAllEvents && (start_time <= (long)(time_now - previousMidnight(time_now))/60 ))
						continue;
					if (iNumEvents >= MAX_EVENTS)
					{
						trace(F("ERROR: Too Many Events!\n"));
					}
					else
					{
						events[iNumEvents].time = start_time;
						events[iNumEvents].command = 0x03;  // load events for schedule i, time j
						events[iNumEvents].data[0] = i;
						events[iNumEvents].data[1] = j;
						events[iNumEvents].data[2] = 0;
						iNumEvents++;
					}
				}
			}
		}
	}
}

// Check to see if there are any events that need to be processed.
static void ProcessEvents()
{
	const time_t local_now = nntpTimeServer.LocalNow();
	const short time_check = (local_now - previousMidnight(local_now)) / 60;
	for (uint8_t i = 0; i < iNumEvents; i++)
	{
		if (events[i].time == -1)
			continue;
		if (time_check >= events[i].time)
		{
			switch (events[i].command)
			{
			case 0x01:  // turn on valves in data[0]
				TurnOnZone(events[i].data[0]);
				runState.ContinueSchedule(events[i].data[0], events[i].data[1] << 8 | events[i].data[2]);
				events[i].time = -1;
				break;
			case 0x02:  // turn off all valves
				TurnOffZones();
				runState.SetSchedule(false);
				events[i].time = -1;
				break;
			case 0x03:  // load events for schedule(data[0]) time(data[1])
				if (runState.isSchedule())  // If we're already running a schedule, push this off 1 minute
					events[i].time++;
				else
				{
					// Load all the individual events for the individual zones on/off
					LoadSchedTimeEvents(events[i].data[0]);
					events[i].time = -1;
				}
				break;
			};
		}
	}
}
static void process_tankFillingPump()
{
	const time_t local_now = getTimeMonotonic();
	const int64_t now_ms = getTimeMonotonicMs();
    static time_t pumpStartTime;
    static time_t errorStartTime;
    static time_t primingStartTime;
    static time_t lastFlowChanged;
    static int64_t lastFlowSampled;
    static uint64_t flowPulseCount;
    static uint64_t flowLastSampledPulseCount;
    static bool     primingDone;


    if( now_ms - lastFlowSampled > 1000 )
    {
        float f = ( theFlowmeterPulseCount - flowLastSampledPulseCount ) * K_FLUX / (  now_ms - lastFlowSampled ) * 1000. * 60;
        theFlow = theFlow * 0.8 + f * 0.2;
        flowLastSampledPulseCount = theFlowmeterPulseCount;
        lastFlowSampled = now_ms;
    }


    switch(thePumpState)
    {
    case PS_STOPPED:
        if( pumpDesiredState() )
        {
            pumpStartTime = local_now;
            trace("*** TURN ON\n");
            digitalWrite( 0, 1 );
            thePumpState = PS_START_DELAY;
            primingDone = false;
        }
        break;
    case PS_START_DELAY:
        if( local_now - pumpStartTime > FLOWMETER_CHECK_TIME )
        {
            flowPulseCount = theFlowmeterPulseCount;
            lastFlowChanged = local_now;
            thePumpState = PS_ACTIVE;
        }
        if( !pumpDesiredState() )
        {
            thePumpState = PS_STOPPED;
            digitalWrite( 0, 0 );
        }
        break;
    case PS_ACTIVE:
        if( theFlowmeterPulseCount - flowPulseCount > K_FLUX * 2) //2 lt
        {
            flowPulseCount = theFlowmeterPulseCount;
            lastFlowChanged = local_now;
        }
        if( local_now - lastFlowChanged > 10)
        {
            if( !primingDone )
            {
                trace("Flow interrupted, try to prime the pump\n");
                thePumpState = PS_PRIMING;
                primingStartTime = local_now;
                digitalWrite( 0, 0 );
                digitalWrite( UTILITY_WATER_GPIO, 1 );
            }
            else
            {
                trace("Still no flow, switch to utility water\n");
                thePumpState = PS_UTILITY_WATER;
                digitalWrite( 0, 0 );
                digitalWrite( UTILITY_WATER_GPIO, 1 );
            }
        }
        if( local_now - pumpStartTime > 30*60 )
        {
            trace("Max pump running time exceeded\n");
            thePumpState = PS_ERROR;
            errorStartTime = local_now;
            digitalWrite( 0, 0 );
        }
        if( !pumpDesiredState() )
        {
            trace("*** TURN OFF\n");
            thePumpState = PS_STOPPED;

            //discharge pressure with section 1
            digitalWrite( 1, 1 );
            //stop pump
            digitalWrite( 0, 0 );
            usleep(300*100);
            trace("Discarge pressure\n");
            digitalWrite( 1, 0 );
        }
        break;
    case PS_PRIMING:
        //pump some water in reverse to fill the pipe
        if( local_now - primingStartTime > 2*60 )
        {
            trace("Priming done, try to restart the pump\n");
            trace("*** TURN ON\n");
            pumpStartTime = local_now;
            primingDone = true;
            digitalWrite( 0, 1 );
            digitalWrite( UTILITY_WATER_GPIO, 0 );
            thePumpState = PS_START_DELAY;
        }
        if( !pumpDesiredState() )
        {
            thePumpState = PS_STOPPED;
            digitalWrite( 0, 0 );
            digitalWrite( UTILITY_WATER_GPIO, 0 );
        }
        break;
    case PS_UTILITY_WATER:
        if( local_now - pumpStartTime > 30*60 )
        {
            trace("Max pump running time exceeded\n");
            thePumpState = PS_ERROR;
            errorStartTime = local_now;
            digitalWrite( UTILITY_WATER_GPIO, 0 );
        }
        if( !pumpDesiredState() )
        {
            trace("*** TURN OFF\n");
            thePumpState = PS_STOPPED;
            digitalWrite( UTILITY_WATER_GPIO, 0 );
        }
        break;
    case PS_ERROR:
        if( local_now - errorStartTime > 4* 3600)
            thePumpState = PS_STOPPED;
        break;
    }
}
void mainLoop()
{
	static bool firstLoop = true;
	static bool bDoneMidnightReset = false;
	if (firstLoop)
	{
		firstLoop = false;
		freeMemory();

		if (IsFirstBoot())
			ResetEEPROM();
		io_setup();

#ifdef LOGGING
		if (!logger.Init())
			exit(EXIT_FAILURE);
#endif

		TurnOffZones();
		ClearEvents();

		//Init the web server
		if (!webServer.Init())
			exit(EXIT_FAILURE);

#ifdef ARDUINO
		//Init the TFTP server
		tftpServer.Init();
#endif

		// Set the clock.
		nntpTimeServer.checkTime();

		ReloadEvents();
		//ShowSockStatus();
		thePumpState = PS_STOPPED;
        theStartupTime = getTimeMonotonic();
	}

	// Check to see if we need to set the clock and do so if necessary.
	nntpTimeServer.checkTime();

	const time_t timeNow = nntpTimeServer.LocalNow();
	// One shot at midnight
	if ((hour(timeNow) == 0) && !bDoneMidnightReset)
	{
		trace(F("Reloading Midnight\n"));
		bDoneMidnightReset = true;
		// TODO:  outstanding midnight events.  See other TODO for how.
		ReloadEvents(true);
	}
	else if (hour(timeNow) != 0)
		bDoneMidnightReset = false;

	//  See if any web clients have connected
	webServer.ProcessWebClients();

	// Process any pending events.
	process_tankFillingPump();
	ProcessEvents();

#ifdef ARDUINO
	// Process the TFTP Server
	tftpServer.Poll();
#else
	// if we've changed the settings, store them to disk
	EEPROM.Store();
#endif

	// latch any output modifications
	io_latch();
}

void stop()
{
    TurnOffZones();
    io_latch();
    process_tankFillingPump();
    
    uint64_t pls = theFlowmeterPulseCount;
    trace("Total flowmeter pulses %llu\n",pls);
    flowmeterShouldRun = 0;
    theFlowmeterThread.join();

    digitalWrite( 0, 0 );
    digitalWrite( UTILITY_WATER_GPIO, 0 );
}
