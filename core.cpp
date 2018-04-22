// core.cpp
// This file constitutes the core functions that run the scheduling for the Sprinkler system.
// Author: Richard Zimmerman
// Copyright (c) 2013 Richard Zimmerman
// 

#include "core.h"
#include "settings.h"
#include "Weather.h"
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
#endif
#include <atomic>
#include <assert.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*a))

#ifdef LOGGING
Logging log;
#endif
static web webServer;
nntp nntpTimeServer;
runStateClass runState;

// A bitfield that defines which zones are currently on.
int ZoneState = 0;

runStateClass::runStateClass() : m_bSchedule(false), m_bManual(false), m_iSchedule(-1), m_zone(-1), m_endTime(0), m_eventTime(0), m_paused(false)
{
}

void runStateClass::LogSchedule()
{
#ifdef LOGGING
	if ((m_eventTime > 0) && (m_zone >= 0))
		log.LogZoneEvent(m_eventTime, m_zone, nntpTimeServer.LocalNow() - m_eventTime, m_bSchedule ? m_iSchedule+1:-1, m_adj.seasonal, m_adj.wunderground);
#endif
}

void runStateClass::SetSchedule(bool val, int8_t iSched, const runStateClass::DurationAdjustments * adj)
{
	LogSchedule();
	m_bSchedule = val;
	m_bManual = false;
	m_paused = false;
	m_zone = -1;
	m_endTime = 0;
	m_iSchedule = val?iSched:-1;
	m_eventTime = nntpTimeServer.LocalNow();
	m_adj = adj?*adj:DurationAdjustments();
}

void runStateClass::ContinueSchedule(int8_t zone, int64_t endTime)
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
	m_paused = false;
	m_bManual = val;
	m_zone = zone;
	m_endTime = 0;
	m_iSchedule = -1;
	m_eventTime = nntpTimeServer.LocalNow();
	m_adj=DurationAdjustments();
}
void runStateClass::PauseSchedule()
{
	m_paused = true;
}
void runStateClass::ResumeSchedule()
{
	m_paused = false;
}

#ifdef ARDUINO
uint8_t ZoneToIOMap[] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37};
#else
uint8_t ZoneToIOMap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
#define SR_CLK_PIN  7
#define SR_NOE_PIN  0
#define SR_DAT_PIN  2
#define SR_LAT_PIN  3
#endif

#define PIN_TANK_FILL_PUMP 10
#define PIN_LOW_LEVEL 25
#define PIN_EMPTY     24

enum EventState{
	CS_WAIT,
	CS_RUN,
	CS_PAUSED,
	CS_DONE,
};
static const char * EVENT_STATES[] = {
	"CS_WAIT",
	"CS_RUN",
	"CS_PAUSED",
	"CS_DONE",
	0,
};

/*
 * FILLING1 : between low and request water
 * FILLING2 : after request water ( time based )
 */
#define TANK_STATES \
    TANK_STATE(TS_FILLING_1) \
    TANK_STATE(TS_FILLING_2) \
    TANK_STATE(TS_FULL) \
    TANK_STATE(TS_ERROR) \
    TANK_STATE(TS_UNKNOWN)

enum TankState {
#define TANK_STATE(s) s,
TANK_STATES
#undef TANK_STATE
};

static const char * TANK_STATES_STR[] = {
#define TANK_STATE(s) #s,
TANK_STATES
#undef TANK_STATE
    0
};

#define PUMP_STATES \
    PUMP_STATE(PS_STOPPED) \
    PUMP_STATE(PS_START_DELAY) \
    PUMP_STATE(PS_ACTIVE) \

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
static Event    theCurrentEvent;
static EventState theCurrentEventState;
static enum TankState  theCurrentTankState;
static int32_t  theCurrentEventElapsed;
static int32_t  theCurrentEventPaused;
static int32_t  theCurrentEventStartTime;
static std::atomic_ullong theFlowmeterPulseCount;
static TankStatus theTankData;
static enum PumpState thePumpState;

static void flowmeter_isr(void)
{
	static uint64_t lastIsr_ns = 0;
	timespec now;
	clock_gettime(CLOCK_MONOTONIC,&now);

	digitalRead(29); //ack the isr

	uint64_t now_ns = now.tv_sec * 1000000000ll+now.tv_nsec;
	if( !lastIsr_ns)
		lastIsr_ns = now_ns;
	if( now_ns - lastIsr_ns < 10000)
		return;
	lastIsr_ns = now_ns;
	theFlowmeterPulseCount++;

}

float TotalLitres()
{
	return theFlowmeterPulseCount / 4.8;
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
		break;
	case OT_DIRECT_POS:
	case OT_DIRECT_NEG:
		for (int i = 0; i <= NUM_ZONES; i++)
		{
			if (eot == OT_DIRECT_POS)
				digitalWrite(ZoneToIOMap[i], (outState&(0x01<<i))?1:0);
			else
				digitalWrite(ZoneToIOMap[i], (outState&(0x01<<i))?0:1);
		}
		break;

	case OT_OPEN_SPRINKLER:
#ifndef ARDUINO
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
		else if (wiringPiSetupSys() == -1) //setup in sys mode for interrupts
		{
			trace("Failed to Setup Outputs\n");
		}
#endif
		if (eot == OT_OPEN_SPRINKLER)
		{
			pinMode(SR_CLK_PIN, OUTPUT);
			digitalWrite(SR_CLK_PIN, 0);
			pinMode(SR_NOE_PIN, OUTPUT);
			digitalWrite(SR_NOE_PIN, 0);
			pinMode(SR_DAT_PIN, OUTPUT);
			digitalWrite(SR_DAT_PIN, 0);
			pinMode(SR_LAT_PIN, OUTPUT);
			digitalWrite(SR_LAT_PIN, 0);
		}
		else
		{
			for (uint8_t i=0; i<sizeof(ZoneToIOMap); i++)
			{
				pinMode(ZoneToIOMap[i], OUTPUT);
				digitalWrite(ZoneToIOMap[i], (eot==OT_DIRECT_NEG)?1:0);
			}
			//setup the input for the flowmeter
			wiringPiISR(29,INT_EDGE_FALLING,flowmeter_isr);
            //setup the external pump
            digitalWrite( PIN_TANK_FILL_PUMP, 0 );
            pinMode(PIN_TANK_FILL_PUMP, OUTPUT);
            pinMode(PIN_LOW_LEVEL, INPUT);
            pinMode(PIN_EMPTY, INPUT);
		}
	}
	outState = 0;
	prevOutState = 1;
	io_latch();
}


static void TurnOffZones()
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

static void pumpControl(bool val)
{
	if (val)
		outState |= 0x01;
	else
		outState &= ~0x01;
}

static void TurnOnZone(int iValve)
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
	if (sched->IsWAdj())
	{
		Weather w;
		char key[17];
		GetApiKey(key);
		char pws[12] = {0};
		GetPWS(pws);
		adj.wunderground = w.GetScale(key, GetZip(), pws, GetUsePWS());   // factor to adjust times by.  100 = 100% (i.e. no adjustment)
	}
	adj.seasonal = GetSeasonalAdjust();
	long scale = ((long)adj.seasonal * (long)adj.wunderground) / 100;
	for (uint8_t k = 0; k < NUM_ZONES; k++)
		sched->zone_duration[k] = spi_min(((long)sched->zone_duration[k] * scale + 50) / 100, 254);
	return adj;
}

// return true if the schedule is enabled and runs today.
static inline bool IsRunToday(const Schedule & sched, time_t time_now)
{
	if ((sched.IsEnabled())
			&& (((sched.IsInterval()) && ((elapsedDays(time_now) % sched.interval) == 0))
					|| (!(sched.IsInterval()) && (sched.day & (0x01 << (weekday(time_now) - 1))))))
		return true;
	return false;
}

void QuickSchedule(const Schedule & sched)
{
	const time_t local_now = nntpTimeServer.LocalNow();
	short start_time = (local_now - previousMidnight(local_now)) / 60;

	for (uint8_t k = 0; k < NUM_ZONES; k++)
	{
		ShortZone zone;
		LoadShortZone(k, &zone);
		if (zone.bEnabled && (sched.zone_duration[k] > 0))
		{
			Event e;
			e.time = start_time;
			e.duration = sched.zone_duration[k];
			e.zone = k;
			e.sched_num = 99;
			e.isQuickSchedule = true;
			eventQueuePush(e);
			trace("Added quick schedule %d %d\n",k,e.duration);
		}
	}
}


// TODO:  Schedules that go past midnight!
//  Pretty simple.  When we one-shot at midnight, check to see if any outstanding events are at time >1400.  If so, move them
//  to the top of the event stack and subtract 1440 (24*60) from their times.

// Loads the events for the current day
void ReloadEvents(bool bAllEvents)
{
	eventQueueClear();
	runState.SetSchedule(false);
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
		if (IsRunToday(sched, time_now))
		{
			// now load up events for each of the start times.
			for (uint8_t j = 0; j <= 3; j++)
			{
				const short start_time = sched.time[j];
				if (start_time != -1)
				{
					if (!bAllEvents && (start_time <= (long)(time_now - previousMidnight(time_now))/60 ))
						continue;
					for (uint8_t k = 0; k < NUM_ZONES; k++)
					{
						ShortZone zone;
						LoadShortZone(k, &zone);
						if (zone.bEnabled && (sched.zone_duration[k] > 0))
						{
							Event e;
							e.time = start_time;
							e.duration = sched.zone_duration[k];
							e.zone = k;
							e.sched_num = i;
							eventQueuePush(e);
						}
					}
				}
			}
		}
	}
	trace("Loaded %d events\n",iNumEvents);
}

bool shouldPause()
{
	return false;
}
// Check to see if there are any events that need to be processed.
void ManualTurnOnZone(int iValve)
{
	const time_t local_now = nntpTimeServer.LocalNow();
	short start_time = (local_now - previousMidnight(local_now)) / 60;
	Event e;
	e.time = start_time;
	e.duration = 60;
	e.zone = iValve - 1;
	e.sched_num = 99;
	e.isManual = true;
	eventQueuePush(e);
	TurnOnZone(iValve);
}
void ManualTurnOffZones()
{
	//reset everything
	TurnOffZones();
	eventQueueClear();
	theCurrentEvent = Event();
	theCurrentEventState = CS_DONE;
	runState.SetManual(false);
}
static void ProcessEvents()
{
	const time_t local_now = nntpTimeServer.LocalNow();
	const short time_check = (local_now - previousMidnight(local_now)) / 60;
	EventState ev = theCurrentEventState;
	switch(theCurrentEventState)
	{
	case CS_WAIT:
		if( time_check >= theCurrentEvent.time)
		{
			int sched_num = theCurrentEvent.sched_num;
			Schedule sched;
			runStateClass::DurationAdjustments adj;
			if (!theCurrentEvent.isQuickSchedule && !theCurrentEvent.isManual)
			{
				const uint8_t iNumSchedules = GetNumSchedules();
				if ((sched_num < 0) || (sched_num >= iNumSchedules))
					return;
				LoadSchedule(sched_num, &sched);
				adj=AdjustDurations(&sched);

				trace("Set duration to %d from %d\n",sched.zone_duration[theCurrentEvent.zone],theCurrentEvent.duration);
				theCurrentEvent.duration = sched.zone_duration[theCurrentEvent.zone];
			}
			else
			{
				trace("Quick/Manual duration %d\n",theCurrentEvent.duration);
			}
			if ( theCurrentEvent.duration )
			{
				//TODO set runState
				//start the zone
				TurnOnZone(theCurrentEvent.zone + 1);
				theCurrentEventElapsed = 0;
				theCurrentEventPaused = 0;
				theCurrentEventStartTime = local_now;
				theCurrentEventState = CS_RUN;
				if(theCurrentEvent.isManual)
				{
					runState.SetManual(true,theCurrentEvent.zone + 1);
				}
				else
				{
					runState.SetSchedule(true, theCurrentEvent.isQuickSchedule ? 99 : sched_num, &adj);
					runState.ContinueSchedule(theCurrentEvent.zone + 1,local_now + theCurrentEvent.duration * 60 + 1);
				}
			}
			else
			{
				//drop event	
				theCurrentEvent = Event();
				theCurrentEventState = CS_DONE;
				trace("Drop 0 duration event\n");
			}
		}
		break;
	case CS_RUN:
		if( shouldPause())
		{
			//stop the zone and calculate the elapsed time
			TurnOffZones();
			theCurrentEventElapsed += local_now - theCurrentEventStartTime;
			theCurrentEventStartTime = local_now;
			theCurrentEventState = CS_PAUSED;
			runState.PauseSchedule();
		}
		else if ( theCurrentEventElapsed + local_now - theCurrentEventStartTime > theCurrentEvent.duration * 60l)
		{
			TurnOffZones();
			//clear the currentEvent
			theCurrentEvent = Event();
			theCurrentEventState = CS_DONE;
			runState.SetSchedule(false);
		}
		break;
	case CS_PAUSED:	
		if( !shouldPause())
		{
			//restart
			TurnOnZone(theCurrentEvent.zone + 1);
			theCurrentEventPaused += local_now - theCurrentEventStartTime;
			theCurrentEventStartTime = local_now;
			theCurrentEventState = CS_RUN;
			runState.ResumeSchedule();
		}
		break;
	case CS_DONE:	
		if(!theCurrentEvent.isValid())
		{
			theCurrentEvent = eventQueueHead();
			eventQueuePop();
			if(theCurrentEvent.isValid())
				theCurrentEventState = CS_WAIT;
		}
		break;
	}
	if( theCurrentEventState != ev)
	{
		trace("Tran %s -> %s\n", EVENT_STATES[ev],EVENT_STATES[theCurrentEventState]);	
	}
}

static void process_tankFillingPump()
{
	const time_t local_now = nntpTimeServer.LocalNow();
    static time_t pumpStartTime;
    switch(thePumpState)
    {
    case PS_STOPPED:
        if( theTankData.tankPumpDesired )
        {
            pumpStartTime = local_now;
            digitalWrite( PIN_TANK_FILL_PUMP, 1 );
            thePumpState = PS_START_DELAY;
        }
        break;
    case PS_START_DELAY:
        if( local_now - pumpStartTime > tankSettings.flowmeterCheckTime )
            thePumpState = PS_ACTIVE;
        if( !theTankData.tankPumpDesired )
        {
            thePumpState = PS_STOPPED;
            digitalWrite( PIN_TANK_FILL_PUMP, 0 );
        }
        break;
    case PS_ACTIVE:
        //TODO insert flowmeter check
        if( local_now - pumpStartTime > 1200 )
        {
            trace("Max pump running time exceeded\n");
            thePumpState = PS_STOPPED;
            digitalWrite( PIN_TANK_FILL_PUMP, 0 );
        }
        if( !theTankData.tankPumpDesired )
        {
            thePumpState = PS_STOPPED;
            digitalWrite( PIN_TANK_FILL_PUMP, 0 );
        }
        break;
    }
}
static void startTankFilling()
{
    theTankData.tankPumpDesired = 1;
}
static void stopTankFilling()
{
    theTankData.tankPumpDesired = 0;
}
static bool tankNeedsRefill()
{
    return theTankData.lowLevelInput;
}
static bool tankEmpty()
{
    return theTankData.emptyInput;
}
static void process_tank()
{
	const time_t local_now = nntpTimeServer.LocalNow();
    static time_t tankFillingStart;

    theTankData.lowLevelInput = digitalRead( PIN_LOW_LEVEL );
    theTankData.emptyInput = digitalRead( PIN_EMPTY );

    switch( theCurrentTankState )
    {
    case TS_UNKNOWN:
        if( !tankNeedsRefill() && !tankEmpty() )
        {
            theCurrentTankState = TS_FULL;
        }
        else if ( !tankNeedsRefill() && tankEmpty() )
        {
            //invalid combination
            if( local_now % 10 == 0 )
                trace("INVALID tank combination, stay here\n");
        }
        else if ( tankNeedsRefill() && !tankEmpty() )
        {
            //we're between level 1 and 2, start with FILL 1
            startTankFilling();
            tankFillingStart = local_now;
            theCurrentTankState = TS_FILLING_1;
        }
        else /*if ( tankNeedsRefill() && tankEmpty() ) */
        {
            //we're empty, start full refill
            startTankFilling();
            tankFillingStart = local_now;
            theCurrentTankState = TS_FILLING_1;
        }
        break;
    case TS_FULL:
        if( tankNeedsRefill() || tankEmpty())
        {
            tankFillingStart = local_now;
            startTankFilling();
            theCurrentTankState = TS_FILLING_1;
        }
        break;
    case TS_FILLING_1:
        if ( !tankNeedsRefill() )
        {
            tankFillingStart = local_now;
            theCurrentTankState = TS_FILLING_2;
        }
        if ( local_now - tankFillingStart > 60 )
        {
            trace("Tank filling 1 timeout\n");
            stopTankFilling();
            theCurrentTankState = TS_ERROR;
        }
        break;
    case TS_FILLING_2:
        if ( local_now - tankFillingStart > 10 )
        {
            stopTankFilling();
            theCurrentTankState = TS_FULL;
        }
        break;
    case TS_ERROR:
        break;
    }
}
TankStatus TankState()
{
    assert( theCurrentTankState < ARRAY_SIZE( TANK_STATES_STR ) );
    assert( thePumpState < ARRAY_SIZE ( PUMP_STATES_STR ) );
    theTankData.fsmState =  TANK_STATES_STR[ theCurrentTankState ];
    theTankData.pumpState = PUMP_STATES_STR[ thePumpState ];
    return theTankData;
}
TankSettings tankSettings;
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
		if (!log.Init())
			exit(EXIT_FAILURE);
#endif

		TurnOffZones();
		eventQueueClear();

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
		//
		theCurrentEventState = CS_DONE;
        theCurrentTankState = TS_UNKNOWN;
        thePumpState = PS_STOPPED;
        tankSettings.step2FillTime = 10;
        tankSettings.step1FillTimeout = 60;
        tankSettings.flowmeterCheckTime = 5;
        tankSettings.ignoreFlowmeter = 1;
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
    process_tank();
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
