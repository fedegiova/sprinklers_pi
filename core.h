// core.h
// This file constitutes the core functions that run the scheduling for the Sprinkler system.
// Author: Richard Zimmerman
// Copyright (c) 2013 Richard Zimmerman
//

#ifndef _CORE_h
#define _CORE_h

#ifdef ARDUINO
#include "nntp.h"
#endif
#include <inttypes.h>
#include "port.h"
#ifdef LOGGING
#include "Logging.h"
extern Logging log;
#endif

#ifndef VERSION
#define VERSION "0.0.0"
#endif

class Schedule;

void mainLoop();
void stop();
void ClearEvents();
void QuickSchedule(const Schedule & sched);
void ReloadEvents(bool bAllEvents = false);
bool isZoneOn(int iNum);
void ManualTurnOnZone(int iValve);
void ManualTurnOffZones();
float TotalLitres();
struct TankStatus{
    const char * fsmState;
    const char * pumpState;
    const char * filterState;
    bool         tankPumpOutput;
    bool         tankPumpDesired;
    bool         filterWaterDesired;
    bool         lowLevelInput;
    bool         emptyInput;
    bool         fullInput;
};
TankStatus TankState();
void io_setup();

class runStateClass
{
public:
	class DurationAdjustments {
	public:
		DurationAdjustments() : seasonal(-1), wunderground(-1) {}
		DurationAdjustments(int16_t val) : seasonal(val), wunderground(val) {}
		int16_t seasonal;
		int16_t wunderground;
	};
public:
	runStateClass();
	void SetSchedule(bool val, int8_t iSchedNum = -1, const runStateClass::DurationAdjustments * adj = 0);
	void ContinueSchedule(int8_t zone, int64_t endTime);
	void SetManual(bool val, int8_t zone = -1);
	void PauseSchedule();
	void ResumeSchedule();
	bool isPaused()
	{
		return m_paused;
	}
	bool isSchedule()
	{
		return m_bSchedule;
	}
	bool isManual()
	{
		return m_bManual;
	}
	int8_t getZone()
	{
		return m_zone;
	}
	int64_t getEndTime()
	{
		return m_endTime;
	}
private:
	void LogSchedule();
	bool m_bSchedule;
	bool m_bManual;
	int8_t m_iSchedule;
	int8_t m_zone;
	int64_t m_endTime;
	time_t m_eventTime;
	bool m_paused;
	DurationAdjustments m_adj;
};

extern runStateClass runState;
extern nntp nntpTimeServer;

#endif

