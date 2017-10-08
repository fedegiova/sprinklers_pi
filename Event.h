// Event.h
// This defines the Event Object used for timing in the Sprinkling System.
// Author: Richard Zimmerman
// Copyright (c) 2013 Richard Zimmerman
//

#ifndef _EVENT_h
#define _EVENT_h

#include <inttypes.h>

class Event
{
public:
	short time;
	uint8_t  zone;
	uint8_t  sched_num;
	uint16_t duration;
	bool     isQuickSchedule;
	bool     isManual;
	Event()
		:time(-1),zone(0),sched_num(0),duration(0),isQuickSchedule(false),isManual(false)
	{
	}
	bool isValid()const
	{
		return time != -1;
	}
};

#define MAX_EVENTS 60

extern Event events[];
extern int iNumEvents;

extern void eventQueueClear();
extern void eventQueuePush(const Event & e);
extern void eventQueuePop();
extern Event eventQueueHead();

#endif
