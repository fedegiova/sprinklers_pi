// Event.cpp
// This defines the Event Object used for timing in the Sprinkling System.
// Author: Richard Zimmerman
// Copyright (c) 2013 Richard Zimmerman
//

#include "Event.h"
#include <algorithm>
#include "port.h"

// global for the events structure.
Event events[MAX_EVENTS];
int iNumEvents = 0;

void eventQueueClear()
{
	iNumEvents = 0;	
}
static void sortEvents()
{
	struct local{
		static bool compare(const Event &a,const Event &b)
		{
			return a.time < b.time;
		}
	};
	std::sort(events,events+iNumEvents,&local::compare);
}
void eventQueuePush(const Event & e)
{
	if( iNumEvents >= MAX_EVENTS)
	{
		trace(F("ERROR: Too Many Events!\n"));
	}
	events[iNumEvents++] = e;
	sortEvents();
}
void eventQueuePop()
{
	if(!iNumEvents) return;
	if(iNumEvents > 1)
		std::swap(events[iNumEvents-1],events[0]);
	iNumEvents--;
	sortEvents();
}
Event eventQueueHead()
{
	if( iNumEvents)
		return events[0];
	return Event();
}
