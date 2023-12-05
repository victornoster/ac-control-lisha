/*FIle ClockCalendar.h*/
#ifndef __CLOCKCALENDAR_H_
#define __CLOCKCALENDAR_H_

#include "Calendar.h"
#include "Clock.h"


class ClockCalendar : public Clock, public Calendar {
    public:
        ClockCalendar(int mo, int d, int y, int h, int m, int s, int pm);
        void advance();
};

#endif
