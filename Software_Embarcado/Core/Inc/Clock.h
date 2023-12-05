/*File Clock.h
Header da class Clock*/
#ifndef __CLOCK_H__
#define __CLOCK_H__

class Clock
{
    protected:
        int hr, min, sec, is_pm;
    public:
        Clock(int h, int s, int m, int pm);
        void setClock(int h, int s, int m, int pm);
        void readClock(int& h, int& s, int& m, int& pm);
        void advanceClock();
};
#endif
