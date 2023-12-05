/*File ClockCalendar.cpp*/
#include "ClockCalendar.h"

ClockCalendar::ClockCalendar(int mo, int d, int y, int h, int m, int s, int pm): Clock(h, m, s, pm), Calendar(m, d, y)
{
    setClock(h, s, m, pm);
    setCalendar(mo, d, y);
}

void ClockCalendar::advance() //avançar o calendário, caso o clock
{
    bool wasPm = is_pm; //mude de PM para AM.
    Clock::advanceClock();
    if (wasPm && !is_pm)
    Calendar::advanceCalendar();
}
