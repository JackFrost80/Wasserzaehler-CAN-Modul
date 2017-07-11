/*
 * time_convert.c
 *
 * Created: 29.08.2015 07:36:47
 *  Author: JackFrost
 */ 
#include "time_convert.h"
#include "ctime.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#define __need_NULL



tm _tb;

/*      static arrays used by gmtime to determine date and time
***************************************************************/
int _lpdays[] = {-1, 30, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365};
int _days[] = {-1, 30, 58, 89, 119, 150, 180, 211, 242, 272, 303, 333, 364};
	
	
void summertime(tm * t )
{
	if( t->tm_mon < 3 || t->tm_mon > 10 )			// month 1, 2, 11, 12
		t->tm_isdst = 0;					// -> Winter
	else
	{
		if(t->tm_mon==10 || t->tm_mon== 3)
		{
			if(((t->tm_mday - t->tm_wday >= 25) && (t->tm_wday == 0 ) && (t->tm_hour >= 3) && (t->tm_isdst == 1) && (t->tm_mon==10)) || ((t->tm_mday - t->tm_wday >= 25) && (t->tm_wday == 0 ) && (t->tm_hour < 2) && (t->tm_isdst == 0) && (t->tm_mon==3)))
				t->tm_isdst = 0;
			else
				t->tm_isdst = 1;
			
		}
		else
		{
			t->tm_isdst = 1;
		}
		
	}
	
}
	
int gurke_r(const time_t * timer, tm * ptm, int8_t timezone)
{
	if(ptm->tm_isdst == 1)
	{
		timezone++;
	}
	time_t ctimer = *timer;     /* var to calculate with */
	ctimer += ((uint16_t)timezone * 3600);
	uint8_t isleapyear = 0;     /* current year is leap year */
	uint32_t tmptimer;
	int *mdays;                 /* pointer to _numdayslp or _numdays */
	if (ptm == NULL)            /* check pointer */
		return -1;
	
	/*
	 First calculate the number of four-year-interval, so calculation
	 of leap year will be simple. Btw, because 2000 IS a leap year and
	 2100 is out of range, this formula is so simple.
	*/
	tmptimer = (uint32_t) (ctimer / _FOUR_YEAR_SEC);
	ctimer -= ((time_t) tmptimer * _FOUR_YEAR_SEC);
	/* Determine the correct year within the interval */
	tmptimer = (tmptimer * 4) + 70;     /* 1970, 1974, 1978,... */
	if (ctimer >= (time_t)_YEAR_SEC) {
	tmptimer++;             /* 1971, 1975, 1979,... */
	ctimer -= _YEAR_SEC;
	if (ctimer >= (time_t)_YEAR_SEC) {
			tmptimer++;         /* 1972, 1976, 1980,... (all leap years!) */
			ctimer -= _YEAR_SEC;
			/* A leap year has 366 days, so compare to _YEAR_SEC + _DAY_SEC */
			if (ctimer >= (time_t)(_YEAR_SEC + _DAY_SEC)) {
				tmptimer++;     /* 1973, 1977, 1981,... */
				ctimer -= (_YEAR_SEC + _DAY_SEC);
			} else
			isleapyear = 1; /*If leap year, set the flag */
		}
	}
	/*
	tmptimer now has the value for tm_year. ctimer now holds the
	number of elapsed seconds since the beginning of that year.
	*/
	ptm->tm_year = tmptimer;
	
	/*
	Calculate days since January 1st and store it to tm_yday.
	Leave ctimer with number of elapsed seconds in that day.
	*/
	ptm->tm_yday = (int) (ctimer / _DAY_SEC);
	ctimer -= (time_t) (ptm->tm_yday) * _DAY_SEC;
	
	/*
	Determine months since January (Note, range is 0 - 11)
	and day of month (range: 1 - 31)
	*/
	if (isleapyear)
		mdays = _lpdays;
	else
		mdays = _days;
	
	for (tmptimer = 1; mdays[tmptimer] < ptm->tm_yday; tmptimer++);
	
	ptm->tm_mon = tmptimer;
	--tmptimer;
	
	ptm->tm_mday = ptm->tm_yday - mdays[tmptimer];
	
	/* Calculate day of week. Sunday is 0 */
	ptm->tm_wday = ((int) (*timer / _DAY_SEC) + _BASE_DOW) % 7;
	/* Calculate the time of day from the remaining seconds */
	ptm->tm_hour = (int) (ctimer / 3600);
	ctimer -= (time_t) ptm->tm_hour * 3600L;
	
	ptm->tm_min = (int) (ctimer / 60);
	ptm->tm_sec = (int) (ctimer - (ptm->tm_min) * 60);
	//ptm->tm_isdst = 0;
	return 0;
 }

tm *gurke(const time_t * timer, int8_t zone)
{
	if (gurke_r(timer, &_tb, zone))
	return NULL;
	else
	return &_tb;
	
}