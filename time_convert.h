/*
 * time_convert.h
 *
 * Created: 29.08.2015 07:37:03
 *  Author: JackFrost
 */ 


#ifndef TIME_CONVERT_H_
#define TIME_CONVERT_H_
	#include <inttypes.h>
	#include <stdlib.h>

// Timezones

#define UTC 0
#define CET 1


	
	typedef uint32_t time_t;
	typedef struct _tm tm;
	
	struct _tm {
		int8_t tm_sec;
		int8_t tm_min;
		int8_t tm_hour;
		int8_t tm_mday;
		int8_t tm_wday;
		int8_t tm_mon;
		int16_t tm_year;
		int16_t tm_yday;
		int16_t tm_isdst;
	};
	void summertime(tm * t );
	int gurke_r(const time_t * timer, tm * theTime, int8_t timezone);
	tm *gurke(const time_t * timer, int8_t zone);
	
	
	
	
	enum _WEEK_DAYS_ {
			SUNDAY,
			MONDAY,
			TUESDAY,
			WEDNESDAY,
			THURSDAY,
			FRIDAY,
			SATURDAY
		};
		/**
		Enumerated labels for the months.
		*/
		enum _MONTHS_ {
			JANUARY,
			FEBRUARY,
			MARCH,
			APRIL,
			MAY,
			JUNE,
			JULY,
			AUGUST,
			SEPTEMBER,
			OCTOBER,
			NOVEMBER,
			DECEMBER
		};
		
		
		
		
	
	



#endif /* TIME_CONVERT_H_ */