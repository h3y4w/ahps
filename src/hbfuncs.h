#ifndef HBFUNCS_H__
#define HBFUNCS_H__

#define HEX_2_DEC(val) (((val)/16)*10+((val)%16))
#define DEC_2_HEX(val) (((val)/10)*16+((val)%10))

void getTime(RTC_TimeTypeDef *time);
void setTime(int hours, int minutes, int seconds);
void setTimeSafely(int hours, int minutes, int seconds, unsigned prev_epoch); //sets time safely for timers and other tasks

void setDate(int month, int day, int year);

unsigned getEpoch(RTC_TimeTypeDef *time, int days);

#endif
