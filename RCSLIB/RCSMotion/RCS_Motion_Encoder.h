	/*
 *@author : kgolym
 *@date:2012-10-05
 *@brief: Read the count of the encoder
 */

#ifndef _RCS_ENCODER_H_
#define _RCS_ENCODER_H_

#include "bsp.h"



extern const uint16_t ENCODER_PPR ;  // number of pulses per revolution
extern const uint16_t COUNTER_RESET;

extern volatile int32_t encoder1OverFlow ;
extern volatile int32_t encoder2OverFlow ;
extern volatile int32_t encoder3OverFlow ;


void InitEncoders();

inline int32_t GetEncoder1Count()
{
	int32_t count = encoder1OverFlow * COUNTER_RESET + ENCODER1_TIMER->CNT + 1;
	return count;
}
inline void ClearEncoder1Count()
{
	encoder1OverFlow = -1;
	ENCODER1_TIMER->CNT = COUNTER_RESET - 1;
}
inline int32_t GetEncoder2Count()
{
	int32_t count = encoder2OverFlow * COUNTER_RESET + ENCODER2_TIMER->CNT + 1;
	return count;
}
inline void ClearEncoder2Count()
{
	encoder2OverFlow = -1;
	ENCODER2_TIMER->CNT = COUNTER_RESET - 1;
}
inline int32_t GetEncoder3Count()
{
	int32_t count = encoder3OverFlow * COUNTER_RESET + ENCODER3_TIMER->CNT + 1;
	return count;
}
inline void ClearEncoder3Count()
{
	encoder3OverFlow = -1;
	ENCODER3_TIMER->CNT = COUNTER_RESET - 1;
}
#endif //_RCS_ENCODER_H_