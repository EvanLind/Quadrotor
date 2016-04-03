#ifndef _RCS_UARTCCD_H_
#define _RCS_UARTCCD_H_
#include "bsp.h"
extern volatile uint8_t g_width ;
extern volatile uint8_t g_shift ;
extern volatile Boolean g_isComplete;
inline uint8_t CCD_GetWidth()
{
	return g_width;
}
inline uint8_t CCD_GetShift()
{
	return g_shift;
}
inline Boolean CCD_GetIsComplete()
{
	return g_isComplete;
}
void CCD_Init();
void CCD_Deinit();
void CCD_Start();
void CCD_Stop();

#endif
