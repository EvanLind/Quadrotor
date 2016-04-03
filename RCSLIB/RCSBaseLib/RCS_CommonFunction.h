#ifndef _RCS_COMMANFUNCTION_H_
#define _RCS_COMMANFUNCTION_H_

#define ABS(x)  ((x)>0?(x):-(x))

//@name:  CombineTwoBytes
//@brief:   将两个8位合并成一个16位
//retval:   0xab
inline uint16_t CombineTwoBytes(uint8_t a, uint8_t b)
{
	uint16_t tmp = a & 0x00ff;
	tmp = tmp << 8 & 0xff00 | b;
	return tmp;
}

//@name:  TakepartTwoBytes
//@brief:   将一个16位分成两个8位
inline void TakepartTwoBytes(uint16_t data, uint8_t *a, uint8_t *b)
{
	*a = (data & 0xff00) >> 8;
	*b = (data & 0x00ff);
}

#endif