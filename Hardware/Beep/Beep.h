#ifndef _BEEP_H_
#define _BEEP_H_

void InitBeep(GPIO_TypeDef *_port, uint32_t _pin);
void Beep(int count);

#endif