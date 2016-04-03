#ifndef _SONAR_H_
#define _SONAR_H_

void InitSonar();
void ECHO_EXTI_IRQHandler(void);
float GetSonarDis(void);


#endif
