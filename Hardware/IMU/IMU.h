#ifndef _IMU_H_
#define _IMU_H_
#include "bsp.h"


void AttitudeInit(AttitudeCtrl * attitudeCtrl);
Attitude AttitudeControl(AttitudeCtrl * attitudeCtrl);
Coordinate AltitudeControl(CoordinateCtrl * coordinateCtrl);

#endif