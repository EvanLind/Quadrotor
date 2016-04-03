
#include "rcs.h"
#include "IMU.h"
//log1: sp0.0016  sd0.06
//log2: sp0.0015  sd0.08
//log3: cp0.1  cd0.05  ci0.001  sp0.10  
//log4: cp0.08  cd0.09  ci0.0015  sp0.0016  
//log5: cp0.04  cd0.1  ci0.0015  sp0.005 sd0.08
//log5: cp0.06  cd0.03  ci0.0012  sp0.013 sd0.08
//log6: cp0.03 	cd0.02 	ci0.0012  sp0.011 si0.0005
#define CONSTRAIN(VAL, VPT)    {VAL>VPT?(VAL = VPT):VAL; VAL<-VPT?(VAL = -VPT):VAL;}

extern volatile int RecvCom[4];
#define INTEGRALMAX		20.0
#define THRMIN	22000

/*PID参数初始化*/
void AttitudeInit(AttitudeCtrl * attitudeCtrl)
{    
	attitudeCtrl->attitudeOut.pitch = 0.0f;
	attitudeCtrl->attitudeOut.roll = 0.0f;
	attitudeCtrl->attitudeOut.yaw = 0.0f;

	attitudeCtrl->shellPitchPID.kp = 0.011f;
	attitudeCtrl->shellPitchPID.ki = 0.0005f;
	attitudeCtrl->shellPitchPID.kd = 0.0f;
	attitudeCtrl->shellRollPID.kp = 0.011f;//0.0013
	attitudeCtrl->shellRollPID.ki = 0.0005f;//0.0
	attitudeCtrl->shellRollPID.kd = 0.0f;//0.06
	attitudeCtrl->shellYawPID.kp = 0.0007f;
	attitudeCtrl->shellYawPID.ki = 0.0001f;
	attitudeCtrl->shellYawPID.kd = 0.002f;

	attitudeCtrl->corePitchPID.kp = 0.03f;//
	attitudeCtrl->corePitchPID.ki = 0.0012f;
	attitudeCtrl->corePitchPID.kd = 0.02f;
	attitudeCtrl->coreRollPID.kp = 0.03f;//0.06
	attitudeCtrl->coreRollPID.ki = 0.0012;//0.0015
	attitudeCtrl->coreRollPID.kd = 0.02f;//0.03
	attitudeCtrl->coreYawPID.kp = 0.0f;
	attitudeCtrl->coreYawPID.ki = 0.0f;
	attitudeCtrl->coreYawPID.kd = 0.0f;
}

double shellPitchError, shellPitchErrorIntegral, shellPitchLastError, shellRollError, shellRollErrorIntegral, shellRollLastError, shellYawError, shellYawErrorIntegral, shellYawLastError;
double corePitchError, corePitchErrorIntegral, corePitchLastError, coreRollError, coreRollErrorIntegral, coreRollLastError, coreYawError, coreYawErrorIntegral, coreYawLastError;
Attitude lastRealAgl;
Attitude lastAttitudeOut;

Attitude AttitudeControl(AttitudeCtrl * attitudeCtrl)
{    
	if (attitudeCtrl->realAgl.err_flag == 0)
	{		
		/***********外环控制*********************************************************************************************************/
		//采用位置型PID，积分采用梯形积分, 计算外环误差（PD）
//ShellPitch
		shellPitchError = attitudeCtrl->expectAgl.pitch - attitudeCtrl->realAgl.pitch;
		shellPitchErrorIntegral += shellPitchError;
		attitudeCtrl->attitudeOut.pitch = attitudeCtrl->shellPitchPID.kp * shellPitchError + attitudeCtrl->shellPitchPID.ki * shellPitchErrorIntegral + attitudeCtrl->shellPitchPID.kd * (shellPitchError - shellPitchLastError);
		shellPitchLastError = shellPitchError;
		CONSTRAIN(shellPitchErrorIntegral, INTEGRALMAX);
		if (RecvCom[0] < THRMIN)
		{
			shellPitchErrorIntegral = 0.0;
		}

//ShellRoll
		shellRollError = attitudeCtrl->expectAgl.roll - attitudeCtrl->realAgl.roll;
		shellRollErrorIntegral += shellRollError;
		attitudeCtrl->attitudeOut.roll = attitudeCtrl->shellRollPID.kp * shellRollError + attitudeCtrl->shellRollPID.ki * shellRollErrorIntegral + attitudeCtrl->shellRollPID.kd * (shellRollError - shellRollLastError);
		shellRollLastError = shellRollError;
		CONSTRAIN(shellRollErrorIntegral, INTEGRALMAX);
		if (RecvCom[0] < THRMIN)
		{
			shellRollErrorIntegral = 0.0;
		}

//ShellYaw
		shellYawError = attitudeCtrl->expectAgl.yaw - attitudeCtrl->realAgl.yaw;
		shellYawErrorIntegral += shellYawError;
		attitudeCtrl->attitudeOut.yaw = attitudeCtrl->shellYawPID.kp * shellYawError + attitudeCtrl->shellYawPID.ki * shellYawErrorIntegral + attitudeCtrl->shellYawPID.kd * (shellYawError - shellYawLastError);
		shellYawLastError = shellYawError;
		CONSTRAIN(shellYawErrorIntegral, INTEGRALMAX);
		if (RecvCom[0] < THRMIN)
		{
			shellYawErrorIntegral = 0.0;
		}


		/*************内环控制**********************************************************************************************************/
//CorePitch
		corePitchError = attitudeCtrl->attitudeOut.pitch - (attitudeCtrl->realAgl.pitch - lastRealAgl.pitch);
		lastRealAgl.pitch = attitudeCtrl->realAgl.pitch;
		corePitchErrorIntegral += corePitchError;
		CONSTRAIN(corePitchErrorIntegral, INTEGRALMAX);
		/*************低油门积分清零**********************************************************************************************************/
		if (RecvCom[0] < THRMIN)
		{
			corePitchErrorIntegral = 0.0;
		}
		attitudeCtrl->attitudeOut.pitch = attitudeCtrl->corePitchPID.kp * corePitchError + attitudeCtrl->corePitchPID.ki * corePitchErrorIntegral + attitudeCtrl->corePitchPID.kd * (corePitchError - corePitchLastError);
		corePitchLastError = corePitchError;
		if (attitudeCtrl->attitudeOut.pitch > 0.25)
		{
			attitudeCtrl->attitudeOut.pitch = 0.25;
		}

//CoreRoll
		coreRollError = attitudeCtrl->attitudeOut.roll - (attitudeCtrl->realAgl.roll - lastRealAgl.roll);
		lastRealAgl.roll = attitudeCtrl->realAgl.roll;
		coreRollErrorIntegral += coreRollError;
		CONSTRAIN(coreRollErrorIntegral, INTEGRALMAX);
		/*************低油门积分清零**********************************************************************************************************/
		if (RecvCom[0] < THRMIN)
		{
			coreRollErrorIntegral = 0.0;
		}
		attitudeCtrl->attitudeOut.roll = attitudeCtrl->coreRollPID.kp * coreRollError + attitudeCtrl->coreRollPID.ki * coreRollErrorIntegral + attitudeCtrl->coreRollPID.kd * (coreRollError - coreRollLastError);
		coreRollLastError = coreRollError;
		if (attitudeCtrl->attitudeOut.roll > 0.25)
		{
			attitudeCtrl->attitudeOut.roll = 0.25;	
		}

// //CoreYaw
// 		coreYawError = attitudeCtrl->attitudeOut.yaw - (attitudeCtrl->realAgl.yaw - lastRealAgl.yaw);
// 		lastRealAgl.yaw = attitudeCtrl->realAgl.yaw;
// 		coreYawErrorIntegral += coreYawError;
// 		/*************积分限幅环节**********************************************************************************************************/
// 		if (coreYawErrorIntegral > INTEGRALMAX)
// 		{
// 			coreYawErrorIntegral = INTEGRALMAX;
// 		}
// 		if (coreYawErrorIntegral < -INTEGRALMAX)
// 		{
// 			coreYawErrorIntegral = -INTEGRALMAX;
// 		}
// 		/*************低油门积分清零**********************************************************************************************************/
// 		if (RecvCom[0] < THRMIN)
// 		{
// 			coreYawErrorIntegral = 0.0;
// 		}
// 		attitudeCtrl->attitudeOut.yaw = attitudeCtrl->corePid.yawKp * coreYawError + attitudeCtrl->corePid.yawKi * coreYawErrorIntegral + attitudeCtrl->corePid.yawKd * (coreYawError - coreYawLastError);
// 		coreYawLastError = coreYawError;
		if (attitudeCtrl->attitudeOut.yaw > 0.25)
		{
			attitudeCtrl->attitudeOut.yaw = 0.25;	
		}
		lastAttitudeOut = attitudeCtrl->attitudeOut;
		return attitudeCtrl->attitudeOut;
	}
	else
	{
		return lastAttitudeOut;
	}
}


double AltitudeError, AltitudeErrorIntegral, AltitudeLastError, RollError, RollErrorIntegral, RollLastError, YawError, YawErrorIntegral, YawLastError;
Coordinate lastRealCoordinate;
Coordinate lastCoordinateOut;

Coordinate AltitudeControl(CoordinateCtrl * coordinateCtrl)
{

	/*************Altitude**********************************************************************************************************/
	AltitudeError = coordinateCtrl->expectCoord.altitude - coordinateCtrl->realCoord.altitude;
	AltitudeErrorIntegral += AltitudeError;
	CONSTRAIN(AltitudeErrorIntegral, 10);
	coordinateCtrl->coordinateOut.altitude = coordinateCtrl->altitudePID.kp * AltitudeError + coordinateCtrl->altitudePID.ki * AltitudeErrorIntegral + coordinateCtrl->altitudePID.kd * (AltitudeError - AltitudeLastError);
	AltitudeLastError = AltitudeError;
	CONSTRAIN(coordinateCtrl->coordinateOut.altitude, 0.2);
	return coordinateCtrl->coordinateOut;
}






// #include "rcs.h"
// #include "IMU.h"
// //log1: sp0.0016  sd0.06
// //log2: sp0.0015  sd0.08
// //log3: cp0.1  cd0.05  ci0.001  sp0.10  
// //log4: cp0.08  cd0.09  ci0.0015  sp0.0016  
// //log5: cp0.04  cd0.1  ci0.0015  sp0.005 sd0.08
// //log5: cp0.06  cd0.03  ci0.0012  sp0.013 sd0.08
// //log6: cp0.03 	cd0.02 	ci0.0012  sp0.011 si0.0005
// extern volatile int RecvCom[4];
// #define INTEGRALMAX		20.0

// /*PID参数初始化*/
// void PIDInit(Quadrotor * _pid)
// {    
// 	// _pid->expectAgl.Pitch = 0.0;
// 	// _pid->expectAgl.Roll = 0.0;
// 	// _pid->expectAgl.Yaw = 0.0;
// 	_pid->realAgl.pitch = 0.0f;
// 	_pid->realAgl.pitch = 0.0f;
// 	_pid->realAgl.pitch = 0.0f;
// 	_pid->adjustOut.pitch = 0.0f;
// 	_pid->adjustOut.roll = 0.0f;
// 	_pid->adjustOut.yaw = 0.0f;

// 	_pid->shellPid.pitchKp = 0.011f;
// 	_pid->shellPid.pitchKi = 0.0005f;
// 	_pid->shellPid.pitchKd = 0.0f;
// 	_pid->shellPid.rollKp = 0.011f;//0.0013
// 	_pid->shellPid.rollKi = 0.0005f;//0.0
// 	_pid->shellPid.rollKd = 0.0f;//0.06
// 	_pid->shellPid.yawKp = 0.0007f;
// 	_pid->shellPid.yawKi = 0.0001f;
// 	_pid->shellPid.yawKd = 0.002f;

// 	_pid->corePid.pitchKp = 0.03f;//
// 	_pid->corePid.pitchKi = 0.0012f;
// 	_pid->corePid.pitchKd = 0.02f;
// 	_pid->corePid.rollKp = 0.03f;//0.06
// 	_pid->corePid.rollKi = 0.0012;//0.0015
// 	_pid->corePid.rollKd = 0.02f;//0.03
// 	_pid->corePid.yawKp = 0.0f;
// 	_pid->corePid.yawKi = 0.0f;
// 	_pid->corePid.yawKd = 0.0f;
// }

// double shellPitchError, shellPitchErrorIntegral, shellPitchLastError, shellRollError, shellRollErrorIntegral, shellRollLastError, shellYawError, shellYawErrorIntegral, shellYawLastError;
// double corePitchError, corePitchErrorIntegral, corePitchLastError, coreRollError, coreRollErrorIntegral, coreRollLastError, coreYawError, coreYawErrorIntegral, coreYawLastError;
// Triaxial lastRealAgl;
// Attitude lastAdjustOut;

// Attitude PIDControl(Quadrotor * _pid, Triaxial _realAgl)
// {    
// 	if (_realAgl.err_flag == 0)
// 	{
			
// 		_pid->realAgl = _realAgl;
// 		/***********外环控制*********************************************************************************************************/
// 		//采用位置型PID，积分采用梯形积分, 计算外环误差（PD）
// //ShellPitch
// 		shellPitchError = _pid->expectAgl.Pitch - _pid->realAgl.pitch;
// 		shellPitchErrorIntegral += shellPitchError;
// 		_pid->adjustOut.pitch = _pid->shellPid.pitchKp * shellPitchError + _pid->shellPid.pitchKi * shellPitchErrorIntegral + _pid->shellPid.pitchKd * (shellPitchError - shellPitchLastError);
// 		shellPitchLastError = shellPitchError;
// 		if (shellPitchErrorIntegral > INTEGRALMAX)
// 		{
// 			shellPitchErrorIntegral = INTEGRALMAX;
// 		}
// 		if (shellPitchErrorIntegral < -INTEGRALMAX)
// 		{
// 			shellPitchErrorIntegral = -INTEGRALMAX;
// 		}
// 		if (RecvCom[0] < 22000)
// 		{
// 			shellPitchErrorIntegral = 0.0;
// 		}

// //ShellRoll
// 		shellRollError = _pid->expectAgl.Roll - _pid->realAgl.roll;
// 		shellRollErrorIntegral += shellRollError;
// 		_pid->adjustOut.roll = _pid->shellPid.rollKp * shellRollError + _pid->shellPid.rollKi * shellRollErrorIntegral + _pid->shellPid.rollKd * (shellRollError - shellRollLastError);
// 		shellRollLastError = shellRollError;
// 		if (shellRollErrorIntegral > INTEGRALMAX)
// 		{
// 			shellRollErrorIntegral = INTEGRALMAX;
// 		}
// 		if (shellRollErrorIntegral < -INTEGRALMAX)
// 		{
// 			shellRollErrorIntegral = -INTEGRALMAX;
// 		}
// 		if (RecvCom[0] < 22000)
// 		{
// 			shellRollErrorIntegral = 0.0;
// 		}

// //ShellYaw
// 		shellYawError = _pid->expectAgl.Yaw - _pid->realAgl.yaw;
// 		shellYawErrorIntegral += shellYawError;
// 		_pid->adjustOut.yaw = _pid->shellPid.yawKp * shellYawError + _pid->shellPid.yawKi * shellYawErrorIntegral + _pid->shellPid.yawKd * (shellYawError - shellYawLastError);
// 		shellYawLastError = shellYawError;
// 		if (shellYawErrorIntegral > INTEGRALMAX)
// 		{
// 			shellYawErrorIntegral = INTEGRALMAX;
// 		}
// 		if (shellYawErrorIntegral < -INTEGRALMAX)
// 		{
// 			shellYawErrorIntegral = -INTEGRALMAX;
// 		}
// 		if (RecvCom[0] < 22000)
// 		{
// 			shellYawErrorIntegral = 0.0;
// 		}


// 		/*************内环控制**********************************************************************************************************/
// //CorePitch
// 		corePitchError = _pid->adjustOut.pitch - (_pid->realAgl.pitch - lastRealAgl.pitch);
// 		lastRealAgl.pitch = _pid->realAgl.pitch;
// 		corePitchErrorIntegral += corePitchError;
// 		/*************积分限幅环节**********************************************************************************************************/
// 		if (corePitchErrorIntegral > INTEGRALMAX)
// 		{
// 			corePitchErrorIntegral = INTEGRALMAX;
// 		}
// 		if (corePitchErrorIntegral < -INTEGRALMAX)
// 		{
// 			corePitchErrorIntegral = -INTEGRALMAX;
// 		}
// 		/*************低油门积分清零**********************************************************************************************************/
// 		if (RecvCom[0] < 22000)
// 		{
// 			corePitchErrorIntegral = 0.0;
// 		}
// 		_pid->adjustOut.pitch = _pid->corePid.pitchKp * corePitchError + _pid->corePid.pitchKi * corePitchErrorIntegral + _pid->corePid.pitchKd * (corePitchError - corePitchLastError);
// 		corePitchLastError = corePitchError;
// 		if (_pid->adjustOut.pitch > 0.25)
// 		{
// 			_pid->adjustOut.pitch = 0.25;
// 		}

// //CoreRoll
// 		coreRollError = _pid->adjustOut.roll - (_pid->realAgl.roll - lastRealAgl.roll);
// 		lastRealAgl.roll = _pid->realAgl.roll;
// 		coreRollErrorIntegral += coreRollError;
// 		/*************积分限幅环节**********************************************************************************************************/
// 		if (coreRollErrorIntegral > INTEGRALMAX)
// 		{
// 			coreRollErrorIntegral = INTEGRALMAX;
// 		}
// 		if (coreRollErrorIntegral < -INTEGRALMAX)
// 		{
// 			coreRollErrorIntegral = -INTEGRALMAX;
// 		}
// 		/*************低油门积分清零**********************************************************************************************************/
// 		if (RecvCom[0] < 22000)
// 		{
// 			coreRollErrorIntegral = 0.0;
// 		}
// 		_pid->adjustOut.roll = _pid->corePid.rollKp * coreRollError + _pid->corePid.rollKi * coreRollErrorIntegral + _pid->corePid.rollKd * (coreRollError - coreRollLastError);
// 		coreRollLastError = coreRollError;
// 		if (_pid->adjustOut.roll > 0.25)
// 		{
// 			_pid->adjustOut.roll = 0.25;	
// 		}

// // //CoreYaw
// // 		coreYawError = _pid->adjustOut.yaw - (_pid->realAgl.yaw - lastRealAgl.yaw);
// // 		lastRealAgl.yaw = _pid->realAgl.yaw;
// // 		coreYawErrorIntegral += coreYawError;
// // 		/*************积分限幅环节**********************************************************************************************************/
// // 		if (coreYawErrorIntegral > INTEGRALMAX)
// // 		{
// // 			coreYawErrorIntegral = INTEGRALMAX;
// // 		}
// // 		if (coreYawErrorIntegral < -INTEGRALMAX)
// // 		{
// // 			coreYawErrorIntegral = -INTEGRALMAX;
// // 		}
// // 		/*************低油门积分清零**********************************************************************************************************/
// // 		if (RecvCom[0] < 22000)
// // 		{
// // 			coreYawErrorIntegral = 0.0;
// // 		}
// // 		_pid->adjustOut.yaw = _pid->corePid.yawKp * coreYawError + _pid->corePid.yawKi * coreYawErrorIntegral + _pid->corePid.yawKd * (coreYawError - coreYawLastError);
// // 		coreYawLastError = coreYawError;
// 		if (_pid->adjustOut.yaw > 0.25)
// 		{
// 			_pid->adjustOut.yaw = 0.25;	
// 		}
// 		// char res[50];
// 		// // sprintf(res, "rollP:%.5f\n\0", _pid->adjustOut.roll);
// 		// // sprintf(res, "coreRollError:%.5f _pid->realAgl.roll:%.5f \n\0", coreRollError, _pid->realAgl.roll);
// 		// sprintf(res, "Yaw:%.5f \n\0", _realAgl.yaw);
// 		// RCS_USART_Send_Str(DEBUGUSART, res);
// 		lastAdjustOut = _pid->adjustOut;
// 		return _pid->adjustOut;
// 	}
// 	else
// 	{
// 		return lastAdjustOut;
// 	}
// }