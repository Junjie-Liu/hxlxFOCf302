
#include "stm32f3xx_hal.h"
#include "arm_math.h"
#include "foc.h"
#include "foc_param.h"

//Debug
#ifdef FOC_DEBUG
int32_t zSVPWM1 = 0;
int32_t zSVPWM2 = 0;
int32_t zSVPWM3 = 0;
int32_t zIa = 0;
int32_t zIb = 0;
#endif
//

FOC_t FOC;           /* FOC算法数据 */
FOCflag_t FOCflag;   /* FOC标志界面 */
FOCuser_t FOCuser;   /* FOC用户界面 */

PIparam_t PIparamQ;  //PI for Iq
PIparam_t PIparamD;  //PI for Id
PIparam_t PIparamV;  //PI for Velocity 
PIDparam_t PIDparamP;  //PI for Position 

uint8_t OpenLoop = 1;
int32_t Vel_Acc_Kp = 0;

/* for SVPWM */
static int32_t T1 = 0;
static int32_t T2 = 0;
static uint16_t Ta = 0;
static uint16_t Tb = 0;
static uint16_t Tc = 0;
SVpwm_t  SVpwm;      //SVPWM output for MCU PWM update */

static void Pcalc_pid(PIDparam_t *pParam);
static void calc_pi(PIparam_t *pParam);
static void do_control(void);
static void calc_svpwm(void);
static void calc_pwm_time(void);
static void velocity_ramp(void);
static void torque_ramp(void);

void foc_loop(FOC_t* pFOC)
{
	static int32_t qei_pos_lpf = 0;
	static int32_t qei_vel_lpf = 0;
	static int32_t mag_vel_lpf = 0;
	static int32_t mag_pos_lpf = 0;	

	static int32_t id_lpf = 0;	
	static int32_t iq_lpf = 0;
	
  int32_t ia = pFOC->Ia;// - pFOC->Ia_offset;
  int32_t ib = pFOC->Ib;// - pFOC->Ib_offset;
	
	pFOC->Ialpha = (ia)*3>>1;
	pFOC->Ibeta  = ((ia) + 2*(ib))*111>>7;
	
	pFOC->Id =  (pFOC->Ialpha*pFOC->Cos>>15) + (pFOC->Ibeta*pFOC->Sin>>15);
	pFOC->Iq = (-pFOC->Ialpha*pFOC->Sin>>15) + (pFOC->Ibeta*pFOC->Cos>>15);
	
	id_lpf += (pFOC->Id - pFOC->IdLpf)*pFOC->IdLpK;
	pFOC->IdLpf = id_lpf>>15;  	

	iq_lpf += (pFOC->Iq - pFOC->IqLpf)*pFOC->IqLpK;
	pFOC->IqLpf = iq_lpf>>15;   	
	
	pFOC->Sin = arm_sin_q15(pFOC->QEItheta);
	pFOC->Cos = arm_cos_q15(pFOC->QEItheta);

	qei_vel_lpf += (pFOC->QEIvel - pFOC->QEIvelLpf)*pFOC->QEIvelLpK;
	pFOC->QEIvelLpf = qei_vel_lpf>>15;  	
	
	qei_pos_lpf += (pFOC->QEIpos - pFOC->QEIposLpf)*pFOC->QEIposLpK;
	pFOC->QEIposLpf = qei_pos_lpf>>15;  	
	
	mag_vel_lpf += (pFOC->GEAR_MAGvel - pFOC->GEAR_MAGvelLpf)*pFOC->GEAR_MAGvelLpK;
	pFOC->GEAR_MAGvelLpf = mag_vel_lpf>>15;		
	
	mag_pos_lpf += (pFOC->GEAR_MAGpos - pFOC->GEAR_MAGposLpf)*pFOC->GEAR_MAGposLpK;
	pFOC->GEAR_MAGposLpf = mag_pos_lpf>>15;			
	
	do_control(); 
	
	pFOC->Valpha =  (pFOC->Vd*pFOC->Cos>>15) - (pFOC->Vq*pFOC->Sin>>15);
	pFOC->Vbeta  =  (pFOC->Vd*pFOC->Sin>>15) + (pFOC->Vq*pFOC->Cos>>15);
	
	pFOC->Va = pFOC->Vbeta;                         
	pFOC->Vb = (-pFOC->Vbeta>>1) + (pFOC->Valpha*111>>7);
	pFOC->Vc = (-pFOC->Vbeta>>1) - (pFOC->Valpha*111>>7);
	
	/* SVPWM */
	calc_svpwm();

  //Debug
#ifdef FOC_DEBUG
	zIa = ia;
	zIb = ib;
#endif
}

/* PI 控制器 */
static void calc_pid(PIDparam_t *pParam)
{
	static int32_t d_err_lpf = 0;
	int32_t d_err = 0;
  int32_t error = 0;
  int32_t u     = 0;
  int32_t out   = 0;
  
  error = pParam->qInRef - pParam->qInMeas;
  u = error*pParam->qKp;
	
	d_err = error - pParam->qLastErr;
	d_err_lpf += (d_err - pParam->qdErrLpf)*pParam->qdErrLpK;
	pParam->qdErrLpf = d_err_lpf>>15;
	
	u = u + pParam->qdErrLpf*pParam->qKd;
  u = u + (pParam->qdSum>>4);
  
	pParam->qLastErr = error;
	
  out = u>>15;
  if(out >  pParam->qOutMax)
    pParam->qOut =  pParam->qOutMax;
  else if(out < pParam->qOutMin)
    pParam->qOut =  pParam->qOutMin;
  else
    pParam->qOut = out;
  
	/* 积分 */
  u = (error * pParam->qKi);
  
	/* 饱和退出 */
  error = out - pParam->qOut;
  u -= (error * pParam->qKc);
  
  pParam->qdSum += u;  
	
	/* 积分限幅 */
	if(pParam->qdSum > pParam->qdSumMax )
	{
		pParam->qdSum = pParam->qdSumMax;
	}
	
	if(pParam->qdSum < pParam->qdSumMin)
	{
		pParam->qdSum = pParam->qdSumMin;
	}	
}

/* PI 控制器 */
static void calc_pi(PIparam_t *pParam)
{
  int32_t error = 0;
  int32_t u     = 0;
  int32_t out   = 0;
  
  error = pParam->qInRef - pParam->qInMeas;
  u = error*pParam->qKp;
  u = u + (pParam->qdSum>>4);
  
  out = u>>7;
  if(out >  pParam->qOutMax)
    pParam->qOut =  pParam->qOutMax;
  else if(out < pParam->qOutMin)
    pParam->qOut =  pParam->qOutMin;
  else
    pParam->qOut = out;
  
	/* 积分 */
  u = (error * pParam->qKi);
  
	/* 饱和退出 */
  error = out - pParam->qOut;
  u -= (error * pParam->qKc);
  
  pParam->qdSum += u;  
	
	/* 积分限幅 */
	if(pParam->qdSum > pParam->qdSumMax )
	{
		pParam->qdSum = pParam->qdSumMax;
	}
	
	if(pParam->qdSum < pParam->qdSumMin)
	{
		pParam->qdSum = pParam->qdSumMin;
	}	
}

static void do_control(void)
{
	
	/* 模式选择 */
	switch(FOCuser.MotorMode)
	{
		case POSITION_MODE:               /* 位置模式 */		
		case VELOCITY_MODE:               /* 速度模式 */
			
			if(FOCuser.MotorMode == POSITION_MODE)
			{
					PIDparamP.qInMeas = FOC.GEAR_MAGposLpf;         /* MAG的当前位置 */
					PIDparamP.qInRef  = FOCuser.PositionCMD;     /* 目标位置参考值 */
					calc_pid(&PIDparamP);
					FOCuser.VelocityCMD = PIDparamP.qOut;
			}  

			velocity_ramp();                 /* VelocityCMD 计算 VelRef*/
		
			/* 指令限幅 */
			if(FOC.VelRef > FOCuser.MaxVelocity)
				FOC.VelRef = FOCuser.MaxVelocity;
			
			if(FOC.VelRef < -FOCuser.MaxVelocity)
				FOC.VelRef = -FOCuser.MaxVelocity;
		
			PIparamV.qInMeas = FOC.GEAR_MAGvelLpf + FOC.GEAR_MAGaccLpf*Vel_Acc_Kp; /* MAG的当前速度 */
			PIparamV.qInRef  = FOC.VelRef;         /* 目标速度参考值 */
			calc_pi(&PIparamV);
			FOC.VqRef = PIparamV.qOut;
			
			break;
			
		case TORQUE_MODE:  /* 扭矩模式 */
			
			torque_ramp();   /* 计算 VqRef*/
			FOC.VelRef = 0;  /* 清 VelRef */
		
			break;	
			
		default:            /* 错误的模式 */
			
			FOCuser.VelocityCMD = 0;			
			FOCuser.TorqueCMD   = 0;
			velocity_ramp(); 
			torque_ramp();
		
			break;
	}
	
	/* 指令限幅 */
	if(FOC.VqRef > FOCuser.MaxTorque)
		FOC.VqRef = FOCuser.MaxTorque;
	
	if(FOC.VqRef < -FOCuser.MaxTorque)
		FOC.VqRef = -FOCuser.MaxTorque;		
	
  PIparamD.qInMeas = FOC.IdLpf;            /* Id测量值 */
  PIparamD.qInRef  = FOC.VdRef;         /* 参考值 */ 
  calc_pi(&PIparamD);
  FOC.Vd = PIparamD.qOut;

	arm_sqrt_q15((Q15(0.98f) - (PIparamD.qOut*PIparamD.qOut>>15)), (int16_t*)&PIparamQ.qOutMax);
  PIparamQ.qOutMin = -PIparamQ.qOutMax;
	
  PIparamQ.qInMeas = FOC.IqLpf;            /* Iq测量值 */
  PIparamQ.qInRef  = FOC.VqRef;         /* 参考值 */
	
  calc_pi(&PIparamQ);
  FOC.Vq = PIparamQ.qOut;       
}

/* 加减速 */
static void velocity_ramp(void)
{
	FOC.VelRef = FOCuser.VelocityCMD;
}

/* 加减速 */
static void torque_ramp(void)
{
	FOC.VqRef = FOCuser.TorqueCMD; 
}

static void calc_svpwm(void)
{ 
  if( FOC.Va >= 0 )
  {       
    // (xx1)
    if( FOC.Vb >= 0 )
    {
      // (x11)
      // Must be Sector 3 since Sector 7 not allowed
      // Sector 3: (0,1,1)  0-60 degrees
      T1 = FOC.Va; //Vb
      T2 = FOC.Vb; //Va
      calc_pwm_time();
      SVpwm.Pwm1 = Ta;
      SVpwm.Pwm2 = Tb;
      SVpwm.Pwm3 = Tc;
    }
    else
    {            
      // (x01)
      if( FOC.Vc >= 0 )
      {
        // Sector 5: (1,0,1)  120-180 degrees
        T1 = FOC.Vc; //Va
        T2 = FOC.Va; //Vc
        calc_pwm_time();
        SVpwm.Pwm1 = Tc;
        SVpwm.Pwm2 = Ta;
        SVpwm.Pwm3 = Tb;
      }
      else
      {
        // Sector 1: (0,0,1)  60-120 degrees
        T1 = -FOC.Vc; //Vb
        T2 = -FOC.Vb; //Vc
        calc_pwm_time();
        SVpwm.Pwm1 = Tb;
        SVpwm.Pwm2 = Ta;
        SVpwm.Pwm3 = Tc;
      }
    }
  }
  else
  {
    // (xx0)
    if( FOC.Vb >= 0 )
    {
      // (x10)
      if( FOC.Vc >= 0 )
      {
        // Sector 6: (1,1,0)  240-300 degrees
        T1 = FOC.Vb; //Vc
        T2 = FOC.Vc; //Vb
        calc_pwm_time();
        SVpwm.Pwm1 = Tb;
        SVpwm.Pwm2 = Tc;
        SVpwm.Pwm3 = Ta;
      }
      else
      {
        // Sector 2: (0,1,0)  300-0 degrees
        T1 = -FOC.Va; //Vc
        T2 = -FOC.Vc; //Va
        calc_pwm_time();
        SVpwm.Pwm1 = Ta;
        SVpwm.Pwm2 = Tc;
        SVpwm.Pwm3 = Tb;
      }
    }
    else
    {            
      // (x00)
      // Must be Sector 4 since Sector 0 not allowed
      // Sector 4: (1,0,0)  180-240 degrees
      T1 = -FOC.Vb; //Va
      T2 = -FOC.Va; //Vb
      calc_pwm_time();
      SVpwm.Pwm1 = Tc;
      SVpwm.Pwm2 = Tb;
      SVpwm.Pwm3 = Ta;
    }
  }	
	
#ifdef FOC_DEBUG
  zSVPWM1 = (int32_t)SVpwm.Pwm1 - (PWM_MOD>>1);
  zSVPWM2 = (int32_t)SVpwm.Pwm2 - (PWM_MOD>>1);
  zSVPWM3 = (int32_t)SVpwm.Pwm3 - (PWM_MOD>>1);
#endif
}

static void calc_pwm_time(void)
{
  T1 = (PWM_MOD*T1>>15);
  T2 = (PWM_MOD*T2>>15);
  Tc = (PWM_MOD-T1-T2)>>1;
  Tb = Tc + T1;
  Ta = Tb + T2;
}        

void openloop_run_motor(uint16_t theta, int16_t Vout)
{  
  theta &= 0x7FFF;
  
  FOC.Va = arm_sin_q15(theta)*Vout>>15;
  FOC.Vb = arm_sin_q15((theta + 10922)&0x7FFF)*Vout>>15;
  FOC.Vc = arm_sin_q15((theta + 21844)&0x7FFF)*Vout>>15;
  
  calc_svpwm();
}


