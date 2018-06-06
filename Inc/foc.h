
#ifndef FOC_H
#define FOC_H

#include <stdint.h>
#define FOC_DEBUG 1

typedef struct
{
  int32_t Vbus;          
  int32_t Ia;                
  int32_t Ib;           
  int32_t Ia_offset;          
  int32_t Ib_offset;
  int32_t ADCperI;	
  int32_t Ialpha;          
  int32_t Ibeta;
  int32_t Id;
  int32_t Iq;
  int32_t IdLpf;
  int32_t IqLpf;		
  int32_t IdLpK;
  int32_t IqLpK;	
  int32_t Vd;             
  int32_t Vq;        
  int32_t Valpha;             
  int32_t Vbeta;		     
  int32_t Va;             
  int32_t Vb;
  int32_t Vc;		
  int32_t Sin;
  int32_t Cos;      
  int32_t VelRef;          
  int32_t VdRef;             
  int32_t VqRef;
  uint32_t QEIen;           
  uint32_t QEIthetaOk;      
  uint32_t QEIcnt;             
  uint32_t QEItheta; 
	int32_t  QEIthetaOffset; 
  int32_t  QEIvel;         
  int32_t  QEIvelLpf;          
  int32_t  QEIvelLpK;
  int32_t  QEIpos;
  int32_t  QEIposLpf;          
  int32_t  QEIposLpK;	
  uint32_t QEIposOk;
  int32_t  GEAR_MAGvel;         
  int32_t  GEAR_MAGvelLpf;          
  int32_t  GEAR_MAGvelLpK;
	int32_t  GEAR_MAGacc;
  int32_t  GEAR_MAGaccLpf;          
  int32_t  GEAR_MAGaccLpK;	
  int32_t  GEAR_MAGpos;
  int32_t  GEAR_MAGposLpf;          
  int32_t  GEAR_MAGposLpK;		
  uint32_t GEAR_MAGposOk;			
}FOC_t;

typedef struct 
{
  int32_t  qdSum;     
  int32_t  qKp;        
  int32_t  qKi;      
  int32_t  qKc;        
  int32_t  qOutMax;    
  int32_t  qOutMin;
  int32_t  qInRef;    
  int32_t  qInMeas;  
  int32_t  qOut;     
	int32_t  qdSumMax;
	int32_t  qdSumMin;	
}PIparam_t;

typedef struct 
{
  int32_t  qdSum;     
  int32_t  qKp;        
  int32_t  qKi;      
  int32_t  qKc;        
  int32_t  qOutMax;    
  int32_t  qOutMin;
  int32_t  qInRef;    
  int32_t  qInMeas;  
  int32_t  qOut;     
	int32_t  qdSumMax;
	int32_t  qdSumMin;
	int32_t  qKd;
	int32_t  qLastErr;
	int32_t  qdErrLpf;
	int32_t  qdErrLpK;
}PIDparam_t;

typedef struct 
{
  uint16_t  Pwm1;    
  uint16_t  Pwm2;
  uint16_t  Pwm3;
}SVpwm_t;	

typedef struct
{
  uint8_t OpenLoop;
  uint8_t ErrorMode;   /* 故障标志 */
}FOCflag_t;

typedef struct
{
  uint8_t MotorMode;   /* 模式标志 ,速度/扭矩/位置 */
  int32_t TorqueCMD;   /* 扭矩指令 */
  int32_t VelocityCMD; /* 速度指令 */
  int32_t PositionCMD; /* 位置指令 */
  int32_t TorqueRamp;  /* 扭矩斜坡 */
  int32_t VelocityRamp; /* 速度斜坡 */
	int32_t MaxTorque;    /* 最大扭力 */
	int32_t MaxVelocity;  /* 最高转速 */
}FOCuser_t;

extern FOC_t FOC;	
extern FOCflag_t FOCflag;
extern FOCuser_t FOCuser;
extern PIparam_t PIparamQ;
extern PIparam_t PIparamD;
extern PIparam_t PIparamV;
extern PIDparam_t PIDparamP;
extern SVpwm_t  SVpwm;

void foc_loop(FOC_t* pFOC);
void openloop_run_motor(uint16_t theta, int16_t Vout);

#endif


