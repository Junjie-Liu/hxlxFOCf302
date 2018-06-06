
#include "stm32f3xx_hal.h"
#include "arm_math.h"
#include "foc.h"
#include "foc_param.h"
#include "QEI.h"

PIparam_t  Q_PI[6]; /* 6个轴的PI参数 */
PIparam_t  D_PI[6]; /* 6个轴的PI参数 */
PIparam_t  V_PI[6]; /* 6个轴的PI参数 */
PIDparam_t  P_PI[6]; /* 6个轴的PI参数 */

void foc_param_init(uint8_t servo_id)
{
  FOC.Vbus = 0;
  FOC.Ia = 0;
  FOC.Ib = 0;
  FOC.Ia_offset = 0;
  FOC.Ib_offset = 0;
	FOC.ADCperI = ADC_PER_MA;
  FOC.Ialpha = 0;
  FOC.Ibeta = 0;
  FOC.Id = 0;
  FOC.Iq = 0;
  FOC.IdLpf = 0;
  FOC.IqLpf = 0;	
  FOC.IdLpK = IDQ_LPFK;
  FOC.IqLpK = IDQ_LPFK;	
  FOC.Vd = 0;
  FOC.Vq = 0;
  FOC.Valpha = 0;
  FOC.Vbeta = 0;		
  FOC.Va = 0;
  FOC.Vb = 0;
  FOC.Vc = 0;		
  FOC.Sin = 0;
  FOC.Cos = 0;
  FOC.VelRef = 0;
  FOC.VdRef = 0;
  FOC.VqRef = 0;

  FOC.QEIen = QEI_EN;
  FOC.QEIthetaOk = 0;
  FOC.QEIcnt = 0;
  FOC.QEItheta = 0;
	FOC.QEIthetaOffset = 0;
  FOC.QEIvel = 0;
  FOC.QEIvelLpf = 0;	
  FOC.QEIvelLpK = QEI_VEL_LPFK;
  FOC.QEIpos = 0;
  FOC.QEIposLpf = 0;	
  FOC.QEIposLpK = QEI_POS_LPFK;	
  FOC.QEIposOk = 0;
	
  FOC.GEAR_MAGvel = 0;         
  FOC.GEAR_MAGvelLpf = 0;          
  FOC.GEAR_MAGvelLpK = MAG_VEL_LPFK;	
  FOC.GEAR_MAGacc = 0;  	
  FOC.GEAR_MAGaccLpf = 0;          
  FOC.GEAR_MAGaccLpK = MAG_ACC_LPFK;		
  FOC.GEAR_MAGpos = 0;
  FOC.GEAR_MAGposLpf = 0;          
  FOC.GEAR_MAGposLpK = MAG_POS_LPFK;		
  FOC.GEAR_MAGposOk = 0;
	

	PIparamD.qKp = A123_D_CURRCNTR_PTERM;       
	PIparamD.qKi = A123_D_CURRCNTR_ITERM;              
	PIparamD.qKc = A123_D_CURRCNTR_CTERM;       
	PIparamD.qOutMax =  A123_D_CURRCNTR_OUTMAX;
	PIparamD.qOutMin = -A123_D_CURRCNTR_OUTMAX;
	PIparamD.qdSum = 0;
	PIparamD.qOut = 0;
	PIparamD.qdSumMax =  A123_Q_CURRCNTR_SUMMAX;	
	PIparamD.qdSumMin = -A123_Q_CURRCNTR_SUMMAX;			
	
	PIparamQ.qKp = A123_Q_CURRCNTR_PTERM;    
	PIparamQ.qKi = A123_Q_CURRCNTR_ITERM;
	PIparamQ.qKc = A123_Q_CURRCNTR_CTERM;
	PIparamQ.qOutMax =  A123_Q_CURRCNTR_OUTMAX;
	PIparamQ.qOutMin = -A123_Q_CURRCNTR_OUTMAX;		
	PIparamQ.qdSum = 0;
	PIparamQ.qOut = 0;	
	PIparamQ.qdSumMax =  A123_D_CURRCNTR_SUMMAX;
	PIparamQ.qdSumMin = -A123_D_CURRCNTR_SUMMAX;
	
	PIparamV.qKp = A123_SPEEDCNTR_PTERM;       
	PIparamV.qKi = A123_SPEEDCNTR_ITERM;       
	PIparamV.qKc = A123_SPEEDCNTR_CTERM;       
	PIparamV.qOutMax =  A123_SPEEDCNTR_OUTMAX;   
	PIparamV.qOutMin = -A123_SPEEDCNTR_OUTMAX;		
	PIparamV.qdSum = 0;
	PIparamV.qOut = 0;
	PIparamV.qdSumMax =  A123_SPEEDCNTR_SUMMAX;	
	PIparamV.qdSumMin = -A123_SPEEDCNTR_SUMMAX;			
	
	PIDparamP.qKp = A123_POSITION_PTERM;       
	PIDparamP.qKi = A123_POSITION_ITERM;    
	PIDparamP.qKd = A123_POSITION_DTERM;  	
	PIDparamP.qKc = A123_POSITION_CTERM;       
	PIDparamP.qOutMax =  A123_POSITION_OUTMAX;   
	PIDparamP.qOutMin = -A123_POSITION_OUTMAX;		
	PIDparamP.qdSum = 0;
	PIDparamP.qOut = 0;
	PIDparamP.qdSumMax =  A123_POSITION_SUMMAX;	
	PIDparamP.qdSumMin = -A123_POSITION_SUMMAX;	
	PIDparamP.qdErrLpf = 0;
	PIDparamP.qdErrLpK = 4096;
  
  FOCuser.MotorMode    = START_MODE;
  FOCuser.TorqueRamp   = 0;
  FOCuser.VelocityRamp = 0;
	FOCuser.MaxTorque    = MT_MAX_TORQUE;
	FOCuser.MaxVelocity  = MT_MAX_VELOCITY;
}