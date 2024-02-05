#include "mc_svpwm.h"

#include "board.h"

#include "drv8301.h"

/***********************HAL库****************************/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"
#include "parameters_conversion.h"
#include "stm32f4xx.h"


_MCPWM_FOC mc_svpm;

int PWMA,PWMB,PWMC,UAA,UBB;
  u8 Step=0,a,b,c;

void Svpwm_Module(void)
{
  float t1,t2,Sum_OverMod;
  float sqrt_3=1.7320508075689/*SVPWM调制比*/;
  float Udc = MOTOR_POWER;  
  
  UAA = mc_svpm.Svpwm.UAlpha*100;
  UBB = mc_svpm.Svpwm.UBeta*100;
  
  //确定扇区
  mc_svpm.Svpwm.Ua=mc_svpm.Svpwm.UBeta;
  mc_svpm.Svpwm.Ub=   sqrt_3 * mc_svpm.Svpwm.UAlpha/2 - mc_svpm.Svpwm.UBeta/2;
  mc_svpm.Svpwm.Uc= - sqrt_3 * mc_svpm.Svpwm.UAlpha/2 - mc_svpm.Svpwm.UBeta/2;  
  
  a=mc_svpm.Svpwm.Ua>0 ? 1 :0;
  b=mc_svpm.Svpwm.Ub>0 ? 1 :0;
  c=mc_svpm.Svpwm.Uc>0 ? 1 :0;
  
  Step= 4*c + 2*b + a;  //计算所处扇区位置
  
  mc_svpm.Svpwm.Ua=sqrt_3*mc_svpm.Svpwm.Ua/Udc*Ts;
  mc_svpm.Svpwm.Ub=sqrt_3*mc_svpm.Svpwm.Ub/Udc*Ts;
  mc_svpm.Svpwm.Uc=sqrt_3*mc_svpm.Svpwm.Uc/Udc*Ts; 
  
  switch(Step)
  {   
    case 0:
             mc_svpm.Svpwm.taOn = Time1_Period / 2;
             mc_svpm.Svpwm.tbOn = Time1_Period / 2;
             mc_svpm.Svpwm.tcOn = Time1_Period / 2;
             break;
    
    case 1:           
             t1=-mc_svpm.Svpwm.Ub;
             t2=-mc_svpm.Svpwm.Uc;
             Sum_OverMod=t1+t2;
             t1=Sum_OverMod>Ts ? t1*Ts/Sum_OverMod : t1;
             t2=Sum_OverMod>Ts ? t2*Ts/Sum_OverMod : t2;
             mc_svpm.Svpwm.tbOn = (Ts-t1-t2)/4;
             mc_svpm.Svpwm.taOn = mc_svpm.Svpwm.tbOn + t1/2;
             mc_svpm.Svpwm.tcOn = mc_svpm.Svpwm.taOn + t2/2;
             break;
    
    case 2:           
             t1=-mc_svpm.Svpwm.Uc;
             t2=-mc_svpm.Svpwm.Ua;
             Sum_OverMod=t1+t2;
             t1=Sum_OverMod>Ts ? t1*Ts/Sum_OverMod : t1;
             t2=Sum_OverMod>Ts ? t2*Ts/Sum_OverMod : t2;
             mc_svpm.Svpwm.taOn = (Ts-t1-t2)/4;
             mc_svpm.Svpwm.tcOn = mc_svpm.Svpwm.taOn + t1/2;
             mc_svpm.Svpwm.tbOn = mc_svpm.Svpwm.tcOn + t2/2;
             break;

    case 3:           
             t1=mc_svpm.Svpwm.Ub;
             t2=mc_svpm.Svpwm.Ua;
             Sum_OverMod=t1+t2;
             t1=Sum_OverMod>Ts ? t1*Ts/Sum_OverMod : t1;
             t2=Sum_OverMod>Ts ? t2*Ts/Sum_OverMod : t2;
             mc_svpm.Svpwm.taOn = (Ts-t1-t2)/4;
             mc_svpm.Svpwm.tbOn = mc_svpm.Svpwm.taOn + t1/2;
             mc_svpm.Svpwm.tcOn = mc_svpm.Svpwm.tbOn + t2/2;
             break;        

    case 4:           
             t1=-mc_svpm.Svpwm.Ua;
             t2=-mc_svpm.Svpwm.Ub;
             Sum_OverMod=t1+t2;
             t1=Sum_OverMod>Ts ? t1*Ts/Sum_OverMod : t1;
             t2=Sum_OverMod>Ts ? t2*Ts/Sum_OverMod : t2;
             mc_svpm.Svpwm.tcOn = (Ts-t1-t2)/4;
             mc_svpm.Svpwm.tbOn = mc_svpm.Svpwm.tcOn + t1/2;
             mc_svpm.Svpwm.taOn = mc_svpm.Svpwm.tbOn + t2/2;
             break;    

    case 5:           
             t1=mc_svpm.Svpwm.Ua;
             t2=mc_svpm.Svpwm.Uc;
             Sum_OverMod=t1+t2;
             t1=Sum_OverMod>Ts ? t1*Ts/Sum_OverMod : t1;
             t2=Sum_OverMod>Ts ? t2*Ts/Sum_OverMod : t2;
             mc_svpm.Svpwm.tbOn = (Ts-t1-t2)/4;
             mc_svpm.Svpwm.tcOn = mc_svpm.Svpwm.tbOn + t1/2;
             mc_svpm.Svpwm.taOn = mc_svpm.Svpwm.tcOn + t2/2;
             break;    

    case 6:           
             t1=mc_svpm.Svpwm.Uc;
             t2=mc_svpm.Svpwm.Ub;
             Sum_OverMod=t1+t2;
             t1=Sum_OverMod>Ts ? t1*Ts/Sum_OverMod : t1;
             t2=Sum_OverMod>Ts ? t2*Ts/Sum_OverMod : t2;
             mc_svpm.Svpwm.tcOn = (Ts-t1-t2)/4;
             mc_svpm.Svpwm.taOn = mc_svpm.Svpwm.tcOn + t1/2;
             mc_svpm.Svpwm.tbOn = mc_svpm.Svpwm.taOn + t2/2;
             break;   

    default:break;    
  } 
  
  //倒三角模式，重新计算占空比
    mc_svpm.Duty.MOTA =Time1_Period - mc_svpm.Svpwm.taOn;
    mc_svpm.Duty.MOTB =Time1_Period - mc_svpm.Svpwm.tbOn;  
    mc_svpm.Duty.MOTC =Time1_Period - mc_svpm.Svpwm.tcOn;  
        
    if(t1==0&&t2==0)
    {
     mc_svpm.Duty.MOTA =0;
     mc_svpm.Duty.MOTB =0;
     mc_svpm.Duty.MOTC =0;      
    }      
  
    mc_svpm.Duty.MOTA = mc_svpm.Duty.MOTA > MAX_Duty ? MAX_Duty :mc_svpm.Duty.MOTA; 
    mc_svpm.Duty.MOTB = mc_svpm.Duty.MOTB > MAX_Duty ? MAX_Duty :mc_svpm.Duty.MOTB; 
    mc_svpm.Duty.MOTC = mc_svpm.Duty.MOTC > MAX_Duty ? MAX_Duty :mc_svpm.Duty.MOTC; 
    
    PWMA= mc_svpm.Duty.MOTA;
    PWMB= mc_svpm.Duty.MOTB; 
    PWMC= mc_svpm.Duty.MOTC;
    
    TIM1->CCR1 = mc_svpm.Duty.MOTA ;
    TIM1->CCR2 = mc_svpm.Duty.MOTB ;    
    TIM1->CCR3 = mc_svpm.Duty.MOTC ;    
		

}






/**
* @brief Computes and return latest converted motor phase currents motor
* @param pHandle ICS F4xx PWM Current Feedback Handle
* @retval Ia and Ib current in ab_t format
*/
uint16_t curr_ia,curr_ib;
void GetPhaseCurrents(void)
{
  int32_t aux;
  uint16_t reg;

  /* disable ADC trigger */
  LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH4);

  /* Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  */
  reg = ( uint16_t )( ( ADC1->JDR1 ) >> 3 );
  aux = ( int32_t )( reg ) - ( int32_t )( mc_svpm.AdcData.offeset[PhaseA] );
	
  /* Saturation of Ia */
  if ( aux < -INT16_MAX )
  {
    mc_svpm.Clack.Ia = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    mc_svpm.Clack.Ia = INT16_MAX;
  }
  else
  {
    mc_svpm.Clack.Ia = ( int16_t )aux;
  }

  /* Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value) */
  reg = ( uint16_t )( ( ADC2->JDR1 ) >>3 );
  aux = ( int32_t )( reg ) - ( int32_t )( mc_svpm.AdcData.offeset[PhaseB] );
	
  /* Saturation of Ib */
  if ( aux < -INT16_MAX )
  {
    mc_svpm.Clack.Ib = -INT16_MAX;
  }
  else  if ( aux > INT16_MAX )
  {
    mc_svpm.Clack.Ib = INT16_MAX;
  }
  else
  {
    mc_svpm.Clack.Ib = ( int16_t )aux;
  }

  mc_svpm.Clack.Ic = -mc_svpm.Clack.Ia - mc_svpm.Clack.Ib;

}




/**
* @brief Enables PWM generation on the proper Timer peripheral acting on MOE bit
* @param pHdl ICS F4xx PWM Current Feedback Handle
*/
void SwitchOnPWM(void)
{

  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIM1, (uint32_t)(PWM_PERIOD_CYCLES/2  >> 1));
  LL_TIM_OC_SetCompareCH2(TIM1, (uint32_t)(PWM_PERIOD_CYCLES/2  >> 1));
  LL_TIM_OC_SetCompareCH3(TIM1, (uint32_t)(PWM_PERIOD_CYCLES/2  >> 1));
  LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(PWM_PERIOD_CYCLES/2 - 5u));

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIM1 );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIM1 );


  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE( TIM1 );
	
	DRV8301_EN_GATE_HIGH;

}


/**
* @brief  Disables PWM generation on the proper Timer peripheral acting on
*         MOE bit
* @param pHdl ICS F4xx PWM Current Feedback Handle
*/
void SwitchOffPWM(void)
{

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIM1 );

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIM1);

	
  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIM1 );

  return;
}



void CurrentsCalibration(void)
{
		
		DRV8301_EN_GATE_LOW;												//关闭MosDriver   
	
		DelayMs(50);

		mc_svpm.AdcData.Calibration = 1;						//开始采集ADC
		
		while(mc_svpm.AdcData.Calibration){};		//等待采集完成
			
		mc_svpm.AdcData.offeset[PhaseA] = mc_svpm.AdcData.offeset[PhaseA] / 50;
		mc_svpm.AdcData.offeset[PhaseB] = mc_svpm.AdcData.offeset[PhaseB] / 50;
			
		DelayMs(50);
	
		DRV8301_EN_GATE_HIGH;												//校准完成，开启MosDriver   

}








