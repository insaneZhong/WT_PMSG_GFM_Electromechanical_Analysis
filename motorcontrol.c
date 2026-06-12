//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//#  FunctionName:   电机模型主函数
//#      Designer:   机侧变流器控制
//#   Description:   目标是获得恒定的直流母线电压
//#      history:    2025-8-13
//##########################################################################################################
//##########################################################################################################
#include        "math.h" 
#include        "motorcontrol.h"
#ifdef ENABLE_SCHEMEA_OVERRIDES
#include        "schemeA_tuning_overrides.h"
#endif

//##########################################################################################################
//##########################################################################################################
//                 电机初始化
//##########################################################################################################
void motor_init(MOTOR *p)
{

}
//##########################################################################################################
//                             电机控制主程序     
//##########################################################################################################
void motor_control(MOTOR *p)
{	
    float iq_ref_target;
    float iq_power_ff;

 	p->val.Id_ref  = 0;
    p->iq_slope_limit.Ts = p->Ts;
    p->iq_slope_limit.Init = 0;
    p->iq_slope_limit.Slope = MOTOR_IQ_REF_SLOPE_LIMIT_MAX;

    if (p->ref.dvc_enable == 0)
    {
        /* Presynchronization: avoid integral accumulation before power export. */
        p->pwm_speed_pi.reset(&p->pwm_speed_pi);
        p->iq_slope_limit.reset(&p->iq_slope_limit);
        p->val.Iq_ref = 0;
    }
    else
    {
        p->pwm_speed_pi.Ref = p->ref.voltage_ref;
        p->pwm_speed_pi.Fdb = p->bak.Udc1;
        motor_PI2_calc(&p->pwm_speed_pi);

        /* MSC-DVC Type a/c switch:
         * Type a uses only DC-voltage PI feedback.
         * Type c adds active-power feedforward to the Type-a baseline.
         */
#if MOTOR_MSC_DVC_TYPE == MOTOR_MSC_DVC_TYPE_C
        iq_power_ff = -MOTOR_IQ_POWER_FF_A_PER_W * p->ref.active_power_ref;
#elif MOTOR_MSC_DVC_TYPE == MOTOR_MSC_DVC_TYPE_A
        iq_power_ff = 0.0f;
#else
#error "Unsupported MOTOR_MSC_DVC_TYPE. Use MOTOR_MSC_DVC_TYPE_A or MOTOR_MSC_DVC_TYPE_C."
#endif
        iq_ref_target = iq_power_ff - p->pwm_speed_pi.Out;
        if (iq_ref_target > MOTOR_IQ_LIMIT_MAX) iq_ref_target = MOTOR_IQ_LIMIT_MAX;
        if (iq_ref_target < -MOTOR_IQ_LIMIT_MAX) iq_ref_target = -MOTOR_IQ_LIMIT_MAX;
        p->iq_slope_limit.In = iq_ref_target;
        motor_slope_limit_calc(&p->iq_slope_limit);
        p->val.Iq_ref = p->iq_slope_limit.Out;
    }
 	//*********电流反馈***********************

    p->val.RotorPos    = p->bak.RotorPos*p->par.Polar;
    if(p->val.RotorPos>MOTOR_2PI_RADIAN)
    {p->val.RotorPos=p->val.RotorPos-MOTOR_2PI_RADIAN;}
    if(p->val.RotorPos<0)
    {p->val.RotorPos=p->val.RotorPos+MOTOR_2PI_RADIAN;}

    p->bak.I_alfa=p->bak.Ia1;
    p->bak.I_beta=MOTOR_1SQRT3*(p->bak.Ia1+2.0*p->bak.Ib1);
    
	p->bak.CosPos     = cos(p->val.RotorPos);
	p->bak.SinPos     = sin(p->val.RotorPos);
	p->bak.Id1         =(p->bak.I_alfa * p->bak.CosPos)+(p->bak.I_beta * p->bak.SinPos);
	p->bak.Iq1         =(p->bak.I_beta * p->bak.CosPos)-(p->bak.I_alfa * p->bak.SinPos);  
       	
    p->id_pi.Ref          = p->val.Id_ref ;
	p->id_pi.Fdb          = p->bak.Id1;
	p->id_pi.calc2(&p->id_pi);
	p->iq_pi.Ref          = p->val.Iq_ref;
	p->iq_pi.Fdb          = MOTOR_IQ_FEEDBACK_SIGN * p->bak.Iq1;
	p->iq_pi.calc2(&p->iq_pi);	

    p->val.Ud_fwd    = p->par.Rs * p->id_pi.Ref -  p->par.Polar*p->bak.We * p->par.Lq * p->iq_pi.Ref;
    p->val.Uq_fwd    = p->par.Rs * p->iq_pi.Ref +  p->par.Polar*p->bak.We *(p->par.Ld * p->id_pi.Ref + p->par.Fm);

    p->val.Ud1_ref = p->id_pi.Out+p->val.Ud_fwd;
    p->val.Uq1_ref = MOTOR_IQ_PI_OUTPUT_SIGN * p->iq_pi.Out+p->val.Uq_fwd;
   /*************************************************************
	*  电压 坐标变换 
 	**************************************************************/ 
	p->out.Us_alfa = (p->val.Ud1_ref * p->bak.CosPos)- (p->val.Uq1_ref * p->bak.SinPos);
	p->out.Us_beta = (p->val.Ud1_ref * p->bak.SinPos)+ (p->val.Uq1_ref * p->bak.CosPos);
}

//##########################################################################################################
//##########################################################################################################
/**************************************************************************        
 FunctionName: void motor_back_calculate(MOTOR *p)	  
***************************************************************************/  
void motor_reset(MOTOR *p)	
{
    p->id_pi.reset(&p->id_pi);
    p->iq_pi.reset(&p->iq_pi);
    p->pwm_speed_pi.reset(&p->pwm_speed_pi);
    p->id_slope_limit.reset(&p->id_slope_limit);
    p->iq_slope_limit.reset(&p->iq_slope_limit);
    p->speed_slope_limit.reset(&p->speed_slope_limit);
}

//##########################################################################################################
//#                            斜率给定                                                                    #                                                  
//##########################################################################################################
void motor_slope_limit_calc(MOTOR_SLOPE_LIMIT *v)
{
	float  DeltTemp=0;
	DeltTemp =  fabsf(v->Slope * v->Ts);
	//上升斜率限制
	if((v->In - v->Out ) > DeltTemp)
	{v->Out= v->Out+ DeltTemp;}
	//下降斜率限制
	else if((v->In - v->Out ) < -DeltTemp)
	{v->Out= v->Out- DeltTemp;}
	else
	{v->Out= v->In;}
}
void motor_slope_limit_reset(MOTOR_SLOPE_LIMIT *v)
{
	v->Out= v->Init;
	v->In = v->Init;
}

//*********增量式PI*********************************************          
void motor_PI1_calc(MOTOR_PI *v)
{	
    float  Temp_out=0,  Error=0;
    
    Error   = v->Ref - v->Fdb;	
	Temp_out= v->Kp*(Error-v->Ui)+(v->Ki*Error)+v->Out;
	v->Ui   = Error;
	v->Out  = Temp_out;

	if(Temp_out>v->OutMax)
	{v->Out = v->OutMax;}
	else if(Temp_out<v->OutMin)
	{v->Out = v->OutMin;}    
}

//##########################################################################################################
//****************TI位置式PI************************************         
void motor_PI2_calc(MOTOR_PI *v)
{   
    //计算误差
	v->Error = v->Ref - v->Fdb;
    //计算比例输出
	v->Up = v->Kp * v->Error;
	//计算积分输出
	v->Ui = v->Ui+ v->Ki * v->Up + v->Kc * v->SatErr;
	//计阄⒅输出
	v->Ud = v->Kd * (v->Up - v->Up_old);
	//计算输出值
	v->OutPreSat=v->Up + v->Ui + v->Ud;

	//限幅输出
	if(v->OutPreSat > v->OutMax)
	{v->Out = v->OutMax;}
	else if(v->OutPreSat < v->OutMin)
	{v->Out = v->OutMin;}
	else
	{v->Out=v->OutPreSat;}

	//计算抗饱和积分差值
    v->SatErr = v->Out- v->OutPreSat;
	//更卤壤涑鲋?
    v->Up_old=v->Up;
}

//##########################################################################################################
//****************复位PI环节***********************************          
void motor_PI_reset(MOTOR_PI *v)
{
	v->Error    = 0;
	v->Up       = 0;
    v->Up_old   = 0 ;
	v->Ui       = 0;
	v->Ud       = 0;
	v->OutPreSat= 0;
    v->SatErr   = 0;
	v->Out      = 0;
}






