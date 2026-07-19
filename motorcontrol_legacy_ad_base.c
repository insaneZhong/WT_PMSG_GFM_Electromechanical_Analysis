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
#include        "motorcontrol_legacy_tunable.h"

extern float legacy_omega_rel_ad;
extern float legacy_msc_iq_ff_a;
extern float system_Time;
extern int legacy_lvrt_active;
extern MOTOR motor;
#ifndef MSC_AD_IQ_GAIN
#define MSC_AD_IQ_GAIN 0.0f
#endif
#ifndef MSC_AD_IQ_LIMIT
#define MSC_AD_IQ_LIMIT 50.0f
#endif
#ifndef MSC_LVRT_IQ_LIMIT_A
#define MSC_LVRT_IQ_LIMIT_A 0.0f
#endif
#ifndef MSC_LVRT_VOLTAGE_MODULATION_LIMIT
#define MSC_LVRT_VOLTAGE_MODULATION_LIMIT 0.0f
#endif
#ifndef MSC_LVRT_VECTOR_AW_GAIN
#define MSC_LVRT_VECTOR_AW_GAIN 0.0f
#endif
#ifndef MSC_LVRT_FREEZE_CURRENT_PI_INTEGRAL
#define MSC_LVRT_FREEZE_CURRENT_PI_INTEGRAL 0
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
    float  Lq_Ld_temp,Temp1,Temp2;
    float  iq_ad;
    float  voltage_magnitude;
    float  voltage_limit;
    float  voltage_scale;
    float  voltage_ud_unsat;
    float  voltage_uq_unsat;

    p->pwm_speed_pi.Ref = p->ref.voltage_ref;
    p->pwm_speed_pi.Fdb = p->bak.Udc1;
    if (system_Time < MSC_DVC_ENABLE_TIME_S)
    {
        p->pwm_speed_pi.reset(&p->pwm_speed_pi);
    }
    else
    {
        motor_PI2_calc(&p->pwm_speed_pi);
    }
    
 	p->val.Id_ref  = 0;
	/* PMSM电磁转矩与Iq同号。omega_t-omega_g为正时增加正转矩，
	 * 使发电机侧加速并抑制相对运动。 */
	iq_ad = MSC_AD_IQ_GAIN * legacy_omega_rel_ad;
	if (iq_ad > MSC_AD_IQ_LIMIT) iq_ad = MSC_AD_IQ_LIMIT;
	if (iq_ad < -MSC_AD_IQ_LIMIT) iq_ad = -MSC_AD_IQ_LIMIT;
 	p->val.Iq_ref  = -legacy_msc_iq_ff_a - p->pwm_speed_pi.Out + iq_ad;
    if (legacy_lvrt_active && MSC_LVRT_IQ_LIMIT_A > 0.0f)
    {
        if (p->val.Iq_ref > MSC_LVRT_IQ_LIMIT_A)
            p->val.Iq_ref = MSC_LVRT_IQ_LIMIT_A;
        if (p->val.Iq_ref < -MSC_LVRT_IQ_LIMIT_A)
            p->val.Iq_ref = -MSC_LVRT_IQ_LIMIT_A;
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
    if (MSC_CURRENT_PI_FREEZE_UNTIL_DVC &&
        system_Time < MSC_DVC_ENABLE_TIME_S)
    {
        p->id_pi.Ui = 0.0f;
        p->id_pi.SatErr = 0.0f;
        p->id_pi.Up_old = 0.0f;
        p->iq_pi.Ui = 0.0f;
        p->iq_pi.SatErr = 0.0f;
        p->iq_pi.Up_old = 0.0f;
    }
	if (legacy_lvrt_active && MSC_LVRT_FREEZE_CURRENT_PI_INTEGRAL)
    {
        float ui_hold = p->id_pi.Ui;
        p->id_pi.calc2(&p->id_pi);
        p->id_pi.Ui = ui_hold;
    }
    else
    {
        p->id_pi.calc2(&p->id_pi);
    }
	p->iq_pi.Ref          = p->val.Iq_ref;
	p->iq_pi.Fdb          = p->bak.Iq1;
	if (legacy_lvrt_active && MSC_LVRT_FREEZE_CURRENT_PI_INTEGRAL)
    {
        float ui_hold = p->iq_pi.Ui;
        p->iq_pi.calc2(&p->iq_pi);
        p->iq_pi.Ui = ui_hold;
    }
    else
    {
        p->iq_pi.calc2(&p->iq_pi);
    }

    p->val.Ud_fwd    = p->par.Rs * p->id_pi.Ref -  p->par.Polar*p->bak.We * p->par.Lq * p->iq_pi.Ref;
    p->val.Uq_fwd    = p->par.Rs * p->iq_pi.Ref +  p->par.Polar*p->bak.We *(p->par.Ld * p->id_pi.Ref + p->par.Fm);

    p->val.Ud1_ref = p->id_pi.Out+p->val.Ud_fwd;
    p->val.Uq1_ref = p->iq_pi.Out+p->val.Uq_fwd;
    if (legacy_lvrt_active &&
        MSC_LVRT_VOLTAGE_MODULATION_LIMIT > 0.0f &&
        p->bak.Udc1 > 1.0f)
    {
        voltage_magnitude = sqrtf(
            p->val.Ud1_ref*p->val.Ud1_ref +
            p->val.Uq1_ref*p->val.Uq1_ref);
        voltage_limit =
            MSC_LVRT_VOLTAGE_MODULATION_LIMIT*p->bak.Udc1/1.5f;
        if (voltage_magnitude > voltage_limit &&
            voltage_magnitude > 1.0e-6f)
        {
            voltage_ud_unsat = p->val.Ud1_ref;
            voltage_uq_unsat = p->val.Uq1_ref;
            voltage_scale = voltage_limit/voltage_magnitude;
            p->val.Ud1_ref *= voltage_scale;
            p->val.Uq1_ref *= voltage_scale;
            p->id_pi.Ui += MSC_LVRT_VECTOR_AW_GAIN *
                (p->val.Ud1_ref - voltage_ud_unsat);
            p->iq_pi.Ui += MSC_LVRT_VECTOR_AW_GAIN *
                (p->val.Uq1_ref - voltage_uq_unsat);
        }
    }
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
    /*
     * The nonnegative-integral option belongs only to the MSC DC-voltage
     * controller.  Applying it in this shared PI routine to every instance
     * prevents the GSC power/voltage/current loops from using a negative
     * integral state and breaks off-nominal-frequency synchronization.
     */
    if (MSC_DVC_NONNEGATIVE_INTEGRAL &&
        v == &motor.pwm_speed_pi && v->Ui < 0.0f)
    {
        v->Ui = 0.0f;
    }
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




