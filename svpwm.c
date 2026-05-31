//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
//##########################################################################################################
// FunctionName:  调制模块
//     Designer:  廖武
//  Description:  
//      history: 2020.3.8
//##########################################################################################################
//##########################################################################################################
#include        "math.h"
#include        "svpwm.h"

void Modify_PWM(PWM *p);

PWMSWICHTABLE    PwmTable[12]= SVPWM_TABLE;
PWMSWICHTABLE1   PwmTable1[12]= SVPWM_TABLE1;

#define   SQRT3            1.732
#define   PWM_1PI6         PWM_1PI6_RADIAN

short   i=0;
//7段氏SVPWM

int const INV[6][7]={{U0,U1,U2,U7,U2,U1,U0},\
                     {U0,U3,U2,U7,U2,U3,U0},\
                     {U0,U3,U4,U7,U4,U3,U0},\
                     {U0,U5,U4,U7,U4,U5,U0},\
                     {U0,U5,U6,U7,U6,U5,U0},\
                     {U0,U1,U6,U7,U6,U1,U0}};
                    
void  svpwm_init(PWM *p)
{      
    short   i=0;
    for(i=0;i<7;i++)
    {
    	p->PwmOutTemp.State1[i]  = U_BLANK;
    	p->PwmOutTemp.Timer1[i]  = (unsigned int)(p->Par.FPGACountPerS*(2.5e-4));//250毫秒
    }
    p->Val.PwmVecterPeriod = 1/(2*p->Par.PWMSwitchFrequency);
}

void  svpwm_reset(PWM *p)
{
    
    for(i=0;i<7;i++)
    {
    	p->PwmOutTemp.State1[i]= U_BLANK;
    	p->PwmOutTemp.Timer1[i]= (unsigned int)(p->Par.FPGACountPerS*(2.5e-4));//250毫秒
    }
    p->PwmTabPtr = PwmTable;
    p->Val.PwmVecterPeriod = 1/(2*p->Par.PWMSwitchFrequency);
}

void  svpwm_calc(PWM *p)
{

    //*****将静止坐标系转换成极坐标系
    p->Ref.Us_Phase  = atan2(p->Ref.U_beta, p->Ref.U_alfa);//如果要求速度，可以用查表来实现
    if(p->Ref.Us_Phase<0)
    {p->Ref.Us_Phase+=PWM_2PI_RADIAN;}
    p->Ref.Us_Amplitude = sqrt(p->Ref.U_alfa*p->Ref.U_alfa+ p->Ref.U_beta*p->Ref.U_beta);
    //******************各种脉冲延时补偿*************************************
    if(p->Ref.Us_Phase> PWM_2PI_RADIAN)                               //将Us相角归一到0-2*PI
    {p->Ref.Us_Phase  = p->Ref.Us_Phase - PWM_2PI_RADIAN;}
    else if(p->Ref.Us_Phase< 0)
    {p->Ref.Us_Phase  = p->Ref.Us_Phase + PWM_2PI_RADIAN;}
    
    p->Val.PwmVecterPeriod = 1.0/(p->Par.PWMSwitchFrequency);
    p->Ref.ModulationDepth=   p->Ref.Us_Amplitude/p->Ref.Udc*1.5;

    //*******************************************************************
    if((p->Ref.Us_Phase>=0)&&(p->Ref.Us_Phase<PWM_1PI3_RADIAN))  //第一扇区
    {
        p->Val.SectorNumber = 1;
    }
    else if((p->Ref.Us_Phase>=PWM_1PI3_RADIAN) && (p->Ref.Us_Phase<PWM_2PI3_RADIAN))  //第二扇区
    {
        p->Val.SectorNumber = 2;
        p->Ref.Us_Phase=p->Ref.Us_Phase-PWM_1PI3_RADIAN;
    }
    else if((p->Ref.Us_Phase>=PWM_2PI3_RADIAN) && (p->Ref.Us_Phase<PWM_PI_RADIAN))    //第三扇区
    {
        p->Val.SectorNumber = 3;
         p->Ref.Us_Phase=p->Ref.Us_Phase-PWM_2PI3_RADIAN;
    }
    else if((p->Ref.Us_Phase>=PWM_PI_RADIAN) && (p->Ref.Us_Phase<PWM_4PI3_RADIAN)) 	  //第四扇区
    {
        p->Val.SectorNumber = 4;
        p->Ref.Us_Phase=p->Ref.Us_Phase-PWM_PI_RADIAN;
    }
    else if((p->Ref.Us_Phase>=PWM_4PI3_RADIAN) && (p->Ref.Us_Phase<PWM_5PI3_RADIAN))  //第五扇区
    {
        p->Val.SectorNumber = 5;
        p->Ref.Us_Phase=p->Ref.Us_Phase-PWM_4PI3_RADIAN;
    }
    else if((p->Ref.Us_Phase>=PWM_5PI3_RADIAN) && (p->Ref.Us_Phase<PWM_2PI_RADIAN))  //第六扇区
    {
        p->Val.SectorNumber = 6;
        p->Ref.Us_Phase=p->Ref.Us_Phase-PWM_5PI3_RADIAN;
    }
    
    p->Val.T1=p->Val.PwmVecterPeriod*p->Ref.ModulationDepth*sin(PWM_1PI3_RADIAN-p->Ref.Us_Phase)/PWM_SQRT3_DIVI_2;
    p->Val.T2=p->Val.PwmVecterPeriod*p->Ref.ModulationDepth*sin(p->Ref.Us_Phase)/PWM_SQRT3_DIVI_2;
    p->Val.T0=p->Val.PwmVecterPeriod- p->Val.T1-p->Val.T2; 
    
    if  (( p->Val.T1+ p->Val.T2)>p->Val.PwmVecterPeriod)
    {
        p->Val.T1=p->Val.T1/( p->Val.T1+ p->Val.T2)*p->Val.PwmVecterPeriod;
        p->Val.T2=p->Val.T2/( p->Val.T1+ p->Val.T2)*p->Val.PwmVecterPeriod;
        p->Val.T0=p->Val.PwmVecterPeriod-p->Val.T1-p->Val.T2;
    }
    if(p->Val.T1<0)
    p->Val.T1=0;
    if(p->Val.T2<0)
    p->Val.T2=0;
    if(p->Val.T0<0)
    p->Val.T0=0;

        for(i=0;i<7;i++)
        {
            p->PwmOutTemp.State1[i]= INV[p->Val.SectorNumber-1][i];
        }
        if((p->Val.SectorNumber==1)||(p->Val.SectorNumber==3)||(p->Val.SectorNumber==5))
        {
            p->PwmOutTemp.Timer1[0] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T0*0.25);
            p->PwmOutTemp.Timer1[1] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T1*0.5);
            p->PwmOutTemp.Timer1[2] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T2*0.5);
            p->PwmOutTemp.Timer1[3] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T0*0.5);
            p->PwmOutTemp.Timer1[4] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T2*0.5);
            p->PwmOutTemp.Timer1[5] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T1*0.5);
            p->PwmOutTemp.Timer1[6] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T0*0.25);
        }
      
        if((p->Val.SectorNumber==2)||(p->Val.SectorNumber==4)||(p->Val.SectorNumber==6))
        {
            p->PwmOutTemp.Timer1[0] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T0*0.25);
            p->PwmOutTemp.Timer1[1] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T2*0.5);
            p->PwmOutTemp.Timer1[2] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T1*0.5);
            p->PwmOutTemp.Timer1[3] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T0*0.5);
            p->PwmOutTemp.Timer1[4] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T1*0.5);
            p->PwmOutTemp.Timer1[5] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T2*0.5);
            p->PwmOutTemp.Timer1[6] = (unsigned int)(p->Par.FPGACountPerS* p->Val.T0*0.25);
        }

    
}


