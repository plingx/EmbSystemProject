/**
* 版权所有(C) 2017 - 2022.　北京一维弦科技有限公司. 著作人保留所有权利.
*
* 本文件包含部分Ewaybot Operating System(EwayOS)代码,为开源代码,在遵守BSD 2-Clause License的前提下，
* 开发者可再发布软件或以源代码及二进制形式使用软件，包括进行修改或不进行修改。
* 公司通信地址：北京市海淀区北三环西路48号北京科技会展中心1B座16L
*
* @file IMU.c
* @brief 本文件的概要说明.
* @details 本文件的详细说明.IMU和拉线传感器
* @author 
* @version 0.0.1
* @date 2017-12-20
* @par Description:
* @par Change History:
*
* <Date> | <Version> | <Author> | <Description>
*
* 2018/01/20 | 0.0.1 | xx       | Create file
*
*/
#include "includes.h"



EwayIMUModule EwayImu={{0,0,0,0,0,0},{0,0}};
extern EwayEmbSysInfoModule EwayEmbSysInfo;



/*
JY901,i2c
0x34,0x35,0x36,0x3D,0x3E,0x3F
Ax    Ay   Az   Wx   Wy   Wz

*/

s8 sysGetImuData(EwayIMUModule* pModule)
{
    u8 Dat[20]={0};
    u8 res;

    if((pModule->Comm.sta)>0)
    {
        pModule->Comm.sta -= 1;

        return ERR_NONE;
    }
    
    res = I2C1_Read(Aclx_ADDR,Dat,6);    //return 12 bytes result,加速度，角速度

    if(res==0)
    {/*
        pModule->Res[3]=(short)((Dat[0]+(Dat[1]<<8))/32768.0*16.0*9.8*100.0);
        pModule->Res[4]=(short)((Dat[2]+(Dat[3]<<8))/32768.0*16.0*9.8*100.0);
        pModule->Res[5]=(short)((Dat[4]+(Dat[5]<<8))/32768.0*16.0*9.8*100.0); 

        pModule->Res[6]=(short)((Dat[6]+(Dat[7]<<8))/32768.0*2000.0*100);
        pModule->Res[7]=(short)((Dat[8]+(Dat[9]<<8))/32768.0*2000.0*100.0);
        pModule->Res[8]=(short)((Dat[10]+(Dat[11]<<8))/32768.0*2000.0*100.0); */
			
			pModule->Res[3]=(short)(Dat[0]+(Dat[1]<<8))/32768.0*16.0*9.8*100.0;
        pModule->Res[4]=(short)(Dat[2]+(Dat[3]<<8))/32768.0*16.0*9.8*100.0;
        pModule->Res[5]=(short)(Dat[4]+(Dat[5]<<8))/32768.0*16.0*9.8*100.0; 

        pModule->Res[6]=(short)(Dat[6]+(Dat[7]<<8))/32768.0*2000.0*100;
        pModule->Res[7]=(short)(Dat[8]+(Dat[9]<<8))/32768.0*2000.0*100.0;
        pModule->Res[8]=(short)(Dat[10]+(Dat[11]<<8))/32768.0*2000.0*100.0;
    }

    pModule->Comm.resIMU = ((pModule->Comm.resIMU)<<1)+res;    
    
    res = I2C1_Read(Roll_ADDR,Dat,3);    //return 6 bytes result,角度

    if(res==0)
    {/*
        pModule->Res[0]=(short)((Dat[0]+(Dat[1]<<8))/32768.0*180.0*100.0);
        pModule->Res[1]=(short)((Dat[2]+(Dat[3]<<8))/32768.0*180.0*100.0);
        pModule->Res[2]=(short)((Dat[4]+(Dat[5]<<8))/32768.0*180.0*100.0);*/
			
			 pModule->Res[0]=(short)((Dat[0])+(Dat[1]<<8))/32768.0*180.0*100.0;
			pModule->Res[1]=(short)((Dat[2])+(Dat[3]<<8))/32768.0*180.0*100.0;
			pModule->Res[2]=(short)((Dat[4])+(Dat[5]<<8))/32768.0*180.0*100.0;
    }
    
    pModule->Comm.resIMU = ((pModule->Comm.resIMU)<<1)+res;

    return ERR_NONE;
}



/* --------------------------------------------------------------------------*/
/**
* @name sysGetRopeDisplacementSensorData
* @brief 
* @details Get sensor voltage value via ADC3_IN14 Channel,the result is average
* of ADC3_IN14 channel collecting 32 times sum.
*
* @param[in] rslt The result data pointer of store rope displacement sensor 
* substract hight of down limit switch.
*
* @returns ERR_NONE Successfully
* Other values Unsuccessfully
*
* @author LingPing
*/
/* --------------------------------------------------------------------------*/
s8 sysGetRopeDisplacementSensorData(EwayRDSModule* pModule)
{
    u32 res=0;
    float Len;
    u32 avg_sample = 0;
    u16 adc_sample[ADC_AVERAGE_SAMPLE_VALUE]={0};
    u8 index = 0;
    
    for(index=0;index<ADC_AVERAGE_SAMPLE_VALUE;index++)    //!<  Get N sample value
    {        
        ADC_SoftwareStartConv(ADC3);                       //!< ADC start convert
        
        while(ADC_GetFlagStatus(ADC3,ADC_FLAG_EOC)== RESET);//!< wait for the result
        
        adc_sample[index] = ADC_GetConversionValue(ADC3);
    }

    for(index=0;index<ADC_AVERAGE_SAMPLE_VALUE;index++)    //!<  compute the sum
    {
        avg_sample += adc_sample[index];
    }

    res = avg_sample>>5;                                   //!< compute the average value,32=2^5

    Len = ((float)(res/4.096))*100;                        //!< 12bit,4096=1M,result Len unit is 0.01mm

    res = (u32)Len;
/*
    if(res<LIMIT_SWITCH_LOW_POS)
    { 
        res = 0;
    }
    else if(res>=(LIMIT_SWITCH_LOW_POS+LIMIT_SWITCH_LENGTH))
    { 
        res = LIMIT_SWITCH_LENGTH;
    }
    else 
    {
        res -= LIMIT_SWITCH_LOW_POS;
    }*/

    //!< 若系统有拉线传感器，则计算实际高度为当前拉线传感器高度减去系统中所存储的下限位高度之差，否则直接给出传感器数据
    pModule->result = res;
/*
    if((EwayEmbSysInfo.Rds.En != 0)&&(EwayEmbSysInfo.Rds.init != 0))
    {
        if(res < (EwayEmbSysInfo.Rds.LowLmt))
        { 
            res = 0;
        }
        else if(res >= (EwayEmbSysInfo.Rds.HigLmt)) //!< else if(res >= (EwayEmbSysInfo.Rds.LowLmt + EwayEmbSysInfo.Rds.Lmt_SwitchLen))
        { 
            res = EwayEmbSysInfo.Rds.HigLmt-EwayEmbSysInfo.Rds.LowLmt;
        }
        else 
        {
            res -= EwayEmbSysInfo.Rds.LowLmt;
        }
    }*/

    pModule->Res = res;

    return ERR_NONE;
}


