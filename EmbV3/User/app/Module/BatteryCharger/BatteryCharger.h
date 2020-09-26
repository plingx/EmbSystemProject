#ifndef _BATTERY_CHARGER_H_
#define _BATTERY_CHARGER_H_

//#include "bsp.h"

#define PSU_HEADER_FLAG        0xEEEE

#define PSU_PACKET_HEADER    2


#define PSU_Capacity_Low_Limit  2000

#define PSU_LowEnergy_MaskBit   0x01

/***************************************************************************************************************************
                                                                锂电池控电控制器的寄存器定义如下
****************************************************************************************************************************/

                                            //寄存器地址    寄存器地址（HEX）0x    名称    描述    读写    取值范围    初始值    备注
                                            //0    0    设备型号主分类    设备型号，主分类    R    0-255    2(0X02)    充电控制板主编号为2
                                            //1    1    设备型号次分类    设备型号，次分类    R    0-255    1(0X01)    充电控制板次编号为1
                                            //2    2    固件版本主版本号    固件版本（主版本号）    R    0-255    1(0X01)    固件主板本尽量与硬件版本对应
                                            //3    3    固件版本副板本号    固件版本（副板本号）    R    0-255    0(0x00)    同主板本下不同软件版本
                                            //4    4    地址号    从站ID    RW    1-253    1(0x01)    
                                            //5    5    波特率    通信波特率    RW    见下方波特率表    3（115200）    默认校验方式8N1
                                            //6    6    反馈方式    控制板信息反馈方式    RW    0-1    0(0X00)    0代表应答模式，1代表主动回应（按照读指令应答格式回应状态寄存器,帧间隔100ms）
                                            //7    7    最高输入电压L    最大输入电压报警值    RW    0-65535    2500    数值放大100倍，单位0.01V
                                            //8    8    最高输入电压H                    
                                            //9    9    最低输入电压L    最小输入电压报警值    RW    0-65535    2000    数值放大100倍，单位0.01V
                                            //10    A    最低输入电压H                    
                                            //11    B    电池放电报警电压或容量值L    电池放电报警电压值    RW    0-65535    2240    数值放大100倍，单位0.01V
                                            //12    C    电池放电报警电压或容量值H                    
                                            //13    D    电池放电截止电压或容量值L    电池截止输出的电压值    RW    0-65535    1960    数值放大100倍，单位0.01V
                                            //14    E    电池放电截止电压或容量值H                    
                                            //15    F    控制板报警温度    控制板报警温度值    RW    0-255    80    单位摄氏度
                            
                            
#define BatteryCharge_CtrlFaultStatus     0x1E      //30            1E    控制板故障状态        R    0-255    0    bit0：过热，bit1：输出过流（输出电流超过10A），bit2输入欠压，bit3：输入过压，bit4：电池欠压，bit5：输出短路，bit6：电池即将输出关闭，bit7：未定义错误；以上0均代表无故障
                                            //31    1F    控制板工作状态        R    0-255    0    bit0：负载供电电源（0是外部电源，1是电池），bit1：电池充满（1是充满），bit6-bit7：00是未充电，01是充电恒流，10是充电恒压，11是充电涓流；其他未定义位补0
                                            //32    20    外部输入电压L        R    0-65535    0    电压V，数值放大了100倍，单位0.01V
                                            //33    21    外部输入电压H                    
                                            //34    22    输出电流L    给负载供电电流    R    0-65535    0    电流A，数值放大了100倍，单位0.01A
                                            //35    23    输出电流H                    
                                            //36    24    驱动板当前温度        R    0-120    25    单位摄氏度
                                            //37    25    电池电压L        R    0-65535    0    电压V，数值放大了100倍，单位0.01V
                                            //38    26    电池电压H                    
                                            //39    27    电池充电电流L        R    0-65535    0    电流A，数值放大了100倍，单位0.01A
                                            //40    28    电池充电电流H                    
                                            //41    29    电池当前容量L        R    0-65536    0    数值放大100倍，单位0.01%(精确到1%)
                                            //42    2A    电池当前容量H                    


typedef struct
{     
     u32 unTimestamp;
     u16 unExVoltage;                      //!< reg:0x20,0x21 外部输入电压
     u16 unOutCurrent;                     //!< reg:0x22,0x23 输出电流
     u16 unBatteryVoltage;                 //!< reg:0x25,0x26 电池电压
     u16 unBatteryInCurrent;               //!< reg:0x27,0x28 电池充电电流
     u16 unBatteryCapacity;                //!< reg:0x29,0x2A 电池容量
     u16 unStatus;                         //!< reg:0x1E,0x1F 控制板工作状态     
     u8  unTemperature;                    //!< reg:0x24 驱动板当前温度
     u8  unBatterySlaveryStatus;                // status 从站状态
}ModuleBatteryCharger;


typedef struct{
    ModuleBatteryCharger sBattCharger;
    PSMPSUReRecord reRecord;
    u8 mCommStatus;                        //!< 与PSU通信状态记录，发查询给PSM之前先清除，待下一周期查询是否有回复
    u8 sStatus;                             //!< 标识PSU当前的状态 bit0:低电量指示  0:normal 1:low energy
}EwayBatteryChargerModule;




#include "BatteryCharger_Cmd.h"
#include "BatteryCharger_Status.h"


s8 SendQuery_BatteryCharger(u8 unSlaveAddr, u8 *unSendBuffer,u8 unLength);
void sysBatteryChargerMaintainWork(void);


#endif
