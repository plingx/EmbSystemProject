#ifndef _SYS_UTILS_H
#define _SYS_UTILS_H


#define LOG_BUFFER_SIZE   256



//�������صĴ���״̬�붨��
#define ERR_NONE                    0
#define ERR_INPUT_PARAMETERS        -1
#define ERR_IS_BUSY                 -2
#define ERR_DATA_OVERFLOW           -3
#define ERR_GET_RESULT_FAILED       -4
#define ERR_PROCESS_FAILED          -5
#define ERR_UART_TX_BUF_FULL        -6
#define ERR_POINTER_NULL            -7
#define ERR_ADD_HEADER              -8
#define ERR_UART_NULL               -9
#define ERR_DATA_NOT_RDY            -10
#define ERR_RX_DATA_HEADER          -11
#define ERR_RX_DATA_ADDR            -12
#define ERR_RX_DATA_CRC             -13
#define ERR_RX_DATA_LEN             -14
#define ERR_RESULT_NOT_EQUAL        -15
#define ERR_PROCESS_FAILED_Alloc    -16
#define ERR_PROCESS_FAILED_BufTake  -17
#define ERR_CHECK_SUM               -18
#define ERR_CHECK_DATA_INVALID      -19
#define ERR_BUFFER_EMPTY            -20
#define ERR_RegsCompare_0           -21     //!< ������üĴ����ȽϽ�����Ĵ����ɸ���
#define ERR_RegsCompare_1           -22     //!< ������üĴ����ȽϽ�����Ĵ������ɸ��ģ�ֻ�ܸ�����Ҫ��ӡ��ʾ��Ϣ







//u8 Sd_Log_Msg( char *fmt,...);
s8 Emb_Log_Msg( char *fmt,...);
u32 SysGetCurrentRunTime(void);

#endif

