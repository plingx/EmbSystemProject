#ifndef __TCP_SERVER_H
#define __TCP_SERVER_H
#include "bsp.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip_comm.h"
#include "tcp_impl.h"

 
extern struct tcp_pcb *tcppcbnew;  	
extern struct tcp_pcb *tcppcbconn;  
extern u8 ss;
#define TCP_SERVER_RX_BUFSIZE	1000	
#define TCP_SERVER_PORT			8088	
extern u8 tcp_tmp_rcvbuf0[TCP_SERVER_RX_BUFSIZE];
//extern u8 tcp_server_senbuf0[TCP_SERVER_RX_BUFSIZE];
//extern u16 tcp_server_senRead;
//extern u16 tcp_server_senWrite;
//extern u16 tcp_server_senCount;
//extern u16 tcp_server_recRead;
//extern u16 tcp_server_recWrite;
//extern u16 tcp_server_recCount;
//extern u8 *tcp_server_sendbuf;
extern u8 flaggg;


//bit7:0,û������Ҫ����;1,������Ҫ����
//bit6:0,û���յ�����;1,�յ�������.
//bit5:0,û�пͻ���������;1,�пͻ�����������.
//bit4~0:����

#define TCP_COMM_FLAG_BIT_TRANSMIT    (0x80)
#define TCP_COMM_FLAG_BIT_DATARECVD   (0x40)
#define TCP_COMM_FLAG_BIT_CLICONNET   (0x20)


#define EMB_CHECK_PHY_LINKSTATUS_Interval           10      //!< �����������������״̬


#define GET_PHY_LINK_STATUS() (ETH_ReadPHYRegister(LAN8720_PHY_ADDRESS,PHY_BSR) & 0x00000004)



//!< tcp����������״̬
typedef enum 
{
	ES_TCPSERVER_NONE = 0,
	ES_TCPSERVER_CONNECTED,
	ES_TCPSERVER_CLOSED,
}Tcp_Server_State_Enum; 

//!<LWIP�ص�����ʹ�õĽṹ��
 struct tcp_server_struct
{
	u8 state;
	struct tcp_pcb *pcb;
	struct pbuf *p;
}; 


typedef struct{	
	u8 *pSnd;		//send
	u8* pRev;		//recv
	u16 SndCnt;
	u8 bsy;
}EmbTcpTransModule;  //!< tcp������ƽṹ��

typedef struct{	
	u8* pRev[2];		//!< recv buffer ptr
	u16 rCnt[2];          //!< ��ǰ��д��ָ��
	u8 state[2];        //!< 0:idle,1:recving,2:storing
}EmbTcpRecvModule;  //!< tcp������ƽṹ��



typedef struct{
	Tcp_Server_State_Enum TCP_State;
	EmbTcpTransModule TCP_Trans;
    u32 runCnt;    
    u32 runTimeout;                 //!< ����ʱ��δ�յ����ݺ����Զ��ر�����
    u16 runStatus;                  //!< ����ʱ��δ�յ������������Զ��ر����ӡ�
    u8 phylink;                    //!< ���������״̬�����ڼ�����߱��ε��ļ�⡣����ʱ����Ϊ4���Ͽ�ʱ����Ϊ0
    u8 Cable_state;                 //!< 
    EmbTcpTransModule tcp_tmp;    
    EmbTcpRecvModule Recs;          //!< ����������ݴ���ģ��
}EwayEmbTCPModule;





typedef struct 
{
 u8 *tcp_recvbuf;
 u16 tcp_recRead;	//!< ��Read���ݵĵ�ַƫ�����������ָ��tcp_recvbuf��˵��ȡֵ��ΧΪ[0,1999]
 u16 tcp_recWrite;	//!< ��Write���ݵĵ�ַƫ����
 u16 tcp_recCount;
	
 u8 *tcp_senbuf;
 u16 tcp_senRead;
 u16 tcp_senWrite;
 u16 tcp_senCount;

}tcp_buffer; 



void TcpServer_Init(void);
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err);
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void tcp_server_error(void *arg,err_t err);
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void tcp_server_senddata(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
//void tcp_server_remove_timewait(void);
s8 SysGeneralTcpSendData(EmbTcpTransModule* ptcpTrans);
void CableLink_Check(void);
s8 tcpServerRecvGetIdleIdx(u8* pidx);
s8 tcpServerRecvGetReadyProcessedIdx(u8 idx);




//extern u8 TCPConnectFlag;
#endif 

