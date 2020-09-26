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


//bit7:0,没有数据要发送;1,有数据要发送
//bit6:0,没有收到数据;1,收到数据了.
//bit5:0,没有客户端连接上;1,有客户端连接上了.
//bit4~0:保留

#define TCP_COMM_FLAG_BIT_TRANSMIT    (0x80)
#define TCP_COMM_FLAG_BIT_DATARECVD   (0x40)
#define TCP_COMM_FLAG_BIT_CLICONNET   (0x20)


#define EMB_CHECK_PHY_LINKSTATUS_Interval           10      //!< 检查网络的物理层连接状态


#define GET_PHY_LINK_STATUS() (ETH_ReadPHYRegister(LAN8720_PHY_ADDRESS,PHY_BSR) & 0x00000004)



//!< tcp服务器连接状态
typedef enum 
{
	ES_TCPSERVER_NONE = 0,
	ES_TCPSERVER_CONNECTED,
	ES_TCPSERVER_CLOSED,
}Tcp_Server_State_Enum; 

//!<LWIP回调函数使用的结构体
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
}EmbTcpTransModule;  //!< tcp传输控制结构体

typedef struct{	
	u8* pRev[2];		//!< recv buffer ptr
	u16 rCnt[2];          //!< 当前读写的指针
	u8 state[2];        //!< 0:idle,1:recving,2:storing
}EmbTcpRecvModule;  //!< tcp传输控制结构体



typedef struct{
	Tcp_Server_State_Enum TCP_State;
	EmbTcpTransModule TCP_Trans;
    u32 runCnt;    
    u32 runTimeout;                 //!< 当长时间未收到数据后，则自动关闭连接
    u16 runStatus;                  //!< 当长时间未收到心跳包后，则自动关闭连接。
    u8 phylink;                    //!< 物理层连接状态，用于检查网线被拔掉的检测。连接时读回为4，断开时读回为0
    u8 Cable_state;                 //!< 
    EmbTcpTransModule tcp_tmp;    
    EmbTcpRecvModule Recs;          //!< 网络接收数据处理模块
}EwayEmbTCPModule;





typedef struct 
{
 u8 *tcp_recvbuf;
 u16 tcp_recRead;	//!< 待Read数据的地址偏移量，相对于指针tcp_recvbuf来说。取值范围为[0,1999]
 u16 tcp_recWrite;	//!< 待Write数据的地址偏移量
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

