#ifndef __TCP_SERVER_DEMO_H
#define __TCP_SERVER_DEMO_H
#include "bsp.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip_comm.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 战舰开发板 V3
//TCP Server 测试代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/3/15
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//*******************************************************************************
//修改信息
//无
////////////////////////////////////////////////////////////////////////////////// 	   
extern struct tcp_pcb *tcppcbnew;  	//定义一个TCP服务器控制块
extern struct tcp_pcb *tcppcbconn;  	//定义一个TCP服务器控制块
extern u8 ss;
#define TCP_SERVER_RX_BUFSIZE	2000	//定义tcp server最大接收数据长度
#define TCP_SERVER_PORT			8088	//定义tcp server的端口
extern u8 tcp_server_recvbuf[TCP_SERVER_RX_BUFSIZE];
extern u8 tcp_server_senbuf[TCP_SERVER_RX_BUFSIZE];
extern u16 tcp_server_senRead;
extern u16 tcp_server_senWrite;
extern u16 tcp_server_senCount;
extern u16 tcp_server_recRead;
extern u16 tcp_server_recWrite;
extern u16 tcp_server_recCount;
extern u8 *tcp_server_sendbuf;
extern u8 flaggg;
extern u8 tcp_server_flag;	
//tcp服务器连接状态
enum tcp_server_states
{
	ES_TCPSERVER_NONE = 0,		//没有连接
	ES_TCPSERVER_ACCEPTED,		//有客户端连接上了 
	ES_TCPSERVER_CLOSING,		//即将关闭连接
}; 
//LWIP回调函数使用的结构体
 struct tcp_server_struct
{
	u8 state;               //当前连接状
	struct tcp_pcb *pcb;    //指向当前的pcb
	struct pbuf *p;         //指向接收/或传输的pbuf
}; 
//TCPbuffer使用的结构体
typedef struct 
{
 u8 *tcp_recvbuf;//接收BUFFER
 u16 tcp_recRead;//读指针
 u16 tcp_recWrite;//写指针
 u16 tcp_recCount;//buffer长度
	
 u8 *tcp_senbuf;//发送
 u16 tcp_senRead;
 u16 tcp_senWrite;
 u16 tcp_senCount;
}tcp_buffer; 
void tcp_server_test(void);//TCP Server测试函数
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err);
err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void tcp_server_error(void *arg,err_t err);
err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void tcp_server_senddata(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
//void tcp_server_remove_timewait(void);
void tcp_server_buffer_ini(void);
extern tcp_buffer tcp_server_buffer;
extern u8 TCPConnectFlag;
#endif 
