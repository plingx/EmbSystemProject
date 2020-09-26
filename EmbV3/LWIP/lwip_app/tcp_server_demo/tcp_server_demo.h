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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ս�������� V3
//TCP Server ���Դ���	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/3/15
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//*******************************************************************************
//�޸���Ϣ
//��
////////////////////////////////////////////////////////////////////////////////// 	   
extern struct tcp_pcb *tcppcbnew;  	//����һ��TCP���������ƿ�
extern struct tcp_pcb *tcppcbconn;  	//����һ��TCP���������ƿ�
extern u8 ss;
#define TCP_SERVER_RX_BUFSIZE	2000	//����tcp server���������ݳ���
#define TCP_SERVER_PORT			8088	//����tcp server�Ķ˿�
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
//tcp����������״̬
enum tcp_server_states
{
	ES_TCPSERVER_NONE = 0,		//û������
	ES_TCPSERVER_ACCEPTED,		//�пͻ����������� 
	ES_TCPSERVER_CLOSING,		//�����ر�����
}; 
//LWIP�ص�����ʹ�õĽṹ��
 struct tcp_server_struct
{
	u8 state;               //��ǰ����״
	struct tcp_pcb *pcb;    //ָ��ǰ��pcb
	struct pbuf *p;         //ָ�����/�����pbuf
}; 
//TCPbufferʹ�õĽṹ��
typedef struct 
{
 u8 *tcp_recvbuf;//����BUFFER
 u16 tcp_recRead;//��ָ��
 u16 tcp_recWrite;//дָ��
 u16 tcp_recCount;//buffer����
	
 u8 *tcp_senbuf;//����
 u16 tcp_senRead;
 u16 tcp_senWrite;
 u16 tcp_senCount;
}tcp_buffer; 
void tcp_server_test(void);//TCP Server���Ժ���
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
