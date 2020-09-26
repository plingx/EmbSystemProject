#include "tcp_server.h" 
#include "includes.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h"  	
#include "bsp_uart_dma.h"
 
//TCP Server接收数据缓冲区
u8 tcp_rcvbuf0[TCP_SERVER_RX_BUFSIZE];
u8 tcp_rcvbuf1[TCP_SERVER_RX_BUFSIZE];
u8 tcp_server_recbuf[TCP_SERVER_RX_BUFSIZE]={0};

u8 tcp_sndbuf0[TCP_SERVER_RX_BUFSIZE];
u8 tcp_sndbuf1[TCP_SERVER_RX_BUFSIZE];//!< 用于待发送数据缓存

//u8 TCPConnectFlag;
struct tcp_pcb *tcppcbnew;  	//定义一个TCP服务器控制块
struct tcp_pcb *tcppcbconn;  	//定义一个TCP服务器控制块
struct tcp_server_struct *EwayTcpServer;
struct tcp_pcb* EwayTcpPcb;

extern EwayEmbSysDebugModule* pDebug;

EwayEmbTCPModule EwayEmbTCP={ES_TCPSERVER_NONE,{tcp_sndbuf0,0,0,0},0,0,0,0,0,{tcp_sndbuf1,0,0,0},{{tcp_rcvbuf0,tcp_rcvbuf1},{0},{0}}};




//TCP Server 连接
void TcpServer_Init(void)
{
	err_t err;  
	
	//TCPConnectFlag=0;
	
	tcppcbnew=tcp_new();	     //!< 创建一个新的pcb
	
	if(tcppcbnew)			     //!< 创建成功
	{ 
		err=tcp_bind(tcppcbnew,IP_ADDR_ANY,TCP_SERVER_PORT);	//!< 将本地IP与指定的端口号绑定在一起,IP_ADDR_ANY为绑定本地所有的IP地址

		if(err==ERR_OK)	        //!< 绑定完成
		{
			tcppcbconn=tcp_listen(tcppcbnew); 			//!< 设置tcppcb进入监听状态
			
			tcp_accept(tcppcbconn,tcp_server_accept); 	//!< 初始化LWIP的tcp_accept的回调函数
		}
	}
} 


//lwIP tcp_accept()的回调函数
err_t tcp_server_accept(void *arg,struct tcp_pcb *newpcb,err_t err)
{
	err_t ret_err;
	struct tcp_server_struct *es; 
//	if(0==TCPConnectFlag)
//	TCPConnectFlag=1;
//	else TCPConnectFlag=10;
#if	PRINT_ETH_DEBUG_INFO
	Bsp_printf("TCP Connect\r\n");
#endif
 	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);
	tcp_setprio(newpcb,TCP_PRIO_MIN);//设置新创建的pcb优先级
	es=(struct tcp_server_struct*)mem_malloc(sizeof(struct tcp_server_struct)); //分配内存
 	if(es!=NULL) //内存分配成功
	{
		es->state=ES_TCPSERVER_CONNECTED;  	//接收连接
		es->pcb=newpcb;
		es->p=NULL;

		EwayTcpServer = es;
		EwayTcpPcb = tcppcbnew;
		
		tcp_arg(newpcb,es);
		tcp_recv(newpcb,tcp_server_recv);	//初始化tcp_recv()的回调函数
		tcp_err(newpcb,tcp_server_error); 	//初始化tcp_err()回调函数
		tcp_poll(newpcb,tcp_server_poll,1);	//初始化tcp_poll回调函数
		tcp_sent(newpcb,tcp_server_sent);  	//初始化发送回调函数
		  
		//tcp_server_buffer.tcp_trsFlag |= TCP_COMM_FLAG_BIT_CLICONNET; 
		EwayEmbTCP.TCP_State = ES_TCPSERVER_CONNECTED;		//!<标记有客户端已经连上
		
		lwipdev.remoteip[0]=newpcb->remote_ip.addr&0xff; 		//IADDR4
		lwipdev.remoteip[1]=(newpcb->remote_ip.addr>>8)&0xff;  	//IADDR3
		lwipdev.remoteip[2]=(newpcb->remote_ip.addr>>16)&0xff; 	//IADDR2
		lwipdev.remoteip[3]=(newpcb->remote_ip.addr>>24)&0xff; 	//IADDR1 

        Bsp_printf("accept remote ip:%d.%d.%d.%d",lwipdev.remoteip[0],lwipdev.remoteip[1],lwipdev.remoteip[2],lwipdev.remoteip[3]);
			
		ret_err=ERR_OK;
	}else ret_err=ERR_MEM;
	return ret_err;
}

s8 tcpServerRecvGetIdleIdx(u8* pidx)
{
    u8 i;

    for(i=0;i<2;i++)
    {
        if(EwayEmbTCP.Recs.state[i]==0)
        {
            break;
        }
    }

    if(i<2)
    {
        *pidx = i;
        
        return ERR_NONE;
    }

    return ERR_IS_BUSY;         //!< 没有空闲的buffer，这种情况理论上不会发生
}

s8 tcpServerRecvGetIdleEmptyIdx(u8* pidx)
{
    u8 i;

    for(i=0;i<2;i++)
    {
        if((EwayEmbTCP.Recs.state[i]==0)&&(EwayEmbTCP.Recs.rCnt[i]==0))
        {
            break;
        }
    }

    if(i<2)
    {
        *pidx = i;
        
        return ERR_NONE;
    }

    return ERR_IS_BUSY;
}



//!< 检查相应buffer是否为闲&接收数据个数不为0
s8 tcpServerRecvGetReadyProcessedIdx(u8 idx)
{
    if(idx >= 2)
    {
        return ERR_INPUT_PARAMETERS;
    }

    if((EwayEmbTCP.Recs.state[idx]==0)&&(EwayEmbTCP.Recs.rCnt[idx] > 0))
    {
        return ERR_NONE;
    }
    else
    {
        return ERR_GET_RESULT_FAILED;       //!< buffer不符合处理的要求
    }
}



/*
1. 获取空闲且rCnt[]==0的buffer
    获取成功，置bufferX的状态为繁忙，并进入 2
    获取失败，X

2. 检查获取到的p->len是否大于当前空闲buffer可存储的
        否，存储p->len长度的数据到buffer,并更新rCnt[]，继续 2
        是，存储可存储的数据到buffer,并更新rCnt[]，并跳出存储循环到 3

3. 置bufferX的状态为空闲

4.
        
        


X.




*/

err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	err_t ret_err;
	u32 data_len = 0;
	//u16 i=0;
	struct pbuf *q;
  	struct tcp_server_struct *es;
	//LWIP_ASSERT("arg != NULL",arg != NULL);
	es=(struct tcp_server_struct *)arg;
    u8 idx=0;
    EmbTcpRecvModule* prcv = &EwayEmbTCP.Recs;
	if(p==NULL) //!< 从客户端接收到空数据
	{
		es->state=ES_TCPSERVER_CLOSED;//!< 需要关闭TCP 连接了
		es->p=p; 
		ret_err=ERR_OK;
	}
	else if(err!=ERR_OK)	//!< 从客户端接收到一个非空数据,但是由于某种原因err!=ERR_OK
	{
		if(p)pbuf_free(p);	//!< 释放接收pbuf
		ret_err=err;
	}
	else if(es->state==ES_TCPSERVER_CONNECTED) 	//!< 处于连接状态
	{
		if(p!=NULL)  //!< 当处于连接状态并且接收到的数据不为空时将其打印出来
		{
            if(tcpServerRecvGetIdleEmptyIdx(&idx)==ERR_NONE)                //!< 从buff0&1中寻找空闲的且空的buffer
            {
                prcv->state[idx] = 1;

                //Bsp_printf("tcp-1_recv() ptr:%d,len:%d.tot:%d.",idx,prcv->rCnt[idx],p->tot_len);

                for(q=p;q!=NULL;q=q->next)  //!< 遍历完整个pbuf链表
			    {			
				    if((q->len + prcv->rCnt[idx]) > TCP_SERVER_RX_BUFSIZE) 
				    {
				        data_len = TCP_SERVER_RX_BUFSIZE - prcv->rCnt[idx];
                        
					    memcpy((prcv->pRev[idx] + prcv->rCnt[idx]),q->payload,data_len);//!< 拷贝数据

                        //Bsp_printf("tcp-1 recv buf(%d) full.lst store:%d.",idx,data_len);

                        prcv->rCnt[idx] += data_len;  //!< 接收缓存已满，更新接收数据个数，跳出
                        
                        break;                  
				    }
				    else 
				    {
					    memcpy((prcv->pRev[idx] + prcv->rCnt[idx]),q->payload,q->len);
				
				        prcv->rCnt[idx] += q->len;
				    }
			    }

                prcv->state[idx] = 0;

                //Bsp_printf("tcp-1_recv() ptr:%d,svd:%d",idx,prcv->rCnt[idx]);
                
            }
            else
            {
                if(tcpServerRecvGetIdleIdx(&idx)==ERR_NONE)             //!< 没有空闲且空的，则寻找空闲的buffer
                {
                    prcv->state[idx] = 1;

                    //Bsp_printf("tcp-2_recv() ptr:%d,len:%d.tot:%d.",idx,prcv->rCnt[idx],q->tot_len);

                    for(q=p;q!=NULL;q=q->next)  //!< 遍历完整个pbuf链表
			        {			
				        if(q->len > (TCP_SERVER_RX_BUFSIZE - prcv->rCnt[idx])) 
				        {
				            data_len = TCP_SERVER_RX_BUFSIZE - prcv->rCnt[idx];
                        
					        memcpy((prcv->pRev[idx] + prcv->rCnt[idx]),q->payload,data_len);//!< 拷贝数据

                            //Bsp_printf("tcp-2 recv buf(%d) full.lst store:%d.",idx,data_len);

                            prcv->rCnt[idx] += data_len;  //!< 接收缓存已满，更新接收数据个数，跳出

                            break;
				        }
				        else 
				        {
					        memcpy((prcv->pRev[idx] + prcv->rCnt[idx]),q->payload,q->len);
				        }
				
				        prcv->rCnt[idx] += q->len;
			        }

                    prcv->state[idx] = 0;

                    //Bsp_printf("tcp-2_recv() ptr:%d,svd:%d.",idx,prcv->rCnt[idx]);

                }
                else
                {
                    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
                    {
	                    Bsp_printf("2.tcp_recv() get IdleIdx failed");
                    }
                }
            }
						
			lwipdev.remoteip[0]=tpcb->remote_ip.addr&0xff; 		//IADDR4
			lwipdev.remoteip[1]=(tpcb->remote_ip.addr>>8)&0xff; //IADDR3
			lwipdev.remoteip[2]=(tpcb->remote_ip.addr>>16)&0xff;//IADDR2
			lwipdev.remoteip[3]=(tpcb->remote_ip.addr>>24)&0xff;//IADDR1 
			
 			tcp_recved(tpcb,p->tot_len);//!< 用于获取接收数据,通知LWIP可以获取更多数据
 			
			pbuf_free(p);  	//!< 释放内存
			
			ret_err=ERR_OK;		
		}
	}
    else//服务器关闭了
	{
		tcp_recved(tpcb,p->tot_len);//用于获取接收数据,通知LWIP可以获取更多数据
		es->p=NULL;
		pbuf_free(p); //释放内存
		ret_err=ERR_OK;
	}

	return ret_err;
}
//lwIP tcp_err函数的回调函数
void tcp_server_error(void *arg,err_t err)
{  
	LWIP_UNUSED_ARG(err);

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
    {
	    Bsp_printf("2.tcp_server_error()arg=0x%x,er=0x%x",(int)arg,(int)err);
    }

	EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED; 		//!< 标记连接断开

	if(arg!=NULL)mem_free(arg);//释放内存
} 

err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
	//u8 i;
	err_t ret_err;
	struct tcp_server_struct *es; 
	es=(struct tcp_server_struct *)arg; 
	
	if(es!=NULL)
	{
		if(es->state==ES_TCPSERVER_CLOSED)//!< 需要关闭连接?执行关闭操作
		{
		    if(es->p!=NULL)
            {
                pbuf_free(es->p);
            }
            
			tcp_server_connection_close(tpcb,es);//!< 关闭连接

            tcp_abort(tpcb);//!< 终止连接,删除pcb控制块

            ret_err=ERR_OK;
		}
        else if(es->state==ES_TCPSERVER_CONNECTED)
        {
            if(EwayEmbTCP.runStatus == 'C')
            {
                EwayEmbTCP.runStatus = 0;
            
                EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED;         //!< 标记连接断开
                
		        if(es->p!=NULL)
                {
                    pbuf_free(es->p);
                }                
            
                tcp_abort(tpcb);//!< 终止连接,删除pcb控制块
                    
                ret_err=ERR_ABRT;             
            }
            else
            {                
                tcp_server_senddata(tpcb,es);
                
		        if(es->p!=NULL)
                {
                    
                    pbuf_free(es->p);
                }
                
                ret_err=ERR_OK;
            }
        }
        else
        {
            if(es->p!=NULL)
            {                    
                pbuf_free(es->p);
            }
        }
	}
	else
	{
        EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED;
        
		tcp_abort(tpcb);//!< 终止连接,删除pcb控制块
		
		ret_err=ERR_ABRT; 
	}
	
	return ret_err;
} 



//lwIP tcp_sent的回调函数(当从远端主机接收到ACK信号后发送数据)
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
	struct tcp_server_struct *es;
	LWIP_UNUSED_ARG(len); 
	es = (struct tcp_server_struct *) arg;
	if(es->p)
    {
        //tcp_server_senddata(tpcb,es);//发送数据
        //Bsp_printf("err!!!tcp_sent() tpcb:0x%x,new:0x%x,conn:0x%x.",tpcb,tcppcbnew,tcppcbconn);
    }

    /*if(es->p)
    {
        pbuf_free(es->p);
    }*/

	return ERR_OK;
} 
//此函数用来发送数据
void tcp_server_senddata(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
	struct pbuf *ptr;
	u16 plen;
	err_t wr_err=ERR_OK;
	 while((wr_err==ERR_OK)&&es->p&&(es->p->len<=tcp_sndbuf(tpcb)))
	 {
		ptr=es->p;
		wr_err=tcp_write(tpcb,ptr->payload,ptr->len,1);
		if(wr_err==ERR_OK)
		{ 
			plen=ptr->len;
			es->p=ptr->next;			//指向下一个pbuf
			if(es->p)pbuf_ref(es->p);	//pbuf的ref加一
			pbuf_free(ptr);
			tcp_recved(tpcb,plen); 		//更新tcp窗口大小
		}else if(wr_err==ERR_MEM)es->p=ptr;
	 }
} 
//关闭tcp连接
void tcp_server_connection_close(struct tcp_pcb *tpcb, struct tcp_server_struct *es)
{
	tcp_close(tpcb);
	tcp_arg(tpcb,NULL);
	tcp_sent(tpcb,NULL);
	tcp_recv(tpcb,NULL);
	tcp_err(tpcb,NULL);
	tcp_poll(tpcb,NULL,0);
	if(es)mem_free(es);
	
	//tcp_server_buffer.tcp_trsFlag &=(~TCP_COMM_FLAG_BIT_CLICONNET);
	EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED;			//!< 标记连接断开
}
extern void tcp_pcb_purge(struct tcp_pcb *pcb);	//在 tcp.c里面 
extern struct tcp_pcb *tcp_active_pcbs;			//在 tcp.c里面 
extern struct tcp_pcb *tcp_tw_pcbs;				//在 tcp.c里面  
//强制删除TCP Server主动断开时的time wait
/*
void tcp_server_remove_timewait(void)
{
	struct tcp_pcb *pcb,*pcb2; 
	u8 t=0;
	while(tcp_active_pcbs!=NULL&&t<200)
	{
		lwip_periodic_handle();	//继续轮询
		t++;
 		bsp_DelayMS(10);			//等待tcp_active_pcbs为空  
	}
	pcb=tcp_tw_pcbs;
	while(pcb!=NULL)//如果有等待状态的pcbs
	{
		tcp_pcb_purge(pcb); 
		tcp_tw_pcbs=pcb->next;
		pcb2=pcb;
		pcb=pcb->next;
		memp_free(MEMP_TCP_PCB,pcb2);	
	}
}
*/


/* --------------------------------------------------------------------------*/
/**
* @name SysGeneralTcpSendData
* @brief 下位机网络发送接口函数
* @details 
*			1.检查网络连接是否正常
*			2.打包发送
*
* @param[in] None
*
* @returns None
* 
* @author 
*/
/* --------------------------------------------------------------------------*/
s8 SysGeneralTcpSendData(EmbTcpTransModule* ptcpTrans)
{	
	err_t rt;
	struct pbuf * pTrans;

	
	if(ptcpTrans->SndCnt>TCP_SERVER_RX_BUFSIZE)
	{
		return ERR_INPUT_PARAMETERS;
	}

    if(EwayTcpServer->p != NULL)
    {
        return ERR_IS_BUSY;             //!< 发送繁忙
    }
	
	pTrans = pbuf_alloc(PBUF_TRANSPORT,ptcpTrans->SndCnt,PBUF_POOL);//!< 申请内存

	if(NULL == pTrans)
	{
		return ERR_PROCESS_FAILED_Alloc;
	}

	EwayTcpServer->p = pTrans;

	rt = pbuf_take(EwayTcpServer->p,ptcpTrans->pSnd,ptcpTrans->SndCnt);

	if(ERR_OK != rt)
	{
        {
            Bsp_printf("pbuf_take() failed at line %d in %s.rt:%d.",__LINE__, __FILE__,rt);
        }
		return ERR_PROCESS_FAILED_BufTake;
	}	

	//tcp_server_senddata(EwayTcpPcb,EwayTcpServer);		//!< 轮询的时候发送要发送的数据
	
	//if(EwayTcpServer->p!=NULL) pbuf_free(EwayTcpServer->p); 	//!< 释放内存	

	return ERR_NONE;
}


/*
s8 SysGeneralTcpSendData(u8* pdat,u16 len)
{	
	EwayTcpServer->p = pbuf_alloc(PBUF_TRANSPORT,tcp_server_buffer.tcp_senCount,PBUF_POOL);//!< 申请内存

	pbuf_take(EwayTcpServer->p,(char*)tcp_server_sendbuf,tcp_server_buffer.tcp_senCount);

	tcp_server_senddata(EwayTcpPcb,EwayTcpServer);		//!< 轮询的时候发送要发送的数据
	
	if(EwayTcpServer->p!=NULL)pbuf_free(EwayTcpServer->p); 	//!< 释放内存	

	return ERR_NONE;
}
*/



void CableLink_Check(void)
{    
    u16 link=0;
    
    if((EwayEmbTCP.runCnt%EMB_CHECK_PHY_LINKSTATUS_Interval)==0)
    {
        EwayEmbTCP.phylink = EwayEmbTCP.phylink << 1;
        
        link = GET_PHY_LINK_STATUS();//检测了网线连接

        if(link != 0)
        {
            EwayEmbTCP.phylink += 1;
        }        
        
        if(((EwayEmbTCP.phylink & 0x03) == 0x00)&&(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)) //网线断开了（以前是连接着的） 
        {
            EwayEmbTCP.runTimeout = 0;
        
            EwayEmbTCP.runStatus = 'C';

            if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
            {
                Bsp_printf("Phy Link Break,Status:%x.Close TCP Connection.",EwayEmbTCP.phylink);
            }          
        }  
    }
}


