#include "tcp_server.h" 
#include "includes.h"
#include "malloc.h"
#include "stdio.h"
#include "string.h"  	
#include "bsp_uart_dma.h"
 
//TCP Server�������ݻ�����
u8 tcp_rcvbuf0[TCP_SERVER_RX_BUFSIZE];
u8 tcp_rcvbuf1[TCP_SERVER_RX_BUFSIZE];
u8 tcp_server_recbuf[TCP_SERVER_RX_BUFSIZE]={0};

u8 tcp_sndbuf0[TCP_SERVER_RX_BUFSIZE];
u8 tcp_sndbuf1[TCP_SERVER_RX_BUFSIZE];//!< ���ڴ��������ݻ���

//u8 TCPConnectFlag;
struct tcp_pcb *tcppcbnew;  	//����һ��TCP���������ƿ�
struct tcp_pcb *tcppcbconn;  	//����һ��TCP���������ƿ�
struct tcp_server_struct *EwayTcpServer;
struct tcp_pcb* EwayTcpPcb;

extern EwayEmbSysDebugModule* pDebug;

EwayEmbTCPModule EwayEmbTCP={ES_TCPSERVER_NONE,{tcp_sndbuf0,0,0,0},0,0,0,0,0,{tcp_sndbuf1,0,0,0},{{tcp_rcvbuf0,tcp_rcvbuf1},{0},{0}}};




//TCP Server ����
void TcpServer_Init(void)
{
	err_t err;  
	
	//TCPConnectFlag=0;
	
	tcppcbnew=tcp_new();	     //!< ����һ���µ�pcb
	
	if(tcppcbnew)			     //!< �����ɹ�
	{ 
		err=tcp_bind(tcppcbnew,IP_ADDR_ANY,TCP_SERVER_PORT);	//!< ������IP��ָ���Ķ˿ںŰ���һ��,IP_ADDR_ANYΪ�󶨱������е�IP��ַ

		if(err==ERR_OK)	        //!< �����
		{
			tcppcbconn=tcp_listen(tcppcbnew); 			//!< ����tcppcb�������״̬
			
			tcp_accept(tcppcbconn,tcp_server_accept); 	//!< ��ʼ��LWIP��tcp_accept�Ļص�����
		}
	}
} 


//lwIP tcp_accept()�Ļص�����
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
	tcp_setprio(newpcb,TCP_PRIO_MIN);//�����´�����pcb���ȼ�
	es=(struct tcp_server_struct*)mem_malloc(sizeof(struct tcp_server_struct)); //�����ڴ�
 	if(es!=NULL) //�ڴ����ɹ�
	{
		es->state=ES_TCPSERVER_CONNECTED;  	//��������
		es->pcb=newpcb;
		es->p=NULL;

		EwayTcpServer = es;
		EwayTcpPcb = tcppcbnew;
		
		tcp_arg(newpcb,es);
		tcp_recv(newpcb,tcp_server_recv);	//��ʼ��tcp_recv()�Ļص�����
		tcp_err(newpcb,tcp_server_error); 	//��ʼ��tcp_err()�ص�����
		tcp_poll(newpcb,tcp_server_poll,1);	//��ʼ��tcp_poll�ص�����
		tcp_sent(newpcb,tcp_server_sent);  	//��ʼ�����ͻص�����
		  
		//tcp_server_buffer.tcp_trsFlag |= TCP_COMM_FLAG_BIT_CLICONNET; 
		EwayEmbTCP.TCP_State = ES_TCPSERVER_CONNECTED;		//!<����пͻ����Ѿ�����
		
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

    return ERR_IS_BUSY;         //!< û�п��е�buffer��������������ϲ��ᷢ��
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



//!< �����Ӧbuffer�Ƿ�Ϊ��&�������ݸ�����Ϊ0
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
        return ERR_GET_RESULT_FAILED;       //!< buffer�����ϴ����Ҫ��
    }
}



/*
1. ��ȡ������rCnt[]==0��buffer
    ��ȡ�ɹ�����bufferX��״̬Ϊ��æ�������� 2
    ��ȡʧ�ܣ�X

2. ����ȡ����p->len�Ƿ���ڵ�ǰ����buffer�ɴ洢��
        �񣬴洢p->len���ȵ����ݵ�buffer,������rCnt[]������ 2
        �ǣ��洢�ɴ洢�����ݵ�buffer,������rCnt[]���������洢ѭ���� 3

3. ��bufferX��״̬Ϊ����

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
	if(p==NULL) //!< �ӿͻ��˽��յ�������
	{
		es->state=ES_TCPSERVER_CLOSED;//!< ��Ҫ�ر�TCP ������
		es->p=p; 
		ret_err=ERR_OK;
	}
	else if(err!=ERR_OK)	//!< �ӿͻ��˽��յ�һ���ǿ�����,��������ĳ��ԭ��err!=ERR_OK
	{
		if(p)pbuf_free(p);	//!< �ͷŽ���pbuf
		ret_err=err;
	}
	else if(es->state==ES_TCPSERVER_CONNECTED) 	//!< ��������״̬
	{
		if(p!=NULL)  //!< ����������״̬���ҽ��յ������ݲ�Ϊ��ʱ�����ӡ����
		{
            if(tcpServerRecvGetIdleEmptyIdx(&idx)==ERR_NONE)                //!< ��buff0&1��Ѱ�ҿ��е��ҿյ�buffer
            {
                prcv->state[idx] = 1;

                //Bsp_printf("tcp-1_recv() ptr:%d,len:%d.tot:%d.",idx,prcv->rCnt[idx],p->tot_len);

                for(q=p;q!=NULL;q=q->next)  //!< ����������pbuf����
			    {			
				    if((q->len + prcv->rCnt[idx]) > TCP_SERVER_RX_BUFSIZE) 
				    {
				        data_len = TCP_SERVER_RX_BUFSIZE - prcv->rCnt[idx];
                        
					    memcpy((prcv->pRev[idx] + prcv->rCnt[idx]),q->payload,data_len);//!< ��������

                        //Bsp_printf("tcp-1 recv buf(%d) full.lst store:%d.",idx,data_len);

                        prcv->rCnt[idx] += data_len;  //!< ���ջ������������½������ݸ���������
                        
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
                if(tcpServerRecvGetIdleIdx(&idx)==ERR_NONE)             //!< û�п����ҿյģ���Ѱ�ҿ��е�buffer
                {
                    prcv->state[idx] = 1;

                    //Bsp_printf("tcp-2_recv() ptr:%d,len:%d.tot:%d.",idx,prcv->rCnt[idx],q->tot_len);

                    for(q=p;q!=NULL;q=q->next)  //!< ����������pbuf����
			        {			
				        if(q->len > (TCP_SERVER_RX_BUFSIZE - prcv->rCnt[idx])) 
				        {
				            data_len = TCP_SERVER_RX_BUFSIZE - prcv->rCnt[idx];
                        
					        memcpy((prcv->pRev[idx] + prcv->rCnt[idx]),q->payload,data_len);//!< ��������

                            //Bsp_printf("tcp-2 recv buf(%d) full.lst store:%d.",idx,data_len);

                            prcv->rCnt[idx] += data_len;  //!< ���ջ������������½������ݸ���������

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
			
 			tcp_recved(tpcb,p->tot_len);//!< ���ڻ�ȡ��������,֪ͨLWIP���Ի�ȡ��������
 			
			pbuf_free(p);  	//!< �ͷ��ڴ�
			
			ret_err=ERR_OK;		
		}
	}
    else//�������ر���
	{
		tcp_recved(tpcb,p->tot_len);//���ڻ�ȡ��������,֪ͨLWIP���Ի�ȡ��������
		es->p=NULL;
		pbuf_free(p); //�ͷ��ڴ�
		ret_err=ERR_OK;
	}

	return ret_err;
}
//lwIP tcp_err�����Ļص�����
void tcp_server_error(void *arg,err_t err)
{  
	LWIP_UNUSED_ARG(err);

    if((pDebug->sysDebugCtrl&SysDebugCtrl_EMB_SYS_MASK)&&(pDebug->sysDebugCtrl&SysDebugCtrl_EMB_Sys_Err_MASK))
    {
	    Bsp_printf("2.tcp_server_error()arg=0x%x,er=0x%x",(int)arg,(int)err);
    }

	EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED; 		//!< ������ӶϿ�

	if(arg!=NULL)mem_free(arg);//�ͷ��ڴ�
} 

err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb)
{
	//u8 i;
	err_t ret_err;
	struct tcp_server_struct *es; 
	es=(struct tcp_server_struct *)arg; 
	
	if(es!=NULL)
	{
		if(es->state==ES_TCPSERVER_CLOSED)//!< ��Ҫ�ر�����?ִ�йرղ���
		{
		    if(es->p!=NULL)
            {
                pbuf_free(es->p);
            }
            
			tcp_server_connection_close(tpcb,es);//!< �ر�����

            tcp_abort(tpcb);//!< ��ֹ����,ɾ��pcb���ƿ�

            ret_err=ERR_OK;
		}
        else if(es->state==ES_TCPSERVER_CONNECTED)
        {
            if(EwayEmbTCP.runStatus == 'C')
            {
                EwayEmbTCP.runStatus = 0;
            
                EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED;         //!< ������ӶϿ�
                
		        if(es->p!=NULL)
                {
                    pbuf_free(es->p);
                }                
            
                tcp_abort(tpcb);//!< ��ֹ����,ɾ��pcb���ƿ�
                    
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
        
		tcp_abort(tpcb);//!< ��ֹ����,ɾ��pcb���ƿ�
		
		ret_err=ERR_ABRT; 
	}
	
	return ret_err;
} 



//lwIP tcp_sent�Ļص�����(����Զ���������յ�ACK�źź�������)
err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
	struct tcp_server_struct *es;
	LWIP_UNUSED_ARG(len); 
	es = (struct tcp_server_struct *) arg;
	if(es->p)
    {
        //tcp_server_senddata(tpcb,es);//��������
        //Bsp_printf("err!!!tcp_sent() tpcb:0x%x,new:0x%x,conn:0x%x.",tpcb,tcppcbnew,tcppcbconn);
    }

    /*if(es->p)
    {
        pbuf_free(es->p);
    }*/

	return ERR_OK;
} 
//�˺���������������
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
			es->p=ptr->next;			//ָ����һ��pbuf
			if(es->p)pbuf_ref(es->p);	//pbuf��ref��һ
			pbuf_free(ptr);
			tcp_recved(tpcb,plen); 		//����tcp���ڴ�С
		}else if(wr_err==ERR_MEM)es->p=ptr;
	 }
} 
//�ر�tcp����
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
	EwayEmbTCP.TCP_State = ES_TCPSERVER_CLOSED;			//!< ������ӶϿ�
}
extern void tcp_pcb_purge(struct tcp_pcb *pcb);	//�� tcp.c���� 
extern struct tcp_pcb *tcp_active_pcbs;			//�� tcp.c���� 
extern struct tcp_pcb *tcp_tw_pcbs;				//�� tcp.c����  
//ǿ��ɾ��TCP Server�����Ͽ�ʱ��time wait
/*
void tcp_server_remove_timewait(void)
{
	struct tcp_pcb *pcb,*pcb2; 
	u8 t=0;
	while(tcp_active_pcbs!=NULL&&t<200)
	{
		lwip_periodic_handle();	//������ѯ
		t++;
 		bsp_DelayMS(10);			//�ȴ�tcp_active_pcbsΪ��  
	}
	pcb=tcp_tw_pcbs;
	while(pcb!=NULL)//����еȴ�״̬��pcbs
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
* @brief ��λ�����緢�ͽӿں���
* @details 
*			1.������������Ƿ�����
*			2.�������
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
        return ERR_IS_BUSY;             //!< ���ͷ�æ
    }
	
	pTrans = pbuf_alloc(PBUF_TRANSPORT,ptcpTrans->SndCnt,PBUF_POOL);//!< �����ڴ�

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

	//tcp_server_senddata(EwayTcpPcb,EwayTcpServer);		//!< ��ѯ��ʱ����Ҫ���͵�����
	
	//if(EwayTcpServer->p!=NULL) pbuf_free(EwayTcpServer->p); 	//!< �ͷ��ڴ�	

	return ERR_NONE;
}


/*
s8 SysGeneralTcpSendData(u8* pdat,u16 len)
{	
	EwayTcpServer->p = pbuf_alloc(PBUF_TRANSPORT,tcp_server_buffer.tcp_senCount,PBUF_POOL);//!< �����ڴ�

	pbuf_take(EwayTcpServer->p,(char*)tcp_server_sendbuf,tcp_server_buffer.tcp_senCount);

	tcp_server_senddata(EwayTcpPcb,EwayTcpServer);		//!< ��ѯ��ʱ����Ҫ���͵�����
	
	if(EwayTcpServer->p!=NULL)pbuf_free(EwayTcpServer->p); 	//!< �ͷ��ڴ�	

	return ERR_NONE;
}
*/



void CableLink_Check(void)
{    
    u16 link=0;
    
    if((EwayEmbTCP.runCnt%EMB_CHECK_PHY_LINKSTATUS_Interval)==0)
    {
        EwayEmbTCP.phylink = EwayEmbTCP.phylink << 1;
        
        link = GET_PHY_LINK_STATUS();//�������������

        if(link != 0)
        {
            EwayEmbTCP.phylink += 1;
        }        
        
        if(((EwayEmbTCP.phylink & 0x03) == 0x00)&&(EwayEmbTCP.TCP_State == ES_TCPSERVER_CONNECTED)) //���߶Ͽ��ˣ���ǰ�������ŵģ� 
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


