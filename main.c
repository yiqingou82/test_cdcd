#include "main.h"

#define BUF ((struct uip_eth_hdr *)uip_buf)

/* ���崮��ģʽ�������ݽṹ */
typedef  struct  UartMode
{  uint8 datab;         // �ֳ��ȣ�5/6/7/8
   uint8 stopb;         // ֹͣλ��1/2
   uint8 parity;    	// ��żУ��λ��0Ϊ��У�飬1����У�飬2Ϊż��У��
}  UARTMODE;

uint8  rcv_buf[180];       		// UART0���ݽ��ջ�����,������һ��д��72�ַ���ʱ��Ϊ���ֵ
uint8  rcv_temp[180];       		// UART0���ݽ��ջ�����,������һ��д��72�ַ���ʱ��Ϊ���ֵ
uint8  send_buf[245];//UART0���ݷ��ͻ�������һ������ȡ120���Ĵ���
uint8  rcv_new;     	// ���յ������ݱ�־

uint16 net_time05=0;
uint16 net_time1=0;
uint16 net_time10=0;
uint16 net_time30=0;
uint8 net_timeover05=0;
uint8 net_timeover1=0;
uint8 net_timeover10=0;	
uint8 net_timeover30=0;



/****************************************************************************
* ���ƣ�IRQ_UART0()
* ���ܣ�����UART0�����жϡ�
* ��ڲ�������
* ���ڲ�������
****************************************************************************/
void   __irq IRQ_UART0(void)
{ 
   uint8  i;
   //�ض�ʱ��
   T0TCR = 0x00;  
   if( 0x04==(U0IIR&0x0F) )
   {
     for(i=0; i<7; i++)
     {
       rcv_buf[rec_count] = U0RBR; 	// ��ȡFIFO�����ݣ�������жϱ�־ 
       rec_count++;          
     }
	 if(rec_count>165)
	 	rec_count=0;
   }
   else if( 0x0c==(U0IIR&0x0F) )//������һ���ַ�����ô�ж��ַ����أ���U0LSR��Ӧλ
   {
     while((U0LSR & 0x01) ==1)
     {
     rcv_buf[rec_count] = U0RBR;
     rec_count++;
     }

	 if(rcv_buf[0]==ip_address)
	 	{
		     for(i=0; i<rec_count; i++)
		     {
		       rcv_temp[i] = rcv_buf[i]; 	// ��ȡFIFO�����ݣ�������жϱ�־ 
		     }
		     rcv_count=rec_count;
  		     rec_count=0;//Ϊ��һ֡������׼��     
		     rcv_new = 1; // ���ý��յ��µ����ݱ�־
		}
	else
		{
 		    rec_count=0;//Ϊ��һ֡������׼��     
		}
   }
   
   //����ʱ��
   T0TCR = 0x01;
   VICVectAddr = 0x00; // �жϴ������
   
}               


/****************************************************************************
* ���ƣ�SendByte()
* ���ܣ��򴮿�UART0�����ֽ����ݡ�
* ��ڲ�����data                Ҫ���͵�����
* ���ڲ�������
****************************************************************************/
void  SendByte(uint8 data)
{  U0THR = data;                      	// ��������
}


/****************************************************************************
* ���ƣ�ISendBuf()
* ���ܣ��������������ݷ��ͻ�����(ʹ��FIFO)�����ȴ�������ϡ�
* ��ڲ�����length
* ���ڲ�������
****************************************************************************/
void  ISendBuf(uint8 length)//16���ֽ�fifo(�շ�)
{    
   uint8 i,j;
   j=0;
   while(length/16)
   {
      for(i=0;i<16;i++)
      {
       SendByte(send_buf[j]);
       j++;
       length--;
      }
      while( (U0LSR&0x20)==0 );
   }
   
   for(;length>0;length--)
   {
    SendByte(send_buf[j]);
    j++;
   }
   
   while( (U0LSR&0x20)==0 );         	// �ȴ����ݷ���
}               
                 
        
/****************************************************************************
* ���ƣ�UART0_Ini()
* ���ܣ���ʼ������0�������乤��ģʽ�������ʡ�
* ��ڲ�����baud                ������
*          set          ģʽ����(UARTMODE���ݽṹ)
* ���ڲ���������ֵΪ1ʱ��ʾ�����ɹ���Ϊ0�����������
****************************************************************************/
uint8  UART0_Ini(uint32 baud, UARTMODE set)
{  uint32  bak;
   
   /* �������� */
   if( (0==baud)||(baud>115200) ) return(0);
   if( (set.datab<5)||(set.datab>8) ) return(0);
   if( (0==set.stopb)||(set.stopb>2) ) return(0);
   if( set.parity>4 ) return(0);

   /* ���ô��ڲ����� */
   U0LCR = 0x80;                        // DLABλ��1
   bak = (Fpclk>>4)/baud;
   U0DLM = bak>>8;
   U0DLL = bak&0xff;
   
   /* ���ô���ģʽ */
   bak = set.datab-5;                   // �����ֳ���
   if(2==set.stopb) bak |= 0x04;        // �ж��Ƿ�Ϊ2λֹͣλ  
   
   if(0!=set.parity) {set.parity = set.parity-1; bak |= 0x08;}
   bak |= set.parity<<4;              	// ������żУ��
      
   U0LCR = bak;
   
   return(1);
}

//��ʼ�����ڣ�ʹ�ܴ���
void init_uart0(void)
{
   UARTMODE  uart0_set={0};
   
   rcv_new = 0;
   
   uart0_set.datab = 8;                 // 8λ����λ
   uart0_set.stopb = 1;                 // 1λֹͣλ
   uart0_set.parity = 0;                // ����żУ��
   UART0_Ini(set_baud, uart0_set);        // ��ʼ������ģʽ
   
   U0FCR = 0x81;                        // ʹ��FIFO�������ô�����Ϊ8�ֽ�
   U0IER = 0x01;                        // ����RBR�жϣ��������ж�
   
  
   VICIntSelect = 0x00000000;           // ��������ͨ��ΪIRQ�ж�
   VICVectCntl1 = 0x26;                 // UART0�ж�ͨ�����䵽IRQ slot 0�������ȼ����
   VICVectAddr1 = (int)IRQ_UART0;       // ����UART0������ַ
   VICIntEnable |= 0x00000040;           // ʹ��UART0�ж�

}


//ÿ��һ��ʱ�䣬����һ�δ����źţ�����CPLD������ʾ

/****************************************************************************
* ���ƣ�IRQ_Time0()
* ���ܣ���ʱ��0�жϷ������
* ��ڲ�������
* ���ڲ�������
****************************************************************************/
void __irq  IRQ_Time0(void)
{  
   newtimer=1;
   if(net_time05++>=25)
   {					 //0.5�������־
	 net_time05=0;
	 net_timeover05=1;	
	}
   if(net_time1++>=50)
   {					 //�������־
	 net_time1=0;
	 net_timeover1=1;	
	}  
   if(net_time10++>=500)
   {				 //10�������־
	 net_time10=0;
	 net_timeover10=1;	
	}
   if(net_time30++>=1500)
   {
        net_time30=0;
	 net_timeover30=1;
   	}
   T0IR = 0x01;	    			            	// ����жϱ�־
   VICVectAddr = 0x00; // ֪ͨVIC�жϴ������
}


/****************************************************************************
* ���ƣ�Time0Init()
* ���ܣ���ʼ����ʱ��0����ʱʱ��Ϊ20mS����ʹ���жϡ� 
* ��ڲ�������
* ���ڲ�������
****************************************************************************/
void  Time0Init(void)
{   /* Fcclk = Fosc*4 = 11.0592MHz*4 = 44.2368MHz
	   Fpclk = Fcclk/4 = 44.2368MHz/4 = 11.0592MHz
	*/
	T0PR = 99;			    					// ���ö�ʱ��0��ƵΪ100��Ƶ����110592Hz
	T0MCR = 0x03;		   						// ƥ��ͨ��0ƥ���жϲ���λT0TC
	T0MR0 = 2212;	    						// �Ƚ�ֵ(20MS��ʱֵ)
	T0TCR = 0x03;		   						// ��������λT0TC
	T0TCR = 0x01; 
	
	/* ���ö�ʱ��0�ж�IRQ */
	VICIntSelect = 0x00;						// �����ж�ͨ������ΪIRQ�ж�
	VICVectCntl0 = 0x24;						// ��ʱ��0�ж�ͨ������������ȼ�(����������0)
	VICVectAddr0 = (uint32)IRQ_Time0; 			// �����жϷ�������ַ���� 
	VICIntEnable = 0x00000010;					// ʹ�ܶ�ʱ��0�ж�
}


//CRCУ��
unsigned int CRC16(unsigned char *puchMsg, unsigned char usDataLen)
{
	unsigned int uchCRCHi = 0xFF ;  /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF ; /* low byte of CRC initialized */
	unsigned char uIndex ;          /* will index into CRC lookup table */
	while (usDataLen--)             /* pass through message buffer */
		{
			uIndex = uchCRCHi ^ *puchMsg++ ;    /* calculate the CRC */
			uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex] ;
			uchCRCLo = auchCRCLo[uIndex] ;
		}
	return (uchCRCHi << 8 | uchCRCLo) ;
}

void
SetSytemrestart(uint8 value)
{
     restart=value;
}

void
SetIni_Init(uint8 value)
{
     initini=value;
}

void
Settcplinecheck(void)
{   
     tcplinetime=2*(data4000[58]*256+data4000[59]);
}

void
SetModbusInterval(uint16 interval)
{
	modbus_state.interval = interval;
}


void
SetModbussignnum(uint8 num)
{
	modbus_state.signnum = num;
}

void
SetModbus0150(uint8 value)
{
	modbus_state.enable0150 = value;
}

void
SetModbusmode(uint8 value)
{
	modbus_state.k_mode = value;
	data0250[4]=value;
	
}
void	
SetModbuslinesign(uint8 value)
{
	modbus_state.linesign = value;
}

void
SetPowerOn(uint8 value)
{
       modbus_state.poweron=value;
}

void checkstart(uint8 value)
{
     checksign=value;
     checktime=value;
}

void 
SetPowerOff(uint8 value)
{
	uint8 i;
	SetPowerOn(value);
      for(i=0;i<modbus_state.signnum;i++)
      	{    
        	FArray[i].Function(value);
      	}

}

void InitVal()
{
  int i,j;
  rec_count=0;//�����ж� ���ռ���
  restart=0;
  initini=0;
 checkstart(data4000[61]);
    for(i=0;i<8;i++)
   {   
   	for(j=0;j<10;j++){
	   ledlane[i].shan[j]=0;
	   ledlane[i].shans[j]=0;
   		}
   	}
}

void InitIO()
{
  IO0DIR|=0x1b800000;
  IO1DIR|=0x00080000;
  IO2DIR|=0xbfc00000;
  
  IO0DIR&=0X9fe3ffff;
  IO2DIR&=0xffa0ffff;
  IO3DIR&=0xca00003f;

  IO0CLR=0x1b000000;
  IO1CLR=0x00080000;
  IO2CLR=0xbf800000;  
  
  IO0DIR|= SCL | SDA;	
  IO0CLR = SCL;
  IO0SET = SDA;

}

void PowerOnInit()
{
  InitVal();
  Time0Init();
  InitParam();
  RTCIni();

}

void ModbusDeal()//���յ��������Ҫ���Ĵ���
{
  uint8 aa;

        for(aa=0;aa<rcv_count;aa++)//������ȫ�����Ƶ����ͻ���
        {
           send_buf[aa]=rcv_temp[aa];
        }
        
        crc=CRC16(send_buf, rcv_count);
        if(crc==0)//crcУ����ȷ
        {
           switch(send_buf[1])//������
           {
              case 0x03:
              MultiRd();
              break;
              case 0x06:
              SingleWr();
              break;
              case 0x10:
              MultiWr();
              break;
              case 0x17:
              MultiRdWr();
              break;
              default:
              ModbusSendError(0x01);//01�Ƿ�������
              break;
            }//switch
           
           if(modbus_state.interval>0)//
           modbus_state.count=modbus_state.interval*data4000[31];//��СͨѶ������������¸�ֵ
        }//if(crc==0)
}

void mb_receive(uint8 * buf,uint16 size)
{   	uint16 count;
	
	rcv_count2=size&0x00ff;

		 	for(count=0;count<size;count++)
				{
					send_buf[count]=buf[count]; 					
				}

			 crc=CRC16(send_buf, size);
			 if(crc==0)//crcУ����ȷ
				{

					if(send_buf[0]==ip_address)
						{
							TCP_ModbusDeal();
						}
				}
		
}    

void TCP_ModbusDeal()
{
	switch(send_buf[1])//������
		{
              case 0x03:
              	TCP_MultiRd();
				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//��СͨѶ������������¸�ֵ
              break;
              case 0x06:
              	TCP_SingleWr();
 				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//��СͨѶ������������¸�ֵ
             break;
              case 0x10:
 	            TCP_MultiWr();
 				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//��СͨѶ������������¸�ֵ
             break;
              case 0x17:
	            TCP_MultiRdWr();
				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//��СͨѶ������������¸�ֵ
              break;
              default:
	              TCP_ModbusSendError(0x01);//01�Ƿ�������
              break;
		}//switch
           
}

//������03������ȡ
void TCP_MultiRd()
 {
   uint16 addr;//��ʼ��ַ
   uint16 num;//��ȡ����
   uint16 i;
   uint8 datbuf;
   uint16 addr0;
   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//���ĩ��ַ
   if(addr0==0)
   {
		TCP_ModbusSendError(0x02);
        return;
   }
   
   addr0=AddrChek(addr);//���ʼ��ַ
   if(addr0==0)
   {
		TCP_ModbusSendError(0x02);
        return;
   }

	num=num*2;
   send_buf[2]=num;//��ȡ�ֽ���
   

    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	for(i=0;i<num;i++)
	{
	datbuf=*IPdata;
	send_buf[i+3]=datbuf;
	IPdata++;
	}

	TCP_ModbusSend(3+num);
 }
 
 //������06����������
 void TCP_SingleWr()
 {
   uint16 addr;
   uint16 addr0;
 
   addr=send_buf[2]*256+send_buf[3];
 
   addr0=AddrChek(addr);
   if(addr0==0)
   {
     TCP_ModbusSendError(0x02);//02�Ƿ���ַ
     return;
   }
   
    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	*IPdata=send_buf[4];
	IPdata++;
	*IPdata=send_buf[5];
    
	TCP_ModbusSend(rcv_count2-2);
   
   //����Ӧ��״̬������ֵ   
  if(addr0==0xa600)
   {
      switch(addr)
     {
      case 0xa600:
      case 0xa601:
      case 0xa602:
      case 0xa603:
		  set_clock();
		  break;
		  
      case 0xa604:
	  	SetModbusmode(data0100[9]);
		break;
		
      case 0xa607:
 	     SetModbusInterval(data0100[14]*256+data0100[15]);
      break;

      case 0xa608:
	  	SetModbussignnum(data0100[17]); 
		break;
	  
      default:
      break;
     }
     memcpy(&data4000[32],data0100,20);
     ISendStr(CSI1025, 0x20,&data4000[32],16); 
     Delay(50000);
     ISendStr(CSI1025, 0x30,&data4000[48],16); 
			
   	}
   else if(addr0==0xa640)//����TCP/IP�Ѿ������ж�����״������λ���Բ��á�
    {    
       SetModbuslinesign(data0140[1]);
   	    }
   else if(addr0==0x4000)
   {   
	 if(data4000[73])                    //mac����
	    {     data4000[73]=0;
		    data4000[75]=0;
		    data4000[77]=0;
		    data4000[79]=0;
		    memcpy(mymac, &data4000[64], 6);
		    ISendStr(CSI1025, 0x40,&data4000[64], 6); 
		    Delay(50000);	
		}
	   else if(data4000[75])           //����IP��ַ���
	    {   data4000[75]=0;
	        ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		 Delay(50000);	
	   	}
	   else if(data4000[77])           //�ָ���������
	   {   
	       SetIni_Init(data4000[77]);
		 data4000[77]=0;
	   	}
	    else if(data4000[79])         //Զ��������
	    {	      
	      SetSytemrestart(data4000[79]);
		data4000[79]=0;
	   	}
	   else if((shan_sign!=data4000[70])||(shan_time!=data4000[71]))
	   	{
	   	   shan_sign=data4000[70];
		   shan_time=data4000[71];
		   ISendStr(CSI1025, 0x46,&data4000[64], 2); 
	   	}
	   else
	   {  
	      set_my_data();
	    	}  
   	}

 }
 
 //������16���������
 void TCP_MultiWr()
 {
    uint16 addr,num;
    uint16 addr0;
    uint8 temp;
    uint8 temp1;
    uint16 i;
    addr=send_buf[2]*256+send_buf[3];
    num=send_buf[4]*256+send_buf[5];
  
    addr0=AddrChek(addr+num-1);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
   
    addr0=AddrChek(addr);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
        
    check_store_address(addr0);

	num=num*2;
	IPdata=IPdata+(addr-addr0)*2;
    for(i=0;i<num;i++)
    {
      temp=send_buf[7+i];
      *IPdata=temp;
      IPdata++;
    }

	TCP_ModbusSend(6);

  
    //����Ӧ��״̬������ֵ
     if(addr0==0xa600)
	{  
	     set_clock();
	     SetModbusmode(data0100[9]);
	     SetModbusInterval(data0100[14]*256+data0100[15]);
	     SetModbussignnum(data0100[17]); 
	     memcpy(&data4000[32],data0100,20);
            ISendStr(CSI1025, 0x20,&data4000[32], 16); 
            Delay(50000);
            ISendStr(CSI1025, 0x30,&data4000[48], 16); 
     	  }
	else if(addr0==0xa640)
	 {
           SetModbuslinesign(data0140[1]);
		   
		}
	else if(addr0==0xa650)
	  { 
	      data0250[4]=data0100[9];
		data0250[5]=0;

             parsemodbus(data0150,num);
			   
		temp=modbus_state.signnum;
	       temp=6*temp+6;
		data0250[temp+1]=data0150[1];
		temp1=data0150[1];
		temp1=9*temp1;
		temp=temp+2;
		for(i=0;i<90;i++)
		{     data0250[temp+i*2]=0;
			data0250[temp+i*2+1]=0;
			}
		for(i=0;i<temp1;i++)
		 {
			 data0250[temp+i*2]=data0150[i*2+2];
			 data0250[temp+i*2+1]=data0150[i*2+3];
				  
		     }
		SetModbus0150(1);
		SetPowerOn(1);	
	    
		  }	
	else if(addr0==0x4000)
	{	
	  if(data4000[73])					  //mac����
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//����IP��ַ���
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		        Delay(50000);  
		 }
		else if(data4000[77])			//�ָ���������
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Զ��������
		 {		   
		   SetSytemrestart(data4000[79]);
		 data4000[79]=0;
		 }
		else if((shan_sign!=data4000[70])||(shan_time!=data4000[71]))
		 {
			shan_sign=data4000[70];
			shan_time=data4000[71];
			ISendStr(CSI1025, 0x46,&data4000[64], 2); 
		 }
		else
		{  
		   set_my_data();
			 }	
	 }

}
 
 //������23���������/��ȡ
void TCP_MultiRdWr()
 {
    uint16 addr,num;
    uint16 addr1;
    uint16 addr0;
    uint8 temp;
    uint16 i;
    addr1=send_buf[6]*256+send_buf[7];
    num=send_buf[8]*256+send_buf[9];
  
    addr0=AddrChek(addr1+num-1);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
   
    addr0=AddrChek(addr1);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
    
    check_store_address(addr0);

	num=num*2;
	IPdata=IPdata+(addr1-addr0)*2;
    for(i=0;i<num;i++)
    {
      temp=send_buf[11+i];
      *IPdata=temp;
      IPdata++;
    }

   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//���ĩ��ַ
   if(addr0==0)
   {
      TCP_ModbusSendError(0x02);//02�Ƿ���ַ
      return;
   }
   
   addr0=AddrChek(addr);//���ʼ��ַ
   if(addr0==0)
   {
      TCP_ModbusSendError(0x02);//02�Ƿ���ַ
      return;
   }

	num=num*2;
   send_buf[2]=num;//��ȡ�ֽ���
   //��ȡ��Ҫ��ȡ���ݵ��׵�ַ�������ݣ���Ӧ��

    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	for(i=0;i<num;i++)
	{
	temp=*IPdata;
	send_buf[i+3]=temp;
	IPdata++;
	}

	TCP_ModbusSend(3+num);

   addr0=AddrChek(addr1);
    if(addr0==0xa600)
	{  
	     set_clock();
	     SetModbusmode(data0100[9]);
	     SetModbusInterval(data0100[14]*256+data0100[15]);
	     SetModbussignnum(data0100[17]); 
            memcpy(&data4000[32],data0100,20);
            ISendStr(CSI1025, 0x20,&data4000[32], 16); 
            Delay(50000);
            ISendStr(CSI1025, 0x30,&data4000[48], 16); 
     	  }
	else if(addr0==0xa640)
	 {
           SetModbuslinesign(data0140[1]);
		   
		}
	else if(addr0==0xa650)
	  {
	      data0250[4]=data0100[9];
		data0250[5]=0;

             parsemodbus(data0150,num);
			   
		temp=modbus_state.signnum;
	       temp=6*temp+6;
		data0250[temp+1]=data0150[1]; 
		temp=temp+2;
		for(i=0;i<90;i++)
		 {
			 data0250[temp+i*2]=data0150[i*2+2];
			 data0250[temp+i*2+1]=data0150[i*2+3];
				  
		     }
		SetModbus0150(1);
		SetPowerOn(1);	
	    
		  }	
	else if(addr0==0x4000)
	{	
	  if(data4000[73])					  //mac����
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//����IP��ַ���
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//�ָ���������
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Զ��������
		 {		   
		   SetSytemrestart(data4000[79]);
		 data4000[79]=0;
		 }
		else if((shan_sign!=data4000[70])||(shan_time!=data4000[71]))
		 {
			shan_sign=data4000[70];
			shan_time=data4000[71];
			ISendStr(CSI1025, 0x46,&data4000[64], 2); 
		 }
		else
		{  
		   set_my_data();
			 }	
	 }


 }

void TCP_ModbusSend(uint8 length)
{
    crc=CRC16(send_buf, length);
    send_buf[length]=crc>>8;
    send_buf[length+1]=crc&0x00ff;
	uip_send(send_buf,length+2);   	
}
   
void TCP_ModbusSendError(uint8 value)
{
	
	send_buf[1] |=0x80;
	send_buf[2]=value;
	uip_send(send_buf,3); 	
}


void use_udp_send(struct uip_udp_conn *conn)
{
	uip_udp_conn = udpmessage.conn;
	
	uip_process(UIP_UDP_TIMER);
	
	if(uip_len > 0)
	{
		uip_arp_out();
		send_packet();
	}
}

uint32 readinput()
{ 
//      IO3DIR&=0Xfffc003f;
	return (IO3PIN & ANIN);
}

void output(uint32 valve)
{    uint8 i;
      uint32 temp1;
      temp1=valve;
      for(i=0;i<modbus_state.signnum;i++)
      	{    if((0== ANIN_data_A[i])||(0==ANIN_data_B[i]))
      	               break;
	      else if((0==(temp1&ANIN_data_A[i]))&&(0==(temp1&ANIN_data_B[i])))
		 	 FArray[i].Function(0);	      
      	       else if(0==(temp1&ANIN_data_A[i]))   
      	             {  
      	                 FArray[i].Function(3);			    
      	       	}
	       else if(0==(temp1&ANIN_data_B[i]))
		   	{
		   	    FArray[i].Function(1);			   
	       	}

      	 }
     
}

void check(void)
{  
    uint8   i;
    uint32 rout_out_data,back_in_data;
    uint32 temp1,temp2,temp3,temp4;
	rout_out_data=ROUT_OUT;
	back_in_data=BACK_IN;
    memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
    for(i=0;i<modbus_state.signnum;i++)
    	{  
    	    temp1=ROUT_data[2*i];
	    temp2=ROUT_data[2*i+1];
	    temp3=BKIN_data[2*i];
           temp4=BKIN_data[2*i+1];
 /*01*/if((0==temp1)||(0==temp3))     //6�Ź���(Ӳ����֧��)
    	    	{    
		       LEDstate.state.devstate=custom_fault;
		       LEDstate.state.hwfault=no_support;    	    	  
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;    	    	       
    	    	}
 /*02*/ else if((rout_out_data&temp1)&&(rout_out_data&temp2))        //���Ϻ�2(���ƿ�Ӳ������)
		{			
		       LEDstate.state.devstate=hardware_fault;
		       LEDstate.state.hwfault=2;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;			   
	    	}
 /*03*/  else if((0==(back_in_data&temp3))&&(0==(back_in_data&temp4)))       //���Ϻ�5(ģ��֡����)
		 {
		       LEDstate.state.devstate=module_frame_fault;
//		       LEDstate.state.hwfault=0;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;			
	      	}
 /*04*/  else if(((rout_out_data&temp1)&&(back_in_data&temp3))||((rout_out_data&temp2)&&(back_in_data&temp4)))      //���Ϻ�4(ģ��ͨѶ����)
		 {
		       LEDstate.state.devstate=module_line_fault;
//			LEDstate.state.hwfault=0;
			LEDstate.state.showstate=0;		       
			LEDstate.state.actionstate=0;
		 }
 /*05*/	 else if(((0==(rout_out_data&temp1))&&(0==(back_in_data&temp3)))||((0==(rout_out_data&temp2))&&(0==(back_in_data&temp4))))    //���Ϻ�3(LED����)
		 {
		       LEDstate.state.devstate=LED_fault;			
//		       LEDstate.state.hwfault=0;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;
		  }
 /*06*/	 else if((rout_out_data&temp1)&&(0==(back_in_data&temp3)))          //���������ʾ
		 {
		         LEDstate.state.actionstate=RED;
			  LEDstate.state.showstate=1;
		  }
/*07*/	  else if((rout_out_data&temp2)&&(0==(back_in_data&temp4)))      //�̵�������ʾ
		  {
			    LEDstate.state.actionstate=GREEN;
			    LEDstate.state.showstate=1;
		    }	  
 /*other*/else if((((0==(rout_out_data&temp1))&&(back_in_data&temp3))||((0==(rout_out_data&temp2))&&(back_in_data&temp4))))
		 {
		      refresram(i+1);
		  }
		 	
		 fresram(i+1);
		 
    	
    	  }
    
   
}

void fresram(uint8 valve)
{
        uint8 i;
        uint8 *ptr;
	  ptr=LEDstate.bytes;
	 for(i=0;i<6;i++)
	 {      
	       data0250[valve*6+i]=*ptr;
		   ptr++;
	 	}

}

void refresram(uint8 valve)
{
      uint8 i;
        uint8 *ptr;
	  ptr=LEDstate.bytes;
	 for(i=0;i<6;i++)
	 {      
	       *ptr=data0250[valve*6+i];
		   ptr++;
	 	}
}

void test(void)
{
     if(1==data4000[21])
      	{   
      	     if(0==(IO0PIN & SW8))
      	     	{  
      	     	    output(0xfffeaabf);	
	          modbus_state.count=0;      	          
	          SetPowerOn(0);
      	     	}
          }
     else if(2==data4000[21])
     	{     
     	      if(0==(IO0PIN & SW8))
     	      	{
     	      	   output(0xfffd557f);	
	          modbus_state.count=0;      	          
	          SetPowerOn(0);
     	      	}
     	}
}

void TimerDeal(void)//ÿ��ʱ�ӽ��ģ�Ҫ���Ĵ���
{     
      uint8 temp1,temp2,temp3;
      uint8 i;
      uint32  ANIN_data;
	  
      ANIN_data=readinput();
      if(ANIN_data!=ANIN)            //�ֶ�
      	{ 
      	    output(ANIN_data);			
	   if(data4000[20])
	    leddelay=3600*((data4000[22]*256)+data4000[23])+60*((data4000[24]*256)+data4000[25]);	  	   	
	   else
	    modbus_state.count=0;
      	    SetModbusmode(1);
	    SetPowerOn(1);
	    checkstart(data4000[63]); 
      	  }	  
       //���ڸ�������СͨѶ���ʱ����δ���ܵ���Ч֡��ϵͳӦ�Զ�������
      if(modbus_state.interval>0 && modbus_state.count>0)                //����״��
       {
         modbus_state.count--;
         if(modbus_state.count==0)
         { 
            if(0==data4000[55])
            {
               SetPowerOff(0);      //�ܹ���               
            	}
	     else if(1==data4000[55])
	      { 
	          output(0xfffeaabf);	
	          modbus_state.count=0;      	          
	          SetPowerOn(0);
	     	    }
		else if(2==data4000[55])
		{
		   output(0xfffd557f);	
	          modbus_state.count=0;      	          
	          SetPowerOn(0);
		  }
	     checkstart(data4000[63]);
          }
	  }
        if(modbus_state.poweron&&modbus_state.enable0150)
        {    
              for(i=0;i<modbus_state.signnum;i++)
                {  
                   plan[i]=ledlane[i].maxplan;
              	}
		  SetModbus0150(0);
		  SetModbusmode(0);			  	
        	}
	
	  if((modbus_state.poweron)&&(0==modbus_state.k_mode))      //�Զ�
	   {     
	          for(i=0;i<modbus_state.signnum;i++)
	          {    
	               temp1=ledlane[i].maxplan-plan[i];
	          	  if(plan[i]==0)
	          	  {    
	          	       temp2=ledlane[i].maxplan-1;              //���һ������
			       temp3=ledlane[i].action[temp2];     //��i������temp2�������еĶ���
	   	              if(ledlane[i].sign[temp2])                //��i������temp2�������е��Ӷ����Ƿ񼤻�
	   	              {    
	   	                  FArray[i].Function(temp3);	      //��i������ִ�ж���(0-7������,�޶������ǿն���)
	   	                }
	          	  	}
			    else
			     {  
			         temp3=ledlane[i].action[temp1];     //��i������temp1�������еĶ���
#ifdef  SHANSHUO			         
			         if(((3==temp3)&&(1==data0250[i*6+11]))&&(0==ledlane[i].shan[temp1])){
//				  if((1==temp3)&&(3==data0250[i*6+11])){
					 	ledlane[i].shan[temp1]=1;
						if(shan_sign){
						 ledlane[i].timelag[temp1]=shan_time+1;
							}
//						ledlane[i].shans[temp1]=1;
			         	} 
#endif
			         if( ledlane[i].timelag[temp1]==0)
			         {
#ifdef  SHANSHUO
			            ledlane[i].shan[temp1]=0;
				     ledlane[i].shans[temp1]=0;
#endif
			             if(ledlane[i].sign[temp1])                //��i������temp1�������е��Ӷ����Ƿ񼤻�
	   	                     {    
	   	                        FArray[i].Function(temp3);	      //��i������ִ�ж���(0-7������,�޶������ǿն���)
	   	                        }
					 plan[i]--;
			         	}
				   else
				    {
				          ledlane[i].timelag[temp1]--;					   
				   	}
			    	 }
	          	}	
		 checkstart(1);
	      
	   	  }
	      else if(data4000[20])                              //�ֶ�״��
	 	 {
	 	      leddelay--;
		      if(0==leddelay)
		      	{  SetPowerOff(0);
		      	    SetModbusmode(0);
			    checkstart(data4000[63]);
		      	}
	      	   }			  	
				  
       if(checksign)  
	 checktime--;     //��������
	timetcp--;          //�������Ӽ��
	
      if( 0==(ILR&0x01) );	    // �ȴ�RTC�����жϱ�־
		{
		    ILR = 0x03;				    // ����жϱ�־
			updata_clock();
		}	

}

void LED01(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));   
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	 IO0CLR=ROUT1;
			 IO2CLR=ROUT2;
			 break;
		 case 0x01:
		 	  IO0CLR=ROUT1;
		 	  IO2SET=ROUT2;
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:		 	 
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
 /*        		if(LEDstate.state.showstate){
			 IO0CLR=ROUT1;
			 IO2CLR=ROUT2;
			 LEDstate.state.showstate=0;
         			}
			else{
			 IO0CLR=ROUT1;
			 IO2SET=ROUT2;				
			 LEDstate.state.showstate=1;
				}*/
		 	 break;
			 
		 case 0x03:
		 	  IO2CLR=ROUT2;
		 	  IO0SET=ROUT1;
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(1);
            
}

void LED02(uint8 action)
{     
       memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	  IO2CLR=ROUT3;
		 	  IO2CLR=ROUT4;		 	
			 break;
		 case 0x01:
		 	  IO2CLR=ROUT3;
		 	  IO2SET=ROUT4;		 	 
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
		 	  IO2CLR=ROUT4;
		 	  IO2SET=ROUT3;			 	
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(2);
            
}

void LED03(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	  IO1CLR=ROUT5;
		 	  IO0CLR=ROUT6;			 	
			 break;
		 case 0x01:
		 	  IO1CLR=ROUT5;
		 	  IO0SET=ROUT6;			 	 
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
		 	  IO0CLR=ROUT6;
		 	  IO1SET=ROUT5;			 	
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(3);
            
}

void LED04(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	  IO2CLR=ROUT7;
		 	  IO2CLR=ROUT8;			 	
			 break;
		 case 0x01:
		 	  IO2CLR=ROUT7;
		 	  IO2SET=ROUT8;			 	 
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
		 	  IO2CLR=ROUT8;
		 	  IO2SET=ROUT7;			 	
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(4);
            
}

void LED05(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	  IO2CLR=ROUT9;
		 	  IO0CLR=ROUT10;			 	
			 break;
		 case 0x01:
		 	  IO2CLR=ROUT9;
		 	  IO0SET=ROUT10;	
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
		 	
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
		 	  IO0CLR=ROUT10;
		 	  IO2SET=ROUT9;			 	
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(5);
            
}

void LED06(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	  IO0CLR=ROUT11;
		 	  IO0CLR=ROUT12;			 	
			 break;
		 case 0x01:
		 	  IO0CLR=ROUT11;
		 	  IO0SET=ROUT12;			 	
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
		 	  IO0CLR=ROUT12;
		 	  IO0SET=ROUT11;			 	
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(6);
            
}

void LED07(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	
			 break;
		 case 0x01:
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(7);
            
}

void LED08(uint8 action)
{     
	memset(LEDstate.bytes,0,sizeof(LEDstate.bytes));  
	LEDstate.state.actionstate=action;
	switch(action)
         {   
		 case 0:
		 	
			 break;
		 case 0x01:
		        LEDstate.state.showstate=1;
			  break;
			  
		 case 0x02:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	  	
		 	 break;
			 
		 case 0x03:
                     LEDstate.state.showstate=1;
		 	 break;
			 
		 case 0x04:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x05:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
              case 0x06:
			 LEDstate.state.devstate=custom_fault;
			 LEDstate.state.hwfault=no_support;	
			 break;
			 
		 case 0x07:
		 	
			 break;
		default:
			break;			 
			  
            }
	fresram(8);
            
}

void updata_clock(void)
{
    
    data0100[0]=0x20;
    data0100[1]=((YEAR%100)/10)*16 + YEAR%10;

    data0100[2]=(MONTH/10)*16+MONTH%10;
    data0100[3]=(DOM/10)*16+DOM%10;

    data0100[4]=(HOUR/10)*16+HOUR%10;
    data0100[5]=(MIN/10)*16+MIN%10;

    data0100[6]=(SEC/10)*16+SEC%10;

	if(HOUR!=time.hour)
	{
		time.hour=HOUR;
		if((0==data0100[4])&&(0==data0100[5]))
		{                   
                   memcpy(&data4000[32],data0100,8);
                   ISendStr(CSI1025, 0x20,&data4000[32],8); 			 
	        }
	       data4000[77]=0;          //ʧЧ
	 }
}

void set_clock(void)
{
	SEC =(data0100[6]/16)*10 + data0100[6]%16;
	MIN	=(data0100[5]/16)*10 + data0100[5]%16;
	HOUR=(data0100[4]/16)*10 + data0100[4]%16;
	DOM =(data0100[3]/16)*10 + data0100[3]%16;
       MONTH =(data0100[2]/16)*10 + data0100[2]%16;			   
       YEAR = 2000 +(data0100[1]/16)*10 + data0100[1]%16;	

	if(HOUR!=time.hour)
		{
		 	time.hour=HOUR;
		}
	if((mymac[0]==0)&&(mymac[1]==1)&&(mymac[2]==1)&&(mymac[3]==1)&&(mymac[4]==1)&&(mymac[5]==1))
	{    
             if(data0100[1]&&data0100[2]&&data0100[3])
		 {   
		     
		      memcpy(&data4000[64], &data0100[1], 6);
		      data4000[64]=0;
		      memcpy(mymac, &data4000[64], 6);
		      ISendStr(CSI1025, 0x40,&data4000[64], 16); 
             	   }
		}
}

void ini_sram(void)
{   uint16 i;
	
    for(i=0;i<236;i++)
     {
         data0250[i]=0;
	}
	data0250[1]=25;
	SetModbusmode(data0100[9]);
	SetModbussignnum(data0100[17]); 
	
}

void ini_eeprom_data(void)
{
	uint8 i;
	for(i=0;i<80;i++)
		{
	        data4000[i]=ini4000[i];
		}
}

void ini_init(void)
{
			ini_eeprom_data();			
			set_baud=data4000[0]*256+data4000[1];

			LocalIP[0]=data4000[4];
			LocalIP[1]=data4000[5];
			LocalIP[2]=data4000[6];
			LocalIP[3]=data4000[7];

			MaskIP[0]=data4000[8];
			MaskIP[1]=data4000[9];
			MaskIP[2]=data4000[10];
			MaskIP[3]=data4000[11];

			GateWay[0]=data4000[12];
			GateWay[1]=data4000[13];
			GateWay[2]=data4000[14];
			GateWay[3]=data4000[15];

			ip_address=data4000[17];
			ServerPort[0]=data4000[18];
			ServerPort[1]=data4000[19];

			memcpy(mymac, &data4000[64], 6);
			memcpy(data0100,&data4000[32],20);
			shan_sign=data4000[70];
			shan_time=data4000[71];

			ISendStr(CSI1025, 0x00, data4000, 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x10,&data4000[16], 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x20,&data4000[32], 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x30,&data4000[48], 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x40,&data4000[64], 16); 
}

void ini_prog(void)
{
    	if((IO2PIN&KEY)==0)
		{
			ini_eeprom_data();
			set_baud=data4000[0]*256+data4000[1];

			LocalIP[0]=data4000[4];
			LocalIP[1]=data4000[5];
			LocalIP[2]=data4000[6];
			LocalIP[3]=data4000[7];

			MaskIP[0]=data4000[8];
			MaskIP[1]=data4000[9];
			MaskIP[2]=data4000[10];
			MaskIP[3]=data4000[11];

			GateWay[0]=data4000[12];
			GateWay[1]=data4000[13];
			GateWay[2]=data4000[14];
			GateWay[3]=data4000[15];

			ip_address=data4000[17];
			ServerPort[0]=data4000[18];
			ServerPort[1]=data4000[19];

			memcpy(mymac, &data4000[64], 6);
			memcpy(data0100,&data4000[32],20);
			shan_sign=data4000[70];
			shan_time=data4000[71];

			ISendStr(CSI1025, 0x00, data4000, 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x10,&data4000[16], 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x20,&data4000[32], 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x30,&data4000[48], 16); 
			Delay(50000);
			ISendStr(CSI1025, 0x40,&data4000[64], 16); 
		}
	else
		{
			if(IRcvStr(CSI1025, 0x00, data4000, 80))
				{
					set_baud=data4000[0]*256+data4000[1];

					LocalIP[0]=data4000[4];
					LocalIP[1]=data4000[5];
					LocalIP[2]=data4000[6];
					LocalIP[3]=data4000[7];
		
					MaskIP[0]=data4000[8];
					MaskIP[1]=data4000[9];
					MaskIP[2]=data4000[10];
					MaskIP[3]=data4000[11];
		
					GateWay[0]=data4000[12];
					GateWay[1]=data4000[13];
					GateWay[2]=data4000[14];
					GateWay[3]=data4000[15];

		                    ip_address=data4000[17];
					ServerPort[0]=data4000[18];
					ServerPort[1]=data4000[19];

					memcpy(mymac, &data4000[64], 6);
	                           memcpy(data0100,&data4000[32],20);
					shan_sign=data4000[70];
					shan_time=data4000[71];


				} 
			else
				{
					ini_eeprom_data();
					set_baud=data4000[0]*256+data4000[1];

					LocalIP[0]=data4000[4];
					LocalIP[1]=data4000[5];
					LocalIP[2]=data4000[6];
					LocalIP[3]=data4000[7];
		
					MaskIP[0]=data4000[8];
					MaskIP[1]=data4000[9];
					MaskIP[2]=data4000[10];
					MaskIP[3]=data4000[11];
		
					GateWay[0]=data4000[12];
					GateWay[1]=data4000[13];
					GateWay[2]=data4000[14];
					GateWay[3]=data4000[15];

		                    ip_address=data4000[17];
					ServerPort[0]=data4000[18];
					ServerPort[1]=data4000[19];

					memcpy(mymac, &data4000[64], 6);
					memcpy(data0100,&data4000[32],20);
					shan_sign=data4000[70];
					shan_time=data4000[71];


					ISendStr(CSI1025, 0x00, data4000, 16); 
					Delay(50000);
					ISendStr(CSI1025, 0x10,&data4000[16], 16); 
					Delay(50000);
					ISendStr(CSI1025, 0x20,&data4000[32], 16); 
					Delay(50000);
					ISendStr(CSI1025, 0x30,&data4000[48], 16); 
					Delay(50000);
			             ISendStr(CSI1025, 0x40,&data4000[64], 16); 
				}
		}
}

void set_my_data(void)
{  
   ip_address=data4000[17];
  if(set_baud!=data4000[0]*256+data4000[1])
    {	      
		set_baud=data4000[0]*256+data4000[1];
  	}
		
 if((LocalIP[0]!=data4000[4])||(LocalIP[1]!=data4000[5])||(LocalIP[2]!=data4000[6])||(LocalIP[3]!=data4000[7])
  	||(MaskIP[0]!=data4000[8])||(MaskIP[1]!=data4000[9])||(MaskIP[2]!=data4000[10])||(MaskIP[3]!=data4000[11])
  	||(GateWay[0]!=data4000[12])||(GateWay[1]!=data4000[13])||(GateWay[2]!=data4000[14])||(GateWay[3]!=data4000[15])
  	||(ServerPort[0]!=data4000[18])||(ServerPort[1]!=data4000[19])) 
  {   	
		LocalIP[0]=data4000[4];
		LocalIP[1]=data4000[5];
		LocalIP[2]=data4000[6];
		LocalIP[3]=data4000[7];

		MaskIP[0]=data4000[8];
		MaskIP[1]=data4000[9];
		MaskIP[2]=data4000[10];
		MaskIP[3]=data4000[11];

		GateWay[0]=data4000[12];
		GateWay[1]=data4000[13];
		GateWay[2]=data4000[14];
		GateWay[3]=data4000[15];

		ServerPort[0]=data4000[18];
		ServerPort[1]=data4000[19];
		}
   
		ISendStr(CSI1025, 0x10,&data4000[16], 16); 
		Delay(50000);
		ISendStr(CSI1025, 0x20,&data4000[32], 16); 
		Delay(50000);
		ISendStr(CSI1025, 0x30,&data4000[48], 16); 
		Delay(50000);		

}


void  WdtFeed(void)
{
	VICIntEnClr = 0x00000050;	
  	WDFEED = 0xAA;
   	WDFEED = 0x55;
   	VICIntEnable = 0x00000050;					
}

#ifdef  SHANSHUO
void shan(void)
{   
     int i;
     int temp1;
     for(i=0;i<modbus_state.signnum;i++)
	{    
	    temp1=ledlane[i].maxplan-plan[i];
	   if(ledlane[i].shan[temp1]){
	    switch(i)
	    {
	    	 case 0:
			if(ledlane[i].shans[temp1])
				 {			   
					   IO0CLR=ROUT1;
					   IO2CLR=ROUT2;
					   ledlane[i].shans[temp1]=0;
				 }
				 else
				 {	  
					   IO0CLR=ROUT1;
//					   IO0SET=ROUT1;
					   IO2SET=ROUT2;
					   ledlane[i].shans[temp1]=1;
				 }
			 break;
		 case 0x01:
			 if(ledlane[i].shans[temp1])
				  { 			
				          IO2CLR=ROUT3;
				          IO2CLR=ROUT4;
					   ledlane[i].shans[temp1]=0;
				  }
				  else
				  {    
					  IO2CLR=ROUT3;
					  IO2SET=ROUT4;
					  ledlane[i].shans[temp1]=1;
				  }
			  break;
			  
		 case 0x02:		 	 
			 if(ledlane[i].shans[temp1])
				  { 			
				             IO1CLR=ROUT5;
				             IO0CLR=ROUT6;	   
						ledlane[i].shans[temp1]=0;
				  }
				  else
				  {    
					     IO1CLR=ROUT5;
					     IO0SET=ROUT6; 	
						ledlane[i].shans[temp1]=1;
				  }
		 	 break;
			 
		 case 0x03:
			 if(ledlane[i].shans[temp1])
				  { 			
				              IO2CLR=ROUT7;
				              IO2CLR=ROUT8; 
						ledlane[i].shans[temp1]=0;
				  }
				  else
				  {    
					      IO2CLR=ROUT7;
					      IO2SET=ROUT8; 
						ledlane[i].shans[temp1]=1;
				  }
		 	 break;
			 
		 case 0x04:
			 if(ledlane[i].shans[temp1])
				  { 			
				             IO2CLR=ROUT9;
				             IO0CLR=ROUT10;    
						ledlane[i].shans[temp1]=0;
				  }
				  else
				  {    
					      IO2CLR=ROUT9;
					      IO0SET=ROUT10;	
						ledlane[i].shans[temp1]=1;
				  }
			 break;
			 
		 case 0x05:
			 if(ledlane[i].shans[temp1])
				  { 			
				           IO0CLR=ROUT11;
				           IO0CLR=ROUT12;
						ledlane[i].shans[temp1]=0;
				  }
				  else
				  {    
					  IO0CLR=ROUT11;
					  IO0SET=ROUT12;
						ledlane[i].shans[temp1]=1;
				  }
			 break;
			 
              case 0x06:
				  if(ledlane[i].shans[temp1])
					   {			 
							 ledlane[i].shans[temp1]=0;
					   }
					   else
					   {	
							 ledlane[i].shans[temp1]=1;
					   }
			 break;
		default:
			break;	
	    }
     	}
     }

}

#endif

/////////////////////////////////////////////////////////////////////
//������
int main(void)
{
  uint8 i;
  
  BusInit( );

  InitIO();

  ini_prog();
  
  PowerOnInit(); 

    WDTC = 0x00FFFFFF;						// ����WDTC��ι����װֵ
   WDMOD = 0x03;	
   WdtFeed();

  init_uart0();		
 
  Initialization();
  
  UDPChannelInit();
  
  test();

  while(1)
  {
     if(restart) 
     {   
        if(initini)  ini_init();			
	 while(1);	  	      //Զ������
     	}
     uip_len=recv_packet();            //������������
     if(uip_len > 0)			    //�յ�����
      {
			/* ����IP���ݰ�(ֻ��У��ͨ����IP���Żᱻ����) */
		if(BUF->type == htons(UIP_ETHTYPE_IP))   //��IP����
		{
			uip_arp_ipin();		   //ȥ����̫��ͷ�ṹ������ARP��		
			uip_input();		   //IP������
				/*
					������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ��� uip_len > 0
					��Ҫ���͵�������uip_buf, ������uip_len  (����2��ȫ�ֱ���)
				*/
			if (uip_len > 0)		//�д����Ӧ����
			{
				uip_arp_out();		//����̫��ͷ�ṹ������������ʱ����Ҫ����ARP����
				send_packet();		//�������ݵ���̫�����豸��������
				timetcp=(data4000[28]*256+data4000[29])*data4000[30];
			}
		 }
			/* ����arp���� */
		 else if (BUF->type == htons(UIP_ETHTYPE_ARP))	//��ARP�����
		 {
			uip_arp_arpin();		//������ARP��Ӧ������ARP����������󣬹����Ӧ���ݰ�
				/*
					������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ��� uip_len > 0
					��Ҫ���͵�������uip_buf, ������uip_len  (����2��ȫ�ֱ���)
				*/
			if (uip_len > 0)		//��ARP����Ҫ���ͻ�Ӧ
			{
				send_packet();		//��ARP��Ӧ����̫���				
			}
		    }
	       }	  
      else if(net_timeover05==1)			/* 0.5�붨ʱ����ʱ */
	 {	net_time05=0;
		net_timeover05=0;			/* ��λ0.5�붨ʱ�� */	
#ifdef  SHANSHUO
		shan();  
#endif
                        
		/* ��������ÿ��TCP����, UIP_CONNSȱʡ��10�� */
		for(i = 0; i < UIP_CONNS; i++)
		{
			uip_periodic(i);		/* ����TCPͨ���¼� */
			/*
				������ĺ���ִ�к������Ҫ�������ݣ���ȫ�ֱ��� uip_len > 0
				��Ҫ���͵�������uip_buf, ������uip_len  (����2��ȫ�ֱ���)
			*/
			if(uip_len > 0)
			{
				uip_arp_out();		//����̫��ͷ�ṹ������������ʱ����Ҫ����ARP����
				send_packet();		//�������ݵ���̫�����豸��������
			}
		}

	#if UIP_UDP
		/* ��������ÿ��UDP����, UIP_UDP_CONNSȱʡ��10�� */
		for(i = 0; i < UIP_UDP_CONNS; i++)
		{     
			uip_udp_periodic(i);	/*����UDPͨ���¼� */
			/* �������ĺ������õ�������Ӧ�ñ����ͳ�ȥ��ȫ�ֱ���uip_len�趨ֵ> 0 */
			if(uip_len > 0)
			{
				uip_arp_out();		//����̫��ͷ�ṹ������������ʱ����Ҫ����ARP����
				send_packet();		//�������ݵ���̫�����豸��������
			}
		}
	#endif /* UIP_UDP */

		/* ÿ��10�����1��ARP��ʱ������ ���ڶ���ARP����,ARP��10�����һ�Σ��ɵ���Ŀ�ᱻ����*/
            if(net_timeover1==1)
		 {    net_time1=0;
		       net_timeover1=0;
#ifdef  SHANSHUO			  
//			shan();         //�̵���˸��ʾ5��
#endif
			 WdtFeed();
		       TimerDeal(); 
			 if(data4000[60]&&checksign&&(0==checktime))
                     {    checkstart(data4000[61]);
				check();
                     	}

	             if (net_timeover10==1)
		        {     net_time10=0;
			        net_timeover10=0;		/* ��λ10�붨ʱ�� */				
			        uip_arp_timer();
				if(net_timeover30==1)
				{
				    net_time30=0;
				    net_timeover30=0;
				    use_udp_send(udpmessage.conn);
					}
			      }
		    }
	 	}
     else if(1==rcv_new)//���յ�һ֡����,���ݳ���Ҫ��
     { 
        rcv_new = 0;
        ModbusDeal();
       }
     if(0==timetcp)              //����û�����ӵ�����
	{     
	    init_8019();
	    timetcp=(data4000[26]*256+data4000[27])*data4000[57];
		
	   }
 /*   else if(clocksave)
    	{   clocksave=0;
    	     data4000[77]=0;          //ʧЧ
	     memcpy(&data4000[32],data0100,8);
            ISendStr(CSI1025, 0x20,&data4000[32],8); 
    	}	
*/

    //       }//else if(newtimer)
    
      }//while(1)

}

void BusInit( void )
{
	PINSEL2 |=0x05804114;
	PINSEL2 &=0xf58c5717;
	
	PINSEL1 |= 0x00000000;
       PINSEL1 &= 0xc0000000;
	   
	PINSEL0 = 0x55550005; 
	
}


///////////////////MODBUS���ݽ���//////////////////////////////  
void
ModbusSend(uint8 length)//length ������CRC��
{
    crc=CRC16(send_buf, length);
    send_buf[length]=crc>>8;
    send_buf[length+1]=crc&0x00ff;
    ISendBuf(length+2);
}

void  
ModbusSendError(uint8 error)
{
	send_buf[1] |= 0x80;
	send_buf[2] = error;
	ModbusSend(3);
}
  
 
uint16 AddrChek(uint16 addr)
{
  uint8 i;
  i=0;
  while(i<5)
  {
     if(addr>=MODBUSADDR[i*2] && addr<=MODBUSADDR[i*2+1])
     {
        return(MODBUSADDR[i*2]);
     }
     i++;
  }
  return 0;
}

//������03������ȡ
void MultiRd()
 {
   uint16 addr;//��ʼ��ַ
   uint16 num;//��ȡ����
   uint16 i;
   uint8 datbuf;
   uint16 addr0;
   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//���ĩ��ַ
   if(addr0==0)
   {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
   }
   
   addr0=AddrChek(addr);//���ʼ��ַ
   if(addr0==0)
   {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
   }

	num=num*2;
   send_buf[2]=num;//��ȡ�ֽ���
   //��ȡ��Ҫ��ȡ���ݵ��׵�ַ�������ݣ���Ӧ��

    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	for(i=0;i<num;i++)
	{
	datbuf=*IPdata;
	send_buf[i+3]=datbuf;
	IPdata++;
	}
   ModbusSend(3+num);
 }
 
 //������06����������
 void SingleWr()
 {
   uint16 addr;
   uint16 addr0;
 
   addr=send_buf[2]*256+send_buf[3];
 
   addr0=AddrChek(addr);
   if(addr0==0)
   {
     ModbusSendError(0x02);//02�Ƿ���ַ
     return;
   }
   
    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	*IPdata=send_buf[4];
	IPdata++;
	*IPdata=send_buf[5];
    
   ModbusSend(rcv_count-2);//rcv_bufԭ���ݷ��أ�CRC����Ҫ���¼���
   
   //����Ӧ��״̬������ֵ
  if(addr0==0xa600)
   {
      switch(addr)
     {
      case 0xa600:
      case 0xa601:
      case 0xa602:
      case 0xa603:
		  set_clock();
		  break;
		  
      case 0xa604:
	  	SetModbusmode(data0100[9]);
		break;
		
      case 0xa607:
 	     SetModbusInterval(data0100[14]*256+data0100[15]);
      break;

      case 0xa608:
	  	SetModbussignnum(data0100[17]); 
		break;
	  
      default:
      break;	  
     }  
      memcpy(&data4000[32],data0100,20);
      ISendStr(CSI1025, 0x20,&data4000[32], 16); 
      Delay(50000);
      ISendStr(CSI1025, 0x30,&data4000[48], 16); 
			
   	}
   else if(addr0==0xa640)//����TCP/IP�Ѿ������ж�����״������λ���Բ��á�
    {    
       SetModbuslinesign(data0140[1]);
   	    }
   else if(addr0==0x4000)
   {   
	 if(data4000[73])                    //mac����
	    {     data4000[73]=0;
		    data4000[75]=0;
		    data4000[77]=0;
		    data4000[79]=0;
		    memcpy(mymac, &data4000[64], 6);
		    ISendStr(CSI1025, 0x40,&data4000[64], 6); 
		    Delay(50000);	
		}
	   else if(data4000[75])           //����IP��ַ���
	    {   data4000[75]=0;
	        ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		 Delay(50000);	
	   	}
	   else if(data4000[77])           //�ָ���������
	   {   
	       SetIni_Init(data4000[77]);
		 data4000[77]=0;
	   	}
	    else if(data4000[79])         //Զ��������
	    {	      
	      SetSytemrestart(data4000[79]);
		data4000[79]=0;
	   	}
	   else if((shan_sign!=data4000[70])||(shan_time!=data4000[71]))
	   	{
	   	   shan_sign=data4000[70];
		   shan_time=data4000[71];
		   ISendStr(CSI1025, 0x46,&data4000[64], 2); 
	   	}
	   else
	   {  
	      set_my_data();
	    	}  
   	}


 }
 
 //������16���������
 void MultiWr()
 {
    uint16 addr,num;
    uint16 addr0;
    uint8 temp;
    uint8 temp1;
    uint16 i;
    addr=send_buf[2]*256+send_buf[3];
    num=send_buf[4]*256+send_buf[5];
  
    addr0=AddrChek(addr+num-1);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
   
    addr0=AddrChek(addr);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
        
    check_store_address(addr0);

	num=num*2;
	IPdata=IPdata+(addr-addr0)*2;
    for(i=0;i<num;i++)
    {
      temp=send_buf[7+i];
      *IPdata=temp;
      IPdata++;
    }

    ModbusSend(6);//rcv_bufǰ6���ֽڷ��أ�CRC����Ҫ���¼���

  
    //����Ӧ��״̬������ֵ
     if(addr0==0xa600)
	{  
	     set_clock();
	     SetModbusmode(data0100[9]);
	     SetModbusInterval(data0100[14]*256+data0100[15]);
	     SetModbussignnum(data0100[17]); 
            memcpy(&data4000[32],data0100,20);
            ISendStr(CSI1025, 0x20,&data4000[32], 16); 
            Delay(50000);
            ISendStr(CSI1025, 0x30,&data4000[48], 16); 		 
     	  }
	else if(addr0==0xa640)
	 {
           SetModbuslinesign(data0140[1]);
		   
		}
	else if(addr0==0xa650)
	  {
	      data0250[4]=data0100[9];
		data0250[5]=0;

             parsemodbus(data0150,num);
			   
		temp=modbus_state.signnum;
	       temp=6*temp+6;
		data0250[temp+1]=data0150[1]; 
		temp1=9*data0150[1];
		temp=temp+2;
		for(i=0;i<90;i++)
		{
			data0250[temp+i*2]=0;
			 data0250[temp+i*2+1]=0;
			}
		for(i=0;i<temp1;i++)
		 {
			 data0250[temp+i*2]=data0150[i*2+2];
			 data0250[temp+i*2+1]=data0150[i*2+3];
				  
		     }
		SetModbus0150(1);
		SetPowerOn(1);	
	    
		  }	
	else if(addr0==0x4000)
	{	
	  if(data4000[73])					  //mac����
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//����IP��ַ���
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//�ָ���������
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Զ��������
		 {		   
		   SetSytemrestart(data4000[79]);
		 data4000[79]=0;
		 }
		else if((shan_sign!=data4000[70])||(shan_time!=data4000[71]))
		 {
			shan_sign=data4000[70];
			shan_time=data4000[71];
			ISendStr(CSI1025, 0x46,&data4000[64], 2); 
		 }
		else
		{  
		   set_my_data();
			 }	
	 }

}
 
 
 //������23���������/��ȡ
void MultiRdWr()
 {
    uint16 addr,num;
    uint16 addr1;
    uint16 addr0;
    uint8 temp;
    uint16 i;
    addr1=send_buf[6]*256+send_buf[7];
    num=send_buf[8]*256+send_buf[9];
  
    addr0=AddrChek(addr1+num-1);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
   
    addr0=AddrChek(addr1);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
    }
    
    check_store_address(addr0);

	num=num*2;
	IPdata=IPdata+(addr1-addr0)*2;
    for(i=0;i<num;i++)
    {
      temp=send_buf[11+i];
      *IPdata=temp;
      IPdata++;
    }

   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//���ĩ��ַ
   if(addr0==0)
   {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
   }
   
   addr0=AddrChek(addr);//���ʼ��ַ
   if(addr0==0)
   {
      ModbusSendError(0x02);//02�Ƿ���ַ
      return;
   }

	num=num*2;
   send_buf[2]=num;//��ȡ�ֽ���
   //��ȡ��Ҫ��ȡ���ݵ��׵�ַ�������ݣ���Ӧ��

    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	for(i=0;i<num;i++)
	{
	temp=*IPdata;
	send_buf[i+3]=temp;
	IPdata++;
	}
   ModbusSend(3+num);

   addr0=AddrChek(addr1);
      if(addr0==0xa600)
	{  
	     set_clock();
	     SetModbusmode(data0100[9]);
	     SetModbusInterval(data0100[14]*256+data0100[15]);
	     SetModbussignnum(data0100[17]); 
            memcpy(&data4000[32],data0100,20);
            ISendStr(CSI1025, 0x20,&data4000[32], 16); 
            Delay(50000);
            ISendStr(CSI1025, 0x30,&data4000[48], 16); 		 
     	  }
	else if(addr0==0xa640)
	 {
           SetModbuslinesign(data0140[1]);
		   
		}
	else if(addr0==0xa650)
	  {
	      data0250[4]=data0100[9];
		data0250[5]=0;

             parsemodbus(data0150,num);
			   
		temp=modbus_state.signnum;
	       temp=6*temp+6;
		data0250[temp+1]=data0150[1]; 
		temp=temp+2;
		for(i=0;i<90;i++)
		 {
			 data0250[temp+i*2]=data0150[i*2+2];
			 data0250[temp+i*2+1]=data0150[i*2+3];
				  
		     }
		SetModbus0150(1);
		SetPowerOn(1);	
	    
		  }
	else if(addr0==0x4000)
	{	
	  if(data4000[73])					  //mac����
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//����IP��ַ���
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//�ָ���������
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Զ��������
		 {		   
		   SetSytemrestart(data4000[79]);
		 data4000[79]=0;
		 }
		else if((shan_sign!=data4000[70])||(shan_time!=data4000[71]))
		 {
			shan_sign=data4000[70];
			shan_time=data4000[71];
			ISendStr(CSI1025, 0x46,&data4000[64], 2); 
		 }
		else
		{  
		   set_my_data();
			 }	
	 }


 }
 

void check_store_address(uint16 addr0)
{
	switch(addr0)
		{

			case 0xa600:
				IPdata = data0100;
					break;

		 	case 0xa640:
				IPdata = data0140;
					break;
					
			case 0xa650:
				IPdata = data0150;
					break;

		 	case 0xa750:
				IPdata = data0250;
				       break;
					   
		 	case 0x4000:
				IPdata = data4000;
				       break;
					   
		}
}

void parsemodbus(uint8 *databuf,uint8 length)
{
   uint8 i,j;
   uint8 efaction;
   uint8 lsign;
   uint8 temp1;
   uint16 temp2;

   for(i=0;i<modbus_state.signnum;i++)
   	{   
   	    ledlane[i].maxplan=databuf[1];  
   	}
   for(j=0;j<ledlane[0].maxplan;j++)
   	{   efaction=databuf[18*j+3];
           for(i=0;i<efaction;i++)
           	{     temp1=databuf[18*j+4+2*i];
           	       temp2=databuf[18*j+4+2*i]*256+databuf[18*j+5+2*i];
		       lsign=(temp1>>5)&0x07;
			ledlane[lsign].sign[j]=1;
			ledlane[lsign].action[j]=(temp1>>2)&0x07;
		       ledlane[lsign].timelag[j]=temp2&0x03FF;
           	}
   	}
   	
    

}


void InitParam()//��ʼ�������� 7000��ʼ��
{
 
 //��ʼ��ͨ�ù�����
  ini_sram();
  SetModbusInterval(data0100[14]*256+data0100[15]);//��СͨѶ���,Ĭ��Ϊ600��
  modbus_state.count=modbus_state.interval*data4000[31];
  Settcplinecheck();
  SetPowerOff(0);//����
  
  //��ʱ��
  time.year=2000 +(data0100[1]/16)*10 + data0100[1]%16;
  time.month=(data0100[2]/16)*10 + data0100[2]%16;
  time.day=(data0100[3]/16)*10 + data0100[3]%16;
  time.hour=(data0100[4]/16)*10 + data0100[4]%16;
  time.minute=(data0100[5]/16)*10 + data0100[5]%16;
  time.second=(data0100[6]/16)*10 + data0100[6]%16;

//����д��SRAM��
 
}


/****************************************************************************
* ���ƣ�RTCIni()
* ���ܣ���ʼ��ʵʱʱ�ӡ�
* ��ڲ�������
* ���ڲ�������
****************************************************************************/
void  RTCIni(void)
{  PREINT = Fpclk / 32768 - 1;	// ���û�׼ʱ�ӷ�Ƶ��
   PREFRAC = Fpclk - (Fpclk / 32768) * 32768;
   
    SEC = time.second;
    MIN	=time.minute;
    HOUR=time.hour;
    DOM =time.day;
    MONTH =time.month;			   
    YEAR =time.year;	
   
   CIIR = 0x01;				    // ������ֵ����������һ���ж�
   
   CCR = 0x01;				    // ����RTC
}   


/***************************************************************************************************
*���ƣ�Initialization()
*��������ʼ��ϵͳ����Ҫ����������ã���RAM��
*��ڲ�������
*���ڲ�������
***************************************************************************************************/
void Initialization(void)
{     
       uip_ipaddr_t ipaddr;
	uip_init();	
	init_8019();    
      uip_arp_init();
      uip_ipaddr(ipaddr, LocalIP[0],LocalIP[1],LocalIP[2],LocalIP[3]);
      uip_sethostaddr(ipaddr);
      uip_ipaddr(ipaddr, GateWay[0],GateWay[1],GateWay[2],GateWay[3]);
      uip_setdraddr(ipaddr);
      uip_ipaddr(ipaddr, MaskIP[0],MaskIP[1],MaskIP[2],MaskIP[3]);
      uip_setnetmask(ipaddr);
      uip_listen(HTONS(ServerPort[0]*256+ServerPort[1])); 
      timetcp=(data4000[26]*256+data4000[27])*data4000[57];      
}

