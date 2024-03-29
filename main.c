#include "main.h"

#define BUF ((struct uip_eth_hdr *)uip_buf)

/* 定义串口模式设置数据结构 */
typedef  struct  UartMode
{  uint8 datab;         // 字长度，5/6/7/8
   uint8 stopb;         // 停止位，1/2
   uint8 parity;    	// 奇偶校验位，0为无校验，1奇数校验，2为偶数校验
}  UARTMODE;

uint8  rcv_buf[180];       		// UART0数据接收缓冲区,当文字一次写满72字符的时候，为最大值
uint8  rcv_temp[180];       		// UART0数据接收缓冲区,当文字一次写满72字符的时候，为最大值
uint8  send_buf[245];//UART0数据发送缓冲区，一次最多读取120个寄存器
uint8  rcv_new;     	// 接收到新数据标志

uint16 net_time05=0;
uint16 net_time1=0;
uint16 net_time10=0;
uint16 net_time30=0;
uint8 net_timeover05=0;
uint8 net_timeover1=0;
uint8 net_timeover10=0;	
uint8 net_timeover30=0;



/****************************************************************************
* 名称：IRQ_UART0()
* 功能：串口UART0接收中断。
* 入口参数：无
* 出口参数：无
****************************************************************************/
void   __irq IRQ_UART0(void)
{ 
   uint8  i;
   //关定时器
   T0TCR = 0x00;  
   if( 0x04==(U0IIR&0x0F) )
   {
     for(i=0; i<7; i++)
     {
       rcv_buf[rec_count] = U0RBR; 	// 读取FIFO的数据，并清除中断标志 
       rec_count++;          
     }
	 if(rec_count>165)
	 	rec_count=0;
   }
   else if( 0x0c==(U0IIR&0x0F) )//至少有一个字符，怎么判断字符数呢？读U0LSR相应位
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
		       rcv_temp[i] = rcv_buf[i]; 	// 读取FIFO的数据，并清除中断标志 
		     }
		     rcv_count=rec_count;
  		     rec_count=0;//为下一帧接受做准备     
		     rcv_new = 1; // 设置接收到新的数据标志
		}
	else
		{
 		    rec_count=0;//为下一帧接受做准备     
		}
   }
   
   //开定时器
   T0TCR = 0x01;
   VICVectAddr = 0x00; // 中断处理结束
   
}               


/****************************************************************************
* 名称：SendByte()
* 功能：向串口UART0发送字节数据。
* 入口参数：data                要发送的数据
* 出口参数：无
****************************************************************************/
void  SendByte(uint8 data)
{  U0THR = data;                      	// 发送数据
}


/****************************************************************************
* 名称：ISendBuf()
* 功能：将缓冲区的数据发送回主机(使用FIFO)，并等待发送完毕。
* 入口参数：length
* 出口参数：无
****************************************************************************/
void  ISendBuf(uint8 length)//16个字节fifo(收发)
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
   
   while( (U0LSR&0x20)==0 );         	// 等待数据发送
}               
                 
        
/****************************************************************************
* 名称：UART0_Ini()
* 功能：初始化串口0。设置其工作模式及波特率。
* 入口参数：baud                波特率
*          set          模式设置(UARTMODE数据结构)
* 出口参数：返回值为1时表示初化成功，为0表除参数出错
****************************************************************************/
uint8  UART0_Ini(uint32 baud, UARTMODE set)
{  uint32  bak;
   
   /* 参数过滤 */
   if( (0==baud)||(baud>115200) ) return(0);
   if( (set.datab<5)||(set.datab>8) ) return(0);
   if( (0==set.stopb)||(set.stopb>2) ) return(0);
   if( set.parity>4 ) return(0);

   /* 设置串口波特率 */
   U0LCR = 0x80;                        // DLAB位置1
   bak = (Fpclk>>4)/baud;
   U0DLM = bak>>8;
   U0DLL = bak&0xff;
   
   /* 设置串口模式 */
   bak = set.datab-5;                   // 设置字长度
   if(2==set.stopb) bak |= 0x04;        // 判断是否为2位停止位  
   
   if(0!=set.parity) {set.parity = set.parity-1; bak |= 0x08;}
   bak |= set.parity<<4;              	// 设置奇偶校验
      
   U0LCR = bak;
   
   return(1);
}

//初始化串口，使能串口
void init_uart0(void)
{
   UARTMODE  uart0_set={0};
   
   rcv_new = 0;
   
   uart0_set.datab = 8;                 // 8位数据位
   uart0_set.stopb = 1;                 // 1位停止位
   uart0_set.parity = 0;                // 无奇偶校验
   UART0_Ini(set_baud, uart0_set);        // 初始化串口模式
   
   U0FCR = 0x81;                        // 使能FIFO，并设置触发点为8字节
   U0IER = 0x01;                        // 允许RBR中断，即接收中断
   
  
   VICIntSelect = 0x00000000;           // 设置所有通道为IRQ中断
   VICVectCntl1 = 0x26;                 // UART0中断通道分配到IRQ slot 0，即优先级最高
   VICVectAddr1 = (int)IRQ_UART0;       // 设置UART0向量地址
   VICIntEnable |= 0x00000040;           // 使能UART0中断

}


//每隔一段时间，发送一次触发信号，触发CPLD更新显示

/****************************************************************************
* 名称：IRQ_Time0()
* 功能：定时器0中断服务程序
* 入口参数：无
* 出口参数：无
****************************************************************************/
void __irq  IRQ_Time0(void)
{  
   newtimer=1;
   if(net_time05++>=25)
   {					 //0.5秒溢出标志
	 net_time05=0;
	 net_timeover05=1;	
	}
   if(net_time1++>=50)
   {					 //秒溢出标志
	 net_time1=0;
	 net_timeover1=1;	
	}  
   if(net_time10++>=500)
   {				 //10秒溢出标志
	 net_time10=0;
	 net_timeover10=1;	
	}
   if(net_time30++>=1500)
   {
        net_time30=0;
	 net_timeover30=1;
   	}
   T0IR = 0x01;	    			            	// 清除中断标志
   VICVectAddr = 0x00; // 通知VIC中断处理结束
}


/****************************************************************************
* 名称：Time0Init()
* 功能：初始化定时器0，定时时间为20mS，并使能中断。 
* 入口参数：无
* 出口参数：无
****************************************************************************/
void  Time0Init(void)
{   /* Fcclk = Fosc*4 = 11.0592MHz*4 = 44.2368MHz
	   Fpclk = Fcclk/4 = 44.2368MHz/4 = 11.0592MHz
	*/
	T0PR = 99;			    					// 设置定时器0分频为100分频，得110592Hz
	T0MCR = 0x03;		   						// 匹配通道0匹配中断并复位T0TC
	T0MR0 = 2212;	    						// 比较值(20MS定时值)
	T0TCR = 0x03;		   						// 启动并复位T0TC
	T0TCR = 0x01; 
	
	/* 设置定时器0中断IRQ */
	VICIntSelect = 0x00;						// 所有中断通道设置为IRQ中断
	VICVectCntl0 = 0x24;						// 定时器0中断通道分配最高优先级(向量控制器0)
	VICVectAddr0 = (uint32)IRQ_Time0; 			// 设置中断服务程序地址向量 
	VICIntEnable = 0x00000010;					// 使能定时器0中断
}


//CRC校验
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
  rec_count=0;//串口中断 接收记数
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

void ModbusDeal()//接收到串口命令，要做的处理
{
  uint8 aa;

        for(aa=0;aa<rcv_count;aa++)//将数据全部复制到发送缓存
        {
           send_buf[aa]=rcv_temp[aa];
        }
        
        crc=CRC16(send_buf, rcv_count);
        if(crc==0)//crc校验正确
        {
           switch(send_buf[1])//功能码
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
              ModbusSendError(0x01);//01非法功能码
              break;
            }//switch
           
           if(modbus_state.interval>0)//
           modbus_state.count=modbus_state.interval*data4000[31];//最小通讯间隔，记数重新赋值
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
			 if(crc==0)//crc校验正确
				{

					if(send_buf[0]==ip_address)
						{
							TCP_ModbusDeal();
						}
				}
		
}    

void TCP_ModbusDeal()
{
	switch(send_buf[1])//功能码
		{
              case 0x03:
              	TCP_MultiRd();
				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//最小通讯间隔，记数重新赋值
              break;
              case 0x06:
              	TCP_SingleWr();
 				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//最小通讯间隔，记数重新赋值
             break;
              case 0x10:
 	            TCP_MultiWr();
 				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//最小通讯间隔，记数重新赋值
             break;
              case 0x17:
	            TCP_MultiRdWr();
				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//最小通讯间隔，记数重新赋值
              break;
              default:
	              TCP_ModbusSendError(0x01);//01非法功能码
              break;
		}//switch
           
}

//功能码03，多点读取
void TCP_MultiRd()
 {
   uint16 addr;//起始地址
   uint16 num;//读取点数
   uint16 i;
   uint8 datbuf;
   uint16 addr0;
   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//检测末地址
   if(addr0==0)
   {
		TCP_ModbusSendError(0x02);
        return;
   }
   
   addr0=AddrChek(addr);//检测始地址
   if(addr0==0)
   {
		TCP_ModbusSendError(0x02);
        return;
   }

	num=num*2;
   send_buf[2]=num;//读取字节数
   

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
 
 //功能码06，单点设置
 void TCP_SingleWr()
 {
   uint16 addr;
   uint16 addr0;
 
   addr=send_buf[2]*256+send_buf[3];
 
   addr0=AddrChek(addr);
   if(addr0==0)
   {
     TCP_ModbusSendError(0x02);//02非法地址
     return;
   }
   
    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	*IPdata=send_buf[4];
	IPdata++;
	*IPdata=send_buf[5];
    
	TCP_ModbusSend(rcv_count2-2);
   
   //给相应的状态变量赋值   
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
   else if(addr0==0xa640)//网络TCP/IP已经可以判断连接状况，该位可以不用。
    {    
       SetModbuslinesign(data0140[1]);
   	    }
   else if(addr0==0x4000)
   {   
	 if(data4000[73])                    //mac更改
	    {     data4000[73]=0;
		    data4000[75]=0;
		    data4000[77]=0;
		    data4000[79]=0;
		    memcpy(mymac, &data4000[64], 6);
		    ISendStr(CSI1025, 0x40,&data4000[64], 6); 
		    Delay(50000);	
		}
	   else if(data4000[75])           //网络IP地址变更
	    {   data4000[75]=0;
	        ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		 Delay(50000);	
	   	}
	   else if(data4000[77])           //恢复出厂设置
	   {   
	       SetIni_Init(data4000[77]);
		 data4000[77]=0;
	   	}
	    else if(data4000[79])         //远程重启动
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
 
 //功能码16，多点设置
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
      TCP_ModbusSendError(0x02);//02非法地址
      return;
    }
   
    addr0=AddrChek(addr);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02非法地址
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

  
    //给相应的状态变量赋值
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
	  if(data4000[73])					  //mac更改
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//网络IP地址变更
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		        Delay(50000);  
		 }
		else if(data4000[77])			//恢复出厂设置
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //远程重启动
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
 
 //功能码23，多点设置/读取
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
      TCP_ModbusSendError(0x02);//02非法地址
      return;
    }
   
    addr0=AddrChek(addr1);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02非法地址
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

   addr0=AddrChek(addr+num-1);//检测末地址
   if(addr0==0)
   {
      TCP_ModbusSendError(0x02);//02非法地址
      return;
   }
   
   addr0=AddrChek(addr);//检测始地址
   if(addr0==0)
   {
      TCP_ModbusSendError(0x02);//02非法地址
      return;
   }

	num=num*2;
   send_buf[2]=num;//读取字节数
   //获取需要读取数据的首地址，读数据，送应答

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
	  if(data4000[73])					  //mac更改
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//网络IP地址变更
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//恢复出厂设置
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //远程重启动
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
 /*01*/if((0==temp1)||(0==temp3))     //6号故障(硬件不支持)
    	    	{    
		       LEDstate.state.devstate=custom_fault;
		       LEDstate.state.hwfault=no_support;    	    	  
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;    	    	       
    	    	}
 /*02*/ else if((rout_out_data&temp1)&&(rout_out_data&temp2))        //故障号2(控制卡硬件故障)
		{			
		       LEDstate.state.devstate=hardware_fault;
		       LEDstate.state.hwfault=2;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;			   
	    	}
 /*03*/  else if((0==(back_in_data&temp3))&&(0==(back_in_data&temp4)))       //故障号5(模组帧故障)
		 {
		       LEDstate.state.devstate=module_frame_fault;
//		       LEDstate.state.hwfault=0;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;			
	      	}
 /*04*/  else if(((rout_out_data&temp1)&&(back_in_data&temp3))||((rout_out_data&temp2)&&(back_in_data&temp4)))      //故障号4(模组通讯故障)
		 {
		       LEDstate.state.devstate=module_line_fault;
//			LEDstate.state.hwfault=0;
			LEDstate.state.showstate=0;		       
			LEDstate.state.actionstate=0;
		 }
 /*05*/	 else if(((0==(rout_out_data&temp1))&&(0==(back_in_data&temp3)))||((0==(rout_out_data&temp2))&&(0==(back_in_data&temp4))))    //故障号3(LED故障)
		 {
		       LEDstate.state.devstate=LED_fault;			
//		       LEDstate.state.hwfault=0;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;
		  }
 /*06*/	 else if((rout_out_data&temp1)&&(0==(back_in_data&temp3)))          //红灯正常显示
		 {
		         LEDstate.state.actionstate=RED;
			  LEDstate.state.showstate=1;
		  }
/*07*/	  else if((rout_out_data&temp2)&&(0==(back_in_data&temp4)))      //绿灯正常显示
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

void TimerDeal(void)//每个时钟节拍，要做的处理
{     
      uint8 temp1,temp2,temp3;
      uint8 i;
      uint32  ANIN_data;
	  
      ANIN_data=readinput();
      if(ANIN_data!=ANIN)            //手动
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
       //若在给定的最小通讯间隔时间内未接受到有效帧，系统应自动黑屏。
      if(modbus_state.interval>0 && modbus_state.count>0)                //连接状况
       {
         modbus_state.count--;
         if(modbus_state.count==0)
         { 
            if(0==data4000[55])
            {
               SetPowerOff(0);      //总关屏               
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
	
	  if((modbus_state.poweron)&&(0==modbus_state.k_mode))      //自动
	   {     
	          for(i=0;i<modbus_state.signnum;i++)
	          {    
	               temp1=ledlane[i].maxplan-plan[i];
	          	  if(plan[i]==0)
	          	  {    
	          	       temp2=ledlane[i].maxplan-1;              //最后一个方案
			       temp3=ledlane[i].action[temp2];     //第i车道第temp2个方案中的动作
	   	              if(ledlane[i].sign[temp2])                //第i车道第temp2个方案中的子动作是否激活
	   	              {    
	   	                  FArray[i].Function(temp3);	      //第i车道灯执行动作(0-7个动作,无动作既是空动作)
	   	                }
	          	  	}
			    else
			     {  
			         temp3=ledlane[i].action[temp1];     //第i车道第temp1个方案中的动作
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
			             if(ledlane[i].sign[temp1])                //第i车道第temp1个方案中的子动作是否激活
	   	                     {    
	   	                        FArray[i].Function(temp3);	      //第i车道灯执行动作(0-7个动作,无动作既是空动作)
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
	      else if(data4000[20])                              //手动状况
	 	 {
	 	      leddelay--;
		      if(0==leddelay)
		      	{  SetPowerOff(0);
		      	    SetModbusmode(0);
			    checkstart(data4000[63]);
		      	}
	      	   }			  	
				  
       if(checksign)  
	 checktime--;     //车道反馈
	timetcp--;          //网络连接检测
	
      if( 0==(ILR&0x01) );	    // 等待RTC增量中断标志
		{
		    ILR = 0x03;				    // 清除中断标志
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
	       data4000[77]=0;          //失效
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
//主程序
int main(void)
{
  uint8 i;
  
  BusInit( );

  InitIO();

  ini_prog();
  
  PowerOnInit(); 

    WDTC = 0x00FFFFFF;						// 设置WDTC，喂狗重装值
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
	 while(1);	  	      //远程重启
     	}
     uip_len=recv_packet();            //网卡接收数据
     if(uip_len > 0)			    //收到数据
      {
			/* 处理IP数据包(只有校验通过的IP包才会被接收) */
		if(BUF->type == htons(UIP_ETHTYPE_IP))   //是IP包吗？
		{
			uip_arp_ipin();		   //去除以太网头结构，更新ARP表		
			uip_input();		   //IP包处理
				/*
					当上面的函数执行后，如果需要发送数据，则全局变量 uip_len > 0
					需要发送的数据在uip_buf, 长度是uip_len  (这是2个全局变量)
				*/
			if (uip_len > 0)		//有带外回应数据
			{
				uip_arp_out();		//加以太网头结构，在主动连接时可能要构造ARP请求
				send_packet();		//发送数据到以太网（设备驱动程序）
				timetcp=(data4000[28]*256+data4000[29])*data4000[30];
			}
		 }
			/* 处理arp报文 */
		 else if (BUF->type == htons(UIP_ETHTYPE_ARP))	//是ARP请求包
		 {
			uip_arp_arpin();		//如是是ARP回应，更新ARP表；如果是请求，构造回应数据包
				/*
					当上面的函数执行后，如果需要发送数据，则全局变量 uip_len > 0
					需要发送的数据在uip_buf, 长度是uip_len  (这是2个全局变量)
				*/
			if (uip_len > 0)		//是ARP请求，要发送回应
			{
				send_packet();		//发ARP回应到以太网�				
			}
		    }
	       }	  
      else if(net_timeover05==1)			/* 0.5秒定时器超时 */
	 {	net_time05=0;
		net_timeover05=0;			/* 复位0.5秒定时器 */	
#ifdef  SHANSHUO
		shan();  
#endif
                        
		/* 轮流处理每个TCP连接, UIP_CONNS缺省是10个 */
		for(i = 0; i < UIP_CONNS; i++)
		{
			uip_periodic(i);		/* 处理TCP通信事件 */
			/*
				当上面的函数执行后，如果需要发送数据，则全局变量 uip_len > 0
				需要发送的数据在uip_buf, 长度是uip_len  (这是2个全局变量)
			*/
			if(uip_len > 0)
			{
				uip_arp_out();		//加以太网头结构，在主动连接时可能要构造ARP请求
				send_packet();		//发送数据到以太网（设备驱动程序）
			}
		}

	#if UIP_UDP
		/* 轮流处理每个UDP连接, UIP_UDP_CONNS缺省是10个 */
		for(i = 0; i < UIP_UDP_CONNS; i++)
		{     
			uip_udp_periodic(i);	/*处理UDP通信事件 */
			/* 如果上面的函数调用导致数据应该被发送出去，全局变量uip_len设定值> 0 */
			if(uip_len > 0)
			{
				uip_arp_out();		//加以太网头结构，在主动连接时可能要构造ARP请求
				send_packet();		//发送数据到以太网（设备驱动程序）
			}
		}
	#endif /* UIP_UDP */

		/* 每隔10秒调用1次ARP定时器函数 用于定期ARP处理,ARP表10秒更新一次，旧的条目会被抛弃*/
            if(net_timeover1==1)
		 {    net_time1=0;
		       net_timeover1=0;
#ifdef  SHANSHUO			  
//			shan();         //绿灯闪烁显示5秒
#endif
			 WdtFeed();
		       TimerDeal(); 
			 if(data4000[60]&&checksign&&(0==checktime))
                     {    checkstart(data4000[61]);
				check();
                     	}

	             if (net_timeover10==1)
		        {     net_time10=0;
			        net_timeover10=0;		/* 复位10秒定时器 */				
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
     else if(1==rcv_new)//接收到一帧数据,数据长度要记
     { 
        rcv_new = 0;
        ModbusDeal();
       }
     if(0==timetcp)              //网卡没有连接到网络
	{     
	    init_8019();
	    timetcp=(data4000[26]*256+data4000[27])*data4000[57];
		
	   }
 /*   else if(clocksave)
    	{   clocksave=0;
    	     data4000[77]=0;          //失效
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


///////////////////MODBUS数据解析//////////////////////////////  
void
ModbusSend(uint8 length)//length 不包括CRC码
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

//功能码03，多点读取
void MultiRd()
 {
   uint16 addr;//起始地址
   uint16 num;//读取点数
   uint16 i;
   uint8 datbuf;
   uint16 addr0;
   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//检测末地址
   if(addr0==0)
   {
      ModbusSendError(0x02);//02非法地址
      return;
   }
   
   addr0=AddrChek(addr);//检测始地址
   if(addr0==0)
   {
      ModbusSendError(0x02);//02非法地址
      return;
   }

	num=num*2;
   send_buf[2]=num;//读取字节数
   //获取需要读取数据的首地址，读数据，送应答

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
 
 //功能码06，单点设置
 void SingleWr()
 {
   uint16 addr;
   uint16 addr0;
 
   addr=send_buf[2]*256+send_buf[3];
 
   addr0=AddrChek(addr);
   if(addr0==0)
   {
     ModbusSendError(0x02);//02非法地址
     return;
   }
   
    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	*IPdata=send_buf[4];
	IPdata++;
	*IPdata=send_buf[5];
    
   ModbusSend(rcv_count-2);//rcv_buf原数据返回，CRC码需要重新计算
   
   //给相应的状态变量赋值
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
   else if(addr0==0xa640)//网络TCP/IP已经可以判断连接状况，该位可以不用。
    {    
       SetModbuslinesign(data0140[1]);
   	    }
   else if(addr0==0x4000)
   {   
	 if(data4000[73])                    //mac更改
	    {     data4000[73]=0;
		    data4000[75]=0;
		    data4000[77]=0;
		    data4000[79]=0;
		    memcpy(mymac, &data4000[64], 6);
		    ISendStr(CSI1025, 0x40,&data4000[64], 6); 
		    Delay(50000);	
		}
	   else if(data4000[75])           //网络IP地址变更
	    {   data4000[75]=0;
	        ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		 Delay(50000);	
	   	}
	   else if(data4000[77])           //恢复出厂设置
	   {   
	       SetIni_Init(data4000[77]);
		 data4000[77]=0;
	   	}
	    else if(data4000[79])         //远程重启动
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
 
 //功能码16，多点设置
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
      ModbusSendError(0x02);//02非法地址
      return;
    }
   
    addr0=AddrChek(addr);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02非法地址
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

    ModbusSend(6);//rcv_buf前6个字节返回，CRC码需要重新计算

  
    //给相应的状态变量赋值
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
	  if(data4000[73])					  //mac更改
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//网络IP地址变更
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//恢复出厂设置
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //远程重启动
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
 
 
 //功能码23，多点设置/读取
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
      ModbusSendError(0x02);//02非法地址
      return;
    }
   
    addr0=AddrChek(addr1);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02非法地址
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

   addr0=AddrChek(addr+num-1);//检测末地址
   if(addr0==0)
   {
      ModbusSendError(0x02);//02非法地址
      return;
   }
   
   addr0=AddrChek(addr);//检测始地址
   if(addr0==0)
   {
      ModbusSendError(0x02);//02非法地址
      return;
   }

	num=num*2;
   send_buf[2]=num;//读取字节数
   //获取需要读取数据的首地址，读数据，送应答

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
	  if(data4000[73])					  //mac更改
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//网络IP地址变更
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//恢复出厂设置
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //远程重启动
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


void InitParam()//初始化参数区 7000开始的
{
 
 //初始化通用功能区
  ini_sram();
  SetModbusInterval(data0100[14]*256+data0100[15]);//最小通讯间隔,默认为600秒
  modbus_state.count=modbus_state.interval*data4000[31];
  Settcplinecheck();
  SetPowerOff(0);//变量
  
  //读时间
  time.year=2000 +(data0100[1]/16)*10 + data0100[1]%16;
  time.month=(data0100[2]/16)*10 + data0100[2]%16;
  time.day=(data0100[3]/16)*10 + data0100[3]%16;
  time.hour=(data0100[4]/16)*10 + data0100[4]%16;
  time.minute=(data0100[5]/16)*10 + data0100[5]%16;
  time.second=(data0100[6]/16)*10 + data0100[6]%16;

//变量写入SRAM区
 
}


/****************************************************************************
* 名称：RTCIni()
* 功能：初始化实时时钟。
* 入口参数：无
* 出口参数：无
****************************************************************************/
void  RTCIni(void)
{  PREINT = Fpclk / 32768 - 1;	// 设置基准时钟分频器
   PREFRAC = Fpclk - (Fpclk / 32768) * 32768;
   
    SEC = time.second;
    MIN	=time.minute;
    HOUR=time.hour;
    DOM =time.day;
    MONTH =time.month;			   
    YEAR =time.year;	
   
   CIIR = 0x01;				    // 设置秒值的增量产生一次中断
   
   CCR = 0x01;				    // 启动RTC
}   


/***************************************************************************************************
*名称：Initialization()
*描述：初始化系统，主要对软件的配置，如RAM等
*入口参数：无
*出口参数：无
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

