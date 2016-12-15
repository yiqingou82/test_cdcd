#include "main.h"

#define BUF ((struct uip_eth_hdr *)uip_buf)

/* ¶¨Òå´®¿ÚÄ£Ê½ÉèÖÃÊı¾İ½á¹¹ */
typedef  struct  UartMode
{  uint8 datab;         // ×Ö³¤¶È£¬5/6/7/8
   uint8 stopb;         // Í£Ö¹Î»£¬1/2
   uint8 parity;    	// ÆæÅ¼Ğ£ÑéÎ»£¬0ÎªÎŞĞ£Ñé£¬1ÆæÊıĞ£Ñé£¬2ÎªÅ¼ÊıĞ£Ñé
}  UARTMODE;

uint8  rcv_buf[180];       		// UART0Êı¾İ½ÓÊÕ»º³åÇø,µ±ÎÄ×ÖÒ»´ÎĞ´Âú72×Ö·ûµÄÊ±ºò£¬Îª×î´óÖµ
uint8  rcv_temp[180];       		// UART0Êı¾İ½ÓÊÕ»º³åÇø,µ±ÎÄ×ÖÒ»´ÎĞ´Âú72×Ö·ûµÄÊ±ºò£¬Îª×î´óÖµ
uint8  send_buf[245];//UART0Êı¾İ·¢ËÍ»º³åÇø£¬Ò»´Î×î¶à¶ÁÈ¡120¸ö¼Ä´æÆ÷
uint8  rcv_new;     	// ½ÓÊÕµ½ĞÂÊı¾İ±êÖ¾

uint16 net_time05=0;
uint16 net_time1=0;
uint16 net_time10=0;
uint16 net_time30=0;
uint8 net_timeover05=0;
uint8 net_timeover1=0;
uint8 net_timeover10=0;	
uint8 net_timeover30=0;



/****************************************************************************
* Ãû³Æ£ºIRQ_UART0()
* ¹¦ÄÜ£º´®¿ÚUART0½ÓÊÕÖĞ¶Ï¡£
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
****************************************************************************/
void   __irq IRQ_UART0(void)
{ 
   uint8  i;
   //¹Ø¶¨Ê±Æ÷
   T0TCR = 0x00;  
   if( 0x04==(U0IIR&0x0F) )
   {
     for(i=0; i<7; i++)
     {
       rcv_buf[rec_count] = U0RBR; 	// ¶ÁÈ¡FIFOµÄÊı¾İ£¬²¢Çå³ıÖĞ¶Ï±êÖ¾ 
       rec_count++;          
     }
	 if(rec_count>165)
	 	rec_count=0;
   }
   else if( 0x0c==(U0IIR&0x0F) )//ÖÁÉÙÓĞÒ»¸ö×Ö·û£¬ÔõÃ´ÅĞ¶Ï×Ö·ûÊıÄØ£¿¶ÁU0LSRÏàÓ¦Î»
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
		       rcv_temp[i] = rcv_buf[i]; 	// ¶ÁÈ¡FIFOµÄÊı¾İ£¬²¢Çå³ıÖĞ¶Ï±êÖ¾ 
		     }
		     rcv_count=rec_count;
  		     rec_count=0;//ÎªÏÂÒ»Ö¡½ÓÊÜ×ö×¼±¸     
		     rcv_new = 1; // ÉèÖÃ½ÓÊÕµ½ĞÂµÄÊı¾İ±êÖ¾
		}
	else
		{
 		    rec_count=0;//ÎªÏÂÒ»Ö¡½ÓÊÜ×ö×¼±¸     
		}
   }
   
   //¿ª¶¨Ê±Æ÷
   T0TCR = 0x01;
   VICVectAddr = 0x00; // ÖĞ¶Ï´¦Àí½áÊø
   
}               


/****************************************************************************
* Ãû³Æ£ºSendByte()
* ¹¦ÄÜ£ºÏò´®¿ÚUART0·¢ËÍ×Ö½ÚÊı¾İ¡£
* Èë¿Ú²ÎÊı£ºdata                Òª·¢ËÍµÄÊı¾İ
* ³ö¿Ú²ÎÊı£ºÎŞ
****************************************************************************/
void  SendByte(uint8 data)
{  U0THR = data;                      	// ·¢ËÍÊı¾İ
}


/****************************************************************************
* Ãû³Æ£ºISendBuf()
* ¹¦ÄÜ£º½«»º³åÇøµÄÊı¾İ·¢ËÍ»ØÖ÷»ú(Ê¹ÓÃFIFO)£¬²¢µÈ´ı·¢ËÍÍê±Ï¡£
* Èë¿Ú²ÎÊı£ºlength
* ³ö¿Ú²ÎÊı£ºÎŞ
****************************************************************************/
void  ISendBuf(uint8 length)//16¸ö×Ö½Úfifo(ÊÕ·¢)
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
   
   while( (U0LSR&0x20)==0 );         	// µÈ´ıÊı¾İ·¢ËÍ
}               
                 
        
/****************************************************************************
* Ãû³Æ£ºUART0_Ini()
* ¹¦ÄÜ£º³õÊ¼»¯´®¿Ú0¡£ÉèÖÃÆä¹¤×÷Ä£Ê½¼°²¨ÌØÂÊ¡£
* Èë¿Ú²ÎÊı£ºbaud                ²¨ÌØÂÊ
*          set          Ä£Ê½ÉèÖÃ(UARTMODEÊı¾İ½á¹¹)
* ³ö¿Ú²ÎÊı£º·µ»ØÖµÎª1Ê±±íÊ¾³õ»¯³É¹¦£¬Îª0±í³ı²ÎÊı³ö´í
****************************************************************************/
uint8  UART0_Ini(uint32 baud, UARTMODE set)
{  uint32  bak;
   
   /* ²ÎÊı¹ıÂË */
   if( (0==baud)||(baud>115200) ) return(0);
   if( (set.datab<5)||(set.datab>8) ) return(0);
   if( (0==set.stopb)||(set.stopb>2) ) return(0);
   if( set.parity>4 ) return(0);

   /* ÉèÖÃ´®¿Ú²¨ÌØÂÊ */
   U0LCR = 0x80;                        // DLABÎ»ÖÃ1
   bak = (Fpclk>>4)/baud;
   U0DLM = bak>>8;
   U0DLL = bak&0xff;
   
   /* ÉèÖÃ´®¿ÚÄ£Ê½ */
   bak = set.datab-5;                   // ÉèÖÃ×Ö³¤¶È
   if(2==set.stopb) bak |= 0x04;        // ÅĞ¶ÏÊÇ·ñÎª2Î»Í£Ö¹Î»  
   
   if(0!=set.parity) {set.parity = set.parity-1; bak |= 0x08;}
   bak |= set.parity<<4;              	// ÉèÖÃÆæÅ¼Ğ£Ñé
      
   U0LCR = bak;
   
   return(1);
}

//³õÊ¼»¯´®¿Ú£¬Ê¹ÄÜ´®¿Ú
void init_uart0(void)
{
   UARTMODE  uart0_set={0};
   
   rcv_new = 0;
   
   uart0_set.datab = 8;                 // 8Î»Êı¾İÎ»
   uart0_set.stopb = 1;                 // 1Î»Í£Ö¹Î»
   uart0_set.parity = 0;                // ÎŞÆæÅ¼Ğ£Ñé
   UART0_Ini(set_baud, uart0_set);        // ³õÊ¼»¯´®¿ÚÄ£Ê½
   
   U0FCR = 0x81;                        // Ê¹ÄÜFIFO£¬²¢ÉèÖÃ´¥·¢µãÎª8×Ö½Ú
   U0IER = 0x01;                        // ÔÊĞíRBRÖĞ¶Ï£¬¼´½ÓÊÕÖĞ¶Ï
   
  
   VICIntSelect = 0x00000000;           // ÉèÖÃËùÓĞÍ¨µÀÎªIRQÖĞ¶Ï
   VICVectCntl1 = 0x26;                 // UART0ÖĞ¶ÏÍ¨µÀ·ÖÅäµ½IRQ slot 0£¬¼´ÓÅÏÈ¼¶×î¸ß
   VICVectAddr1 = (int)IRQ_UART0;       // ÉèÖÃUART0ÏòÁ¿µØÖ·
   VICIntEnable |= 0x00000040;           // Ê¹ÄÜUART0ÖĞ¶Ï

}


//Ã¿¸ôÒ»¶ÎÊ±¼ä£¬·¢ËÍÒ»´Î´¥·¢ĞÅºÅ£¬´¥·¢CPLD¸üĞÂÏÔÊ¾

/****************************************************************************
* Ãû³Æ£ºIRQ_Time0()
* ¹¦ÄÜ£º¶¨Ê±Æ÷0ÖĞ¶Ï·şÎñ³ÌĞò
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
****************************************************************************/
void __irq  IRQ_Time0(void)
{  
   newtimer=1;
   if(net_time05++>=25)
   {					 //0.5ÃëÒç³ö±êÖ¾
	 net_time05=0;
	 net_timeover05=1;	
	}
   if(net_time1++>=50)
   {					 //ÃëÒç³ö±êÖ¾
	 net_time1=0;
	 net_timeover1=1;	
	}  
   if(net_time10++>=500)
   {				 //10ÃëÒç³ö±êÖ¾
	 net_time10=0;
	 net_timeover10=1;	
	}
   if(net_time30++>=1500)
   {
        net_time30=0;
	 net_timeover30=1;
   	}
   T0IR = 0x01;	    			            	// Çå³ıÖĞ¶Ï±êÖ¾
   VICVectAddr = 0x00; // Í¨ÖªVICÖĞ¶Ï´¦Àí½áÊø
}


/****************************************************************************
* Ãû³Æ£ºTime0Init()
* ¹¦ÄÜ£º³õÊ¼»¯¶¨Ê±Æ÷0£¬¶¨Ê±Ê±¼äÎª20mS£¬²¢Ê¹ÄÜÖĞ¶Ï¡£ 
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
****************************************************************************/
void  Time0Init(void)
{   /* Fcclk = Fosc*4 = 11.0592MHz*4 = 44.2368MHz
	   Fpclk = Fcclk/4 = 44.2368MHz/4 = 11.0592MHz
	*/
	T0PR = 99;			    					// ÉèÖÃ¶¨Ê±Æ÷0·ÖÆµÎª100·ÖÆµ£¬µÃ110592Hz
	T0MCR = 0x03;		   						// Æ¥ÅäÍ¨µÀ0Æ¥ÅäÖĞ¶Ï²¢¸´Î»T0TC
	T0MR0 = 2212;	    						// ±È½ÏÖµ(20MS¶¨Ê±Öµ)
	T0TCR = 0x03;		   						// Æô¶¯²¢¸´Î»T0TC
	T0TCR = 0x01; 
	
	/* ÉèÖÃ¶¨Ê±Æ÷0ÖĞ¶ÏIRQ */
	VICIntSelect = 0x00;						// ËùÓĞÖĞ¶ÏÍ¨µÀÉèÖÃÎªIRQÖĞ¶Ï
	VICVectCntl0 = 0x24;						// ¶¨Ê±Æ÷0ÖĞ¶ÏÍ¨µÀ·ÖÅä×î¸ßÓÅÏÈ¼¶(ÏòÁ¿¿ØÖÆÆ÷0)
	VICVectAddr0 = (uint32)IRQ_Time0; 			// ÉèÖÃÖĞ¶Ï·şÎñ³ÌĞòµØÖ·ÏòÁ¿ 
	VICIntEnable = 0x00000010;					// Ê¹ÄÜ¶¨Ê±Æ÷0ÖĞ¶Ï
}


//CRCĞ£Ñé
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
  rec_count=0;//´®¿ÚÖĞ¶Ï ½ÓÊÕ¼ÇÊı
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

void ModbusDeal()//½ÓÊÕµ½´®¿ÚÃüÁî£¬Òª×öµÄ´¦Àí
{
  uint8 aa;

        for(aa=0;aa<rcv_count;aa++)//½«Êı¾İÈ«²¿¸´ÖÆµ½·¢ËÍ»º´æ
        {
           send_buf[aa]=rcv_temp[aa];
        }
        
        crc=CRC16(send_buf, rcv_count);
        if(crc==0)//crcĞ£ÑéÕıÈ·
        {
           switch(send_buf[1])//¹¦ÄÜÂë
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
              ModbusSendError(0x01);//01·Ç·¨¹¦ÄÜÂë
              break;
            }//switch
           
           if(modbus_state.interval>0)//
           modbus_state.count=modbus_state.interval*data4000[31];//×îĞ¡Í¨Ñ¶¼ä¸ô£¬¼ÇÊıÖØĞÂ¸³Öµ
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
			 if(crc==0)//crcĞ£ÑéÕıÈ·
				{

					if(send_buf[0]==ip_address)
						{
							TCP_ModbusDeal();
						}
				}
		
}    

void TCP_ModbusDeal()
{
	switch(send_buf[1])//¹¦ÄÜÂë
		{
              case 0x03:
              	TCP_MultiRd();
				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//×îĞ¡Í¨Ñ¶¼ä¸ô£¬¼ÇÊıÖØĞÂ¸³Öµ
              break;
              case 0x06:
              	TCP_SingleWr();
 				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//×îĞ¡Í¨Ñ¶¼ä¸ô£¬¼ÇÊıÖØĞÂ¸³Öµ
             break;
              case 0x10:
 	            TCP_MultiWr();
 				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//×îĞ¡Í¨Ñ¶¼ä¸ô£¬¼ÇÊıÖØĞÂ¸³Öµ
             break;
              case 0x17:
	            TCP_MultiRdWr();
				if(modbus_state.interval>0)//
                           modbus_state.count=modbus_state.interval*data4000[31];//×îĞ¡Í¨Ñ¶¼ä¸ô£¬¼ÇÊıÖØĞÂ¸³Öµ
              break;
              default:
	              TCP_ModbusSendError(0x01);//01·Ç·¨¹¦ÄÜÂë
              break;
		}//switch
           
}

//¹¦ÄÜÂë03£¬¶àµã¶ÁÈ¡
void TCP_MultiRd()
 {
   uint16 addr;//ÆğÊ¼µØÖ·
   uint16 num;//¶ÁÈ¡µãÊı
   uint16 i;
   uint8 datbuf;
   uint16 addr0;
   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//¼ì²âÄ©µØÖ·
   if(addr0==0)
   {
		TCP_ModbusSendError(0x02);
        return;
   }
   
   addr0=AddrChek(addr);//¼ì²âÊ¼µØÖ·
   if(addr0==0)
   {
		TCP_ModbusSendError(0x02);
        return;
   }

	num=num*2;
   send_buf[2]=num;//¶ÁÈ¡×Ö½ÚÊı
   

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
 
 //¹¦ÄÜÂë06£¬µ¥µãÉèÖÃ
 void TCP_SingleWr()
 {
   uint16 addr;
   uint16 addr0;
 
   addr=send_buf[2]*256+send_buf[3];
 
   addr0=AddrChek(addr);
   if(addr0==0)
   {
     TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
     return;
   }
   
    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	*IPdata=send_buf[4];
	IPdata++;
	*IPdata=send_buf[5];
    
	TCP_ModbusSend(rcv_count2-2);
   
   //¸øÏàÓ¦µÄ×´Ì¬±äÁ¿¸³Öµ   
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
   else if(addr0==0xa640)//ÍøÂçTCP/IPÒÑ¾­¿ÉÒÔÅĞ¶ÏÁ¬½Ó×´¿ö£¬¸ÃÎ»¿ÉÒÔ²»ÓÃ¡£
    {    
       SetModbuslinesign(data0140[1]);
   	    }
   else if(addr0==0x4000)
   {   
	 if(data4000[73])                    //mac¸ü¸Ä
	    {     data4000[73]=0;
		    data4000[75]=0;
		    data4000[77]=0;
		    data4000[79]=0;
		    memcpy(mymac, &data4000[64], 6);
		    ISendStr(CSI1025, 0x40,&data4000[64], 6); 
		    Delay(50000);	
		}
	   else if(data4000[75])           //ÍøÂçIPµØÖ·±ä¸ü
	    {   data4000[75]=0;
	        ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		 Delay(50000);	
	   	}
	   else if(data4000[77])           //»Ö¸´³ö³§ÉèÖÃ
	   {   
	       SetIni_Init(data4000[77]);
		 data4000[77]=0;
	   	}
	    else if(data4000[79])         //Ô¶³ÌÖØÆô¶¯
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
 
 //¹¦ÄÜÂë16£¬¶àµãÉèÖÃ
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
      TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
    }
   
    addr0=AddrChek(addr);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
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

  
    //¸øÏàÓ¦µÄ×´Ì¬±äÁ¿¸³Öµ
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
	  if(data4000[73])					  //mac¸ü¸Ä
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//ÍøÂçIPµØÖ·±ä¸ü
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		        Delay(50000);  
		 }
		else if(data4000[77])			//»Ö¸´³ö³§ÉèÖÃ
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Ô¶³ÌÖØÆô¶¯
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
 
 //¹¦ÄÜÂë23£¬¶àµãÉèÖÃ/¶ÁÈ¡
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
      TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
    }
   
    addr0=AddrChek(addr1);
    if(addr0==0)
    {
      TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
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

   addr0=AddrChek(addr+num-1);//¼ì²âÄ©µØÖ·
   if(addr0==0)
   {
      TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
   }
   
   addr0=AddrChek(addr);//¼ì²âÊ¼µØÖ·
   if(addr0==0)
   {
      TCP_ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
   }

	num=num*2;
   send_buf[2]=num;//¶ÁÈ¡×Ö½ÚÊı
   //»ñÈ¡ĞèÒª¶ÁÈ¡Êı¾İµÄÊ×µØÖ·£¬¶ÁÊı¾İ£¬ËÍÓ¦´ğ

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
	  if(data4000[73])					  //mac¸ü¸Ä
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//ÍøÂçIPµØÖ·±ä¸ü
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//»Ö¸´³ö³§ÉèÖÃ
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Ô¶³ÌÖØÆô¶¯
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
 /*01*/if((0==temp1)||(0==temp3))     //6ºÅ¹ÊÕÏ(Ó²¼ş²»Ö§³Ö)
    	    	{    
		       LEDstate.state.devstate=custom_fault;
		       LEDstate.state.hwfault=no_support;    	    	  
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;    	    	       
    	    	}
 /*02*/ else if((rout_out_data&temp1)&&(rout_out_data&temp2))        //¹ÊÕÏºÅ2(¿ØÖÆ¿¨Ó²¼ş¹ÊÕÏ)
		{			
		       LEDstate.state.devstate=hardware_fault;
		       LEDstate.state.hwfault=2;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;			   
	    	}
 /*03*/  else if((0==(back_in_data&temp3))&&(0==(back_in_data&temp4)))       //¹ÊÕÏºÅ5(Ä£×éÖ¡¹ÊÕÏ)
		 {
		       LEDstate.state.devstate=module_frame_fault;
//		       LEDstate.state.hwfault=0;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;			
	      	}
 /*04*/  else if(((rout_out_data&temp1)&&(back_in_data&temp3))||((rout_out_data&temp2)&&(back_in_data&temp4)))      //¹ÊÕÏºÅ4(Ä£×éÍ¨Ñ¶¹ÊÕÏ)
		 {
		       LEDstate.state.devstate=module_line_fault;
//			LEDstate.state.hwfault=0;
			LEDstate.state.showstate=0;		       
			LEDstate.state.actionstate=0;
		 }
 /*05*/	 else if(((0==(rout_out_data&temp1))&&(0==(back_in_data&temp3)))||((0==(rout_out_data&temp2))&&(0==(back_in_data&temp4))))    //¹ÊÕÏºÅ3(LED¹ÊÕÏ)
		 {
		       LEDstate.state.devstate=LED_fault;			
//		       LEDstate.state.hwfault=0;
//			LEDstate.state.showstate=1;
//			LEDstate.state.actionstate=7;
		  }
 /*06*/	 else if((rout_out_data&temp1)&&(0==(back_in_data&temp3)))          //ºìµÆÕı³£ÏÔÊ¾
		 {
		         LEDstate.state.actionstate=RED;
			  LEDstate.state.showstate=1;
		  }
/*07*/	  else if((rout_out_data&temp2)&&(0==(back_in_data&temp4)))      //ÂÌµÆÕı³£ÏÔÊ¾
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

void TimerDeal(void)//Ã¿¸öÊ±ÖÓ½ÚÅÄ£¬Òª×öµÄ´¦Àí
{     
      uint8 temp1,temp2,temp3;
      uint8 i;
      uint32  ANIN_data;
	  
      ANIN_data=readinput();
      if(ANIN_data!=ANIN)            //ÊÖ¶¯
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
       //ÈôÔÚ¸ø¶¨µÄ×îĞ¡Í¨Ñ¶¼ä¸ôÊ±¼äÄÚÎ´½ÓÊÜµ½ÓĞĞ§Ö¡£¬ÏµÍ³Ó¦×Ô¶¯ºÚÆÁ¡£
      if(modbus_state.interval>0 && modbus_state.count>0)                //Á¬½Ó×´¿ö
       {
         modbus_state.count--;
         if(modbus_state.count==0)
         { 
            if(0==data4000[55])
            {
               SetPowerOff(0);      //×Ü¹ØÆÁ               
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
	
	  if((modbus_state.poweron)&&(0==modbus_state.k_mode))      //×Ô¶¯
	   {     
	          for(i=0;i<modbus_state.signnum;i++)
	          {    
	               temp1=ledlane[i].maxplan-plan[i];
	          	  if(plan[i]==0)
	          	  {    
	          	       temp2=ledlane[i].maxplan-1;              //×îºóÒ»¸ö·½°¸
			       temp3=ledlane[i].action[temp2];     //µÚi³µµÀµÚtemp2¸ö·½°¸ÖĞµÄ¶¯×÷
	   	              if(ledlane[i].sign[temp2])                //µÚi³µµÀµÚtemp2¸ö·½°¸ÖĞµÄ×Ó¶¯×÷ÊÇ·ñ¼¤»î
	   	              {    
	   	                  FArray[i].Function(temp3);	      //µÚi³µµÀµÆÖ´ĞĞ¶¯×÷(0-7¸ö¶¯×÷,ÎŞ¶¯×÷¼ÈÊÇ¿Õ¶¯×÷)
	   	                }
	          	  	}
			    else
			     {  
			         temp3=ledlane[i].action[temp1];     //µÚi³µµÀµÚtemp1¸ö·½°¸ÖĞµÄ¶¯×÷
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
			             if(ledlane[i].sign[temp1])                //µÚi³µµÀµÚtemp1¸ö·½°¸ÖĞµÄ×Ó¶¯×÷ÊÇ·ñ¼¤»î
	   	                     {    
	   	                        FArray[i].Function(temp3);	      //µÚi³µµÀµÆÖ´ĞĞ¶¯×÷(0-7¸ö¶¯×÷,ÎŞ¶¯×÷¼ÈÊÇ¿Õ¶¯×÷)
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
	      else if(data4000[20])                              //ÊÖ¶¯×´¿ö
	 	 {
	 	      leddelay--;
		      if(0==leddelay)
		      	{  SetPowerOff(0);
		      	    SetModbusmode(0);
			    checkstart(data4000[63]);
		      	}
	      	   }			  	
				  
       if(checksign)  
	 checktime--;     //³µµÀ·´À¡
	timetcp--;          //ÍøÂçÁ¬½Ó¼ì²â
	
      if( 0==(ILR&0x01) );	    // µÈ´ıRTCÔöÁ¿ÖĞ¶Ï±êÖ¾
		{
		    ILR = 0x03;				    // Çå³ıÖĞ¶Ï±êÖ¾
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
	       data4000[77]=0;          //Ê§Ğ§
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
//Ö÷³ÌĞò
int main(void)
{
  uint8 i;
  
  BusInit( );

  InitIO();

  ini_prog();
  
  PowerOnInit(); 

    WDTC = 0x00FFFFFF;						// ÉèÖÃWDTC£¬Î¹¹·ÖØ×°Öµ
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
	 while(1);	  	      //Ô¶³ÌÖØÆô
     	}
     uip_len=recv_packet();            //Íø¿¨½ÓÊÕÊı¾İ
     if(uip_len > 0)			    //ÊÕµ½Êı¾İ
      {
			/* ´¦ÀíIPÊı¾İ°ü(Ö»ÓĞĞ£ÑéÍ¨¹ıµÄIP°ü²Å»á±»½ÓÊÕ) */
		if(BUF->type == htons(UIP_ETHTYPE_IP))   //ÊÇIP°üÂğ£¿
		{
			uip_arp_ipin();		   //È¥³ıÒÔÌ«ÍøÍ·½á¹¹£¬¸üĞÂARP±í		
			uip_input();		   //IP°ü´¦Àí
				/*
					µ±ÉÏÃæµÄº¯ÊıÖ´ĞĞºó£¬Èç¹ûĞèÒª·¢ËÍÊı¾İ£¬ÔòÈ«¾Ö±äÁ¿ uip_len > 0
					ĞèÒª·¢ËÍµÄÊı¾İÔÚuip_buf, ³¤¶ÈÊÇuip_len  (ÕâÊÇ2¸öÈ«¾Ö±äÁ¿)
				*/
			if (uip_len > 0)		//ÓĞ´øÍâ»ØÓ¦Êı¾İ
			{
				uip_arp_out();		//¼ÓÒÔÌ«ÍøÍ·½á¹¹£¬ÔÚÖ÷¶¯Á¬½ÓÊ±¿ÉÄÜÒª¹¹ÔìARPÇëÇó
				send_packet();		//·¢ËÍÊı¾İµ½ÒÔÌ«Íø£¨Éè±¸Çı¶¯³ÌĞò£©
				timetcp=(data4000[28]*256+data4000[29])*data4000[30];
			}
		 }
			/* ´¦Àíarp±¨ÎÄ */
		 else if (BUF->type == htons(UIP_ETHTYPE_ARP))	//ÊÇARPÇëÇó°ü
		 {
			uip_arp_arpin();		//ÈçÊÇÊÇARP»ØÓ¦£¬¸üĞÂARP±í£»Èç¹ûÊÇÇëÇó£¬¹¹Ôì»ØÓ¦Êı¾İ°ü
				/*
					µ±ÉÏÃæµÄº¯ÊıÖ´ĞĞºó£¬Èç¹ûĞèÒª·¢ËÍÊı¾İ£¬ÔòÈ«¾Ö±äÁ¿ uip_len > 0
					ĞèÒª·¢ËÍµÄÊı¾İÔÚuip_buf, ³¤¶ÈÊÇuip_len  (ÕâÊÇ2¸öÈ«¾Ö±äÁ¿)
				*/
			if (uip_len > 0)		//ÊÇARPÇëÇó£¬Òª·¢ËÍ»ØÓ¦
			{
				send_packet();		//·¢ARP»ØÓ¦µ½ÒÔÌ«ÍøÉ				
			}
		    }
	       }	  
      else if(net_timeover05==1)			/* 0.5Ãë¶¨Ê±Æ÷³¬Ê± */
	 {	net_time05=0;
		net_timeover05=0;			/* ¸´Î»0.5Ãë¶¨Ê±Æ÷ */	
#ifdef  SHANSHUO
		shan();  
#endif
                        
		/* ÂÖÁ÷´¦ÀíÃ¿¸öTCPÁ¬½Ó, UIP_CONNSÈ±Ê¡ÊÇ10¸ö */
		for(i = 0; i < UIP_CONNS; i++)
		{
			uip_periodic(i);		/* ´¦ÀíTCPÍ¨ĞÅÊÂ¼ş */
			/*
				µ±ÉÏÃæµÄº¯ÊıÖ´ĞĞºó£¬Èç¹ûĞèÒª·¢ËÍÊı¾İ£¬ÔòÈ«¾Ö±äÁ¿ uip_len > 0
				ĞèÒª·¢ËÍµÄÊı¾İÔÚuip_buf, ³¤¶ÈÊÇuip_len  (ÕâÊÇ2¸öÈ«¾Ö±äÁ¿)
			*/
			if(uip_len > 0)
			{
				uip_arp_out();		//¼ÓÒÔÌ«ÍøÍ·½á¹¹£¬ÔÚÖ÷¶¯Á¬½ÓÊ±¿ÉÄÜÒª¹¹ÔìARPÇëÇó
				send_packet();		//·¢ËÍÊı¾İµ½ÒÔÌ«Íø£¨Éè±¸Çı¶¯³ÌĞò£©
			}
		}

	#if UIP_UDP
		/* ÂÖÁ÷´¦ÀíÃ¿¸öUDPÁ¬½Ó, UIP_UDP_CONNSÈ±Ê¡ÊÇ10¸ö */
		for(i = 0; i < UIP_UDP_CONNS; i++)
		{     
			uip_udp_periodic(i);	/*´¦ÀíUDPÍ¨ĞÅÊÂ¼ş */
			/* Èç¹ûÉÏÃæµÄº¯Êıµ÷ÓÃµ¼ÖÂÊı¾İÓ¦¸Ã±»·¢ËÍ³öÈ¥£¬È«¾Ö±äÁ¿uip_lenÉè¶¨Öµ> 0 */
			if(uip_len > 0)
			{
				uip_arp_out();		//¼ÓÒÔÌ«ÍøÍ·½á¹¹£¬ÔÚÖ÷¶¯Á¬½ÓÊ±¿ÉÄÜÒª¹¹ÔìARPÇëÇó
				send_packet();		//·¢ËÍÊı¾İµ½ÒÔÌ«Íø£¨Éè±¸Çı¶¯³ÌĞò£©
			}
		}
	#endif /* UIP_UDP */

		/* Ã¿¸ô10Ãëµ÷ÓÃ1´ÎARP¶¨Ê±Æ÷º¯Êı ÓÃÓÚ¶¨ÆÚARP´¦Àí,ARP±í10Ãë¸üĞÂÒ»´Î£¬¾ÉµÄÌõÄ¿»á±»Å×Æú*/
            if(net_timeover1==1)
		 {    net_time1=0;
		       net_timeover1=0;
#ifdef  SHANSHUO			  
//			shan();         //ÂÌµÆÉÁË¸ÏÔÊ¾5Ãë
#endif
			 WdtFeed();
		       TimerDeal(); 
			 if(data4000[60]&&checksign&&(0==checktime))
                     {    checkstart(data4000[61]);
				check();
                     	}

	             if (net_timeover10==1)
		        {     net_time10=0;
			        net_timeover10=0;		/* ¸´Î»10Ãë¶¨Ê±Æ÷ */				
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
     else if(1==rcv_new)//½ÓÊÕµ½Ò»Ö¡Êı¾İ,Êı¾İ³¤¶ÈÒª¼Ç
     { 
        rcv_new = 0;
        ModbusDeal();
       }
     if(0==timetcp)              //Íø¿¨Ã»ÓĞÁ¬½Óµ½ÍøÂç
	{     
	    init_8019();
	    timetcp=(data4000[26]*256+data4000[27])*data4000[57];
		
	   }
 /*   else if(clocksave)
    	{   clocksave=0;
    	     data4000[77]=0;          //Ê§Ğ§
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


///////////////////MODBUSÊı¾İ½âÎö//////////////////////////////  
void
ModbusSend(uint8 length)//length ²»°üÀ¨CRCÂë
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

//¹¦ÄÜÂë03£¬¶àµã¶ÁÈ¡
void MultiRd()
 {
   uint16 addr;//ÆğÊ¼µØÖ·
   uint16 num;//¶ÁÈ¡µãÊı
   uint16 i;
   uint8 datbuf;
   uint16 addr0;
   addr=send_buf[2]*256+send_buf[3];
   num=send_buf[4]*256+send_buf[5];

   addr0=AddrChek(addr+num-1);//¼ì²âÄ©µØÖ·
   if(addr0==0)
   {
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
   }
   
   addr0=AddrChek(addr);//¼ì²âÊ¼µØÖ·
   if(addr0==0)
   {
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
   }

	num=num*2;
   send_buf[2]=num;//¶ÁÈ¡×Ö½ÚÊı
   //»ñÈ¡ĞèÒª¶ÁÈ¡Êı¾İµÄÊ×µØÖ·£¬¶ÁÊı¾İ£¬ËÍÓ¦´ğ

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
 
 //¹¦ÄÜÂë06£¬µ¥µãÉèÖÃ
 void SingleWr()
 {
   uint16 addr;
   uint16 addr0;
 
   addr=send_buf[2]*256+send_buf[3];
 
   addr0=AddrChek(addr);
   if(addr0==0)
   {
     ModbusSendError(0x02);//02·Ç·¨µØÖ·
     return;
   }
   
    check_store_address(addr0);

	IPdata=IPdata+(addr-addr0)*2;
	*IPdata=send_buf[4];
	IPdata++;
	*IPdata=send_buf[5];
    
   ModbusSend(rcv_count-2);//rcv_bufÔ­Êı¾İ·µ»Ø£¬CRCÂëĞèÒªÖØĞÂ¼ÆËã
   
   //¸øÏàÓ¦µÄ×´Ì¬±äÁ¿¸³Öµ
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
   else if(addr0==0xa640)//ÍøÂçTCP/IPÒÑ¾­¿ÉÒÔÅĞ¶ÏÁ¬½Ó×´¿ö£¬¸ÃÎ»¿ÉÒÔ²»ÓÃ¡£
    {    
       SetModbuslinesign(data0140[1]);
   	    }
   else if(addr0==0x4000)
   {   
	 if(data4000[73])                    //mac¸ü¸Ä
	    {     data4000[73]=0;
		    data4000[75]=0;
		    data4000[77]=0;
		    data4000[79]=0;
		    memcpy(mymac, &data4000[64], 6);
		    ISendStr(CSI1025, 0x40,&data4000[64], 6); 
		    Delay(50000);	
		}
	   else if(data4000[75])           //ÍøÂçIPµØÖ·±ä¸ü
	    {   data4000[75]=0;
	        ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		 Delay(50000);	
	   	}
	   else if(data4000[77])           //»Ö¸´³ö³§ÉèÖÃ
	   {   
	       SetIni_Init(data4000[77]);
		 data4000[77]=0;
	   	}
	    else if(data4000[79])         //Ô¶³ÌÖØÆô¶¯
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
 
 //¹¦ÄÜÂë16£¬¶àµãÉèÖÃ
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
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
    }
   
    addr0=AddrChek(addr);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
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

    ModbusSend(6);//rcv_bufÇ°6¸ö×Ö½Ú·µ»Ø£¬CRCÂëĞèÒªÖØĞÂ¼ÆËã

  
    //¸øÏàÓ¦µÄ×´Ì¬±äÁ¿¸³Öµ
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
	  if(data4000[73])					  //mac¸ü¸Ä
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//ÍøÂçIPµØÖ·±ä¸ü
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//»Ö¸´³ö³§ÉèÖÃ
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Ô¶³ÌÖØÆô¶¯
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
 
 
 //¹¦ÄÜÂë23£¬¶àµãÉèÖÃ/¶ÁÈ¡
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
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
    }
   
    addr0=AddrChek(addr1);
    if(addr0==0)
    {
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
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

   addr0=AddrChek(addr+num-1);//¼ì²âÄ©µØÖ·
   if(addr0==0)
   {
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
   }
   
   addr0=AddrChek(addr);//¼ì²âÊ¼µØÖ·
   if(addr0==0)
   {
      ModbusSendError(0x02);//02·Ç·¨µØÖ·
      return;
   }

	num=num*2;
   send_buf[2]=num;//¶ÁÈ¡×Ö½ÚÊı
   //»ñÈ¡ĞèÒª¶ÁÈ¡Êı¾İµÄÊ×µØÖ·£¬¶ÁÊı¾İ£¬ËÍÓ¦´ğ

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
	  if(data4000[73])					  //mac¸ü¸Ä
		 {	   data4000[73]=0;
			 data4000[75]=0;
			 data4000[77]=0;
			 data4000[79]=0;
			 memcpy(mymac, &data4000[64], 6);
			 ISendStr(CSI1025, 0x40,&data4000[64], 6); 
			 Delay(50000);	 
		 }
		else if(data4000[75])			//ÍøÂçIPµØÖ·±ä¸ü
		 {	 data4000[75]=0;
			 ISendStr(CSI1025, 0x00,&data4000[0], 16); 
		  Delay(50000);  
		 }
		else if(data4000[77])			//»Ö¸´³ö³§ÉèÖÃ
		{	
			SetIni_Init(data4000[77]);
		  data4000[77]=0;
		 }
		 else if(data4000[79])		   //Ô¶³ÌÖØÆô¶¯
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


void InitParam()//³õÊ¼»¯²ÎÊıÇø 7000¿ªÊ¼µÄ
{
 
 //³õÊ¼»¯Í¨ÓÃ¹¦ÄÜÇø
  ini_sram();
  SetModbusInterval(data0100[14]*256+data0100[15]);//×îĞ¡Í¨Ñ¶¼ä¸ô,Ä¬ÈÏÎª600Ãë
  modbus_state.count=modbus_state.interval*data4000[31];
  Settcplinecheck();
  SetPowerOff(0);//±äÁ¿
  
  //¶ÁÊ±¼ä
  time.year=2000 +(data0100[1]/16)*10 + data0100[1]%16;
  time.month=(data0100[2]/16)*10 + data0100[2]%16;
  time.day=(data0100[3]/16)*10 + data0100[3]%16;
  time.hour=(data0100[4]/16)*10 + data0100[4]%16;
  time.minute=(data0100[5]/16)*10 + data0100[5]%16;
  time.second=(data0100[6]/16)*10 + data0100[6]%16;

//±äÁ¿Ğ´ÈëSRAMÇø
 
}


/****************************************************************************
* Ãû³Æ£ºRTCIni()
* ¹¦ÄÜ£º³õÊ¼»¯ÊµÊ±Ê±ÖÓ¡£
* Èë¿Ú²ÎÊı£ºÎŞ
* ³ö¿Ú²ÎÊı£ºÎŞ
****************************************************************************/
void  RTCIni(void)
{  PREINT = Fpclk / 32768 - 1;	// ÉèÖÃ»ù×¼Ê±ÖÓ·ÖÆµÆ÷
   PREFRAC = Fpclk - (Fpclk / 32768) * 32768;
   
    SEC = time.second;
    MIN	=time.minute;
    HOUR=time.hour;
    DOM =time.day;
    MONTH =time.month;			   
    YEAR =time.year;	
   
   CIIR = 0x01;				    // ÉèÖÃÃëÖµµÄÔöÁ¿²úÉúÒ»´ÎÖĞ¶Ï
   
   CCR = 0x01;				    // Æô¶¯RTC
}   


/***************************************************************************************************
*Ãû³Æ£ºInitialization()
*ÃèÊö£º³õÊ¼»¯ÏµÍ³£¬Ö÷Òª¶ÔÈí¼şµÄÅäÖÃ£¬ÈçRAMµÈ
*Èë¿Ú²ÎÊı£ºÎŞ
*³ö¿Ú²ÎÊı£ºÎŞ
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

