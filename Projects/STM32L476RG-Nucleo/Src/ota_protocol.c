/***************************************************************************************************
*                    (c) Copyright 2008-2018  Syncretize technologies co.,ltd.
*										All Rights Reserved
*
*\File          ota_protocol.c
*\Description   
*\Note          
*\Log           2018.01.19    Ver 1.0    Job
*               �����ļ���
***************************************************************************************************/
#include "ota_protocol.h"
#include "flash_if.h"
#include "iap_if.h"
#include "app_board.h"
//#include "lora.h"
#include "iap_if.h"
//OTAЭ���־
u8 OTA_UpdateFlag = 0;// 0 δ������1 ��ʼ��2 �����У�3 �������

u16 OTA_Version = 0;//�汾
u64 OTA_TotalSize = 0;//�����ܳ��ȣ�ѹ����ʱ��ѹ������ܳ��ȣ�δѹ��ʱ��BIN�ļ����ܳ���
u32 OTA_SubSize = 0;//�ְ��̶���
u64 OTA_CRC32 = 0;//У��

u16 OTA_Frame_Num = 0;//���������

//����Э��ʹ��
u8 OTACommBuf[2048];
u8 OTASendBuf[1024];
u16 OTACommBufLen = 0;//���յ����ݵĳ��ȣ��ڻص������и�ֵ���ڽ�������������
//u16 OTASourceId = 0;//�յ����ݵ�ID
 
static int OTAWaitTicks;

extern LoRa_PP_type LoRa_PP;


uint32_t iap_param_read(uint8_t* buf, uint32_t len)
{
	u32 data_len;
	int i=0;
	//��ѯ��ǰID �ŵ��Ƿ��flash�浵��һ�£�һ����ʼͨ�ţ���һ�¿�ʼ����flash
	data_len=29;
	if (len < data_len)
    {
        return 0;
    }

	buf[i++] = COMM_PROTOCOL_HEAD;
	buf[i++] = (data_len)%256;
	buf[i++] = (data_len)/256;
	buf[i++] = FUC_PARAM_IAP_READ;//������	
	buf[i++] = iap_param.ID%256;
	buf[i++] = iap_param.ID/256;
	buf[i++] = (iap_param.Channel-410);
	buf[i++] = iap_param.GatewayID%256;
	buf[i++] = iap_param.GatewayID/256;	
	
	buf[i++] = iap_param.ChipUniqueID[0]&0xFF;
	buf[i++] = (iap_param.ChipUniqueID[0]>>8)&0xFF;
	buf[i++] = iap_param.ChipUniqueID[1]&0xFF;
	buf[i++] = (iap_param.ChipUniqueID[1]>>8)&0xFF;	
	buf[i++] = iap_param.ChipUniqueID[2]&0xFF;
	buf[i++] = (iap_param.ChipUniqueID[2]>>8)&0xFF;	
	buf[i++] = iap_param.ChipUniqueID[3]&0xFF;
	buf[i++] = (iap_param.ChipUniqueID[3]>>8)&0xFF;	
	buf[i++] = iap_param.ChipUniqueID[4]&0xFF;
	buf[i++] = (iap_param.ChipUniqueID[4]>>8)&0xFF;
	buf[i++] = iap_param.ChipUniqueID[5]&0xFF;
	buf[i++] = (iap_param.ChipUniqueID[5]>>8)&0xFF;
	buf[i++] =(u8)iap_param.lora_rssi;//iap_param.Lora_fourth;
	buf[i++] = ((u8)(iap_param.lora_rssi>>8));//iap_param.Lora_six;
	buf[i++] = iap_param.IAP_flag;
	buf[i++] = (u8)iap_param.lora_snr;
	buf[i++] = 0x00;
	buf[i++] = iap_param.version;
	buf[i++] = Get_Sum8(buf + 1, data_len - 3);
	buf[i++] = COMM_PROTOCOL_TAIL;
	return i;
}


void iap_param_analysis(u8* buf)
{
	 u16 data = 0;

	data =  buf[5];
	data = (data << 8) | buf[4];
	iap_param.ID=data;
	iap_param.Channel=buf[6]+410;
	data =  buf[8];
	data = (data << 8) | buf[7];
	iap_param.GatewayID=data;
//	iap_param.Lora_fourth=buf[21];
//	iap_param.Lora_six=buf[22];		

}


void iap_param_send_lora(void)
{
	u32 send_len = 0;
	send_len = iap_param_read(OTASendBuf, sizeof(OTASendBuf));		
	if (send_len)
	{
		LoRa_Send(&LoRa_PP, iap_param.src_addr, OTASendBuf, send_len);  
	}
}

///***************************************************************************************************
//*\Function      Get_Sum8
//*\Description   У���
//*\Parameter     buf
//*\Parameter     len
//*\Return        u8
//*\Note          
//*\Log           2018.01.19    Ver 1.0    Job
//*               ����������
//***************************************************************************************************/
//u8 Get_Sum8(u8* buf, u32 len)
//{
//    u8 sum = 0;
//    u32 i = 0;

//    for (i = 0; i < len; i++)
//    {
//        sum += buf[i];
//    }

//    return sum;
//}

///***************************************************************************************************
//*\Function      ArrayU8ToU16
//*\Description   
//*\Parameter     src
//*\Return        u16
//*\Note          
//*\Log           2018.01.19    Ver 1.0    Job
//*               ����������
//***************************************************************************************************/
//u16 ArrayU8ToU16(void* src)
//{
//    return (*(u8*)src) + ((*((u8*)src+1))<<8);
//}
///***************************************************************************************************
//*\Function      ArrayU8ToU32
//*\Description   
//*\Parameter     src
//*\Return        uint32_t
//*\Note          
//*\Log           2018.01.19    Ver 1.0    Job
//*               ����������
//***************************************************************************************************/
//u32 ArrayU8ToU32(void* src)
//{
//    return ((u32)*(u8*)src) + (((u32)*((u8*)src+1))<<8) +
//           (((u32)*((u8*)src+2))<<16) + (((u32)*((u8*)src+3))<<24);
//}


/***************************************************************************************************
*\Function      OTA_Start_ACK_Construct
*\Description   OTA��ʼACK����
*\Parameter     buf
*\Parameter     len
*\Return        u32
*\Note          
*\Log           2018.01.19    Ver 1.0    Job
*               ����������
***************************************************************************************************/
u32 OTA_Start_ACK_Construct(u8* buf, u32 len)
{
    u32 data_len = 8;
    u32 i = 0;

    if (len < data_len)
    {
        return 0;
    }
    buf[i++] = COMM_PROTOCOL_HEAD;
    buf[i++] = (u8)(data_len);
    buf[i++] = (u8)((data_len) >> 8);
    buf[i++] = OTA_Start;
    buf[i++] = iap_param.src_addr/256;//ID
    buf[i++] = iap_param.src_addr%256;//ID
    buf[i++] = Get_Sum8(buf + 1, data_len - 3);
    buf[i++] = COMM_PROTOCOL_TAIL;

    return i;
}

/***************************************************************************************************
*\Function      OTA_Update_ACK_Construct
*\Description   ���������ACK
*\Parameter     buf
*\Parameter     len
*\Return        u32
*\Note          
*\Log           2018.01.19    Ver 1.0    Job
*               ����������
***************************************************************************************************/
u32 OTA_Update_ACK_Construct(u8* buf, u32 len, u8 state_code)
{
    u32 data_len = 11;
    u32 i = 0;

    if (len < data_len)
    {
        return 0;
    }
    buf[i++] = COMM_PROTOCOL_HEAD;
    buf[i++] = (u8)(data_len);
    buf[i++] = (u8)((data_len) >> 8);
    buf[i++] = OTA_Update;
    buf[i++] = iap_param.src_addr/256;//ID
    buf[i++] = iap_param.src_addr%256;//ID

    buf[i++] = (u8)OTA_Frame_Num;
    buf[i++] = (u8)(OTA_Frame_Num >> 8);

    buf[i++] = state_code;

    buf[i++] = Get_Sum8(buf + 1, data_len - 3);
    buf[i++] = COMM_PROTOCOL_TAIL;

    return i;
}
/***************************************************************************************************
*\Function      OTA_End_ACK_Construct
*\Description   
*\Parameter     buf
*\Parameter     len
*\Return        u32
*\Note          
*\Log           2018.01.19    Ver 1.0    Job
*               ����������
***************************************************************************************************/
u32 OTA_End_ACK_Construct(u8* buf, u32 len)
{
    u32 data_len = 8;
    u32 i = 0;

    if (len < data_len)
    {
        return 0;
    }
    buf[i++] = COMM_PROTOCOL_HEAD;
    buf[i++] = (u8)(data_len);
    buf[i++] = (u8)((data_len) >> 8);
    buf[i++] = OTA_End;
    buf[i++] = iap_param.src_addr/256;//ID
    buf[i++] = iap_param.src_addr%256;//ID
    buf[i++] = Get_Sum8(buf + 1, data_len - 3);
    buf[i++] = COMM_PROTOCOL_TAIL;

    return i;
}
/***************************************************************************************************
*\Function      OTA_Protocol_Anylise
*\Description   
*\Parameter     buf
*\Parameter     len
*\Return        del_info_type
*\Note          
*\Log           2018.01.19    Ver 1.0    Job               
				����������
***************************************************************************************************/
extern void system_reset(void);
del_info_type OTA_Protocol_Anylise(u8* buf, u32 len)
{
    del_info_type info = {anylise_waiting, 0};
    u32 i = 0;
    u32 data_len = 0;
    u16 dev_ID = 0;
    u32 updata_len = 0;//������С������
    u32 send_len = 0;

    if (len >= COMM_PROTOCOL_MIN_LEN)
    {
        /*��ʼ����*/
        i = 0;
        while (1)
        {
            /*ͬ��ͷ��β����ȷ����У��Ͳ���ȷ*/
            if ((buf[i] !=  COMM_PROTOCOL_HEAD))
            {
                for (i += 1; i < len; i++)
                {
                    if (buf[i] == COMM_PROTOCOL_HEAD)
                    {
                        break;
                    }
                }
                if (len - i >=  COMM_PROTOCOL_MIN_LEN)
                {
                    continue;
                }
                else
                {
                    info.state = anylise_reject;
                    info.del_len = i;
                    return info;
                }      
            }
            else
            {
                /*�������ݰ�����*/
                data_len = buf[i + 2];
                data_len = (data_len << 8) + buf[i + 1];
                 //���ȺϷ��ж� Ŀǰ��������8-1200
                if (data_len > COMM_PROTOCOL_MAX_LEN || data_len < COMM_PROTOCOL_MIN_LEN)
                {
                    i++;
                    continue;
                }
                /*����������ݳ���С��֡���ɳ��� �����ȴ�*/
                if (len - i < data_len)
                {
                    /*i == 0��ʾ��ȷ��֡ͷǰ��û�д������� ���صȴ�,����ɾ�����������*/
                    if (i == 0)
                    {
                        info.state = anylise_waiting;
                        info.del_len = 0;
                    }
                    else
                    {
                        info.state = anylise_reject;
                        info.del_len = i;
                    }
                    return info;
                }
                /*���֡β,֡β���Ϲ��� ����Ϊ����һ������������֡*/
                if (buf[i + data_len - 1] == COMM_PROTOCOL_TAIL && 
                    buf[i + data_len - 2] == Get_Sum8(buf + i + 1, data_len - 3))
                {
                    break;
                }
                else//֡βУ�鲻���� ɾ��һ���ֽ� ��������
                {
                    i += 1;
                    continue;
                }
            }
        }
        //��֤ͨ��������Э�鴦��
        info.state = anylise_accept;//��Ԥ��Ϊ����
        //��ɸѡ��ID ����
        dev_ID = buf[i + 5];
        dev_ID = (dev_ID << 8) | buf[i + 4];
        switch (buf[i + 3])
        {
        case OTA_Start:
			      HAL_Delay(20);
            //�����汾 ���� �ְ��� ��У��
            OTA_UpdateFlag = 1;

            OTA_Version = buf[i+6];
            OTA_Version = (OTA_Version << 8)|buf[i+7];

            OTA_TotalSize = ArrayU8ToU32(buf + i + 10);

            OTA_SubSize = ArrayU8ToU16(buf + i + 14);

            OTA_CRC32 = ArrayU8ToU32(buf + i + 16);
            //�洢bin����+crc32
            Flash_Storage_Header(buf + i + 8);
            //������д����
            Flash_Erase_Page(FLASH_APP_COPY_ADDR, OTA_TotalSize);
            //�ظ�ACK
            send_len = OTA_Start_ACK_Construct(OTASendBuf, sizeof(OTASendBuf));
            if (send_len)
            {
                LoRa_Send(&LoRa_PP, iap_param.src_addr, OTASendBuf, send_len);
            }
            break;
        case OTA_Update:
            OTA_UpdateFlag = 2;
            //������ ����
            OTA_Frame_Num = ArrayU8ToU16(buf + i + 6);
            
            if (data_len > 10 && OTA_Frame_Num * OTA_SubSize < OTA_BIN_MAX_SIZE)
            {
                //updata_lenӦ����8�ı���
                updata_len = data_len - 10;
                Flash_Write(FLASH_APP_COPY_ADDR + (OTA_Frame_Num) * OTA_SubSize, buf + i + 8, updata_len);
                //ack
                send_len = OTA_Update_ACK_Construct(OTASendBuf, sizeof(OTASendBuf), 0);
                if (send_len)
                {
									LoRa_Send(&LoRa_PP, iap_param.src_addr, OTASendBuf, send_len);
                }
            }
            else
            {
                send_len = OTA_Update_ACK_Construct(OTASendBuf, sizeof(OTASendBuf), 1);
                if (send_len)
                {
									LoRa_Send(&LoRa_PP, iap_param.src_addr, OTASendBuf, send_len);
                }
            }
            
            break;
        case OTA_End:
            //������� ��������
            OTA_UpdateFlag = 3;
            //�ظ�ACK
            send_len = OTA_End_ACK_Construct(OTASendBuf, sizeof(OTASendBuf));
            if (send_len)
            {
                LoRa_Send(&LoRa_PP, iap_param.src_addr, OTASendBuf, send_len);
								HAL_Delay(2000);
            }
            UpdateAppMode();
//            NVIC_SystemReset();
			system_reset();
            break;
			case FUC_PARAM_IAP_READ:
					send_len = iap_param_read(OTASendBuf, sizeof(OTASendBuf));		
					if (send_len)
					{
						LoRa_Send(&LoRa_PP, iap_param.src_addr, OTASendBuf, send_len);  
					}
			break;	
			case FUC_PARAM_IAP_WRITE://�޷��� ��Ҫ����
					iap_param_analysis(buf);
				break;					
			case FUC_PARAMSET_SAVE:
				iap_param_save();
				break;
			case FUC_PARAMSET_RESET:
				HAL_Delay(10);
//				NVIC_SystemReset();
				system_reset();
				break;
			default:
				//printf("the cmd[%d] is no support!\n", buf[i + 4]);
				break;				
				
        }

        info.del_len = data_len + i; 
        return info;
    }
    else
    {
        return info;
    } 
}
/***************************************************************************************************
*\Function      OTA_Periodic_Handle
*\Description   Э�鴦���߳�
*\Parameter     
*\Return        void
*\Note          
*\Log           2018.01.19    Ver 1.0    Job               
				����������
***************************************************************************************************/
void OTA_Periodic_Handle(void)
{
    u32 len = 0;
    del_info_type info = {anylise_waiting, 0};
    int tick_diff;

    if (OTACommBufLen == 0)
    {
        OTAWaitTicks = HAL_GetTick();
        return;
    }
    
		len = OTACommBufLen;
		OTACommBufLen = 0;

    info = OTA_Protocol_Anylise(OTACommBuf, len);

//    if (info.state != anylise_waiting)
//    {
//        //ɾ��
//        stm32_uart_del(&UartTTL, info.del_len);
//        OTAWaitTicks = HAL_GetTick();
//    }
//    else
//    {
//        tick_diff = HAL_GetTick() - OTAWaitTicks;
//        if (Abs(tick_diff) > 5*1000)//����5S�������ɹ�������ջ��������ݣ����߱���һ�㣬ɾ��һ���ֽڣ�������������Ƚ�����
//        {
//            stm32_uart_clear(&UartTTL);
//            OTAWaitTicks = HAL_GetTick();
//        } 
//    }
}
