/**
******************************************************************************
* @file    FLASH/FLASH_EraseProgram/Src/main.c
* @author  MCD Application Team
* @version V1.5.0
* @date    29-April-2016
* @brief   This example provides a description of how to erase and program the
*          STM32L4xx FLASH.
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iap_if.h"
#include "flash_if.h"
#include "app_board.h"
#include "usdl_calc.h"
#include "drv_watchdog.h"
#include "lora_pp.h"
#include "low_power.h"
#include "ota_protocol.h"
#include "zlib.h"

#define LOWPOWER_MODE
typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;
iap_updata_param_type IapUpdataParam;//IAP升级时的各种信息
extern u8 OTACommBuf[];
extern  u16 OTACommBufLen;
//extern  u16 OTASourceId;
extern  iap_param_type iap_param;

LoRa_PP_type LoRa_PP;//LoRa协议

static uint8_t check_buf[FLASH_PAGE_SIZE];//检验缓冲区
extern void system_reset(void);
extern void iap_param_send_lora(void);
/**
* @brief  System Clock Configuration
*         The system Clock is configured as follows :
*            System Clock source            = PLL (MSI)
*            SYSCLK(Hz)                     = 80000000
*            HCLK(Hz)                       = 80000000
*            AHB Prescaler                  = 1
*            APB1 Prescaler                 = 1
*            APB2 Prescaler                 = 1
*            MSI Frequency(Hz)              = 4000000
*            PLL_M                          = 1
*            PLL_N                          = 40
*            PLL_R                          = 2
*            PLL_P                          = 7
*            PLL_Q                          = 4
*            Flash Latency(WS)              = 4
* @param  None
* @retval None
*/
//#define SYSTEM_CLOCK_24M
#define SYSTEM_CLOCK_48M
//#define SYSTEM_CLOCK_80M
void SystemClock_Config(void)
{
#ifdef SYSTEM_CLOCK_80M

    //配置为80M时钟 MSI
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* MSI is enabled after System reset, activate PLL with MSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;//24:48M 40:80M
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLP = 7;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
    clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
#endif
#ifdef SYSTEM_CLOCK_48M

    //配置为48M时钟 MSI
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};

    /* MSI is enabled after System reset, activate PLL with MSI as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 24;//40;//24:48M 40:80M
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLP = 7;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
    clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
#endif
#ifdef SYSTEM_CLOCK_24M

  //系统时钟为24M
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* MSI is enabled after System reset, update MSI to 24Mhz (RCC_MSIRANGE_9) */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_9;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
     while(1);
  }
  
  /* Select MSI as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI; 
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
     while(1);
  }

#endif
}
/***************************************************************************************************
*\Function      stm32_flash_RDP
*\Description   读保护，加密后必须断电重启一次
*\Parameter     
*\Return        void
*\Note          
*\Log           2017.01.09    Ver 1.0    Job               
创建函数。
***************************************************************************************************/
void stm32_flash_RDP(void)
{
    FLASH_OBProgramInitTypeDef OBInit;

    HAL_FLASHEx_OBGetConfig(&OBInit);
    if (OBInit.RDPLevel != OB_RDP_LEVEL_0)
    {
        return;
    }
	/* Allow Access to Flash control registers and user Falsh */
    HAL_FLASH_Unlock();
    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();

    /* Enable FLASH_WRP_SECTORS write protection */
    OBInit.OptionType = OPTIONBYTE_RDP;
    OBInit.RDPLevel = OB_RDP_LEVEL_1;
    HAL_FLASHEx_OBProgram(&OBInit);

    /* Start the Option Bytes programming process */
    if (HAL_FLASH_OB_Launch() != HAL_OK)
    {

    }

    /* Prevent Access to option bytes sector */
    HAL_FLASH_OB_Lock();

    /* Disable the Flash option control register access (recommended to protect
    the option Bytes against possible unwanted operations) */
    HAL_FLASH_Lock();
}

void LoRaRecv_CallBack(void* p, u16 src_addr, u8* buf, u16 len, s16 rssi, s8 snr)
{
	iap_param.src_addr=src_addr;
	iap_param.lora_rssi=rssi;//
	iap_param.lora_snr=snr;//	
		
	memcpy(OTACommBuf, buf, len);
	OTACommBufLen = len;
}
/*************************************************************************************************
*\Function      main
*\Description   主函数入口
*\Parameter     
*\Return        int
*\Note          
*\Log           2018.01.18    Ver 1.0    Job               
				创建函数。
***************************************************************************************************/
int main(void)
{
    int i = 0;
    uint32_t addr = 0;
    uint64_t bin_size = 0;
    uint64_t check_code = 0;
    uint32_t page_num = 0;//页数
    uint32_t page_left = 0;//整数页剩余的字节
    CRC32_CTX CRC32;
    uint32_t Count = 0;
	u32 count = 0;
	uint32_t ComSizeCnt = 0, UncomSizeCnt = 0, ComSizeCntTmp = 0, UncomSizeCntTmp = 0;//解压前大小和解压后大小,解压前每包大小临时变量，解压后数据每包大小临时变量
	int err;
    /* STM32L4xx HAL library initialization:
    - Configure the Flash prefetch
    - Systick timer is configured by default as source of time base, but user 
    can eventually implement his proper time base source (a general purpose 
    timer for example or other time source), keeping in mind that Time base 
    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
    handled in milliseconds basis.
    - Set NVIC Group Priority to 4
    - Low Level Initialization
    */   
    HAL_Init();
    /* Configure the system clock to 80 MHz */
    SystemClock_Config(); 

	//读保护
    stm32_flash_RDP();
    //读取标志
    IAP_FLAG = ReadIAPMode();
//     printf("load app...\n");
    if (IAP_FLAG == OTA_RUN_APP)
    {
RunAPP:
        /* Check if valid stack address (RAM address) then jump to user application */
        if (((*(__IO uint32_t*)FLASH_APP_RUN_ADDR) & 0x2FFE0000 ) == 0x20000000)
        {
//            printf("load app...\n");
            /* Jump to user application */
            JumpAddress = *(__IO uint32_t*) (FLASH_APP_RUN_ADDR + 4);
            Jump_To_Application = (pFunction) JumpAddress;
            /* Initialize user application's Stack Pointer */
            __set_MSP(*(__IO uint32_t*) FLASH_APP_RUN_ADDR);
            Jump_To_Application();
        }
        else
        {   /* Otherwise, do nothing */
            //跳转地址不合法，警告
//            printf("\n\r Addr is invalid! Jump IAP...\n\r");
            ForceUpdateAppMode();//下次直接进入IAP模式
            goto OTAoE;
        }
    }
    else if (IAP_FLAG == OTA_Update_APP)
    {
		//读取IAP升级信息
		Flash_Read(FLASH_APP_INFO_ADDR, (uint8_t*)&IapUpdataParam, sizeof(IapUpdataParam));
			
		//先校验升级包是否正确（不管是不是压缩数据）
        //计算文件的校验
        page_num = IapUpdataParam.UpdatesTotalSize / sizeof(check_buf);
        page_left = IapUpdataParam.UpdatesTotalSize % sizeof(check_buf);
        if (IapUpdataParam.UpdatesTotalSize > OTA_BIN_MAX_SIZE)
        {
            //文件错误
            //更改标记
            RunAppMode();
            goto RunAPP;
        }
								
        CRC32_Init(&CRC32);
        for (i = 0; i < page_num; i++)
        {
            Flash_Read(FLASH_APP_COPY_ADDR + i * sizeof(check_buf), check_buf, sizeof(check_buf));
            CRC32_Update(&CRC32, check_buf, sizeof(check_buf));
        }
        if (page_left)
        {
            Flash_Read(FLASH_APP_COPY_ADDR + page_num * sizeof(check_buf), check_buf, page_left);
            CRC32_Update(&CRC32, check_buf, page_left);
        }
        //计算完成 判断校验是否正确
        if (IapUpdataParam.UpdatesCRC32 != CRC32_Final(&CRC32))
        {
            //校验错误 还是跳转到原来的程序启动
            //更改标记
            RunAppMode();			
            goto RunAPP;
        }
				
		//判断是否是压缩数据
		if(IapUpdataParam.CompressFlag == 1)//如果是压缩数据，需要边解压边写入Falsh数据
		{
			uint8_t *pSrcData = (uint8_t *)FLASH_APP_COPY_ADDR;
			Flash_Unlock();
			Count = 0;
					
			//循环写入直接全部写入正确为止
			while (1)
			{
				Count++;
				if (Count > 3)
				{
					//失败次数3次 退出升级
					Flash_Lock();
					//写入3次失败
					ForceUpdateAppMode();
					goto OTAoE;
				}
				//校验正确 开始copy
				//先擦除 运行区程序
				ComSizeCnt = 0, UncomSizeCnt = 0;
				Flash_Erase_Page(FLASH_APP_RUN_ADDR, IapUpdataParam.ComPackOriginalSize);
              
				for (; ComSizeCnt < IapUpdataParam.UpdatesTotalSize;)//边解压边存储
				{
					ComSizeCntTmp = IapUpdataParam.PerComPackOriginalSize * 2;
					UncomSizeCntTmp = IapUpdataParam.PerComPackOriginalSize * 2;							
				    err = uncompress(check_buf, &UncomSizeCntTmp, pSrcData, &ComSizeCntTmp);									
					FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
					Flash_Write(FLASH_APP_RUN_ADDR + UncomSizeCnt, check_buf, UncomSizeCntTmp);
					//读取一次确认 最后确认是否写入正确
//                	Flash_Read(FLASH_APP_RUN_ADDR + UncomSizeCnt, check_buf, UncomSizeCntTmp);
                
					pSrcData += ComSizeCntTmp;
					ComSizeCnt += ComSizeCntTmp;
					UncomSizeCnt += UncomSizeCntTmp;
				} 
				pSrcData = (uint8_t *)FLASH_APP_RUN_ADDR;
				CRC32_Init(&CRC32);
				CRC32_Update(&CRC32, pSrcData, UncomSizeCnt);
				if((IapUpdataParam.ComPackOriginalCRC32 == CRC32_Final(&CRC32)) && (IapUpdataParam.ComPackOriginalSize == UncomSizeCnt))
				{
					Flash_Lock();
					//写入正确 进入跳转
					RunAppMode();
					system_reset();
//								  NVIC_SystemReset();
					goto RunAPP;
				}
			}
		}
		else
		{
			//解锁
			Flash_Unlock();			
			Count = 0;
			//循环写入直接全部写入正确为止
			while (1)
			{
				Count++;
				if (Count > 3)
				{
					//失败次数3次 退出升级
					Flash_Lock();
					//写入3次失败
					ForceUpdateAppMode();
					goto OTAoE;
				}
				//校验正确 开始copy
				//先擦除 运行区程序
				Flash_Erase_Page(FLASH_APP_RUN_ADDR, IapUpdataParam.UpdatesTotalSize);
				CRC32_Init(&CRC32);
				for (i = 0; i < page_num; i++)
				{
					FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
					Flash_Read(FLASH_APP_COPY_ADDR + i * sizeof(check_buf), check_buf, sizeof(check_buf));
					Flash_Write(FLASH_APP_RUN_ADDR + i * sizeof(check_buf), check_buf, sizeof(check_buf));
					//读取一次确认 最后确认是否写入正确
					Flash_Read(FLASH_APP_RUN_ADDR + i * sizeof(check_buf), check_buf, sizeof(check_buf));
					CRC32_Update(&CRC32, check_buf, sizeof(check_buf));
				}
				if (page_left)
				{
					FLASH_WaitForLastOperation((uint32_t)FLASH_TIMEOUT_VALUE);
					Flash_Read(FLASH_APP_COPY_ADDR + i * sizeof(check_buf), check_buf, page_left);
					Flash_Write(FLASH_APP_RUN_ADDR + i * sizeof(check_buf), check_buf, page_left);
					//读取剩余
					Flash_Read(FLASH_APP_RUN_ADDR + i * sizeof(check_buf), check_buf, page_left);
					CRC32_Update(&CRC32, check_buf, page_left);
				}
				if (IapUpdataParam.UpdatesCRC32 == CRC32_Final(&CRC32))
				{
					Flash_Lock();
					//写入正确 进入跳转
					RunAppMode();
					system_reset();
//								  NVIC_SystemReset();
					goto RunAPP;
				}
			}
		}       
    }
    /* enter in IAP mode */
    else
    {
OTAoE:
		//HAL_DBGMCU_EnableDBGStopMode();		
        app_board_init();
		iap_param_load();
		while (count++ < 4)
		{
			Watchdog_Refresh(); 
			HAL_Delay(300);//小心看门狗
			stm32_switch_evert(&MSG_LED);
		}	 
        stm32_switch_off(&MSG_LED);
		/* Ensure that MSI is wake-up system clock */ 
		__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);
		stm32_Inrtc_EnWakeup(4*1000);
		if(iap_param.ID==0)
		{
			iap_param.ID=0x1234;
		}
#ifdef LOWPOWER_MODE		
		LoRa_Init(&LoRa_PP, &LoRa_Dev, iap_param.Channel*1000*1000, iap_param.ID, 60 * 1000, FALSE, stm32_lowpower_exit);		
#else
		LoRa_Init(&LoRa_PP, &LoRa_Dev, iap_param.Channel*1000*1000, iap_param.ID, 60 * 1000, FALSE, NULL);
#endif
		LoRa_Recv_CallBack_Set(&LoRa_PP, LoRaRecv_CallBack, NULL);
		LoRa_Start(&LoRa_PP);			
//LoRa_Stop(&LoRa_PP);
		iap_param_send_lora();//上电发送lora iap参数

        while (1)
        {			
            //喂狗任务
            Watchdog_Refresh();
            //只要不是在升级 则检测每10分钟重启一次
            if (OTA_UpdateFlag == 0)
            {
                //重启任务
                AutoReset_Periodic_Handle();
            } 
            //协议解析
            if (OTACommBufLen)
            {
                OTA_Periodic_Handle();
            }
#ifdef LOWPOWER_MODE           
			stm32_lowpower_enter();			
			stm32_lowpower_exit();
#endif			
        }
    }
    return 0;

}

//系统断电重启
void system_reset(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
		/*Configure GPIO pins :PC0*/
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);	//CPU断电
	HAL_Delay(3000);//延时3000ms	如果执行到下面，则可能是因为电容太大 则会执行软重启
	HAL_NVIC_SystemReset();//系统重启	  
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t *file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
