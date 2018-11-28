/***************************************************************************************************
*                    (c) Copyright 2008-2018  Syncretize technologies co.,ltd.
*										All Rights Reserved
*
*\File          app_board.c
*\Description   
*\Note          
*\Log           2018.01.19    Ver 1.0    Job
*               创建文件。
***************************************************************************************************/
#include "app_board.h"

//LED
switch_type MSG_LED = {GPIOB, GPIO_PIN_4, FALSE};

//串口
static GpioType UartTx = {GPIOC, GPIO_PIN_4, GPIO_AF7_USART3};
static GpioType UartRx = {GPIOC, GPIO_PIN_5, GPIO_AF7_USART3};
static u8 UartRxBuf[1024];

UartDevType UartTTL = {USART3, &UartTx, &UartRx, NULL, 115200, UartRxBuf, sizeof(UartRxBuf)};
//LoRa
static switch_type ResetPin = {GPIOC, GPIO_PIN_12, FALSE};
static GpioType  DIO0 = {GPIOC, GPIO_PIN_6};

static switch_type SPI2_CS = {GPIOB, GPIO_PIN_12, FALSE};
static GpioType SPI2_CLK = {GPIOB, GPIO_PIN_13, GPIO_AF5_SPI2};
static GpioType SPI2_MOSI = {GPIOB, GPIO_PIN_15, GPIO_AF5_SPI2};
static GpioType SPI2_MISO = {GPIOB, GPIO_PIN_14, GPIO_AF5_SPI2};

static SpiDevType SPI2_Dev = {SPI2, &SPI2_CS, &SPI2_CLK, &SPI2_MOSI, &SPI2_MISO, \
                        10*1000*1000, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_FIRSTBIT_MSB};

LoRa_Dev_type LoRa_Dev = {NULL, &ResetPin, &SPI2_Dev, &DIO0, 475*1000*1000, 17};


/***************************************************************************************************
*\Function      stm32_board_lowpower_cfg
*\Description   低功耗配置
*\Parameter     
*\Return        void
*\Note          
*\Log           2018.05.25    Ver 1.0    Job               
				创建函数。
***************************************************************************************************/
void stm32_board_lowpower_cfg(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    //以下引脚例外 不配置
    //PA13 PA14  PB3 SWO
    //PH0 PH1 HSE 暂时未使用 可以设置
    //PC14 PC5 LSE

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    //GPIOA
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    //例外
    GPIO_InitStruct.Pin &= (~(GPIO_PIN_13|GPIO_PIN_14));

    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 

    //GPIOB
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    //例外
    GPIO_InitStruct.Pin &= (~(GPIO_PIN_3));

    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 

    //GPIOC
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    //例外
    GPIO_InitStruct.Pin &= (~(GPIO_PIN_14|GPIO_PIN_15));

    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); 

    //GPIOD
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); 

    //GPIOE
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct); 

    //GPIOF
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct); 

    //GPIOG
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct); 

    //GPIOH
    GPIO_InitStruct.Pin   = GPIO_PIN_All;
    GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct); 

    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();
    __HAL_RCC_GPIOC_CLK_DISABLE();
    __HAL_RCC_GPIOD_CLK_DISABLE();
    __HAL_RCC_GPIOE_CLK_DISABLE();
    __HAL_RCC_GPIOF_CLK_DISABLE();
    __HAL_RCC_GPIOG_CLK_DISABLE();
    __HAL_RCC_GPIOH_CLK_DISABLE();
}
/***************************************************************************************************
*\Function      acs_app_board_init
*\Description   
*\Parameter     
*\Return        void
*\Note          
*\Log           2018.01.19    Ver 1.0    Job               
				创建函数。
***************************************************************************************************/
extern  void sensor_gpio_Init(void);
void app_board_init(void)
{
    //全部引脚配置一次
    stm32_board_lowpower_cfg();
	//sensor_gpio_Init();//
    //看门狗先初始化
    watchdog_init();
    //串口 如果使用该串口调试 则优先初始化
//    stm32_uart_init(&UartTTL);
    stm32_switch_init(&MSG_LED);
    GetRestFlag();
    //RTC
    stm32_Inrtc_init(FALSE);   
}


/* Private function prototypes -----------------------------------------------*/
#ifdef USING_USART_PRINTF
int fputc(int ch, FILE *f)
{
    stm32_uart_send(&UartTTL, (u8*)&ch, 1);
    return ch;
}
#endif /* __GNUC__ */
