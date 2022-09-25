/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/04/30
* Description        : Main program body.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*
*
*
*
*
*
*
*******************************************************************************/

#include <stdbool.h>
#include "ch32v30x.h"
#include <rtthread.h>
#include <rthw.h>
#include "drivers/pin.h"
#include "ch32v30x_usbotg_device.h"

/* Global typedef */

/* Global define */

/* LED0ͨ��rt��pin�����ӿ�����  */
#define LED0_PIN  18   //PC3

#define THREAD_PRIORITY         15
#define THREAD_STACK_SIZE       512
#define THREAD_TIMESLICE        5



#define ADC_DEV_CHANNEL 5
#define REFER_VOLTAGE       330         /* Reference voltage 3.3V, data accuracy multiplied by 100 and reserve 2 decimal places*/
#define CONVERT_BITS        (1 << 12)   /* The number of conversion bits is 12  */
/* Global Variable */

/*********************************************************************
 * @fn      LED1_BLINK_INIT
 *
 * @brief   LED1ͨ��ֱ�ӵ��õײ�����
 *
 * @return  none
 */
void LED1_BLINK_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
u16 Get_ConversionVal1(s16 val);
// typedef struct ReadAdcParam
// {
// //  rt_adc_device_t* adc;
//   rt_sem_t* print_sem;
//
// }ReadAdcParam;
//
 void ADC1_2_IRQHandler(void) __attribute((naked));
 void ADC1_2_IRQHandler_impl(void);
//
// void read_adc(void *parameter);

s16 Calibration_Val1 = 0;
volatile u16 Adc_Val[2];
volatile u32 temp;
volatile bool adc1_avail;
/*********************************************************************
 * @fn      main
 *
 * @brief   mainֻ��һ���߳�֮һ������֮�⻹��tshell,idle
 *          ��mainֻ��һ��LED��˸��main�̵߳�ע����rtthread_startup�У�tshellʹ���˴���
 *          �����жϣ��ж�ջ���߳�ջʹ�÷ֿ�.
 *
 * @return  none
 */
int main(void)
{
 // Delay_Init();
  //RCC_PCLK2Config(RCC_HCLK_Div2);
 
  adc1_avail = false;
    rt_kprintf("\r\n MCU: CH32V307\r\n");
	rt_kprintf(" SysClk: %dHz\r\n",SystemCoreClock);
    rt_kprintf(" www.wch.cn\r\n");
	LED1_BLINK_INIT();

USBOTG_Init( );
//  rt_sem_t print_sem = rt_sem_create("print_sem",1,RT_IPC_FLAG_PRIO);

//Initialize adc



 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  , ENABLE );
 RCC_ADCCLKConfig(RCC_PCLK2_Div8);

RCC_ClocksTypeDef clocks;
RCC_GetClocksFreq(&clocks);

rt_kprintf("SYSCLK_Frequency: %d\n",clocks.SYSCLK_Frequency);
rt_kprintf("ADCCLK_Frequency: %d\n",clocks.ADCCLK_Frequency);


 ADC_InitTypeDef ADC_InitStructure={0};
 GPIO_InitTypeDef GPIO_InitStructure={0};
 NVIC_InitTypeDef NVIC_InitStructure={0};


 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_1;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 ADC_DeInit(ADC1);


 NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 //NVIC_Init(&NVIC_InitStructure);

 ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
 ADC_InitStructure.ADC_ScanConvMode = DISABLE;
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
 ADC_InitStructure.ADC_NbrOfChannel = 2;
 //ADC_InitStructure.ADC_OutputBuffer = ADC_OutputBuffer_Disable;
 //ADC_InitStructure.ADC_Pga = ADC_Pga_1; //Gain configuration


 ADC_Init(ADC1, &ADC_InitStructure);
 //ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5 );
// ADC_ITConfig( ADC1, ADC_IT_EOC, ENABLE);

 ADC_Cmd(ADC1, ENABLE);

 ADC_BufferCmd(ADC1, DISABLE);   //disable buffer
 ADC_ResetCalibration(ADC1);
 while(ADC_GetResetCalibrationStatus(ADC1));
 ADC_StartCalibration(ADC1);
 while(ADC_GetCalibrationStatus(ADC1));
 Calibration_Val1 = Get_CalibrationValue(ADC1);
ADC_BufferCmd(ADC1, ENABLE);   //enable buffer

//ADC_Cmd(ADC1, ENABLE);
// ReadAdcParam rap;
 ////rap.adc = &adc1;

// rap.print_sem = & print_sem;

 //rt_thread_t read_adc_thread;

 //rt_err_t result = rt_thread_init(&read_adc_thread,"radct",read_adc,&rap,THREAD_STACK_SIZE,THREAD_PRIORITY, THREAD_TIMESLICE);


//ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5 );
ADC_SoftwareStartConvCmd(ADC1, ENABLE);


	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	while(1)
	{
    u16 val[2];

    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5 );
    ADC_Cmd(ADC1, ENABLE);
    //ADC_SoftwareStartConvCmd(ADC1, ENABLE);
   while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
   val[0] = ADC_GetConversionValue(ADC1);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5 );
    ADC_Cmd(ADC1, ENABLE);
    //ADC_SoftwareStartConvCmd(ADC1, ENABLE);
   while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));

    val[1] = ADC_GetConversionValue(ADC1);
//   rt_kprintf("\r\nADC1 soft trigger ch2=%d\r\n",val/*Get_ConversionVal1(Adc_Val[0])*/);

   rt_kprintf("\r\nADC1 soft trigger (%d,%d),  \r\n",val[0],val[1]/*Get_ConversionVal1(Adc_Val[0])*/);
    if(adc1_avail == true)
    {

      adc1_avail = false;
      u32 val = ADC_GetConversionValue(ADC1);
      rt_kprintf("\r\nADC1 ch2=%d\r\n",val/*Get_ConversionVal1(Adc_Val[0])*/);
      //ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5 );
  //    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    }
      
      pEP1_IN_DataBuf[0] = (u8)(val[0]);
      pEP1_IN_DataBuf[1] = (u8)(val[0]>>8);
      pEP1_IN_DataBuf[2] = (u8)(val[1]);
      pEP1_IN_DataBuf[3] = (u8)(val[1]>>8);
      pEP1_IN_DataBuf[4] = (u8)0x0F;
     while( USBHD_Endp1_Up_Flag );
    // memcpy((u8*)&pEP1_IN_DataBuf[0],(u8*)&val[0],4);
     DevEP1_IN_Deal( 4 );

     GPIO_SetBits(GPIOA,GPIO_Pin_0);
     rt_thread_mdelay(5);
     GPIO_ResetBits(GPIOA,GPIO_Pin_0);
     rt_thread_mdelay(5);
     //Delay_Ms(1000);

	    // GPIO_SetBits(GPIOA,GPIO_Pin_0);
      //
	    // rt_thread_mdelay(500);
	    // GPIO_ResetBits(GPIOA,GPIO_Pin_0);
      //
	    // rt_thread_mdelay(500);

	}
}

/*********************************************************************
 * @fn      led
 *
 * @brief   ����ʹ�������ӿڲ���I/O��
 *
 * @return  none
 */
int led(void)
{
    rt_uint8_t count;

    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    printf("led_SP:%08x\r\n",__get_SP());
    for(count = 0 ; count < 10 ;count++)
    {
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_kprintf("led on, count : %d\r\n", count);
        rt_thread_mdelay(500);

        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_kprintf("led off\r\n");
        rt_thread_mdelay(500);
    }
    return 0;
}
MSH_CMD_EXPORT(led,  led sample by using I/O drivers);


// void read_adc(void *parameter)
// {
//  ReadAdcParam *rap = (ReadAdcParam*)parameter;
//
//
//   while(1)
//   {
//     rt_uint32_t value;
//
//     // value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL);
//     // float vol = value * REFER_VOLTAGE / CONVERT_BITS;
//     // rt_sem_take(*(rap->print_sem),RT_WAITING_FOREVER);
//     // rt_kprintf("the voltage is :%d.%02d \n", vol / 100, vol % 100);
//     // rt_sem_release(*(rap->print_sem));
//   }
// }


void ADC1_2_IRQHandler()
{
  __asm volatile ("call ADC1_2_IRQHandler_impl; mret");
}

void ADC1_2_IRQHandler_impl()
{

  GET_INT_SP();
  rt_interrupt_enter();

    if(ADC_GetITStatus( ADC1, ADC_IT_EOC)){

        adc1_avail = true;
        //temp=ADC1->RDATAR;
        //Adc_Val[0]=temp&0xffff;
    //    Adc_Val[1]=(temp>>16)&0xffff;
#if 0
        rt_kprintf("\r\nADC1 ch2=%d\r\n",Get_ConversionVal1(Adc_Val[0]));
  //      printf("\r\nADC2 ch2=%d\r\n",Get_ConversionVal2(Adc_Val[1]));
#endif
        ADC_ClearITPendingBit( ADC1, ADC_IT_EOC);
      //  ADC_ClearITPendingBit( ADC2, ADC_IT_EOC);
    }


    rt_interrupt_leave();
    FREE_INT_SP();

}


u16 Get_ConversionVal1(s16 val)
{
	if((val+Calibration_Val1)<0) return 0;
	if((Calibration_Val1+val)>4095) return 4095;
	return (val+Calibration_Val1);
}
