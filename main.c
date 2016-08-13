/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Templates/main.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    18-January-2013
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm324xg_eval.h"
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include <stdlib.h>
//#include <string.h>
#include "main.h"
#include "Msg.h"
#include "arm_math.h"
#include "math.h"
#include "attitudeSolution.c"
#include "disrefine.h"
/*
   using namespace std;
   using std::size_t;
   using namespace techsoft;

   using techsoft::epsilon;
   using techsoft::isVecEq;
   */
//using std::exception;

#define arm_matrix_instance_f32 arm_matrix_instance_f32

/** @addtogroup STM32F4xx_StdPeriph_Templates
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
typedef union uFloatInt{
    uint32_t ui;
    float fl;
} floatInt;

typedef union uMsgBuf{
    Msg_t msg;
    uint8_t buf[sizeof(Msg_t)];
} MsgBuf;

typedef union uPointMsgBuf{
    Point_msg_t msg;
    uint8_t buf[17];
} PointMsgBuf;


/* Private define ------------------------------------------------------------*/
#define TX_BUF_LEN 256
#define RX_BUF_LEN 256
#define DATA_BUFFER_SIZE 512
#define __IO volatile
#define SZ_BUF_LEN 100
#define SMSG_BUF_LEN 50
#define UART_BUF_SIZE 100
#define MAX_MSG_CNT 10
#define RANGE_FILTER_THRESHOLD 40


#if defined (USE_STM324xG_EVAL)
#define MESSAGE1   "     STM32F40xx     "
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "   STM324xG-EVAL    "

#else /* USE_STM324x7I_EVAL */
#define MESSAGE1   "     STM32F427x     "
#define MESSAGE2   " Device running on  "
#define MESSAGE3   "   STM324x7I-EVAL   "
#endif

#define WINDOW_SIZE 7
/* Private macro -------------------------------------------------------------*/
uint8_t range_data[DATA_BUFFER_SIZE];
//CircularBuffer rangeBuffer(DATA_BUFFER_SIZE,range_data);

/* Private variables ---------------------------------------------------------*/
uint8_t FlagRxUart1;
uint8_t FlagRxUart2;
volatile uint8_t RxBufferOfUart1[BUFFER_SIZE];

volatile uint8_t ID,Seq,pktReady = 0,wrongPkt = 0;
volatile uint8_t flushToFlash,wrongCoordPkt = 0,CoordPktReady =0,offsetPktReady =0 ,ADCReady =0;
volatile uint8_t IMUReady;
float distCM,beaconX,beaconY,beaconZ,thisOffset;
volatile uint16_t firstRisingEdge;
/*public valurable-------------------------------------------------------------*/
__IO uint8_t numBeacon ;
__IO uint8_t numBeaconToWrite;
sRecvStage curStage,uart1Stage;
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;
RCC_ClocksTypeDef    RCC_Clocks;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
//volatile uint16_t CCR1_Val = 40961;
/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);
uint8_t lastSeq;

__IO uint16_t rawUSValue[ADC_VALUE_LEN];
float lastPos[2][3] ={{0,0,0},{0,0,0}};
float targetPos[] ={100.0f,100.0f,100.0f};
#define MAX_SPI_BUF_LEN 100
#define MAX_USART_BUF_LEN 40
__IO uint8_t spiBuf[2][MAX_SPI_BUF_LEN];
__IO uint8_t *spiInBuf;
__IO uint8_t *spiOutBuf;
__IO uint16_t spiInBufLen,spiOutBufLen;

__IO uint8_t USARTBuf[2][MAX_USART_BUF_LEN];
__IO uint8_t *USARTInBuf;
__IO uint8_t *USARTOutBuf;
__IO uint8_t USARTInBufLen,USARTOutBufLen;
__IO uint8_t USARTpktReady;
__IO uint8_t badSPI;
__IO floatInt converter;

arm_matrix_instance_f32 xl;
float32_t xlm[3];
float32_t residue[WINDOW_SIZE];
uint32_t v;
uint8_t p;
uint8_t EKFInit = 0;

float32_t threshold = 80; 
float32_t JUMPTHRESHOLD = 40;
uint8_t MOVESIZE = 1; 
uint32_t ttime = 0;
uint32_t meanval1=0,meanval2=0;
UsDisTab_t RecDisList[EFFECTIVE_BEACON_NUM];
float BeaconAzmAng[EFFECTIVE_BEACON_NUM];
float B[20][3];

/* Private functions ---------------------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#endif

#ifdef __SIMULATOR__
#define PUTCHAR_PROTOTYPE int hello(int ch, FILE *f)
#else
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

float calDisFromBeacon(int i, float y[])
{
    float dis = 0.0;
#ifdef DEBUG_LSQ
    printf("i= %d, targetpos=%f %f %f\n\r", i, y[0], y[1], y[2]);
#endif

    dis = sqrt((B[i][0]-y[0])*(B[i][0]-y[0]) + (B[i][1]-y[1])*(B[i][1]-y[1]) + (B[i][2]-y[2])*(B[i][2]-y[2]));
    return dis;
}

int main(void){
 
    STM_EVAL_LEDInit(LED1);
    STM_EVAL_LEDInit(LED2);
    
    SPI_Config();
    SPI_Cmd(EVAL_SPI,ENABLE);
    SPI_I2S_ITConfig(EVAL_SPI,SPI_I2S_IT_RXNE,ENABLE);
    spiInBuf =&(spiBuf[0][0]);
    spiOutBuf =&(spiBuf[1][0]) ;
    
    USART_Config(COM1);
    USART_NVIC_Config(COM1);
    USART_ITConfig(EVAL_COM1,USART_IT_RXNE,ENABLE);
    initTimer();

    STM_EVAL_LEDOn(LED1);
    STM_EVAL_LEDOn(LED2);
    STM_EVAL_LEDToggle(LED1);
    MsgBuf thisMsgBuf;
    badSPI = 0;

    printf("LBS ver 1.0\n\r");
    pktReady = 0;

	memset(&RecDisList,0,sizeof(RecDisList));
	
    /* Infinite loop */
    while (1){
        if(badSPI){
            badSPI = 0;
            SPI_Cmd(EVAL_SPI,DISABLE);
            SPI_Config();
            SPI_Cmd(EVAL_SPI,ENABLE);
            SPI_I2S_ITConfig(EVAL_SPI,SPI_I2S_IT_RXNE,ENABLE);
        }
        if(pktReady){
            pktReady = 0;
            uint16_t i,j;
            j=0;
            for(i=0;i<spiOutBufLen-1;i++){
                if(spiOutBuf[i]==0x7E)
                    continue;
                else if(spiOutBuf[i]==0x7D&&spiOutBuf[i+1]==0x5D){
                    thisMsgBuf.buf[j++] = 0x7D;
                    i++;
                }else if(spiOutBuf[i]==0x7D&&spiOutBuf[i+1]==0x5E){
                    thisMsgBuf.buf[j++] = 0x7E;
                    i++;
                }else{
                    thisMsgBuf.buf[j++] = spiOutBuf[i];
                }
            }
            if(j!=sizeof(Msg_t)){
                printf("wrong spi pkt %u\r\n",j);
                continue;
            }
            //distCM = (thisMsgBuf.msg.risingEdge[0]*34/250)+offset[thisMsgBuf.msg.ID-1];
            distCM = (thisMsgBuf.msg.risingEdge[0]*344/2500);
            ID = thisMsgBuf.msg.ID;
//=======add by TTHR 0803========================
// to mitigate false alarm
		DisStat_t DisStat;
		DisStat=checkNewDisMeasurement(&thisMsgBuf.msg);
		switch(DisStat){
			case INIT:
				printf("### %u, %3.2f I'm cute!!\r\n",ID,distCM);
			case OK:
				//printf("### %u, %3.2f Dis_Measure OK!!\r\n",ID,distCM);
				BeaconAzmAng[0]=getBeaconAzmAng(&thisMsgBuf.msg);	
				upDateDisMeasurement(&thisMsgBuf.msg);
				break;
			case doEKF:
				printf("### %u, %3.2f Before Rectification!!\r\n",ID,distCM);
				//doEKF rectification
				//[coord, Res]=DoEKF(distCM);

				//if(Res<300){
				//	rectifyDisMeasurement(&thisMsgBuf.msg,coord);
				//	}

				upDateDisMeasurement(&thisMsgBuf.msg);
			    break;
			case Bad:

				printf("### %u, %3.2f BAD BAD BAD!!\r\n",ID,distCM);
				break;
			default:
				printf("### %u, %3.2f You Got me!!\r\n",ID,distCM);
				break;
			}
//==============end====================================
 	if(distCM >40 && distCM <1000){
                STM_EVAL_LEDToggle(LED1);
 				}
    }//finish infinit loop
    	}
}

/**
 * @brief  Inserts a delay time.
 * @param  nTime: specifies the delay time length, in milliseconds.
 * @retval None
 */
void Delay(__IO uint32_t nTime) {
    uwTimingDelay = nTime;
    while(uwTimingDelay != 0);
}

/**
 * @brief  Decrements the TimingDelay variable.
 * @param  None
 * @retval None
 */
void TimingDelay_Decrement(void){
    if (uwTimingDelay != 0x00){
        uwTimingDelay--;
    }
}

/** @addtogroup Template_Project
 * @{
 */

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM1, (uint8_t) ch);

    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
    {}

    return ch;
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
