/*
 * =====================================================================================
 *
 *       Filename:  Msg.hh
 *
 *    Description:  message format for the msg_t
 *
 *        Version:  1.0
 *        Created:  9/24/2015 3:26:09 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *   Organization:
 *
 * =====================================================================================
 */
#ifndef __MSG_HH__
#define __MSG_HH__
#define MAX_EDGE_NUM 10
#define DOUBLED_MSG_SIZE 100
#define DOUBLED_POINT_PKT_SIZE 26

/*
typedef struct {
    volatile uint8_t ID;
    //volatile uint8_t Seq;
    volatile uint8_t numRising;
    //volatile uint8_t numFalling;
    volatile uint16_t risingEdge[MAX_EDGE_NUM];
    //volatile uint16_t fallingEdge[MAX_EDGE_NUM];
    volatile uint32_t TimerTick;
} Msg_t;*/

#pragma pack(1)
//risingEdge[0]~risingEdge[9]存放10个通道的超声数据
typedef struct {
    volatile uint8_t ID;
    volatile uint8_t numRising;
    volatile uint16_t risingEdge[MAX_EDGE_NUM];
    volatile uint32_t TimerTick;
} Msg_t;



#pragma pack(1)
typedef struct{
    uint8_t ID;
    float x;
    float y;
    float z;
    float offset;
} Point_msg_t;
#pragma pack()

#endif

