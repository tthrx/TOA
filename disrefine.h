#ifndef _DISREFINE_H
#define _DISREFINE_H

#include "Msg.h"


#define R0x 0  //mm
#define R0y 0  //mm
#define R1x 27.2  //mm
#define R1y 0  //mm
#define R2x 0  //mm
#define R2y 25.2  //mm



#define DISTIMEWINLEN  6
#define RECNUM 3

#define EFFECTIVE_BEACON_NUM 9


typedef enum{
	    OK=1,
		doEKF=2,
		Bad=3,
		INIT=4	
}DisStat_t;


typedef struct {
    uint8_t BeaconID;
	volatile uint32_t TimeStamp[DISTIMEWINLEN];
	volatile uint16_t Distance[RECNUM][DISTIMEWINLEN];
	volatile uint16_t DisSum;
    volatile uint16_t TimeWid[RECNUM][DISTIMEWINLEN];
	volatile uint16_t TimeWidSum;
	float BeaconAzmAng[EFFECTIVE_BEACON_NUM];// the cosine value of Azmuth Angle which is defined as the angle between Rec0-Beacon to the line that Rec0 and Rec1 hold. 
	volatile uint8_t  CurPtr;
	volatile uint16_t FullTimes;
} UsDisTab_t;



//public fun

void upDateDisMeasurement(Msg_t*);
DisStat_t checkNewDisMeasurement(Msg_t*);







//uint32_t getTimeWidfromDis(float Dis);







#endif