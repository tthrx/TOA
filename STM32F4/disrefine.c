#include "stm324xg_eval.h"

#include "Msg.h"
#include "arm_math.h"
#include "math.h"
#include "disrefine.h"


extern UsDisTab_t RecDisList[EFFECTIVE_BEACON_NUM];


float x,y,z;
//static float R1x=DR0R1,R2x=100,R2y=100;//mm


extern float calDisFromBeacon(int,float*);

DisStat_t checkNewDisMeasurement(Msg_t* pMsg){
		uint16_t val1,val2;
		uint32_t val3;
		int i;
		uint8_t idx=pMsg->ID-1;
		uint8_t DisTimeWidIdx=RecDisList[idx].CurPtr-1;
		uint16_t newDis=(pMsg->risingEdge[0]*344/2500);

		uint32_t newTimeStamp=pMsg->TimerTick;

		if(DisTimeWidIdx>=DISTIMEWINLEN)DisTimeWidIdx=DISTIMEWINLEN-1;// overflow check, CurPtr-1=255 when CurPtr=0; 
		val1=0;val3=0;
		for(i=0;i<RECNUM;i++){
		val1+=abs(newDis-RecDisList[idx].Distance[idx][DisTimeWidIdx]);
			}
		val3=newTimeStamp-RecDisList[idx].TimeStamp[DisTimeWidIdx];
		if(RecDisList[idx].FullTimes>0){
			if(val1>200*3)return Bad;
			if(val1>30*3){ // greater than 1m
				if(val3<5000){ // 250 per sec, 100~0.4sec
					return doEKF;
				}else{
					return OK;
					}
				}else{return OK;}
		}else{
			return INIT;
			}
}

	

void upDateDisMeasurement(Msg_t* pMsg){

		int i;
		uint8_t idx=pMsg->ID-1;
		uint8_t DisTimeWidIdx=RecDisList[idx].CurPtr;
		uint16_t distR[3];
		distR[0]=(pMsg->risingEdge[0]*344/2500);
		distR[1]=(pMsg->risingEdge[1]*344/2500);
		distR[2]=(pMsg->risingEdge[2]*344/2500);
		
		if(pMsg->ID<EFFECTIVE_BEACON_NUM){
			
			RecDisList[idx].BeaconID=idx+1;


			RecDisList[idx].TimeStamp[DisTimeWidIdx]=pMsg->TimerTick;
			
			//Rec1DisList[idx].DisSum-=Rec1DisList[idx].Distance[DisTimeWidIdx];
			//Rec1DisList[idx].DisSum+=distCM;		
			for(i=0;i<RECNUM;i++){
			RecDisList[idx].Distance[i][DisTimeWidIdx]=distR[i];

			//Rec1DisList[idx].TimeWidSum-=Rec1DisList[idx].TimeWid[DisTimeWidIdx];
			//Rec1DisList[idx].TimeWidSum+=pMsg->fallingEdge[0]-pMsg->risingEdge[0];
			//RecDisList[idx].TimeWid[i][DisTimeWidIdx]=pMsg->fallingEdge[i]-pMsg->risingEdge[i];
				}
			//meanval1=Rec1DisList[idx].DisSum/DISTIMEWINLEN;
			//meanval1=abs(meanval1-distCM);//
			//meanval2=Rec1DisList[idx].TimeWidSum/DISTIMEWINLEN;
			//meanval2=abs(meanval2-Rec1DisList[idx].TimeWid[DisTimeWidIdx]);//
			DisTimeWidIdx++;			
			if(DisTimeWidIdx>=DISTIMEWINLEN){
				DisTimeWidIdx=0;
				RecDisList[idx].FullTimes++;
				}
			RecDisList[idx].CurPtr=DisTimeWidIdx;				
			}
		
		//printf("###Dis=%3.2f,meanDis=%d,meanTW=%d\n",distCM,meanval1,meanval2);
}

void rectifyDisMeasurement(Msg_t* pMsg,float* y){

		uint8_t i=0;
		uint8_t Curidx;
		uint32_t val1;
		uint8_t DisTimeWidIdx;
		uint16_t Dist;
		
		
		for(i=0;i<EFFECTIVE_BEACON_NUM;i++){
			if(i==pMsg->ID-1)continue;
			DisTimeWidIdx=RecDisList[i].CurPtr-1;
			if(DisTimeWidIdx>=DISTIMEWINLEN)DisTimeWidIdx=DISTIMEWINLEN-1;// overflow check, CurPtr-1=255 when CurPtr=0;
			val1=abs(pMsg->TimerTick-RecDisList[i].TimeStamp[DisTimeWidIdx]);
			if(val1>5000){
				Curidx=RecDisList[i].CurPtr;
				RecDisList[i].TimeStamp[Curidx]=pMsg->TimerTick;

				//Dist=calDisFromBeacon(i, y);
				//RecDisList[i].Distance=Dist;
				//printf("### %u, %3.2f After Rectification!!\r\n",i+1,Dist);

				
				RecDisList[i].CurPtr++;
				if(RecDisList[i].CurPtr>=DISTIMEWINLEN){
				RecDisList[i].CurPtr=0;
				}
			}
		}	
}

float getBeaconAzmAng(Msg_t* pMsg){

	float val1,CosAzmAng;
	
	float distR0=(pMsg->risingEdge[0]*344/2500);
	float distR1=(pMsg->risingEdge[1]*344/2500);
	float distR2=(pMsg->risingEdge[2]*344/2500);

	float sqR0=distR0*distR0;
	float sqR1=distR1*distR1;
	float sqR2=distR2*distR2;

	x=sqR0-sqR1+R1x*R1x;
	x=x/R1x/2;
	y=sqR0-sqR2+R2y*R2y;
	y=y/R2y/2;
        printf("###x=%.2f##y=%.2f",x,y);
	y=sqrt(x*x+y*y);
        printf("###R=%.2f",y);
	if(y<0.001){y=1;}
	else{
	y=x/y;
		}
        printf("##costheta=%.2f\r\n",y);
	return y;

}


















/*
uint32_t getTimeWidfromDis(float Dis){

		
		float y;
		y=-0.007*Dis*Dis+1.6104*Dis+3138.6;
		
		return (uint32_t) y;
}
*/
