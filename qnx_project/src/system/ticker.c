/*
 * ticker.c
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */
#include<stdio.h>
#include<time.h>
#include<sys/neutrino.h>
#include<sys/netmgr.h>
#include"../system/ticker.h"

int timer_pulse_start(int chid,int pulse_code,int period_ms){
	struct itimerspec its;
	struct sigevent ev;
	int coid;
	timer_t tid;

	coid = ConnectAttach(ND_LOCAL_NODE,0,chid,_NTO_SIDE_CHANNEL,0);
	if(coid==-1){
		perror("ticker:connectAttach");
		return -1;
	}
	SIGEV_PULSE_INIT(&ev,coid,SIGEV_PULSE_PRIO_INHERIT,pulse_code,0);

	if(timer_create(CLOCK_MONOTONIC,&ev,&tid)==-1){
		perror("ticker: timer_create");
		return -1;
	}
	its.it_value.tv_sec=period_ms/1000;
	its.it_value.tv_nsec=(long)(period_ms % 1000)*1000000L;
	its.it_interval = its.it_value;

	if(timer_settime(tid,0,&its,NULL)==-1)
	{
		perror("clicker:timerSettime");
		return -1;
	}
	return 0;
}



