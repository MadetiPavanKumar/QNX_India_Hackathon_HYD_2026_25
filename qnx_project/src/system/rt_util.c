/*
 * rt_util.c
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */

#include<stdio.h>
#include<time.h>
#include<string.h>
#include<errno.h>
#include<sched.h>
#include<pthread.h>
#include<sys/neutrino.h>
#include"../system/rt_util.h"

uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* now_ms: same clock, milliseconds. */
uint64_t now_ms(void)
{
    return now_us() / 1000ULL;
}

int create_rt_thread(void* (*fn) (void*),void *arg, int priority,const char *name){
	pthread_attr_t attr;
	struct sched_param param;
	pthread_t tid;
	int rc;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
	pthread_attr_setschedpolicy(&attr,SCHED_FIFO);
	memset(&param,0,sizeof(param));
	param.sched_priority=priority;
	pthread_attr_setschedparam(&attr,&param);
	pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);

	rc = pthread_create(&tid,&attr,fn,arg);
	pthread_attr_destroy(&attr);

	if(rc!=0){
		fprintf(stderr,"warning %s SCHED_FIFO prio %d failed with error (%s)\n",name,priority,strerror(rc));
		pthread_attr_init(&attr);
		pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
		rc = pthread_create(&tid,&attr,fn,arg);
		if(rc!=0){
			fprintf(stderr,"warning %s pthread_create_failed with default attributes and priority %s\n",name,strerror(rc));
			return -1;
		}
	}
	pthread_setname_np(tid,name);
	return 0;
}

int msg_send_timeout(int coid,const void *smsg,int sbytes,void *rmsg,int rbytes,int timeout_ms){
	struct sigevent ev;
	uint64_t timeout_ns = (uint64_t)timeout_ms*1000000ULL;
	SIGEV_UNBLOCK_INIT(&ev);
	TimerTimeout(CLOCK_MONOTONIC,_NTO_TIMEOUT_SEND | _NTO_TIMEOUT_REPLY,&ev,&timeout_ns,NULL);
	return MsgSend(coid,smsg,sbytes,rmsg,rbytes);
}
