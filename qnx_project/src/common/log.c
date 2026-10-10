/*
 * log.c
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */


#include<stdio.h>
#include<stdarg.h>
#include<pthread.h>
#include"../system/rt_util.h"
#include"../common/log.h"

static pthread_mutex_t log_lock;
static uint64_t start_ms;

void log_init(void){
	pthread_mutexattr_t attr;
	pthread_mutexattr_init(&attr);
	pthread_mutexattr_setprotocol(&attr,PTHREAD_PRIO_INHERIT);
	pthread_mutex_init(&log_lock,&attr);
	pthread_mutexattr_destroy(&attr);

	start_ms = now_ms();
}

void log_msg(const char *fmt,...){
	char buf[256];
	va_list ap;

	va_start(ap,fmt);
	vsnprintf(buf,sizeof(buf),fmt,ap);
	va_end(ap);

	pthread_mutex_lock(&log_lock);
	printf("[%7llu ms] %s\n",(unsigned long long)(now_ms()-start_ms),buf);
	pthread_mutex_unlock(&log_lock);
}

