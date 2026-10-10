/*
 * rt_util.h
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */

#ifndef SYSTEM_RT_UTIL_H_
#define SYSTEM_RT_UTIL_H_
#include<stdint.h>
#include<pthread.h>

uint64_t now_ms(void);

uint64_t now_us(void);

int create_rt_thread(void* (*fn)(void*),void *arg,int priority,const char *name);

int msg_send_timeout(int coid,const void *smsg, int sbytes, void *rmsg, int rbytes,int timeout_ms); //for sending msg with a time out fumction even if a reciver or send stucks then the timeout will automatically unblocks the receiver/sender


#endif /* SYSTEM_RT_UTIL_H_ */
