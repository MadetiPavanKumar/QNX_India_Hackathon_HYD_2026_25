/*
 * watchdog.c
 *
 *  Created on: Oct 6, 2026
 *      Author: pavan
 */

#include<stdio.h>
#include<errno.h>
#include<string.h>
#include<sys/neutrino.h>
#include<sys/netmgr.h>
#include"../common/messages.h"
#include"../common/config.h"
#include"../common/log.h"
#include"../system/rt_util.h"
#include"../system/ticker.h"
#include"../motor/motor.h"
#include"../drivers/drv8833.h"

int wd_chid=-1;
int started[NUM_TASKS] = {0};

int watchdog_get_chid(void)
{
	return wd_chid;
}

int watchdog_connect(void){
	return ConnectAttach(ND_LOCAL_NODE,0,wd_chid,_NTO_SIDE_CHANNEL,0);
}
static const char *task_name[NUM_TASKS] = {"SENSOR","NAVIGATION","MOTOR","SAFETY"};

void watchdog_beat(int coid,task_id_t id){
	MsgSendPulse(coid,-1,PULSE_HEARTBEAT,(int)id);
}

static void force_stop_motor(int motor_coid){
	motor_msg_t msg;
	ack_reply_t reply;

	msg.type=MSG_MOTOR_CMD;
	msg.cmd=CMD_STOP;
	msg.speed_percent=0;
    if(msg_send_timeout(motor_coid,&msg,sizeof(msg),&reply,sizeof(reply),MOTOR_TIMEOUT_MS)==-1){
    	 drv8833_emergency_stop();
    }


}

int timeout_for(task_id_t id){
	return (id==TASK_SENSOR) ? SENSOR_HEARTBEAT_TIMEOUT_MS : HEARTBEAT_TIMEOUT_MS;
}

static void *watchdog_thread(void *arg){
	struct _pulse pulse;
	int motor_coid =-1;
	uint64_t last_beat[NUM_TASKS];
	int alive[NUM_TASKS];
	int i;
	(void)arg;

	 if (timer_pulse_start(wd_chid, PULSE_WD_CHECK, WATCHDOG_CHECK_MS) != 0)
	        return NULL;

	for(i=0;i<NUM_TASKS;i++){
		last_beat[i]=now_ms();
		alive[i]=1;
	}
	for(;;){
		int rcvid = MsgReceive(wd_chid,&pulse,sizeof(pulse),NULL);
		if(rcvid==-1)
			continue;
		if(pulse.code==PULSE_HEARTBEAT){
			int id = pulse.value.sival_int;
			if(id<0 || id>=NUM_TASKS)
				continue;
			last_beat[id]=now_ms();
			started[id]=1;
			if(!alive[id]){
				alive[id]=1;
				log_msg("[watchdog] task %s recovered",task_name[id]);
			}
		}else if(pulse.code==PULSE_WD_CHECK){
			uint64_t current_time = now_ms();
			if(motor_coid<0 && motor_controller_get_chid()>=0){
				motor_coid=ConnectAttach(ND_LOCAL_NODE,0,motor_controller_get_chid(),_NTO_SIDE_CHANNEL,0);
			}
			for(i=0;i<NUM_TASKS;i++){
				if(started[i] && alive[i] && (current_time - last_beat[i])>(uint64_t)timeout_for(i)){
					alive[i]=0;
					log_msg("task %s detected as dead",task_name[i]);
					if(motor_coid>=0){
						force_stop_motor(motor_coid);
					}
				}
			}
		}
	}
	return NULL;
}
int watchdog_start(void){
	wd_chid = ChannelCreate(0);
	if(wd_chid==-1){
//		perror("watchdog wd_chid");
		return -1;
	}
	return create_rt_thread(watchdog_thread,NULL,PRIO_WATCHDOG,"watchdog");
}
