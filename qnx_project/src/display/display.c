/*
 * display.c
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */

#include<stdio.h>
#include<errno.h>
#include<sys/neutrino.h>
#include"../system/rt_util.h"
#include"../common/messages.h"
#include"../common/log.h"
#include"../display/display.h"
#include"../drivers/oled_sh1106.h"

int display_chid = -1;
#define BIG_DISPLAY_PAGE 4

int display_get_chid(void){
	return display_chid;
}

static const char *state_text(int state){
	switch(state){
	case STATE_SAFE: return "SAFE";
	case STATE_WARNING: return "WARNING";
	case STATE_EMERGENCY: return "EMERGENCY";
	default : return "FAULT";
	}
}

static void distance_text(char *out,size_t n,int tenths){
	if(tenths==NO_DISPLAY_VALUE){
		snprintf(out,n,"---");
	}
	else{
		snprintf(out,n,"%d.%d",tenths/10,tenths%10);
	}
}

static void draw_screen(int packet){
	char d0[16],d1[16],line[24];

	distance_text(d0,sizeof(d0),packet>>2&0xFFF);
	distance_text(d1,sizeof(d1),packet>>14&0xFFF);
	oled_clear();
	snprintf(line,sizeof(line),"US1 : %s CM",d0);
	oled_print(0,0,line);
	snprintf(line,sizeof(line),"US2 : %s CM",d1);
	oled_print(0,1,line);
	oled_print_big(BIG_DISPLAY_PAGE,state_text(packet&0x03));
	oled_flush();
}

static void *display_thread(void *arg){
	struct _pulse pulse;
	int oled_ok = (oled_init(OLED_I2C_DEV)==0);
	int last_drawn = -1;
	(void) arg;
	if(!oled_ok)
		log_msg("[diplay] not availabe on %s --check connection or i2c dev address",OLED_I2C_DEV);
	for(;;){
		int rcvid = MsgReceive(display_chid,&pulse,sizeof(pulse),NULL);
		if(rcvid!=0 || pulse.code!=PULSE_DISPLAY)
			continue;
		if(!oled_ok)
			continue;
		if(pulse.value.sival_int!=last_drawn){
			last_drawn = pulse.value.sival_int;
			draw_screen(last_drawn);
		}
	}
	return NULL;
}

int display_start(void){
	display_chid = ChannelCreate(0);
	if(display_chid==-1){
		perror("display_chid");
		return -1;
	}
	return create_rt_thread(display_thread,NULL,PRIO_DISPLAY,"display");
}


