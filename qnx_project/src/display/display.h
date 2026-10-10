/*
 * display.h
 *
 *  Created on: Oct 5, 2026
 *      Author: pavan
 */

#ifndef DISPLAY_DISPLAY_H_
#define DISPLAY_DISPLAY_H_
#include<stdint.h>
#include"../common/messages.h"
#define NO_DISPLAY_VALUE 4095

int display_start(void);
int display_get_chid(void);

static inline int display_tenths(float cm){
	int t = (int)(cm*10.0f+0.5f);
	if((cm<=0)||t==NO_DISPLAY_VALUE){
		return NO_DISPLAY_VALUE;
	}
	return t;
}
static inline int display_pack(safety_state_t state,float d0_cm,float d1_cm){
	return state | (display_tenths(d0_cm)<<2) | (display_tenths(d1_cm)<<14);
}


#endif /* DISPLAY_DISPLAY_H_ */
