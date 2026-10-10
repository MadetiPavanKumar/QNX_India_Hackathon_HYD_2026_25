/*
 * watchdog.h
 *
 *  Created on: Oct 6, 2026
 *      Author: pavan
 */

#ifndef SYSTEM_WATCHDOG_H_
#define SYSTEM_WATCHDOG_H_
#include"../common/messages.h"
int watchdog_start(void);
int watchdog_get_chid(void);
int watchdog_connect(void);
void watchdog_beat(int coid,task_id_t id);

#endif /* SYSTEM_WATCHDOG_H_ */
