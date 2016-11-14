/*
 * cap_touch.h
 *
 *  Created on: 11 Nov 2016
 *      Author: 0xFRED
 */

#ifndef CAP_TOUCH_H_
#define CAP_TOUCH_H_

#include "hal.h"

volatile bool touched;

void initCapTouch(void);
void captureCapTouchMeasurement(void);
bool touchDetected();


#endif /* CAP_TOUCH_H_ */
