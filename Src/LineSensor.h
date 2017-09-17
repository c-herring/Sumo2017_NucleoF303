/*
 * LineSensor.h
 *
 *  Created on: Sep 16, 2017
 *      Author: HeZ
 */

#ifndef LINESENSOR_H_
#define LINESENSOR_H_


class LineSensor {
public:
	LineSensor();
	virtual ~LineSensor();

private:
	uint32_t stopWatch;
	uint8_t pollState;
	uint8_t sensorState;


};

#endif /* LINESENSOR_H_ */
