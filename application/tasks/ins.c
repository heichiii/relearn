//
// Created by GaoKailong on 25-7-18.
//

#include "ins.h"
#include "MahonyAHRS.h"
INS ins;
BMI088 bmi088 = {0};
void testINS(void)
{
	// initBMI088(&bmi088);
	updateBMI088(&bmi088);
}

