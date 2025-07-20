//
// Created by GaoKailong on 25-7-18.
//

#ifndef INS_H
#define INS_H

#include "bmi088.h"

typedef struct ins
{
  float yaw,pitch,roll;
}INS;

extern INS ins;
extern BMI088 bmi088;
void testINS(void);

#endif //INS_H
