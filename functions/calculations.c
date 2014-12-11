#include "stm32f4xx.h"
#include<stdio.h>
#include<math.h>

//defines für moving average
#define alpha 0.1


/******************************************************/
// Moving average

int16_t oldValueGyro[] = {0,0,0};


int16_t MovingAverage (int16_t *oldValue, int16_t newValue) {
	if (*oldValue == 0) {
		*oldValue = newValue;
		return *oldValue;
	} else {
		*oldValue = *oldValue +alpha * (newValue - *oldValue);
		return *oldValue;
	}
}

/******************************************************/
//die 3 raw daten in smoothed umwandeln

void Calc_smoothValues (int16_t raw[], int16_t smoothed[], int size) {
	int i;
	for (i =0; i<size; i++){
		smoothed[i]=MovingAverage(&oldValueGyro[i], raw[i]);
	}
}



double average(double data[], int count)
{
  int i = 0;
  double sum = 0.0;
  for( i = 0 ; i<count ; sum += data[i++])
    ;
  return sum/count;
}

