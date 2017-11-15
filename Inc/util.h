/*
 * util.h
 *
 *  Created on: Nov 14, 2017
 *      Author: Station13
 */
#include "stm32l4xx_hal.h"
#include <stdlib.h>


#ifndef UTIL_H_
#define UTIL_H_


#define MASK_8BIT 0xFF

void fp_to_uint_array(float fp, uint8_t* arr, int* index);
void merge_uint_to_fp(uint8_t* uint_arr, float* fp_arr,int float_arr_size);

#endif /* UTIL_H_ */
