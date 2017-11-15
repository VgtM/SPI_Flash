/*
 * util.c
 *
 *  Created on: Nov 14, 2017
 *      Author: Station13
 */


#include "util.h"


void fp_to_uint_array(float fp, uint8_t* arr, int* index)
{
	float f = fp;
	uint8_t* intermediate_fp;

	intermediate_fp = (uint8_t*)(&f);

	arr[*index] = intermediate_fp[0];
	*index = *index +1;
	arr[*index] = intermediate_fp[1];
	*index = *index +1;
	arr[*index] = intermediate_fp[2];
	*index = *index +1;
	arr[*index] = intermediate_fp[3];
	*index = *index +1;

	return;
}

void merge_uint_to_fp(uint8_t* uint_arr, float* fp_arr,int float_arr_size)
{
	for(int i  = 0; i<float_arr_size;i++)
	{
		fp_arr[i] = (float)(uint_arr[i*4]<<24 | uint_arr[4*i+1]<<16 | uint_arr[4*i+2]<<8 | uint_arr[4*i+3]);
	}

}
