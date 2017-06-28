/*
Copyright (c) 2017, BigCat Wireless Pvt Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.



THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <complex.h>
#include <math.h>

#include "goertzel_spectrum.h"
#include "goertzel_capture.h"
#include "goertzel_firmware.h"

int DebugLevel=DEBUG_LEVEL_ERROR;
float goertzel_function (void* goertzel_dsp_handler)
{
	int y0_goertzel_real_value,y0_goertzel_imag_value,y1_goertzel_real_value,y1_goertzel_imag_value;
	double complex y0_goertzel_complex;
	double complex y1_goertzel_complex;
	double complex data_out;
	float data_out_abs;
	//int val;

	pdevice_handler goertzel_dsp_handler_ptr = (pdevice_handler)goertzel_dsp_handler;

	pgoertzel_data_structure goertzel_data_structure_ptr = goertzel_dsp_handler_ptr->private_data;

#ifdef GOERTZEL_STANDALONE
	//Memory streaming interface streaming trigger
	*(goertzel_data_structure_ptr->mem2st_addr) = 2;
	*(goertzel_data_structure_ptr->mem2st_addr) = 3;
#endif

	//Configuration of b_coefficient value
	*(goertzel_data_structure_ptr->goertzel_module_addr + 0x8) = goertzel_data_structure_ptr->b_coeff;

#ifdef GOERTZEL_STANDALONE
	// Configuration for cpri rfp
	*(goertzel_data_structure_ptr->goertzel_streaming_addr + 0x1) = 6;

	//Goertzel capture module input stream capture
	*(goertzel_data_structure_ptr->goertzel_streaming_addr) = 0;
	*(goertzel_data_structure_ptr->goertzel_streaming_addr) = 1;
	usleep(500);
#endif

	*(goertzel_data_structure_ptr->goertzel_streaming_addr) = 1;

	//Goertzel spectrum dsp module flush
	*(goertzel_data_structure_ptr->goertzel_module_addr + 0x4) = 1;
	*(goertzel_data_structure_ptr->goertzel_module_addr + 0x4) = 0;

	//Goertzel capture module output pattern stream start
	*(goertzel_data_structure_ptr->goertzel_streaming_addr) = 3;
	usleep(500);

	y0_goertzel_real_value = *(goertzel_data_structure_ptr->goertzel_module_addr + 0xC);

	y0_goertzel_imag_value = *(goertzel_data_structure_ptr->goertzel_module_addr + 0x10);

	y1_goertzel_real_value = *(goertzel_data_structure_ptr->goertzel_module_addr + 0x14);

	y1_goertzel_imag_value = *(goertzel_data_structure_ptr->goertzel_module_addr + 0x18);

	DEBUG_PRINT(DEBUG_LEVEL_INFO,("y0 real value: %d\n",y0_goertzel_real_value));
	DEBUG_PRINT(DEBUG_LEVEL_INFO,("y0 imag value: %d\n",y0_goertzel_imag_value));
	DEBUG_PRINT(DEBUG_LEVEL_INFO,("y1 real value: %d\n",y1_goertzel_real_value));
	DEBUG_PRINT(DEBUG_LEVEL_INFO,("y1 imag value: %d\n",y1_goertzel_imag_value));

	y0_goertzel_complex = (((float)y0_goertzel_real_value)/pow(2,16)) + ((((float)y0_goertzel_imag_value)/pow(2,16)) * I);
	y1_goertzel_complex = (((float)y1_goertzel_real_value)/pow(2,16)) + ((((float)y1_goertzel_imag_value)/pow(2,16)) * I);

	DEBUG_PRINT(DEBUG_LEVEL_INFO,("The y0 complex value = %.2f %+.2fi\n", creal(y0_goertzel_complex), cimag(y0_goertzel_complex)));
	DEBUG_PRINT(DEBUG_LEVEL_INFO,("The y1 complex value = %.2f %+.2fi\n", creal(y1_goertzel_complex), cimag(y1_goertzel_complex)));

	data_out = (y0_goertzel_complex) - (y1_goertzel_complex*goertzel_data_structure_ptr->w1_coeff);

	data_out_abs = (float)(sqrt(pow(creal(data_out),2) + pow(cimag(data_out),2)));

	DEBUG_PRINT(DEBUG_LEVEL_INFO,("Output abs value: %f\n",data_out_abs));

	return data_out_abs;
}

void goertzel_measurement_capture(void* goertzel_dsp_handler,void* goertzel_capture_handler,float *result_ptr,unsigned int iteration_count)
{
	int i,j;
	pdevice_handler goertzel_dsp_handler_ptr = (pdevice_handler)goertzel_dsp_handler;
	pdevice_handler goertzel_capture_handler_ptr = (pdevice_handler)goertzel_capture_handler;
	pgoertzel_data_structure goertzel_data_structure_ptr;
	double output_array[8];
	pgoertzel_coeff_structure calculated_coeff;
	//int retval;

	if (goertzel_dsp_handler_ptr->private_data == NULL)
	{
		goertzel_dsp_handler_ptr->private_data = (pgoertzel_data_structure)malloc(sizeof(goertzel_data_structure));
	}

	goertzel_data_structure_ptr = goertzel_dsp_handler_ptr->private_data;

	goertzel_data_structure_ptr->goertzel_streaming_addr = (unsigned int *)goertzel_capture_handler_ptr->reg_remap_addr[0];
	goertzel_data_structure_ptr->goertzel_module_addr = (unsigned int *)goertzel_dsp_handler_ptr->reg_remap_addr[0];

	calculated_coeff = malloc(sizeof(goertzel_coeff_structure));

	for(j=0;j<ACLR_POINTS;j++)
	{
		output_array[j]=0.0;
	}

	for(i=0;i<iteration_count;i++)
	{
		sleep(goertzel_data_structure_ptr->iteration_time_int);

#ifndef GOERTZEL_STANDALONE
		//Goertzel capture module input stream capture
		*(goertzel_data_structure_ptr->goertzel_streaming_addr) = 0;
		*(goertzel_data_structure_ptr->goertzel_streaming_addr) = 1;
#endif

		for(j=0;j<ACLR_POINTS;j++)
		{
			switch(j){
			case 0:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 1:
			    goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 2:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 3:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 4:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 5:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 6:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			case 7:
				goertzel_parameter_generate(goertzel_dsp_handler_ptr,j,calculated_coeff);
				goertzel_data_structure_ptr->b_coeff = calculated_coeff->b_coeff;
				goertzel_data_structure_ptr->w1_coeff = calculated_coeff->w1_coeff;
				break;
			}

			result_ptr[(i*iteration_count) + j] = goertzel_function(goertzel_dsp_handler_ptr);
		}

	}

	//Calculating the output goertzel components
	for(j=0;j<ACLR_POINTS;j++)
	{
		for(i=0;i<iteration_count;i++)
		{
			output_array[j] = output_array[j] + (double)result_ptr[(i*iteration_count) + j];
		}

		output_array[j]=output_array[j]/iteration_count;
		printf("Frequency Component value %d : %lf\n",j,output_array[j]);
	}

	free(calculated_coeff);
}

int goertzel_parameter_generate(void* goertzel_dsp_handler,int iteration,pgoertzel_coeff_structure calculated_coeff)
{
	pdevice_handler goertzel_handler_ptr = (pdevice_handler)goertzel_dsp_handler;
	pgoertzel_data_structure goertzel_data_structure_ptr = goertzel_handler_ptr->private_data;
	double freq_bin_size;
	double freq_comp = 0;
	double freq_bin;


	freq_bin_size = ((double)goertzel_data_structure_ptr->sampling_freq/(double)goertzel_data_structure_ptr->bin_len);

	switch (iteration) {
	case 0:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)-(((double)goertzel_data_structure_ptr->bw_freq)/2.0)-((double)goertzel_data_structure_ptr->aclr1);
		break;
	case 1:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)-(((double)goertzel_data_structure_ptr->bw_freq)/2.0)-((double)goertzel_data_structure_ptr->aclr2);
		break;
	case 2:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)-(3*(((double)goertzel_data_structure_ptr->bw_freq)/8.0));
		break;
	case 3:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)-(((double)goertzel_data_structure_ptr->bw_freq)/8.0);
		break;
	case 4:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)+(((double)goertzel_data_structure_ptr->bw_freq)/8.0);
		break;
	case 5:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)+(3*(((double)goertzel_data_structure_ptr->bw_freq)/8.0));
		break;
	case 6:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)+(((double)goertzel_data_structure_ptr->bw_freq)/2.0)+((double)goertzel_data_structure_ptr->aclr2);
		break;
	case 7:
		freq_comp=((double)goertzel_data_structure_ptr->centre_freq)+(((double)goertzel_data_structure_ptr->bw_freq)/2.0)+((double)goertzel_data_structure_ptr->aclr1);
		break;
	}

	if(freq_comp<0) {
		freq_bin = ((double)goertzel_data_structure_ptr->bin_len) - (double)(fabs(freq_comp/freq_bin_size));
	}
	else {
		freq_bin = (double)(freq_comp/freq_bin_size);
	}

	freq_bin=roundf(freq_bin);

	goertzel_coeff_calculate(goertzel_handler_ptr,freq_bin,calculated_coeff);

	return 0;
}

int goertzel_coeff_calculate(void* goertzel_handler_ptr,double freq_bin,pgoertzel_coeff_structure calculated_coeff)
{
	pdevice_handler pdev = (pdevice_handler)goertzel_handler_ptr;
	pgoertzel_data_structure goertzel_data_structure_ptr = pdev->private_data;
	double pi_term;
	double complex w_coeff,w1_coeff;
	int b_coeff;

	pi_term = (double)((2*M_PI*freq_bin)/(double)goertzel_data_structure_ptr->bin_len);
	w_coeff = cos(pi_term) + sin(pi_term)*I;
	w1_coeff = (cos(pi_term)) + (-1*sin(pi_term))*I;
	b_coeff = (int)(round((2*creal(w_coeff))* pow(2.0,13.0)));

	calculated_coeff->b_coeff = b_coeff;
	calculated_coeff->w1_coeff = w1_coeff;

	return 0;
}

void goertzel_config_param(pdevice_handler goertzel_dsp_handler,float fc,float fbw,float fs,float aclr1,float aclr2,int iteration_time_interval)
{
	pgoertzel_data_structure goertzel_data_structure_ptr;

	if (goertzel_dsp_handler->private_data == NULL)
	{
		goertzel_dsp_handler->private_data = (pgoertzel_data_structure) malloc(sizeof(goertzel_data_structure));
	}

	goertzel_data_structure_ptr = goertzel_dsp_handler->private_data;

	goertzel_data_structure_ptr->iteration_time_int = iteration_time_interval;

	goertzel_data_structure_ptr->bin_len = 4096;
	goertzel_data_structure_ptr->centre_freq = fc;
	goertzel_data_structure_ptr->bw_freq = fbw;
	goertzel_data_structure_ptr->sampling_freq = fs;
	goertzel_data_structure_ptr->aclr1 = aclr1;
	goertzel_data_structure_ptr->aclr2 = aclr2;
}

int main() {



	pdevice_handler goertzel_capture_handler;
	pdevice_handler goertzel_spectrum_handler;
	int instance_num = 0;
	float fc,fbw,fs,aclr1,aclr2;
	int iteration_time_interval;
	goertzel_capture_handler = hepta_a10_2x4_goertzel_capture_get_handler (instance_num);
	goertzel_spectrum_handler = hepta_a10_2x4_goertzel_spectrum_get_handler (instance_num);
	float *data_ptr;
	data_ptr = (float*)malloc(sizeof(float)*ACLR_ITERATION_COUNT*ACLR_POINTS);
	// Configure input values
	printf("Enter iteration time interval: ");
		scanf("%d",&iteration_time_interval);

		printf("Enter center frequency: ");
		scanf("%f",&fc);

		printf("Enter bw frequency: ");
		scanf("%f",&fbw);

		printf("Enter sampling frequency: ");
		scanf("%f",&fs);

		printf("Enter aclr1: ");
		scanf("%f",&aclr1);

		printf("Enter aclr2: ");
		scanf("%f",&aclr2);

		printf("Configuring Goertzel module parameters\n");
	goertzel_config_param(goertzel_spectrum_handler,fc,fbw,fs,aclr1,aclr2,iteration_time_interval);
	goertzel_measurement_capture(goertzel_spectrum_handler,goertzel_capture_handler,data_ptr,ACLR_ITERATION_COUNT);
return 0;
}


