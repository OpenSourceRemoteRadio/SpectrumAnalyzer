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
/**
 * @file hepta_a10_2x4_goertzel_spectrum.h
 * @defgroup hepta_a10_2x4_goertzel_spectrum HEPTA A10 2X4 GOERTZEL SPECTRUM
 * @brief Header file for goertzel_spectrum module
 * */

#ifndef HEPTA_A10_2X4_GOERTZEL_SPECTRUM_H_
#define HEPTA_A10_2X4_GOERTZEL_SPECTRUM_H_

#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************
 * Include public/global Header files
******************************************************************************/

#include <stdio.h>		
#define _GNU_SOURCE 
#define __USE_GNU	 
#include <stdlib.h>      
#include <string.h>      
#include <unistd.h>      
#include <sys/types.h>   
#include <sys/socket.h>  
#include <sys/select.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>   
#include <netdb.h>       
#include <signal.h>      
#include <unistd.h>      
#include <stdio.h>       
#include <stdlib.h>      
#include <string.h>      
#include <unistd.h>      
#include <sys/types.h>   
#include <sys/socket.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>   
#include <sys/mman.h>    
#include <netdb.h>       
#include <sys/stat.h>    
#include <fcntl.h>       
#include <pthread.h>     
#include <sys/ioctl.h>   
#include <stdbool.h>
#include <complex.h>

/******************************************************************************
 * Include private Header files
******************************************************************************/

//#include "geortzel_protocol.h"
#include <goertzel_firmware.h>
//#include "geortzel_command_format.h"

#define hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX 0

#define hepta_a10_2x4_goertzel_spectrum_CONFIG_COUNT 1
#define hepta_a10_2x4_goertzel_spectrum_SERVICE_COUNT 1

#define ACLR_POINTS	8
#define ACLR_ITERATION_COUNT 30
#define OUTPUT_VALUES 12

/******************************************************************************
 * Enumeration definitions
******************************************************************************/


/**
 * @ingroup hepta_a10_2x4_downlink
 * @brief downlink config command enumeration
 * */


typedef enum
{
    Hepta_A10_2x4_goertzel_spectrum_Config_Param=0,    				/**< Write DPD LUT values and switches LUT memory selection*/
}Hepta_A10_2x4_goertzel_spectrum_Config_Enum_Def;



/**
 * @ingroup hepta_a10_2x4_downlink
 * @brief downlink service command enumeration
 * */


typedef enum
{
    Hepta_A10_2x4_goertzel_spectrum_Mag_Measure =0,    /**< CPRI Link 0 Input Cariier Power is zero for 10ms*/
}Hepta_A10_2x4_goertzel_spectrum_Service_Enum_Def;


/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief flush goertzel module
 * */


typedef enum
{
    GOERTZEL_SPECTRUM_start_goertzel_module_start=1,    /**< start*/
    GOERTZEL_SPECTRUM_start_goertzel_module_stop=0,    /**< stop*/
}GOERTZEL_SPECTRUM_goertzel_flush_start_goertzel_module_Enum_Def;


/******************************************************************************
 * Register bit map definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Definition of goertzel_flush
 * */


typedef union
{
    struct
   {
        unsigned int start_goertzel_module : 1;	/**< flush goertzel module*/
        unsigned int Resrvd1 : 31;	
   }reg;
    unsigned int value;
}goertzel_spectrum_goertzel_flush_Def,*pgoertzel_spectrum_goertzel_flush_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Definition of freq_bin
 * */


typedef union
{
    struct
   {
        unsigned int data : 32;	
   }reg;
    unsigned int value;
}goertzel_spectrum_freq_bin_Def,*pgoertzel_spectrum_freq_bin_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Definition of y0_real_output_data
 * */


typedef union
{
    struct
   {
        unsigned int data : 32;	
   }reg;
    unsigned int value;
}goertzel_spectrum_y0_real_output_data_Def,*pgoertzel_spectrum_y0_real_output_data_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Definition of y0_imag_output_data
 * */


typedef union
{
    struct
   {
        unsigned int data : 32;	
   }reg;
    unsigned int value;
}goertzel_spectrum_y0_imag_output_data_Def,*pgoertzel_spectrum_y0_imag_output_data_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Definition of y1_real_output_data
 * */


typedef union
{
    struct
   {
        unsigned int data : 32;	
   }reg;
    unsigned int value;
}goertzel_spectrum_y1_real_output_data_Def,*pgoertzel_spectrum_y1_real_output_data_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Definition of y1_imag_output_data
 * */


typedef union
{
    struct
   {
        unsigned int data : 32;	
   }reg;
    unsigned int value;
}goertzel_spectrum_y1_imag_output_data_Def,*pgoertzel_spectrum_y1_imag_output_data_Def;


/******************************************************************************
 * register_map Register map definition
******************************************************************************/

typedef struct
{
    volatile unsigned int Resrvd1[4];
    volatile goertzel_spectrum_goertzel_flush_Def goertzel_flush;
    volatile unsigned int Resrvd2[3];
    volatile goertzel_spectrum_freq_bin_Def freq_bin;
    volatile unsigned int Resrvd3[3];
    volatile goertzel_spectrum_y0_real_output_data_Def y0_real_output_data;
    volatile unsigned int Resrvd4[3];
    volatile goertzel_spectrum_y0_imag_output_data_Def y0_imag_output_data;
    volatile unsigned int Resrvd5[3];
    volatile goertzel_spectrum_y1_real_output_data_Def y1_real_output_data;
    volatile unsigned int Resrvd6[3];
    volatile goertzel_spectrum_y1_imag_output_data_Def y1_imag_output_data;
}hepta_a10_2x4_goertzel_spectrum_register_map,*phepta_a10_2x4_goertzel_spectrum_register_map;

typedef struct
{
#ifdef GOERTZEL_STANDALONE
	unsigned int* mem2st_addr;
#endif
	unsigned int* goertzel_streaming_addr;
	unsigned int* goertzel_module_addr;
	int bin_len;
	float centre_freq;
	float bw_freq;
	float sampling_freq;
	float aclr1;
	float aclr2;
	int iteration_time_int;
	int b_coeff;
	double complex w1_coeff;
}goertzel_data_structure,*pgoertzel_data_structure;

typedef struct
{
	int b_coeff;
	double complex w1_coeff;
}goertzel_coeff_structure,*pgoertzel_coeff_structure;



/******************************************************************************
 * Function declaration
******************************************************************************/

pdevice_handler hepta_a10_2x4_goertzel_spectrum_get_handler(unsigned int instance_num);
void * hepta_a10_2x4_goertzel_spectrum_isr_thread(void * goertzel_spectrum_handler_ptr);
void * hepta_a10_2x4_goertzel_spectrum_register_isr(void * goertzel_spectrum_handler_ptr,isr_callback callback_function,void * callback_argument);
void hepta_a10_2x4_goertzel_spectrum_goertzel_flush_start_goertzel_module_config_start(pdevice_handler goertzel_spectrum_device_handler);
void hepta_a10_2x4_goertzel_spectrum_goertzel_flush_start_goertzel_module_config_stop(pdevice_handler goertzel_spectrum_device_handler);
void hepta_a10_2x4_goertzel_spectrum_goertzel_flush_config_start_goertzel_module(pdevice_handler goertzel_spectrum_device_handler,GOERTZEL_SPECTRUM_goertzel_flush_start_goertzel_module_Enum_Def start_goertzel_module_configuration);
GOERTZEL_SPECTRUM_goertzel_flush_start_goertzel_module_Enum_Def hepta_a10_2x4_goertzel_spectrum_goertzel_flush_read_start_goertzel_module(pdevice_handler goertzel_spectrum_device_handler);
bool hepta_a10_2x4_goertzel_spectrum_goertzel_flush_isstart_goertzel_module_start(pdevice_handler goertzel_spectrum_device_handler);
bool hepta_a10_2x4_goertzel_spectrum_goertzel_flush_isstart_goertzel_module_stop(pdevice_handler goertzel_spectrum_device_handler);


int goertzel_validation_test(void);
void goertzel_measurement_capture(void*,void*,float*,unsigned int);
int goertzel_parameter_generate(void*,int,pgoertzel_coeff_structure);
int goertzel_coeff_calculate(void*,double,pgoertzel_coeff_structure);
void goertzel_config_param(pdevice_handler,float,float,float,float,float,int);


#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
