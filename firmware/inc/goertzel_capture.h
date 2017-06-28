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
 * @file hepta_a10_2x4_goertzel_capture.h
 * @defgroup hepta_a10_2x4_goertzel_capture HEPTA A10 2X4 GOERTZEL CAPTURE
 * @brief Header file for goertzel_capture module
 * */

#ifndef HEPTA_A10_2X4_GOERTZEL_CAPTURE_H_
#define HEPTA_A10_2X4_GOERTZEL_CAPTURE_H_

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

/******************************************************************************
 * Include private Header files
******************************************************************************/

//#include "geortzel_protocol.h"
#include <goertzel_firmware.h>
//#include "geortzel_command_format.h"

#define hepta_a10_2x4_goertzel_capture_REGISTER_MAP_RESOURCE_INDEX 0

#define hepta_a10_2x4_goertzel_capture_CONFIG_COUNT 0
#define hepta_a10_2x4_goertzel_capture_SERVICE_COUNT 0
/******************************************************************************
 * Enumeration definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_goertzel_capture
 * @brief capture_start
 * */


typedef enum
{
    GOERTZEL_CAPTURE_capture_start_stop=0,    /**< stop*/
    GOERTZEL_CAPTURE_capture_start_start=1,    /**< start*/
}GOERTZEL_CAPTURE_module_config_capture_start_Enum_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_capture
 * @brief pattern_start
 * */


typedef enum
{
    GOERTZEL_CAPTURE_pattern_start_stop=0,    /**< stop*/
    GOERTZEL_CAPTURE_pattern_start_start=1,    /**< start*/
}GOERTZEL_CAPTURE_module_config_pattern_start_Enum_Def;


/******************************************************************************
 * Register bit map definitions
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_goertzel_capture
 * @brief Definition of module_config
 * */


typedef union
{
    struct
   {
        unsigned int capture_start : 1;	/**< capture_start*/
        unsigned int pattern_start : 1;	/**< pattern_start*/
        unsigned int Resrvd1 : 30;	
   }reg;
    unsigned int value;
}goertzel_capture_module_config_Def,*pgoertzel_capture_module_config_Def;

/**
 * @ingroup hepta_a10_2x4_goertzel_capture
 * @brief Definition of cpri_rfp
 * */


typedef union
{
    struct
   {
        unsigned int data : 32;	
   }reg;
    unsigned int value;
}goertzel_capture_cpri_rfp_Def,*pgoertzel_capture_cpri_rfp_Def;


/******************************************************************************
 * register_map Register map definition
******************************************************************************/

typedef struct
{
    volatile goertzel_capture_module_config_Def module_config;
    volatile goertzel_capture_cpri_rfp_Def cpri_rfp;
}hepta_a10_2x4_goertzel_capture_register_map,*phepta_a10_2x4_goertzel_capture_register_map;

/******************************************************************************
 * Function declaration
******************************************************************************/

pdevice_handler hepta_a10_2x4_goertzel_capture_get_handler(unsigned int instance_num);
void * hepta_a10_2x4_goertzel_capture_isr_thread(void * goertzel_capture_handler_ptr);
void * hepta_a10_2x4_goertzel_capture_register_isr(void * goertzel_capture_handler_ptr,isr_callback callback_function,void * callback_argument);
void hepta_a10_2x4_goertzel_capture_module_config_capture_start_config_stop(pdevice_handler goertzel_capture_device_handler);
void hepta_a10_2x4_goertzel_capture_module_config_capture_start_config_start(pdevice_handler goertzel_capture_device_handler);
void hepta_a10_2x4_goertzel_capture_module_config_config_capture_start(pdevice_handler goertzel_capture_device_handler,GOERTZEL_CAPTURE_module_config_capture_start_Enum_Def capture_start_configuration);
GOERTZEL_CAPTURE_module_config_capture_start_Enum_Def hepta_a10_2x4_goertzel_capture_module_config_read_capture_start(pdevice_handler goertzel_capture_device_handler);
bool hepta_a10_2x4_goertzel_capture_module_config_iscapture_start_stop(pdevice_handler goertzel_capture_device_handler);
bool hepta_a10_2x4_goertzel_capture_module_config_iscapture_start_start(pdevice_handler goertzel_capture_device_handler);
void hepta_a10_2x4_goertzel_capture_module_config_pattern_start_config_stop(pdevice_handler goertzel_capture_device_handler);
void hepta_a10_2x4_goertzel_capture_module_config_pattern_start_config_start(pdevice_handler goertzel_capture_device_handler);
void hepta_a10_2x4_goertzel_capture_module_config_config_pattern_start(pdevice_handler goertzel_capture_device_handler,GOERTZEL_CAPTURE_module_config_pattern_start_Enum_Def pattern_start_configuration);
GOERTZEL_CAPTURE_module_config_pattern_start_Enum_Def hepta_a10_2x4_goertzel_capture_module_config_read_pattern_start(pdevice_handler goertzel_capture_device_handler);
bool hepta_a10_2x4_goertzel_capture_module_config_ispattern_start_stop(pdevice_handler goertzel_capture_device_handler);
bool hepta_a10_2x4_goertzel_capture_module_config_ispattern_start_start(pdevice_handler goertzel_capture_device_handler);


#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
