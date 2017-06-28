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
 * @file hepta_a10_2x4_goertzel_spectrum.c
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Application file for goertzel_spectrum module
 * */

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

#include "goertzel_spectrum.h"
//#include "geortzel_protocol.h"
#include <goertzel_firmware.h>
//#include "geortzel_command_format.h"
//extern psocket_manager_header hepta_a10_2x4_socket_manager;
psocket_manager_header hepta_a10_2x4_socket_manager;
/******************************************************************************
 * Device handler Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Driver handler function. Interacts with goertzel_spectrum driver and gets address map information
 * @param instance_num Instance number of the goertzel_spectrum module for which handler is to be returned
 * @retval Handler Pointer Requested Instance of goertzel_spectrum module exits
 * @retval NULL Requested Instance of goertzel_spectrum module doesnot exit
 * */


pdevice_handler hepta_a10_2x4_goertzel_spectrum_get_handler(unsigned int instance_num)
{
    int fd;
    unsigned char ret;
    char number[4];
    char device_node[256];
    static pdevice_handler device_handler_ptr=NULL;
    static unsigned int device_num_of_instance=-1;
    
    unsigned int i=0;
    int pid;
    memory_map_resource *memory_map_resource_ptr;
    unsigned int num_of_resource=0;
    unsigned int current_resource=0;
    bus_address mmap_address;
    int service_index = 0;
    
    
    if(device_handler_ptr==NULL)
    {
    	//memset(device_node,0,256);	//erase device_node
    	for(i=0;i<256;i++)
    	    	{
    	    		device_node[i]=0;
    	    	}
    	snprintf(device_node,sizeof(device_node),"/proc/%s/num_of_instance","goertzel_spectrum");
    	DEBUG_PRINT(DEBUG_LEVEL_INFO,("open file path %s \n",device_node));
    	fd=open(device_node,O_RDWR); //open corresponding proc file
    	if(fd <=0)
    	{
    		DEBUG_PRINT(DEBUG_LEVEL_ERROR,("unable to open %s \n",device_node));
    		return NULL;
    	}
    	ret=read((int)fd,number,4);  
    	close(fd);
    	number[ret-1]='\0';
    	device_num_of_instance=atoi(number);
    	
    	
    	device_handler_ptr = malloc(sizeof(device_handler)*device_num_of_instance);
    	if(device_handler_ptr == NULL)
    	{
    		DEBUG_PRINT(DEBUG_LEVEL_ERROR,("Unable allocate memory \n"));
    		return NULL;			
    	}
    	
    	
    	for(i=0;i<device_num_of_instance;i++)
    	{
    		//memset(device_node,0,256);	//erase device_node content
    		for(i=0;i<256;i++)
    		    	{
    		    		device_node[i]=0;
    		    	}
    		i = 0;
    		/**< assembling device file name from module type*/
    		snprintf(device_node,sizeof(device_node),"%s%s%d","/dev/","goertzel_spectrum",i);
    		//DEBUG_PRINT(DEBUG_LEVEL_INFO,("%s %d %s \n",__FUNCTION__,__LINE__,device_node));
    		device_handler_ptr[i].dev_name = strdup(device_node);
    		device_handler_ptr[i].dev_file_handler = open(device_node,O_RDWR);
    		device_handler_ptr[i].device_instance_number = i;
    
    		if(device_handler_ptr[i].dev_file_handler < 1)
    		{
    			DEBUG_PRINT(DEBUG_LEVEL_ERROR,("Resource is not available \n"));
    			return NULL;
    		}
    		
    		pid = getpid();
    		ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_SET_PID,(unsigned long)&pid);
    
    		ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_GET_NUM_RESOURCE,(unsigned long)&num_of_resource);
    		if(num_of_resource > 0)
    		{
    			memory_map_resource_ptr=(memory_map_resource *)malloc(sizeof(memory_map_resource)*num_of_resource);
    			ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_GET_RESOURCES,(unsigned long)memory_map_resource_ptr);
    
    
    			device_handler_ptr[i].reg_phy_addr = (bus_address *)malloc(sizeof(bus_address)*(num_of_resource));
    			device_handler_ptr[i].reg_remap_addr = (bus_address *)malloc(sizeof(bus_address)*(num_of_resource));
    
    			for(current_resource=0;current_resource<num_of_resource;current_resource++)
    			{
    				mmap_address = mmap(NULL,memory_map_resource_ptr[current_resource].size_in_bytes, PROT_READ|PROT_WRITE, MAP_SHARED, \
    						device_handler_ptr[i].dev_file_handler,(off_t)memory_map_resource_ptr[current_resource].start_address);
    				if (mmap_address == MAP_FAILED)
    				{
    					perror("mmap");
    					DEBUG_PRINT(DEBUG_LEVEL_ERROR,("Memory Map Failed  \n"));
    					return NULL;
    				}
    
    				device_handler_ptr[i].reg_phy_addr[current_resource]=memory_map_resource_ptr[current_resource].start_address;
    				device_handler_ptr[i].reg_remap_addr[current_resource]=mmap_address;
    			}
    			free(memory_map_resource_ptr);
    		}
    
    		ioctl(device_handler_ptr[i].dev_file_handler,IOCTL_GET_UIO_MINOR_NUMBER,(unsigned long)& (device_handler_ptr[i].uio_irq_device_number));
    
    		if(hepta_a10_2x4_goertzel_spectrum_SERVICE_COUNT > 0)
    		{
    			device_handler_ptr[i].service_id_array =  malloc(sizeof(unsigned int)* hepta_a10_2x4_goertzel_spectrum_SERVICE_COUNT );
    			device_handler_ptr[i].service_socket_index_array =  malloc(sizeof(unsigned int)* hepta_a10_2x4_goertzel_spectrum_SERVICE_COUNT );

    			for(service_index=0;service_index< hepta_a10_2x4_goertzel_spectrum_SERVICE_COUNT ; service_index++)
    			{
    				device_handler_ptr[i].service_id_array[service_index] = (unsigned int) (-1);
    			}
    		}
    	}
    }
    
    if(instance_num >= device_num_of_instance)
    {
    	return NULL;
    }
    else
    {
    	return (device_handler_ptr+instance_num);
    }	
}

/******************************************************************************
 * ISR Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Interrupt Service Routine. Interacts with goertzel_spectrum UIO driver and gets interrupt status
 * @param goertzel_spectrum_handler_ptr Device handler of the goertzel_spectrum module
 * */


void * hepta_a10_2x4_goertzel_spectrum_isr_thread(void * goertzel_spectrum_handler_ptr)
{
    pdevice_handler pgoertzel_spectrum_handler = (pdevice_handler)goertzel_spectrum_handler_ptr;
    
    int uio_fd;
    char number[4];
    char device_node[256];
    //unsigned int no_of_interrupts_triggered = 0;
    unsigned char ret;
    unsigned int i = 0;
    
    //memset(device_node,0,256);	//erase device_node
    for(i=0;i<256;i++)
        	{
        		device_node[i]=0;
        	}
    i = 0;
    snprintf(device_node,sizeof(device_node),"/dev/uio%d",pgoertzel_spectrum_handler->uio_irq_device_number);
    
    uio_fd = open(device_node,O_RDONLY);
    
    if(uio_fd <= 0)
    {
    	return NULL;
    }
    
    while(1)
    {
    	ret = read(uio_fd,number,4);
    	DEBUG_PRINT(DEBUG_LEVEL_INFO,("goertzel_spectrum %d Interrupt received\n",pgoertzel_spectrum_handler->device_instance_number));
    	number[ret-1]='\0';
    	//no_of_interrupts_triggered=atoi(number);
    	
    	if(pgoertzel_spectrum_handler->ISR_callback_function_ptr != NULL)
    	{
    		pgoertzel_spectrum_handler->ISR_callback_function_ptr(pgoertzel_spectrum_handler,pgoertzel_spectrum_handler->ISR_callback_argument);
    	}
    	else
    	{	
    		break;
    	}
    }
    
    close(uio_fd);
    
    return NULL;
}

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Interrupt Service Register API. Register callback function for goertzel_spectrum interrupt
 * @param goertzel_spectrum_handler_ptr Device handler of the goertzel_spectrum module
 * @param callback_function Callback function for goertzel_spectrum IRQ
 * @param callback_argument Argument to be passed to goertzel_spectrum ISR callback function
 * */


void * hepta_a10_2x4_goertzel_spectrum_register_isr(void * goertzel_spectrum_handler_ptr,isr_callback callback_function,void * callback_argument)
{
    pdevice_handler pgoertzel_spectrum_handler = (pdevice_handler)goertzel_spectrum_handler_ptr;
    if(pgoertzel_spectrum_handler->uio_irq_device_number >=0 )
    {
    	pgoertzel_spectrum_handler->ISR_callback_function_ptr = callback_function;
    	pgoertzel_spectrum_handler->ISR_callback_argument = callback_argument;
    	pthread_create(&(pgoertzel_spectrum_handler->ISR_thread_handler),NULL, hepta_a10_2x4_goertzel_spectrum_isr_thread,pgoertzel_spectrum_handler);
    }
    return NULL;
}

/******************************************************************************
 * User defined Function definition
******************************************************************************/

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Configure start
 * @param goertzel_spectrum_device_handler Device handler goertzel_spectrum required instance 
 * */


void hepta_a10_2x4_goertzel_spectrum_goertzel_flush_start_goertzel_module_config_start(pdevice_handler goertzel_spectrum_device_handler)
{
    phepta_a10_2x4_goertzel_spectrum_register_map resource_ptr = (phepta_a10_2x4_goertzel_spectrum_register_map) goertzel_spectrum_device_handler->reg_remap_addr[hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->goertzel_flush.reg.start_goertzel_module = GOERTZEL_SPECTRUM_start_goertzel_module_start;
    return;
}

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Configure stop
 * @param goertzel_spectrum_device_handler Device handler goertzel_spectrum required instance 
 * */


void hepta_a10_2x4_goertzel_spectrum_goertzel_flush_start_goertzel_module_config_stop(pdevice_handler goertzel_spectrum_device_handler)
{
    phepta_a10_2x4_goertzel_spectrum_register_map resource_ptr = (phepta_a10_2x4_goertzel_spectrum_register_map) goertzel_spectrum_device_handler->reg_remap_addr[hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->goertzel_flush.reg.start_goertzel_module = GOERTZEL_SPECTRUM_start_goertzel_module_stop;
    return;
}

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Configure start_goertzel_module based on input parameter
 * @param goertzel_spectrum_device_handler Device handler goertzel_spectrum required instance 
 * @param start_goertzel_module_configuration Input configuration of start_goertzel_module
 * */


void hepta_a10_2x4_goertzel_spectrum_goertzel_flush_config_start_goertzel_module(pdevice_handler goertzel_spectrum_device_handler,GOERTZEL_SPECTRUM_goertzel_flush_start_goertzel_module_Enum_Def start_goertzel_module_configuration)
{
    phepta_a10_2x4_goertzel_spectrum_register_map resource_ptr = (phepta_a10_2x4_goertzel_spectrum_register_map)goertzel_spectrum_device_handler->reg_remap_addr[hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX];
    resource_ptr->goertzel_flush.reg.start_goertzel_module = start_goertzel_module_configuration;
    return;
}

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Read start_goertzel_module
 * @param goertzel_spectrum_device_handler Device handler goertzel_spectrum required instance 
 * @retval GOERTZEL_SPECTRUM_start_goertzel_module_start start
 * @retval GOERTZEL_SPECTRUM_start_goertzel_module_stop stop
 * */


GOERTZEL_SPECTRUM_goertzel_flush_start_goertzel_module_Enum_Def hepta_a10_2x4_goertzel_spectrum_goertzel_flush_read_start_goertzel_module(pdevice_handler goertzel_spectrum_device_handler)
{
    phepta_a10_2x4_goertzel_spectrum_register_map resource_ptr = (phepta_a10_2x4_goertzel_spectrum_register_map) goertzel_spectrum_device_handler->reg_remap_addr[hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX];
    return resource_ptr->goertzel_flush.reg.start_goertzel_module;
}

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Check start_goertzel_module status is start
 * @param goertzel_spectrum_device_handler Device handler goertzel_spectrum required instance 
 * @retval true start_goertzel_module status is start
 * @retval false start_goertzel_module status is not start
 * */


bool hepta_a10_2x4_goertzel_spectrum_goertzel_flush_isstart_goertzel_module_start(pdevice_handler goertzel_spectrum_device_handler)
{
    phepta_a10_2x4_goertzel_spectrum_register_map resource_ptr = (phepta_a10_2x4_goertzel_spectrum_register_map) goertzel_spectrum_device_handler->reg_remap_addr[hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->goertzel_flush.reg.start_goertzel_module == GOERTZEL_SPECTRUM_start_goertzel_module_start);
}

/**
 * @ingroup hepta_a10_2x4_goertzel_spectrum
 * @brief Check start_goertzel_module status is stop
 * @param goertzel_spectrum_device_handler Device handler goertzel_spectrum required instance 
 * @retval true start_goertzel_module status is stop
 * @retval false start_goertzel_module status is not stop
 * */


bool hepta_a10_2x4_goertzel_spectrum_goertzel_flush_isstart_goertzel_module_stop(pdevice_handler goertzel_spectrum_device_handler)
{
    phepta_a10_2x4_goertzel_spectrum_register_map resource_ptr = (phepta_a10_2x4_goertzel_spectrum_register_map) goertzel_spectrum_device_handler->reg_remap_addr[hepta_a10_2x4_goertzel_spectrum_REGISTER_MAP_RESOURCE_INDEX];
    return (resource_ptr->goertzel_flush.reg.start_goertzel_module == GOERTZEL_SPECTRUM_start_goertzel_module_stop);
}

/******************************************************************************
 * Custom Function definition
******************************************************************************/


