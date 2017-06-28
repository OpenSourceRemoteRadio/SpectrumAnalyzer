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
 * @file hepta_a10_2x4_firmware.h
 * @brief Firmware Header
 * */

#ifndef HEPTA_A10_2X4_FIRMWARE_H_
#define HEPTA_A10_2X4_FIRMWARE_H_
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
//#include "geortzel_command_format.h"

#define IOCTL_GET_NUM_RESOURCE _IOR('b',0,unsigned int *)
#define IOCTL_GET_RESOURCES _IOR('b',1,memory_map_resource *)
#define IOCTL_SET_PID _IOW('b',2,unsigned int *)
#define IOCTL_GET_NUM_INTERRUPT _IOR('b',3,unsigned int *)
#define IOCTL_GET_UIO_MINOR_NUMBER _IOR('b',4,int *)



#define REGISTER_SIZE_16	2
#define REGISTER_SIZE_32	4
#define REGISTER_SIZE_64	8


/* define */
#define MAX_SERVER_COUNT 2
#define BIT_ENABLE(x) (0x1<<x)
#define BIT_DISABLE(x) ~(0x1<<x)

#define RESERVED_MEM 0x20000000
#define RESERVED_MEM_SIZE (0x1<<29)


#define	DEBUG_LEVEL_ALL   0x0
#define	DEBUG_LEVEL_INIT  0x1
#define DEBUG_LEVEL_EXIT  0x2
#define DEBUG_LEVEL_INFO  0x3
#define	DEBUG_LEVEL_WARN  0x4
#define	DEBUG_LEVEL_ERROR 0X5


extern int DebugLevel;
#define _DEBUG_PRINT_

#ifdef _DEBUG_PRINT_
#define DEBUG_PRINT(X,Y) if(X>=DebugLevel) {do{ printf(" %s : %d ",__FUNCTION__,__LINE__); printf Y; } while(0); } else{ do { }while(0);}
#else
#define DEBUG_PRINT(X,Y)
#endif




//typedef void (*command_response)(pCommand_Header pCmdHeader,unsigned int packet_id);
typedef void (*isr_callback)(void *device_handler_ptr,void * callback_argument);
typedef volatile unsigned int * bus_address;

/**
* @ingroup HandleManager
* @brief Hepta device handler structure.
*/

typedef struct{
	char 					*dev_name; 					/**< Device node path and name as a string. */
	unsigned int 			dev_file_handler;			/**< File handler for the opened device node. */
	unsigned int			device_instance_number;		/**< Instance number of the device. */
	bus_address 			*reg_phy_addr; 				/**< Array of Physical address for Register Bank and resource*/
	bus_address 			*reg_remap_addr; 			/**< Array of Virtual address of Register Bank and resource*/
	pthread_cond_t	   		condition_signal; 			/**< Condition to signal the read/write threads*/
	pthread_mutex_t    		lock; 						/**< Lock to synchronise the access to the fields in the scheduler header*/
	unsigned int 			*service_id_array;			/**< Pointer to service id array. */
	unsigned int 			*service_socket_index_array;/**< Pointer to service socket. */
	int 					uio_irq_device_number;		/**< UIO Number for IRQ. */
	pthread_t	 			ISR_thread_handler;			/**< Interrupt Service routine thread handler. */
	isr_callback   			ISR_callback_function_ptr;	/**< Interrupt Service routine callback function. */
	void *		 			ISR_callback_argument;	/**< Interrupt Service routine callback argument. */
	void *					private_data;  				/**< Private data for device. Goertzel Algorithm control */
}device_handler, *pdevice_handler;


/**
 * @ingroup CommandManager
 * @brief Link list to maintain the commands in the scheduler.
 */


typedef struct
{
	struct command_queue *Next;	/**< Pointer to the next element in queue.*/
	void * pQueueData;			/**< Command of the current queue element.*/
	unsigned int id;				/**< Id for the current command.*/
}command_queue,*pcommand_queue;

/**
 * @ingroup CommandManager
 * @brief Link list Heads to maintain the commands in the scheduler.
 */

typedef struct
{
	pthread_mutex_t queue_lock;		/**< Mutex lock to synchronise queueing and dequeuing .*/
	struct command_queue *Head;		/**< Pointer to the first command in the queue.*/
	char *queue_name;			/**< Name of the command queue.*/
	unsigned int num_of_nodes;			/**< Number of command elements in the queue.*/
}command_queue_head, *pcommand_queue_head;

/**
 * @ingroup SocketManager
 * @brief Socket descpriptors.
 */

typedef struct
{
	int 	connectedfd 	; 					/**< Descriptor for the connected socket for read and write. */
	char * 	pserver_ip_address ; 				/**< Server IPv4 address */
	char * 	port_no      	;
	struct addrinfo	* addrinfo_ptr;
	bool	connected		;
	bool	connecting		;
	bool	reading			;
	unsigned int server_index;
}socket_handles;
/**
 * @ingroup CommandManager
 * @ingroup SocketManager
 * @brief Scheduler to maintain the firmware details.
 */

typedef struct
{
	command_queue_head        	WriteQueueHead;									/**< Queue head for commands to be sent to server*/
	command_queue_head        	CommandQueueHead;								/**< Queue head for valid commands to be executed*/
	pthread_t 		          	pServerConnectThreadHandle[MAX_SERVER_COUNT]; 	/**< Thread to perform write in the hardware modules*/
	pthread_t 		          	pTransportReadThreadHandle[MAX_SERVER_COUNT]; 	/**< Thread to perform write in the hardware modules*/
	pthread_t 		          	pTransportWriteThreadHandle; 					/**< Thread to perform read from the hardware modules*/
	pthread_cond_t	          	WriteThreadCondition;							/**< Condition signal to signal write thread*/
	pthread_t 		          	pCommandThreadHandle; 							/**< Thread to manage hardware modules*/
	pthread_cond_t            	CommandThreadCondition;							/**< Condition signal to signal command thread*/
	pthread_cond_t	          	condition_signal; 								/**< Condition to signal the read/write threads*/
	pthread_mutex_t           	lock; 											/**< Lock to synchronise the access to the fields in the scheduler header*/
	bool			          	is_thread		;								/**< Condition to proceed to next iteration in threads*/
	socket_handles	  		  	ServerClientDetails[MAX_SERVER_COUNT]; 			/**< Heptaconnection handles to get server client details*/
	unsigned int				number_of_server;								/**< Number of servers initiated*/
	bus_address        		  	ddrmemmapped;									/**< Virtual address for DDR3 memory*/
}socket_manager_header,*psocket_manager_header;




/**
* @ingroup HandleManager
* @brief Parameter to exchange information of the resources from driver to the user space.
*/

typedef struct
{
	bus_address  start_address; /**< Base address of the resource. */
	unsigned int size_in_bytes; /**< Size of the resource in bytes. */
}__attribute__((__packed__))memory_map_resource;

//int send_service_message(psocket_manager_header socket_handler,unsigned int server_index,unsigned int Id,Module_Index_Enum_Def module_index,unsigned int *pdata,unsigned int Length);

#ifdef __cplusplus
}/* close the extern "C" { */
#endif
#endif
