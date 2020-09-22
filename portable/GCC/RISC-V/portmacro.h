/*
 * FreeRTOS Kernel V10.4.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 * 1 tab == 4 spaces!
 */


#ifndef PORTMACRO_H
#define PORTMACRO_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Port specific definitions.
 *
 * The settings in this file configure FreeRTOS correctly for the
 * given hardware and compiler.
 *
 * These settings should not be altered.
 *-----------------------------------------------------------
 */

/* Type definitions. */
#if __riscv_xlen == 64
	#define portSTACK_TYPE			uint64_t
	#define portBASE_TYPE			int64_t
	#define portUBASE_TYPE			uint64_t
	#define portMAX_DELAY 			( TickType_t ) 0xffffffffffffffffULL
	#define portPOINTER_SIZE_TYPE 	uint64_t
#elif __riscv_xlen == 32
	#define portSTACK_TYPE	uint32_t
	#define portBASE_TYPE	int32_t
	#define portUBASE_TYPE	uint32_t
	#define portMAX_DELAY ( TickType_t ) 0xffffffffUL
#else
	#error Assembler did not define __riscv_xlen
#endif


typedef portSTACK_TYPE StackType_t;
typedef portBASE_TYPE BaseType_t;
typedef portUBASE_TYPE UBaseType_t;
typedef portUBASE_TYPE TickType_t;

/* Legacy type definitions. */
#define portCHAR		char
#define portFLOAT		float
#define portDOUBLE		double
#define portLONG		long
#define portSHORT		short

/* 32-bit tick type on a 32-bit architecture, so reads of the tick count do
not need to be guarded with a critical section. */
#define portTICK_TYPE_IS_ATOMIC 1
/*-----------------------------------------------------------*/

extern __attribute__((naked)) void vPortSyscall( uint32_t );
extern __attribute__((naked)) void vPortUpdatePrivilegeStatus( UBaseType_t);

/******************************************************************************/
/******************************  PMP settings  ********************************/
/******************************************************************************/

#if( portUSING_MPU_WRAPPERS == 1 )
#include "pmp.h"

/* Privilege bit used to declare a Task as Privileged */
#define portPRIVILEGE_BIT						( 0x80000000UL )

/**************************/
/* System call commands */
/**************************/

/* Command to indicate system call handler that yield is requested */
#define portSVC_YIELD							0
/* Command to indicate system call handler that interrupts should be disabled */
#define portSVC_DISABLE_INTERRUPTS				1
/* Command to indicate system call handler that interrupts should be disabled */
#define portSVC_ENABLE_INTERRUPTS				2
/* Command to indicate system call handler to switch to machine mode */
#define portSVC_SWITCH_TO_MACHINE				3
/* Command to indicate system call handler to switch to user mode */
#define portSVC_SWITCH_TO_USER					4

/**********************/
/* PMP configs rights */
/**********************/

/* Used to give a PMP region read right */
#define portPMP_REGION_READ_ONLY				( 0x01UL )
/* Used to give a PMP region read/write right */
#define portPMP_REGION_READ_WRITE				( 0x03UL )
/* Used to give a PMP region execute right */
#define portPMP_REGION_EXECUTE					( 0x04UL )
/** 
 * Used to indicate that a PMP region as no right, and therefore only 
 * Machine/Supervisor mode access is possible
 */
#define portPMP_REGION_PRIVILEGED_ACCESS_ONLY	( 0x00UL )


/********************/
/* PMP configs mode */
/********************/

/* Used to indicate that a PMP region should be disable */
#define portPMP_REGION_OFF						( 0x00UL )
/* Used to indicate that a PMP region should use TOR mode */
#define portPMP_REGION_ADDR_MATCH_TOR			( 0x08UL )
/* Used to indicate that a PMP region should use NA4 mode */
#define portPMP_REGION_ADDR_MATCH_NA4			( 0x10UL )
/* Used to indicate that a PMP region should use NAPOT mode */
#define portPMP_REGION_ADDR_MATCH_NAPOT			( 0x18UL )

/********************/
/* PMP configs lock */
/********************/

/**
 * Lock a PMP region (apply right on Machine mode too, can only be unlock
 * by reset)
 */
#define portPMP_REGION_LOCK						( 0x80UL )

/****************************/
/* PMP regions for FreeRTOS */
/****************************/

/* Start of unprivileged section that contain code to execute */
#define portUNPRIVILEGED_EXECUTE_REGION_START   ( 0UL )
/* End of unprivileged section that contain code to execute */
#define portUNPRIVILEGED_EXECUTE_REGION_END		( 1UL )
/* Privilege status that allow FreeRTOS to save current execution mode */
#define portPRIVILEGE_STATUS_REGION	            ( 2UL )
/* Start of task stack region */
#define portSTACK_REGION_START					( 3UL )
/* End of task stack region */
#define portSTACK_REGION_END					( 4UL )
/* Fisrt configurable region */
#define portFIRST_CONFIGURABLE_REGION	        ( 5UL )
/**
 * End configurable region (base on the Maxiumum number of PMP regions that
 * RiscV allow available regions).
 * Should be use only when static allocation is required, otherwise use 
 * portLAST_CONFIGURABLE_REGION_REAL(max_nb_pmp) (after init_pmp() execution)
 */
#define portLAST_CONFIGURABLE_REGION		    ( MAX_PMP_REGION - 1)
/** 
 * Maximum number of configurable regions
 * Should be use only when static allocation is required, otherwise use 
 * portNUM_CONFIGURABLE_REGIONS_REAL(max_nb_pmp) (after init_pmp() execution)
 */
#define portNUM_CONFIGURABLE_REGIONS		    ( ( portLAST_CONFIGURABLE_REGION - portFIRST_CONFIGURABLE_REGION ) + 1 )
#define portTOTAL_NUM_REGIONS				    ( portNUM_CONFIGURABLE_REGIONS + 2 ) // Plus 2 to make space for the stack region.
#define portTOTAL_NUM_CFG_REG				    ( NB_PMP_CFG_REG )

/**
 * the number of pmp available for one hart(core) is dynamically determinated
 * so we need to reevaluate the last configurable region available
 */
#define portLAST_CONFIGURABLE_REGION_REAL(max_nb_pmp)	( (max_nb_pmp) - 1 )
/**
 * the number of pmp available for one hart(core) is dynamically determinated
 * so we need to reevaluate the number of configurable regions
 */
#define portNUM_CONFIGURABLE_REGIONS_REAL(max_nb_pmp)	( ( portLAST_CONFIGURABLE_REGION_REAL( max_nb_pmp ) - portFIRST_CONFIGURABLE_REGION ) + 1 )

/**
 * Bit shift to apply on a PMP config to reach the region specified in parameter
 */
#define portPMPCFG_BIT_SHIFT(region)	( ((region) % SIZE_PMP_CFG_REG) << 3 )
/**
 * Get the pmpcfgx register index associated to the region input
 */
#define portGET_PMPCFG_IDX(region) 		((region) / SIZE_PMP_CFG_REG)

/* minimal number of pmp to use FreeRTOS with pmp */
#define portMINIMAL_NB_PMP				( 5UL )

/**
 * @brief PMP settings used to store PMP configs in task TCB
 */
typedef struct MPU_SETTINGS
{
    /* Configuration register configuration (pmpcfgx values) */
	UBaseType_t uxPmpConfigRegAttribute [portTOTAL_NUM_CFG_REG];
    /* Configuration register mask (pmpcfgx mask)*/
	UBaseType_t uxPmpConfigRegMask [portTOTAL_NUM_CFG_REG];
    /* Address register configuration (pmpaddrx values) */
	UBaseType_t uxRegionBaseAddress [ portTOTAL_NUM_REGIONS ];
} xMPU_SETTINGS;

/**
 * @brief Do ecall to raise privilege
 */
void vRaisePrivilege( void );
/**
 * @brief Do ecall to reset privilege state
 */
void vResetPrivilege( void );

/**
 * @brief Supported execution modes 
 */
enum ePortPRIVILEGE_MODE {
    /* User mode */
    ePortUSER_MODE = 0,
    /* Supervisor mode */
    ePortSUPERVISOR_MODE =1,
    /* Machine mode */
    ePortMACHINE_MODE = 3,
};

/**
 * @brief Determine the current execution mode of the hart
 * 
 * @return true (1) if the hart execute in machine mode
 * @return false (0) otherwise
 */
BaseType_t xIsPrivileged( void );

/**
 * @brief Determine the current execution mode of the hart
 * 
 * @return true (1) if the hart execute in machine mode
 * @return false (0) otherwise
 */
#define portIS_PRIVILEGED()			xIsPrivileged()

/**
 * @brief Do ecall to raise privilege
 */
#define portRAISE_PRIVILEGE()		vRaisePrivilege()

/**
 * @brief Do ecall to lower privilege level
 */
#define portRESET_PRIVILEGE()		vResetPrivilege()

#else /* if( portUSING_MPU_WRAPPERS == 1 ) */
/**************************/
/* System call commands */
/**************************/

/* Command to indicate system call handler that yield is requested */
#define portSVC_YIELD							0
#endif /* if( portUSING_MPU_WRAPPERS == 1 ) */
/*-----------------------------------------------------------*/

/* Architecture specifics. */
#define portSTACK_GROWTH			( -1 )
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
#ifdef __riscv64
	#error This is the RV32 port that has not yet been adapted for 64.
	#define portBYTE_ALIGNMENT			16
#else
	#define portBYTE_ALIGNMENT			16
#endif
/*-----------------------------------------------------------*/


/* Scheduler utilities. */
extern void vTaskSwitchContext( void );
#define portYIELD() 	vPortSyscall(portSVC_YIELD)

#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired ) vTaskSwitchContext()
#define portYIELD_FROM_ISR( x ) portEND_SWITCHING_ISR( x )
/*-----------------------------------------------------------*/


/* Critical section management. */
#define portCRITICAL_NESTING_IN_TCB					1
extern void vTaskEnterCritical( void );
extern void vTaskExitCritical( void );

#define portSET_INTERRUPT_MASK_FROM_ISR() 0
#define portCLEAR_INTERRUPT_MASK_FROM_ISR( uxSavedStatusValue ) ( void ) uxSavedStatusValue
#if( portUSING_MPU_WRAPPERS == 1 )
    #define portDISABLE_INTERRUPTS()	vPortSyscall(portSVC_DISABLE_INTERRUPTS)
    #define portENABLE_INTERRUPTS()		vPortSyscall(portSVC_ENABLE_INTERRUPTS)
#else
    #define portDISABLE_INTERRUPTS()	__asm volatile ( "csrc mstatus, 8" )
    #define portENABLE_INTERRUPTS()		__asm volatile ( "csrs mstatus, 8" )
#endif /* ( portUSING_MPU_WRAPPERS == 1 ) */
#define portENTER_CRITICAL()	vTaskEnterCritical()
#define portEXIT_CRITICAL()		vTaskExitCritical()

/*-----------------------------------------------------------*/

/* Architecture specific optimisations. */
#ifndef configUSE_PORT_OPTIMISED_TASK_SELECTION
	#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#endif

#if( configUSE_PORT_OPTIMISED_TASK_SELECTION == 1 )

	/* Check the configuration. */
	#if( configMAX_PRIORITIES > 32 )
		#error configUSE_PORT_OPTIMISED_TASK_SELECTION can only be set to 1 when configMAX_PRIORITIES is less than or equal to 32.  It is very rare that a system requires more than 10 to 15 difference priorities as tasks that share a priority will time slice.
	#endif

	/* Store/clear the ready priorities in a bit map. */
	#define portRECORD_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) |= ( 1UL << ( uxPriority ) )
	#define portRESET_READY_PRIORITY( uxPriority, uxReadyPriorities ) ( uxReadyPriorities ) &= ~( 1UL << ( uxPriority ) )

	/*-----------------------------------------------------------*/

	#define portGET_HIGHEST_PRIORITY( uxTopPriority, uxReadyPriorities ) uxTopPriority = ( 31UL - __builtin_clz( uxReadyPriorities ) )

#endif /* configUSE_PORT_OPTIMISED_TASK_SELECTION */


/*-----------------------------------------------------------*/

/* Task function macros as described on the FreeRTOS.org WEB site.  These are
not necessary for to use this port.  They are defined so the common demo files
(which build with all the ports) will build. */
#define portTASK_FUNCTION_PROTO( vFunction, pvParameters ) void vFunction( void *pvParameters )
#define portTASK_FUNCTION( vFunction, pvParameters ) void vFunction( void *pvParameters )

/*-----------------------------------------------------------*/

#define portNOP() __asm volatile 	( " nop " )

#define portINLINE	__inline

#ifndef portFORCE_INLINE
	#define portFORCE_INLINE inline __attribute__(( always_inline))
#endif

#define portMEMORY_BARRIER() __asm volatile( "" ::: "memory" )
/*-----------------------------------------------------------*/


/* configCLINT_BASE_ADDRESS is a legacy definition that was replaced by the
configMTIME_BASE_ADDRESS and configMTIMECMP_BASE_ADDRESS definitions.  For
backward compatibility derive the newer definitions from the old if the old
definition is found. */
#if defined( configCLINT_BASE_ADDRESS ) && !defined( configMTIME_BASE_ADDRESS ) && ( configCLINT_BASE_ADDRESS == 0 )
	/* Legacy case where configCLINT_BASE_ADDRESS was defined as 0 to indicate
	there was no CLINT.  Equivalent now is to set the MTIME and MTIMECMP
	addresses to 0. */
	#define configMTIME_BASE_ADDRESS 	( 0 )
	#define configMTIMECMP_BASE_ADDRESS ( 0 )
#elif defined( configCLINT_BASE_ADDRESS ) && !defined( configMTIME_BASE_ADDRESS )
	/* Legacy case where configCLINT_BASE_ADDRESS was set to the base address of
	the CLINT.  Equivalent now is to derive the MTIME and MTIMECMP addresses
	from the CLINT address. */
	#define configMTIME_BASE_ADDRESS 	( ( configCLINT_BASE_ADDRESS ) + 0xBFF8UL )
	#define configMTIMECMP_BASE_ADDRESS ( ( configCLINT_BASE_ADDRESS ) + 0x4000UL )
#elif !defined( configMTIME_BASE_ADDRESS ) || !defined( configMTIMECMP_BASE_ADDRESS )
	#error configMTIME_BASE_ADDRESS and configMTIMECMP_BASE_ADDRESS must be defined in FreeRTOSConfig.h.  Set them to zero if there is no MTIME (machine time) clock.  See https://www.FreeRTOS.org/Using-FreeRTOS-on-RISC-V.html
#endif



#ifdef __cplusplus
}
#endif

#endif /* PORTMACRO_H */

