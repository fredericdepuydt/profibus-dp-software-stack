/*
 * PROFIBUS_DP_Slave.h
 */
#ifndef PROFIBUS_DP_Slave_h
#define PROFIBUS_DP_Slave_h

/************************   Library version   ****************************/
#define HW_VERSION_HIGH 5
#define HW_VERSION_LOW 0

#define DEVICE_PARAMETERS 2

/************************   ARDUINO OR NXT    ****************************/
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define __HARDWARE_TARGET_ARDUINO__
#else
    #define __HARDWARE_TARGET_NXT__
#endif

/************************      LIBRARIES      ****************************/
#ifdef __HARDWARE_TARGET_ARDUINO__
    // ARDUINO LIBRARIES
    #include "Arduino.h"
    #include "HardwareSerial.h"
    #include "TimerOne.h"
    #include "TimerOne.cpp"
#endif
#ifdef __HARDWARE_TARGET_NXT__
    // NXT LIBRARIES
    #include <stddef.h>
    #include <math.h>
    #include "kernel.h"
    #include "kernel_id.h"
    #include "ecrobot_interface.h"
    #include "hs.h"
    #include "hs.c"
#endif

// COMMON LIBRARIES
#include <stdbool.h>
#include <stdlib.h>
#include "PROFIBUS_DP_Settings.h"
#include "PROFIBUS_DP_V1.h"
#include "PROFIBUS_DP_V0.h"
#include "PROFIBUS_DP_Fail_Safe.h"


/************************       DEFINES       ****************************/
#ifdef __HARDWARE_TARGET_NXT__
    // NXT DEFINES
    #define uint8_t uint8_T
    #define uint32_t uint32_T
    #define size_t uint8_T
#endif

#define TBIT 1000000/BAUDRATE // TBIT in µs

// PROFIBUS RW STATES
#define PROFIBUS_IDLE 0x00

#define PROFIBUS_READSTART 0x20
#define PROFIBUS_READ 0x21
#define PROFIBUS_READ_NEXT_BYTE 0x22
#define PROFIBUS_READ_NEXT_PACKET 0x23
#define PROFIBUS_READ_MORE 0x24

#define PROFIBUS_WRITESTART 0x30
#define PROFIBUS_WRITE 0x31
#define PROFIBUS_WRITESTOP 0x32

// PROFIBUS SD Telegram Formats
#define PROFIBUS_SD1 0x10
#define PROFIBUS_SD2 0x68
#define PROFIBUS_SD3 0xA2
#define PROFIBUS_SD4 0xDC
#define PROFIBUS_SC 0xE5

// PROFIBUS Station Mode
#define PROFIBUS_SLAVE 0x00
#define PROFIBUS_MASTER_NOT_READY 0x10
#define PROFIBUS_MASTER_READY 0x20
#define PROFIBUS_MASTER_IN_RING 0x30

// PROFIBUS Slave State
#define PROFIBUS_SS_Power_ON 0x00
#define PROFIBUS_SS_Wait_PRM 0x01
#define PROFIBUS_SS_Wait_CFG 0x02
#define PROFIBUS_SS_Pre_Data_XCHG 0x03
#define PROFIBUS_SS_Data_XCHG 0x04

// PROFIBUS Master State
#define PROFIBUS_MS_CLEAR 0x01
#define PROFIBUS_MS_OPERATE 0x00
#define PROFIBUS_MS_FDLSTATUS 0xFE
#define PROFIBUS_MS_PASSTOKEN 0xFF

// PROFIBUS SSAP
#define SSAP_DP_MS0 0x3E
#define SSAP_DP_MS1 0x33

// PROFIBUS DSAP
#define DSAP_SET_SLAVE_ADDRESS 0x37
#define DSAP_READ_INPUT 0x38
#define DSAP_READ_OUTPUT 0x39
#define DSAP_GLOBAL_CONTROL 0x3A
#define DSAP_GET_CONFIG 0x3B
#define DSAP_SLAVE_DIAGNOSIS 0x3C
#define DSAP_SET_PARAMETERS 0x3D
#define DSAP_CHECK_CONFIG 0x3E
#define DSAP_BROADCAST 0x3F

#define DSAP_DPV1_Read_Write_Res 0x33

// PROFIBUS DIAGNOSTIC BITS
// Byte 1
#define DIAG_station_non_existent DIAG_1.b0
#define DIAG_station_not_ready DIAG_1.b1
#define DIAG_configuration_fault DIAG_1.b2
#define DIAG_extended_diagnostic DIAG_1.b3
#define DIAG_not_supported DIAG_1.b4
#define DIAG_invalid_slave_response DIAG_1.b5
#define DIAG_parameter_fault DIAG_1.b6
#define DIAG_master_lock DIAG_1.b7
// Byte 2
#define DIAG_parameter_request DIAG_2.b0
#define DIAG_status_diagnostics DIAG_2.b1
#define DIAG_watchdog_on DIAG_2.b3
#define DIAG_freeze_mode DIAG_2.b4
#define DIAG_sync_mode DIAG_2.b5
#define DIAG_deactivated DIAG_2.b7
// Byte 3
#define DIAG_diagnostic_overflow DIAG_3.b7

// PROFIBUS DIAGNOSTIC BITS
// Byte 1
#define PRM_watchdog_on PRM_1.b3
#define PRM_freeze_mode PRM_1.b4
#define PRM_sync_mode PRM_1.b5
#define PRM_unlock_request PRM_1.b6
#define PRM_lock_request PRM_1.b7

union uint8_u
{
    uint8_t byte;
    struct
    {
        uint8_t b0: 1;
        uint8_t b1: 1;
        uint8_t b2: 1;
        uint8_t b3: 1;
        uint8_t b4: 1;
        uint8_t b5: 1;
        uint8_t b6: 1;
        uint8_t b7: 1;
    };
};
extern void PROFIBUS_begin();
extern void PROFIBUS_setConfig(const uint8_t,const uint8_t,const uint8_t);
extern void PROFIBUS_start();

extern void PROFIBUS_run();
extern void PROFIBUS_S_State();

extern void PROFIBUS_stop();
extern void PROFIBUS_writestart();
extern void PROFIBUS_write();
extern void PROFIBUS_writestop();
extern void PROFIBUS_readstart();
extern void PROFIBUS_read();

extern void createSD1(const uint8_t,const uint8_t,const uint8_t);
extern void createSD2(const uint8_t,const uint8_t,const uint8_t,const size_t);
extern void createSD4(const uint8_t,const uint8_t);
extern void createSC();


uint8_t RW_State;
uint8_t M_State;
uint8_t MS_State;
uint8_t S_State;
uint8_t S_Ready;
uint8_t station;
void process_packet(const uint8_t*);

uint8_t TX_buf[256];
uint8_t TX_size;
uint8_t RX_buf[1024];
uint8_t RX_size;
uint8_t RX_prev_size;
uint8_t RX_pointer;

uint8_t number_Of_Inputs;
uint8_t number_Of_Outputs;
uint8_t number_Of_Parameters;

uint8_t *OUT;
uint8_t *IN;
uint8_t *PARAM;

uint8_t *STORED_OUT;

uint8_t wait;

uint8_t ADDR;

uint8_t SA;
uint8_t DA;
uint8_t FC;

uint8_t i,j;

uint8_t BUS_PRM_HSA;
uint8_t BUS_PRM_GAP_factor;

// Slave settings
union uint8_u DIAG_1;
union uint8_u DIAG_2;
union uint8_u DIAG_3;
union uint8_u PRM_1;

uint16_t BUS_threshold;
uint32_t BUS_curr_time;
uint32_t BUS_last_time;
uint8_t BUS_active;

uint16_t WD_threshold;
uint32_t WD_curr_time;
uint32_t WD_last_time;
uint8_t Data_Exch_Packet;


uint8_t min_TSDR;
uint8_t group_allocation;
uint8_t master_address;
uint8_t ident_H;
uint8_t ident_L;

void process_SAP(const uint8_t*, const size_t);

static uint8_t CRC(const uint8_t*, const size_t);

void DPV1_Write_request(const uint8_t*, const size_t);
void DPV1_Read_request(const uint8_t*, size_t);

void FailSafe();

#include "PROFIBUS_DP_Stack.c"

#endif
