
#define BAUDRATE 500000

#ifdef __HARDWARE_TARGET_ARDUINO__
    #define pin_RE 24
    #define pin_DE 22
    #define Serial_PB Serial3
	
	#define PC_DEBUGGING
	
#endif

#ifdef PC_DEBUGGING
		#define Serial_Debug Serial
#endif
