
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define y_width 1


#if !defined(MATLAB_MEX_FILE)
    #include "PROFIBUS_DP_Stack.h"
    uint8_T initiate = 0;
#endif


/*
 * Output functions
 *
 */
extern "C" void PROFIBUS_DP_Outputs_wrapper(const uint8_T *Ready_for_DE,
			const uint8_T *Inputs,
			uint8_T *DP_Slave_State,
			uint8_T *Outputs,
            uint8_T *Parameters,
			const uint8_T *Initial_DP_Address,
			const uint8_T *Number_Of_Inputs,
			const uint8_T *Number_Of_Outputs,
            const uint8_T *Number_Of_Parameters)
{
#if !defined(MATLAB_MEX_FILE)
    if(initiate == 0){
        PROFIBUS_begin();
        PROFIBUS_setConfig(Number_Of_Inputs[0], Number_Of_Outputs[0], Number_Of_Parameters[0]);
        PROFIBUS_start();
        ADDR = Initial_DP_Address[0];
        initiate = 1;
    }else{
		DIAG_station_not_ready = ~Ready_for_DE[0];
		IN = Inputs;
        OUT = Outputs;
        PARAM = Parameters;
        PROFIBUS_run();
		DP_Slave_State[0] = S_State;
        
    }
#endif
}


