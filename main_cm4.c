#include "project.h"
#include <stdio.h>
#include <math.h>

#define LIS3DH_CTRL_REG1    0x20 
#define LIS3DH_OUT_X_L      0x28 
#define LIS3DH_OUT_X_H      0x29  
#define LIS3DH_OUT_Y_L      0x2A  
#define LIS3DH_OUT_Y_H      0x2B 
#define LIS3DH_OUT_Z_L      0x2C  
#define LIS3DH_OUT_Z_H      0x2D 

#define NORM_FACTOR 1700
#define STEP_THRES 0.6

static cy_stc_scb_i2c_master_xfer_config_t register_setting;
static uint8 rbuff[2]; 
static uint8 wbuff[2]; 
volatile uint8_t timerTriggered = 0;
volatile int numSteps = 0;
volatile int numReads = 0;
volatile int timeSinceStep10ms = 0;

void TimerInt_Handler(void) //flag to take readings from the accelerometer
{
    Cy_TCPWM_ClearInterrupt(Counter_HW, Counter_CNT_NUM, CY_TCPWM_INT_ON_TC);
    timerTriggered = 1; 
    timeSinceStep10ms++;
}

void stepTaken(void)
{
}

static void WaitForOperation() {
    while(0 != (SensorBus_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY)) {
        CyDelayUs(1);
    }
}

static void WriteRegister(uint8 reg_addr, uint8 data) {
    wbuff[0] = reg_addr;
    wbuff[1] = data;
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
}

static uint8 ReadRegister(uint8 reg_addr) {
    wbuff[0] = reg_addr;
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
    register_setting.buffer = rbuff;
    register_setting.xferPending = false;
    SensorBus_MasterRead(&register_setting);
    WaitForOperation();
    return rbuff[0];
}

int main(void)
{
    __enable_irq(); 

    UART_Start();
    SensorBus_Start();
    Cy_SysClk_PeriphEnableDivider(Clock_DIV_TYPE, Clock_DIV_NUM);
    Counter_Start();
    
    Cy_SysInt_Init(&TimerInt_cfg, TimerInt_Handler);
    NVIC_EnableIRQ(TimerInt_cfg.intrSrc);

    register_setting.slaveAddress = 0x19; 
    
    WriteRegister(LIS3DH_CTRL_REG1, 0x57); 
    WriteRegister(0x21, 0x08); // ENABLE HPF
    //WriteRegister(0x21, 0x00); // DISABLE HPF

    char msg[64];
    UART_PutString("Logging LIS3DH via Timer Interrupt...\r\n");

    for(;;)
    {
        if(timerTriggered)
        {
            
            timerTriggered = 0;
            int16_t x = (int16_t)(ReadRegister(LIS3DH_OUT_X_L) | (ReadRegister(LIS3DH_OUT_X_H) << 8));
            int16_t y = (int16_t)(ReadRegister(LIS3DH_OUT_Y_L) | (ReadRegister(LIS3DH_OUT_Y_H) << 8));
            int16_t z = (int16_t)(ReadRegister(LIS3DH_OUT_Z_L) | (ReadRegister(LIS3DH_OUT_Z_H) << 8));
            
            float normx = (float) x/NORM_FACTOR;
            float normy = (float) y/NORM_FACTOR;
            float normz = (float) z/NORM_FACTOR;
            
            float total_accel = sqrt(normx*normx + normy*normy + normz*normz);
            
            numReads++;
            if(numReads > 100)
            {
                //numSteps++;
                sprintf(msg, "numSteps: %d total accel: %0.2f\r\n", numSteps, total_accel);
                if(timeSinceStep10ms > 30 && total_accel > STEP_THRES)
                {
                    numSteps++;
                    timeSinceStep10ms = 0;
                }
                UART_PutString(msg);
                numReads = 0;
            }
        }
    }
}