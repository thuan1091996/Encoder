/*
 * Double_H_Bridges.c
 *
 *  Created on: Jul 8, 2019
 *      Author: Itachi
 */

#include "Userlibs.h"
#include "BridgeH.h"


/* -----------BridgeH_GPIO_Init---------------
 * GPIO Initialization for Bridge-H to control motors's direction (IN1 -> IN4 Pins)
 * Input: No
 * Output: No
 * Note: Must configure at least 8 mA for Bridge H input pins
 */
void BridgeH_GPIO_Init(void)
{
    SysCtlPeripheralEnable(BRIDGEH_GPIO_PERIPH);                                                                //Enable Bridge-H GPIO port
    while(!SysCtlPeripheralReady(BRIDGEH_GPIO_PERIPH));
    GPIOPinTypeGPIOOutput(BRIDGEH_GPIO_PORT,IN1_PIN| IN2_PIN| IN3_PIN| IN4_PIN);
    GPIOPadConfigSet(BRIDGEH_GPIO_PORT, IN1_PIN|IN2_PIN|IN3_PIN|IN4_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD); //Setup pins output 8mA is a MUST
}

/* -----------BridgeH_PWM_Init---------------
 * PWM Initialization for Bridge-H to control motors's speed (ENA, ENB Pins)
 * Input: F_PWM
 * Output: No
 * Note: Must configure at least 8 mA for Bridge H input pins
 */
void BridgeH_PWM_Init(void)
{
    SysCtlPeripheralEnable(BRIDGEH_PWM_GPIO_PERIPH);
    while(!SysCtlPeripheralReady(BRIDGEH_PWM_GPIO_PERIPH));
    SysCtlPeripheralEnable(BRIDGEH_PWM_PERIPH);
    while(!SysCtlPeripheralReady(BRIDGEH_PWM_PERIPH));

    GPIOPinConfigure(GPIO_ENACONFIG);
    GPIOPinConfigure(GPIO_ENBCONFIG);
    GPIOPinTypePWM(BRIDGEH_PWM_GPIO_PORT, (ENA_PIN|ENB_PIN));

    PWMClockSet(BRIDGEH_PWM_PORT,PWM_SYSCLK_DIV_8);
    PWMGenConfigure(BRIDGEH_PWM_PORT, BRIDGEH_PWM_GEN, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(BRIDGEH_PWM_PORT, BRIDGEH_PWM_GEN, LOAD_PWM);
    PWMGenEnable(BRIDGEH_PWM_PORT, BRIDGEH_PWM_GEN);
}
void Motor1_Forward(void)
{
    uint8_t     temp=0;
    temp=DATA_PE_R;
    temp&= MOTOR1;
    if(temp!=MOTOR1_FORWARD)
    {
        DATA_PE_R&=~MOTOR1;
//        SysCtlDelay(SysCtlClockGet()/6);
        DATA_PE_R|=MOTOR1_FORWARD;

    }
}
void Motor1_Backward(void)
{
    uint8_t     temp=0;
    temp=DATA_PE_R;
    temp&= MOTOR1;
    if(temp!=MOTOR1_BACKWARD)
    {
        DATA_PE_R&=~MOTOR1;
//        SysCtlDelay(SysCtlClockGet()/6);
        DATA_PE_R|=MOTOR1_BACKWARD;

    }
}
void Motor1_Stop(void)
{
    DATA_PE_R&=~MOTOR1;
}
void Motor2_Forward(void)
{
    uint8_t     temp=0;
    temp=DATA_PE_R;
    temp&= MOTOR2;
    if(temp!=MOTOR2_FORWARD)
    {
        DATA_PE_R&=~MOTOR2;
//        SysCtlDelay(SysCtlClockGet()/6);
        DATA_PE_R|=MOTOR2_FORWARD;
    }
}
void Motor2_Backward(void)
{
    uint8_t     temp=0;
    temp=DATA_PE_R;
    temp&= MOTOR2;
    if(temp!=MOTOR2_BACKWARD)
    {
        DATA_PE_R&=~MOTOR2;
//        SysCtlDelay(SysCtlClockGet()/6);
        DATA_PE_R|=MOTOR2_BACKWARD;
    }
}
void Motor2_Stop(void)
{
    DATA_PE_R &= ~MOTOR2;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Update_Speed(uint8_t ui8_M1Duty, uint8_t ui8_M2Duty)
{
    static uint16_t temp1_prev=0;
    static uint16_t temp2_prev=0;

    uint16_t temp_load1=LOAD_PWM/100*ui8_M1Duty;
    uint16_t temp_load2=LOAD_PWM/100*ui8_M2Duty;

    if ( (temp_load1!=temp1_prev) || (temp_load2!=temp2_prev) )
    {
        PWMPulseWidthSet(BRIDGEH_PWM_PORT, PWM_OUT_4,temp_load1);
        PWMPulseWidthSet(BRIDGEH_PWM_PORT, PWM_OUT_5,temp_load2);

        if(ui8_M1Duty!=0)
        PWMOutputState(BRIDGEH_PWM_PORT, PWM_OUT_4_BIT, true);
        else
        PWMOutputState(BRIDGEH_PWM_PORT, PWM_OUT_4_BIT, false);

        if(ui8_M2Duty!=0)
        PWMOutputState(BRIDGEH_PWM_PORT, PWM_OUT_5_BIT, true);
        else
        PWMOutputState(BRIDGEH_PWM_PORT, PWM_OUT_5_BIT, false);

        temp1_prev=temp_load1;
        temp2_prev=temp_load2;
    }
}
