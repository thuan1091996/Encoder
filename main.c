/* --------0.Project information--------------------
 * Measuring Encoder output signals
 * Author : TRAN MINH THUAN
 * Date: Sept, 23 2019
 * Project associate with TM4C123, CCS version 9.
---------------------------------------------------*/


/* --------1.System requirement---------------------
1. Read Channel A output signal with external interrupts PD6 (both edges)
    1.1: Dong co quay 1 vong -> Encoder quay 33 vong (Ty so truyen 1:33)
    1.2: Encoder co 11 xung, moi xung tao ra 2 xung canh len/xuong
    1.3: So xung dong co tao ra sau 1 vong quay = 33*11*2 = 726 (xung/vong)
2. Read pulses created by both channel A & B with QEI
3. Calculate velocity of DC motor with QEI
---------------------------------------------------*/



////// ----------------2. Pre-processor Directives Section--------------------///////
#include "UserLibraries/Userlibs.h"
#include "UserLibraries/BridgeH.h"
#include "UserLibraries/Systick.h"
#include "driverlib/gpio.c"

#define SPEED_SAMPLE_TIME       0.01  //0.01s
#define RESOLUTION              1324  //From real-experiments
//////------------------------------------------------------------------------///////

////// ----------------3.Global Declarations Section--------------------------///////
unsigned char ui8FreshFlag=0;           //Speed fresh flag
unsigned char ui8Duty1=0,ui8Duty2;
unsigned long ui32CountA=0;             //Store output signals of Channel A
unsigned long ui32CountAB=0;            //Store output signals of both Channels A and B
unsigned long ui32SampleCount=0;        //Count number of sample recorded
float         deg=0.0;
float         Speed=0;
float         pos_offset=0;                     //Position offset
float         Ref_Speed=300;
uint8_t Test_Bridge=0;
int8_t  Test_Duty1=0,Test_Duty2=0;
//////------------------------------------------------------------------------///////

/* -----------CHANNELA_ISREDGE---------------
 * Count rising/falling edges creating each revolution
 * External interrupt PA6 handler, both edges
 * Optional: Configure for PD7 as well to count pulse edges on both channels
 * Input: No
 * Output: No
 */
void CHANNELA_ISREDGE()
{
    if(GPIOIntStatus(GPIO_PORTD_BASE,true) == GPIO_INT_PIN_6)
    {
        GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_6);
        ui32CountA++;
    }
//    Optional
//    if(GPIOIntStatus(GPIO_PORTD_BASE,true) == GPIO_INT_PIN_7)
//    {
//        GPIOIntClear(GPIO_PORTD_BASE,GPIO_INT_PIN_7);
//        ui32CountAB++;
//    }
};

/* -----------QEI0_INTHandler---------------
 * Velocity Timer expire of QEI0 interrupt handler
 * Use global variable - " ui8FreshFlag " to indicate new period of sample time
 *                     - " ui32CountA "   to show number of sample
 * Input: No
 * Output: No
 */
void QEI0_INTHandler()
{
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);
    ui8FreshFlag=1;
    ui32SampleCount++;
};

/* -----------GPIO_Init---------------
 * GPIO Initialization to read Rising/Falling edge on Channel A per motor Revolution
 * Setup external interrupt on both edges
 * Optional: Configure for PD7 as well to count pulse edges on both channels
 * Input: No
 * Output: No
 */
void GPIO_Init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);    //Pull-up is necessary
    //---------Interrupt configuration---------
    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_6, GPIO_BOTH_EDGES);
    GPIOIntRegister(GPIO_PORTD_BASE, CHANNELA_ISREDGE);
    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_6);
//  Optional
//    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY; //Unlock PD7
//    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;
//    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);    //Pull-up is necessary
//    GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_7, GPIO_BOTH_EDGES);
//    GPIOIntEnable(GPIO_PORTD_BASE, GPIO_INT_PIN_7);
};

/* -----------QEI_Init ---------------
 * QEI Initialization for PD6 PD7
 * QEI Quadrature mode, no swap, no index,no Filter, "RESOLUTION" rising/falling edges per motor revolution
 * Input: ui32NumbPulses - Number of Rising/Falling edges per motor revolution
 * Output: No
 */
void QEI_Init(uint32_t ui32NumbPulses)
{
    //Enable peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));
    //Unlock PD7
    HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE+GPIO_O_CR) |= GPIO_PIN_7;
    //QEI Configure
    QEIDisable(QEI0_BASE);  //First disable  to configure
    QEIIntDisable(QEI0_BASE, (QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX));
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_7);
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    // instruction below *100 because in case Sample Time >1s, QEIMAXPOS reg may not overload because there more than 1324 pulses /sec
    // no need cuz QEI_COUNT_R and QEI_MAXPOS work independently
    QEIConfigure(QEI0_BASE, QEI_CONFIG_CAPTURE_A_B|QEI_CONFIG_NO_RESET|QEI_CONFIG_QUADRATURE|QEI_CONFIG_NO_SWAP, ui32NumbPulses-1);
    QEIPositionSet(QEI0_BASE, 0);       //Reset counter
//  Filter
//    QEIFilterConfigure(QEI0_BASE, QEI_FILTCNT_7);
//    QEIFilterEnable(QEI0_BASE);
    QEIEnable(QEI0_BASE);
};

/* -----------QEI_VelocityInit ---------------
 * QEI Velocity Initialization with Timer clock = System clock
 * Setting for QEI Timer Expire interrupt to interrupt every period
 * Input: Sample time (sec)
 * Output: No
 */
void QEI_VelocityInit(double SampleTime)
{
    QEIVelocityDisable(QEI0_BASE);  //Disable before configure
    QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1, (SysCtlClockGet()*SampleTime));
    QEIVelocityEnable(QEI0_BASE);
    //Interrupt configure
    QEIIntRegister(QEI0_BASE, QEI0_INTHandler);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);  //Velocity timer expires
};

/* -----------Update_Position ---------------
 * Update position by accumulated position overtime
 * Input: No
 * Output: No
 */
void Update_Position(void)
{
    static uint32_t pos_temp=0;
    static int8_t   dir_temp=0;
    float pos_offset=0;                             //Position offset
    pos_temp=QEIPositionGet(QEI0_BASE);
    dir_temp=QEIDirectionGet(QEI0_BASE);
    if( dir_temp==1)                                //CW direction
    {
        if(pos_temp!=0)
        {
            QEIPositionSet(QEI0_BASE, 0);           //Reset position
            pos_offset=(pos_temp*360)/1323;
            deg+=pos_offset;
        }
    }
    else if (dir_temp==-1)                          //CCW direction
    {
        if(pos_temp!=0)
        {
            QEIPositionSet(QEI0_BASE, 0);           //Reset position
            pos_offset=360.0-((pos_temp*360)/1323);
            deg-=pos_offset;
        }
    }
}

/* -----------QEI0_INTHandler---------------
 * Velocity Timer expire of QEI0 interrupt handler
 * Use global variable - " ui8FreshFlag " to indicate new period of sample time
 *                     - " ui32CountA "   to show number of sample
 * Input: No
 * Output: No
 */
uint8_t PID_Controller(double Kp,
                    double Ki,
                    double Kd,
                    float Error,
                    float CurrentState)
{
    uint8_t pid_drive=(uint8_t)Error*Kp;
    return pid_drive;
};

////// ----------------4. Subroutines Section---------------------------------///////
void main(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_2_5| SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80MHz
//    GPIO_Init();
    QEI_Init(RESOLUTION);                   //QEI Initialization
    QEI_VelocityInit(SPEED_SAMPLE_TIME);    //QEI Velocity Initialization
    BridgeH_GPIO_Init();                    //Bridge - H GPIO Initialization
    BridgeH_PWM_Init();                     //Bridge - H PWM Initialization
//    Motor1_Forward();
//    Motor2_Forward();
    Motor1_Stop();
    Motor2_Stop();
    IntMasterEnable();                      //Enable Interrupt
    while(1)
    {
        /* /////////////////////// Test Bridge H ////////////////////// */
//        if(Test_Bridge == 10)
//        {
//            Motor1_Stop();
//        }
//        if(Test_Bridge == 11)
//        {
//            Motor1_Forward();
//        }
//        if(Test_Bridge == 12)
//        {
//            Motor1_Backward();
//        }
//        if(Test_Bridge == 20)
//        {
//            Motor2_Stop();
//        }
//        if(Test_Bridge == 21)
//        {
//            Motor2_Forward();
//        }
//        if(Test_Bridge == 22)
//        {
//            Motor2_Backward();
//        }
        Update_Speed(Test_Duty1,Test_Duty2);
        //////////////////////////////////////////////////////////////////
    }
//          Update_Position();
//        if(ui8FreshFlag==1)
//        {
//            Speed=(QEIVelocityGet(QEI0_BASE)*60)/(1324*SPEED_SAMPLE_TIME);
////            ui8Duty1=PID_Controller(3, 0, 0, Ref_Speed-Speed, Speed);
//            Update_Speed(ui8Duty1, ui8Duty2);
//        }
//        SysCtlDelay(SysCtlClockGet()/30);
//    }
}
//////------------------------------------------------------------------------///////
