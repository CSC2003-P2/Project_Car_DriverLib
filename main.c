/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

//Counter for wheel encoder
volatile uint32_t counter =0;
//TickPeriod for Timer A0
uint32_t volatile tickPeriod =65535;

//static void Delay(uint32_t loop)
//{
//    volatile uint32_t i;
//
//    for (i = 0 ; i < loop ; i++);
//}




void ConfigureTimerA0_UpMode(void)
{
    /* Timer_A UpMode Configuration Parameter */
    const Timer_A_UpModeConfig upConfig =
    {
            TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
            TIMER_A_CLOCKSOURCE_DIVIDER_64,          // 1/(3M/64) = 21.33us per tick
            tickPeriod,                                  // tick period
            TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
            TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
            TIMER_A_DO_CLEAR                        // Clear value
    };

    /* Configuring Timer_A0 for Up Mode */
    Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);

    /* Enabling interrupts and starting the timer */
    Interrupt_enableInterrupt(INT_TA0_0);
    Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);

}

///////////////////////////////////////////PWM for motor////////////////////////////////////////////////
/* Timer_A PWM Configuration Parameter */
//PIN 2.4
Timer_A_PWMConfig pwmConfigLeftMotor =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        1000
};

/* Timer_A PWM Configuration Parameter */
//PIN 2.6
Timer_A_PWMConfig pwmConfigRightMotor =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        1000
};
///////////////////////////////////////////PWM for motor////////////////////////////////////////////////

/////////////////////////////////////////////Motor////////////////////////////////////////////////
void startWheels(void)
{
    //PORT 5 for MOTOR OUTPUT
    GPIO_setAsOutputPin(GPIO_PORT_P5, 0xf3);//11110011
    //MOTOR PWM PINS 5.5 = left, 5.6 = right
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0x30);
    //direction
    // 5.0/5.6=1 & 5.1/5.7=0 = move foward
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, 0x41);//01000001
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0x82);//10000010
}
void motorLeftForward(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN7);
}
void motorLeftReverse(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
}
void motorLeftStop(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
}
void motorRightForward(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
}
void motorRightReverse(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
}
void motorRightStop(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
}
void bothMotorStop(){
    motorLeftStop();
    motorRightStop();
}
void BothMotorForward(){
    motorLeftForward();
    motorRightForward();
}
void BothMotorReverse(){
    motorLeftReverse();
    motorRightReverse();
}
void motorTurnLeft(){
    motorRightForward();
    motorLeftReverse();
}
void motorTurnRight(){
    motorRightReverse();
    motorLeftForward();
}

/////////////////////////////////////////////Motor////////////////////////////////////////////////

uint32_t main(void)
{


    // Configure P1.0 as output
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); //on led
    // Configure P2RGB as output
    GPIO_setAsOutputPin(GPIO_PORT_P2, 0x07);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0x07); //off led

    /* Configuring P1.1 (button) as an input and enable PORT1 interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN1,GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    /* Configuring P1.4 (button) as an input and enable PORT1 interrupts */
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);
    GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN4,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN4);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN4);

    /* Configuring P1.6 for wheel encoder interrupt */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

    /* Configuring P2.4(TA0.1) for Left Motor PWM */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeftMotor);
    /* Configuring P2.6(TA0.3) for Right Motor PWM */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigRightMotor);


    /* Enabling interrupts and starting the watchdog timer */
    Interrupt_enableInterrupt(INT_PORT1);
    Interrupt_enableSleepOnIsrExit();
    Interrupt_enableMaster();

    //TIMER A0 UP MODE
    ConfigureTimerA0_UpMode();

    startWheels();


    while(1)
    {
        PCM_gotoLPM3();
    }
}

//Interrupt handler for PORT 1
void PORT1_IRQHandler(void)
{
    uint32_t status;
    status = GPIO_getInterruptStatus( GPIO_PORT_P1,P1IN);
    GPIO_clearInterruptFlag ( GPIO_PORT_P1, status);

    //Sw1 input at 1.1 to stop both motors
    if(status & GPIO_PIN1){
        //MOTOR PWM PINS 5.6 = left, 5.0 = right
        GPIO_toggleOutputOnPin(GPIO_PORT_P5, 0x41);//0100 0001
    }

    //Sw2 input at 1.4 Increase Motor PWM duty Cycle
    if(status & GPIO_PIN4){
        if(pwmConfigLeftMotor.dutyCycle == 9000)
            pwmConfigLeftMotor.dutyCycle = 1000;
        else
            pwmConfigLeftMotor.dutyCycle += 1000;
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeftMotor);

        if(pwmConfigRightMotor.dutyCycle == 9000)
            pwmConfigRightMotor.dutyCycle = 1000;
        else
            pwmConfigRightMotor.dutyCycle += 1000;
        Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigRightMotor);
    }

    //Wheel Encoder Input at P1.6, blink red led for every 20 notch
    if(status & GPIO_PIN6){
        //One notch in wheel Encoder
        counter= counter+1;
        //If wheel encoder makes one round
        if(counter == 20){
            GPIO_toggleOutputOnPin ( GPIO_PORT_P1, GPIO_PIN0 );
            counter= 0;
        }
    }


}




//TIMER A0 INTERRUPT
void TA0_0_IRQHandler(void)
{
    /* Increment global variable (count number of interrupt occurred) */
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, 0x04);

    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
}



//Interrupt handler for PORT2 *****INITIALIZE in main first
void PORT2_IRQHandler(void){
    uint32_t status;
    status = GPIO_getInterruptStatus( GPIO_PORT_P2 , P2IN  );
    GPIO_clearInterruptFlag ( GPIO_PORT_P2, status);

    if(status & GPIO_PIN5){

    }
}

//////////////////////////////////////////////////////////////////////////////Ultrasonic To Be Integrated with proj above///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
//
///* Standard Includes */
//#include <stdint.h>
//#include <stdbool.h>
//// Start pulse for Ultrasonic timer
//bool start_pulse=0;
//
//const Timer_A_ContinuousModeConfig contmConfig =
//{
//       TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
//
//         TIMER_A_CLOCKSOURCE_DIVIDER_64,          // SMCLK/1 = 3MHz
//        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
//        TIMER_A_DO_CLEAR                        // Clear value
//};
//
//const Timer_A_CompareModeConfig CCR2Config =
//{
//        TIMER_A_CAPTURECOMPARE_REGISTER_2,
//        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
//        TIMER_A_OUTPUTMODE_SET_RESET,
//        0x4000//0xFFFE for very slow
//};
//
//int main(void)
//{
//
//    //Tval1= timer value start of echo, Tval2 = Timer value at end of echo
//    uint32_t tval1, tval2;
//
//    //No. of timer ticks from start of echo to end of echo
//    uint32_t noOfTicksFromUltraSonic;
//
//    //Distance of obstacle from ultraSonicSensor
//    float dist;
//
//    /* Halting the Watchdog */
//    MAP_WDT_A_holdTimer();
//
//    /* Configuring P1.0 as output and P1.1 (switch) as input */
//    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
//
//
//    GPIO_setAsOutputPin(GPIO_PORT_P2, 0x07);
//    GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0x07); //off led
//
//
//    /* IVM Configure P2.5 to output timer TA0.2 (secondary module function, output)*/
//    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION);
//
//    /* IVM Configure P3.0 as input */
//    MAP_GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN0);
//
//
//    /* Configuring Timer_A0 CCR0 and CCR2 then set for continuous Mode */
//    MAP_Timer_A_initCompare(TIMER_A0_BASE, &CCR2Config);   //CCR2 (turns OUT2 on)
//    MAP_Timer_A_configureContinuousMode(TIMER_A0_BASE, &contmConfig);
//    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
//
//    /* Configuring P1.1 (button) as an input and enable PORT1 interrupts */
//    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
//    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
//    MAP_Interrupt_enableInterrupt(INT_PORT1);
//
//    /* Configuring P3.0 (ECHO) as an input and enable PORT3 interrupts */
//    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
//    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN0);
//    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN0, GPIO_LOW_TO_HIGH_TRANSITION);
//    MAP_Interrupt_enableInterrupt(INT_PORT3);
//
//    /* Enabling MASTER interrupts */
//    MAP_Interrupt_enableMaster();
//
//    printf("Starting now\n");
//
//    while (1)
//    {
//        // If distance < 20cm
//        if (dist<20){
//            //Turn ON LED 2
//            GPIO_setOutputHighOnPin(GPIO_PORT_P2, 0x07); //off led
//        }// Distance > 20cm
//        else{
//            // TUrn OFF LED
//            GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0x07); //off led
//        }
// //       MAP_PCM_gotoLPM3(); /* Going to Low Power Mode 3*/
//          if(start_pulse==1)
//          {
//              printf("Starting now\n");
//              //Get timer value of start
//              tval1 = MAP_Timer_A_getCounterValue(TIMER_A0_BASE);
//              //busy wait for the timer input value to drop
//              while((MAP_GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN0) & 0x1)==1);
//              //Get timer value of end
//              tval2 = MAP_Timer_A_getCounterValue(TIMER_A0_BASE);
//              //Reset Pulse
//              start_pulse=0;
//
//              //If tval1 > tval2 indicating timer overflow
//              if(tval1 > tval2)   //it means the 16-bit counter hit 0xffff and wrapped back to 0x0000
//                   tval2=tval2+0xffff;
//
//              //Interval of ticks between start and end
//              noOfTicksFromUltraSonic = tval2-tval1;
//
//              printf(" noOfTicksFromUltraSonic: %d \n", (int)(noOfTicksFromUltraSonic));
//
//
//              //Distance = (SPEED OF SOUND) * ((TimerAtickDuration *noOfTicksFromUltraSonic)/2)
//              dist=( 100.0 * 340.0) * ( (0.000021333 * (float)noOfTicksFromUltraSonic )*0.5)  ;
//
//              printf("%d cm\n", (int)(dist));
//          }
//
//    }
//}
//
///* GPIO ISR for PORT 1 (the button) */
//void PORT1_IRQHandler(void)
//{
//    uint32_t status;
//
//    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);
//
//    /* Toggling the output on the LED */
//    if(status & GPIO_PIN1)
//    {
//        MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
//    }
//
//}
//
//
///* GPIO ISR for PORT 3 (the echo value) */
//void PORT3_IRQHandler (void)
//{
//    uint32_t status;
//
//    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
//    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);
//
//    if(status & GPIO_PIN0) //if this was triggered by PIN0
//        start_pulse=1;
//}


