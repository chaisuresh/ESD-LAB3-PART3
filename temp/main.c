/*
 * Author- Chaithra Suresh
 * Description: MSP432 code for ESD-lab3 part3
 * when the code is compiled, we get an output PWM with 55% duty cycle. typing p (small case) on the terminal gives out the current dutycycle
 * the serial port echoes back anything that is pressed
 * pressing button4, decreases the duty cycle by 10%, whereas pressing the button1 increases the duty cycle by 10%
 * Temperature sensor is integrated. for every 0.5degC change, PWM changes by 10%
 * supplement- pressing the plus sign- + on the terminal increases duty cycle by 10% and minus sign - decreases duty cycle by 10%
 * supplemnt2- pressing capital T gives temperature in Farenheit
 *
 *
 * Functions: main.c contains all the interrupt handlers and function calls
 * initialise.c contains initilaisation for all ports
 *
 *
 * reference: MSP432 examples- UART, ADC14
 *
 */
#include "msp.h"
#include "initialise.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


/*
 *
 * declaration of global vraiables and functions
 */
volatile uint32_t duty=2750; // initialising dudty cycle to 55%
void timer_PWM(duty);          // calling the function that decides PWM
volatile char data;             //data read by the terminal
volatile float temp;            //raw data read by the temp sensor
float IntDegF;
float IntDegC;

int main(void)
{
    // Variables to store the ADC temperature reference calibration value
    int32_t adcRefTempCal_1_2v_30;
    int32_t adcRefTempCal_1_2v_85;

    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
                 WDT_A_CTL_HOLD;

     init_function();

    // Read the ADC temperature reference calibration value
    adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;
    adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;

    // Initialize the shared reference module
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);// If ref generator busy, WAIT
    REF_A->CTL0 |= REF_A_CTL0_VSEL_0 |      // Enable internal 1.2V reference
            REF_A_CTL0_ON;                  // Turn reference on

    REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;       // Enable temperature sensor

    // Configure ADC - Pulse sample mode; ADC14_CTL0_SC trigger
    ADC14->CTL0 |= ADC14_CTL0_SHT0_6 |      // ADC ON,temperature sample period>5us
            ADC14_CTL0_ON |
            ADC14_CTL0_SHP;
    ADC14->CTL1 |= ADC14_CTL1_TCMAP;        // Enable internal temperature sensor
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_1 |  // ADC input ch A22 => temp sense
            ADC14_MCTLN_INCH_22;
    ADC14->IER0 = 0x0001;                   // ADC_IFG upon conv result-ADCMEM0

    // Wait for reference generator to settle
    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));

    ADC14->CTL0 |= ADC14_CTL0_ENC;


    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
        TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
        TIMER_A0->CCR[0] = 5000;
        timer_PWM(duty);
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK, UP mode
                               TIMER_A_CTL_MC__UP;

    // Enable global interrupt
    __enable_irq();

    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);// Enable ADC interrupt in NVIC module
    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31); //enable switches interrupt
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);//enable uart interrupt

    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Wake up on exit from ISR

    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();
    float count=0;

    while(1)
    {
        ADC14->CTL0 |= ADC14_CTL0_SC;       // Sampling and conversion start

        __sleep();
        __no_operation();                   // Only for debugger

        // Temperature in Celsius
        // The temperature (Temp, C)=

        IntDegC = (( temp - adcRefTempCal_1_2v_30) * (85 - 30)) /
                       (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;

        // Temperature in Fahrenheit
        // Tf = (9/5)*Tc | 32
        IntDegF = ((9 * IntDegC) / 5) + 32;

        printf(" %f ", IntDegC);
        printf("\n");

                            if(IntDegC>27)
                            {
                                count=(IntDegC-27)/0.5;
                                while(((duty/50)- (int)count)<5) count--; //makes sure the duty cycle doesnt exceed the boundary

                                duty= duty - ((int)count*500); //for every 0.5deg change, duty cycle changes by 10%
                                timer_PWM(duty);




                            }
                            else if(IntDegC<27)
                            {
                                count=(27-IntDegC)/0.5;
                                while (((duty/50)+ (int)count)>95) count--; //makes sure the duty cycle doesnt exceed the boundary


                                duty= duty + ((int)count*500); //for every 0.5deg change, duty cycle changes by 10%
                                timer_PWM(duty);


                            }



        __no_operation();                   // Only for debugger
    }
}

// ADC14 interrupt service routine for temperature sensor
void ADC14_IRQHandler(void)
{
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG0)
    {
        temp = ADC14->MEM[0];
    }
}

//Function that creates required PWM
void timer_PWM(duty)
{
        TIMER_A0->CCTL[1] = 0x0040; // reset-set mode
        TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_CCIE;
        TIMER_A0->CCR[1] = duty; //load duty value - volatile - which controls the dutycycle
}


// inetruupt for Port1- switches 1 and 4
void PORT1_IRQHandler(void)
{
    volatile uint32_t i=0,j=0;

    /*
     * If switch 4 is pressed and duty cycle is not at its limit, decrease it by 10%
     */
    if(P1->IFG & BIT4)
         {
            if(duty<4749)
            {
             duty=duty+500;
             timer_PWM(duty);
            }

         }


         /*
         * If switch 1 is pressed and duty cycle is not at its limit, increase it by 10%
         */
    if(P1->IFG & BIT1)
             {
                   if(duty>251)
                    {
                 duty=duty-500;
                 timer_PWM(duty);
                    }
             }

                    for(i = 0; i < 10000; i++)
                    {
                     P1->IFG &= ~BIT4;
                     P1->IFG &= ~BIT1;
                    }

}

/*
 * UART Interrupt handler
 * reads from serial port, echoes back
 * if p is pressed, prints duty cycle
 * if t is pressed, prints temperature in C
 * if T is pressed, prints temperature in F
 * if + is pressed, increases duty cyle by 10%
 * if - is pressed , decreases duty cycle by 10%
 *
 */
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {

        uint32_t duty_temp,div;
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

        data= EUSCI_A0->RXBUF;

        if(data == 112)
           {
            duty_temp=duty/50;
            div=10;
            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF= '\n';
            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF= '\r';
            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF= 'p';
            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
            EUSCI_A0->TXBUF= '=';

            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
             while(div!=0)
                {


                while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                EUSCI_A0->TXBUF= (duty_temp/div)+'0';

                duty_temp= duty_temp%div ;
                div=div/10;
                }

            }

        else if((data == 116) | (data == 84))
        {
                        if (data == 116) duty_temp=IntDegC;
                        else if(data == 84) duty_temp=IntDegF;
                        div=1000;
                        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                        EUSCI_A0->TXBUF= '\n';
                        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                        EUSCI_A0->TXBUF= '\r';
                        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                        EUSCI_A0->TXBUF= 't';
                        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                        EUSCI_A0->TXBUF= '=';

                        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                         while(div!=0)
                            {


                            while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                            EUSCI_A0->TXBUF= (duty_temp/div)+'0';

                            duty_temp= duty_temp%div ;
                            div=div/10;
                            }


                         if (data == 116)
                         {
                             while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                                                     EUSCI_A0->TXBUF= 'C';
                         }
                         else if(data == 84)
                         {
                             while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
                                                     EUSCI_A0->TXBUF= 'F';
                         }
          }

        else
                    // Echo the received character back
                                             EUSCI_A0->TXBUF = data;


         if(data== 45)
        {

            if(duty!=250)
              {
                 duty=duty-500;
                 timer_PWM(duty);
              }


        }

        else if(data== 43)
               {

                if(duty!=4750)
                 {
                   duty=duty+500;
                    timer_PWM(duty);
                 }


               }



}
}



