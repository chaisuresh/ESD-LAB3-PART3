/*
 * Author- Chaithra Suresh
 * Description: MSP432 code for ESD-lab3 part3
 * when the code is compiled, we get an output PWM with 55% duty cycle. typing p (small case) on the terminal gives out the current dutycycle
 * the serial port echoes back anything that is pressed
 * pressing button4, decreases the duty cycle by 10%, whereas pressing the button1 increases the duty cycle by 10%
 * Temperature sensor is integrated.- but itsnt working fine, It is seperately submitted.
 * supplement- pressing the plus sign- + on the terminal increases duty cycle by 10% and minus sign - decreases duty cycle by 10%
 *
 *
 * Functions: main.c contains all the interrupt handlers and function calls
 * init_funct.c contains initilaisation for all ports
 *
 */






#include "msp.h"
#include "init_func.h"

#include <stdint.h>
#include <stdint.h>
#include <stdio.h>



/*
 *
 * declaration of global vraiables and functions
 */
volatile uint32_t duty=2750; // initialising dudty cycle to 55%
void timer_PWM(duty);          // calling the function that decides PWM
void temperature(void);         // converts raw temperature data and converts to degC and degF
volatile char data;             //data read by the terminal
volatile float temp;            //raw data read by the temp sensor
volatile float IntDegF;         //data in degree farenheit
volatile float IntDegC;         //data in degree Celsius





int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW |             // Stop WDT
            WDT_A_CTL_HOLD;

    init_function();        //initalises all ports


    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;    // Enable sleep on exit from ISR


    // Ensures SLEEPONEXIT takes effect immediately
    __DSB();

    // Enable global interrupt
    __enable_irq();


    NVIC->ISER[1] = 1 << ((PORT1_IRQn) & 31); //enable switches interrupt
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31); //enable timer interrupt
    NVIC->ISER[0] = 1 << ((EUSCIA0_IRQn) & 31);//enable uart interrupt
    NVIC->ISER[0] = 1 << ((ADC14_IRQn) & 31);// Enable ADC interrupt in NVIC module


    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    TIMER_A0->CCTL[0] = TIMER_A_CCTLN_CCIE; // TACCR0 interrupt enabled
    TIMER_A0->CCR[0] = 5000;
    timer_PWM(duty);
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK | // SMCLK, UP mode
                           TIMER_A_CTL_MC__UP;



}

//Function that creates required PWM
void timer_PWM(duty)
{
        TIMER_A0->CCTL[1] = 0x0040; // reset-set mode
        TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_CCIE;
        TIMER_A0->CCR[1] = duty; //load duty value - volatile - which controls the dutycycle
}


// ADC14 interrupt service routine
void ADC14_IRQHandler(void)
{
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG0)
    {
        temp = ADC14->MEM[0]; //read temperature data
        void temperature();
    }
}


void PORT1_IRQHandler(void)
{
    volatile uint32_t i=0,j=0;

    /*
     * If switch 4 is pressed and duty cycle is not at its limit, decrease it by 10%
     */
    if(P1->IFG & BIT4)
         {
            if(duty!=4750)
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
                   if(duty!=250)
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
 * if P is pressed, prints duty cycle
 * if T is pressed, prints temperature
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

        else if(data == 116)
          {
                        duty_temp=IntDegC;
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


// Timer A0_0 interrupt service routine

void TA0_0_IRQHandler(void) {
    // Clear the compare interrupt flag
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

    P1->OUT ^= BIT0;                        // Toggle P1.0 LED
}

/*
 * Enables temp interrupt and converts raw temperature to deg Celsius and deg Farenheit
 */

void temperature()
{


    ADC14->CTL0 |= ADC14_CTL0_SC;       // Sampling and conversion start
    float count=0;


                // Temperature in Celsius
                // The temperature (Temp, C)=
                IntDegC = (( temp - adcRefTempCal_1_2v_30) * (85 - 30)) /
                (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;


                 // Temperature in Fahrenheit
                 // Tf = (9/5)*Tc | 32
                 IntDegF = ((9 * IntDegC) / 5) + 32;

                 printf(" %f ", IntDegC);
                 printf("\n");

                 if(temp>25)
                 {
                     count=temp-25.0f;
                     duty= duty - ((count/0.2)*10);
                     timer_PWM(duty);
                 }
                 else if(temp<25)
                 {
                     count=25.0f-temp;
                     duty= duty + ((count/0.2)*10);
                     timer_PWM(duty);
                 }


}
