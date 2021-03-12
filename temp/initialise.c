/*
 * initialise.c
 *
 *  Created on: Mar 12, 2021
 *      Author: chsu2570
 */

#include "msp.h"
#include "initialise.h"

void init_function()
{




 /******************************************************GPIO**********************************************************************/
        // Configure GPIO
        P1->DIR |= BIT0;
        P1->OUT |= BIT0;

        //configue PWM OUT pin
        P2->SEL0 |= 0x30;
        P2->SEL1 &= ~0x30;
        P2->DIR |= 0x030;

        //Configue switch 4
        P1->DIR = ~(uint8_t) BIT4;
        P1->OUT = BIT4;
        P1->REN = BIT4;                         // Enable pull-up resistor (P1.1 output high)
        P1->SEL0 = 0;
        P1->SEL1 = 0;
        P1->IES = BIT4;                         // Interrupt on high-to-low transition
        P1->IFG = 0;                            // Clear all P1 interrupt flags
        P1->IE = BIT4;

        //Configue switch 1
        P1->DIR = ~(uint8_t) BIT1;
        P1->OUT |= BIT1;
        P1->REN |= BIT1;                         // Enable pull-up resistor (P1.1 output high)
        P1->SEL0 = 0;
        P1->SEL1 = 0;
        P1->IES |= BIT1;                         // Interrupt on high-to-low transition
        P1->IFG = 0;                            // Clear all P1 interrupt flags
        P1->IE |= BIT1;


/********************************************************UART**********************************************************************/
            //configure UART- serial communication
            CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
            CS->CTL0 = 0;                           // Reset tuning parameters
            CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
            CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
                        CS_CTL1_SELS_3 |                // SMCLK = DCO
                        CS_CTL1_SELM_3;                 // MCLK = DCO
            CS->KEY = 0;                            // Lock CS module from unintended accesses





              // Configure UART pins
              P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pin as secondary function

                  // Configure Port J
               PJ->DIR |= (BIT0| BIT1 | BIT2 | BIT3);
               PJ->OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3);

              // Enable PCM rude mode, which allows to device to enter LPM3 without waiting for peripherals
                PCM->CTL1 = PCM_CTL0_KEY_VAL | PCM_CTL1_FORCE_LPM_ENTRY;




                EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
                 EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
                   EUSCI_B_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
                 // Baud Rate calculation
                 // 12000000/(16*9600) = 78.125
                 // Fractional portion = 0.125
                 // User's Guide Table 21-4: UCBRSx = 0x10
                 // UCBRFx = int ( (78.125-78)*16) = 2
                 EUSCI_A0->BRW = 78;                     // 12000000/16/9600
                 EUSCI_A0->MCTLW = (2 << EUSCI_A_MCTLW_BRF_OFS) |
                                     EUSCI_A_MCTLW_OS16;

                 EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST; // Initialize eUSCI
                 EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;    // Clear eUSCI RX interrupt flag
                 EUSCI_A0->IE |= EUSCI_A_IE_RXIE;        // Enable USCI_A0 RX interrupt







}
