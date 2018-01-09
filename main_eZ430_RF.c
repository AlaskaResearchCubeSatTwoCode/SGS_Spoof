/* --COPYRIGHT--,BSD
 * Copyright (c) 2011, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//  Demo Application for MSP430 eZ430-RF2500 / CC1100-2500 Interface
//  Main code application library v1.2
//
// W. Goh
// Version 1.2
// Texas Instruments, Inc
// December 2009
// Built with IAR Embedded Workbench Version: 4.20
//******************************************************************************
// Change Log:
//******************************************************************************
// Version:  1.2
// Comments: Add startup delay for startup difference between MSP430 and CCxxxx
// Version:  1.1
// Comments: Main application code designed for eZ430-RF2500 board
// Version:  1.00
// Comments: Initial Release Version
//******************************************************************************

#include "include.h"
#include <msp430.h>

extern char paTable[];
extern char paTableLen;
extern char GS_COMM_RF_OFF[36];

char txBuffer[4];
char rxBuffer[4];
unsigned int i = 0;

void main (void)
{

  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // 5ms delay to compensate for time to startup between MSP430 and CC1100/2500
  __delay_cycles(5000);
  TI_CC_SPISetup();                         // Initialize SPI port
  P2SEL = 0;                                // Sets P2.6 & P2.7 as GPIO
  TI_CC_PowerupResetCCxxxx();               // Reset CCxxxx
  writeRFSettings();                        // Write RF settings to config reg
  TI_CC_SPIWriteBurstReg(TI_CCxxx0_PATABLE, paTable, paTableLen);//Write PATABLE

  // Configure ports -- switch inputs, LEDs, GDO0 to RX packet info from CCxxxx
  TI_CC_SW_PxREN = TI_CC_SW1;               // Enable Pull up resistor
  TI_CC_SW_PxOUT = TI_CC_SW1;               // Enable pull up resistor
  TI_CC_SW_PxIES = TI_CC_SW1;               // Int on falling edge
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);           // Clr flags
  TI_CC_SW_PxIE = TI_CC_SW1;                // Activate interrupt enables
  TI_CC_LED_PxOUT &= ~(TI_CC_LED1 + TI_CC_LED2); // Outputs = 0
  TI_CC_LED_PxDIR |= TI_CC_LED1 + TI_CC_LED2;// LED Direction to Outputs

  TI_CC_GDO0_PxIES |= TI_CC_GDO0_PIN;       // Int on falling edge (end of pkt)
  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // Clear flag
  TI_CC_GDO0_PxIE |= TI_CC_GDO0_PIN;        // Enable int on end of packet

  TI_CC_SPIStrobe(TI_CCxxx0_SRX);           // Initialize CCxxxx in RX mode.
                                            // When a pkt is received, it will
                                            // signal on GDO0 and wake CPU
  TI_CC_SW_PxIFG = TI_CC_SW1;

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, enable interrupts
}


// The ISR assumes the interrupt came from a pressed button
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR (void)
{
  unsigned int k;
  // If Switch was pressed
  if(TI_CC_SW_PxIFG & TI_CC_SW1)
  {
    char GS_COMM_RF_OFF[36]={0x55,0x2F,0x20,0x23,0x0D,0xEB,0x8D,0xD5,0xCB,0x30,0xAE,0x17,0xAC,0xDE,0x6F,0x30,0x0E,0xB9,0xA5,0xA3,0xDA,0x17,0x81,0x6B,0xEF,0xD0,0x62,0x8D,0x12,0x33,0x15,0x34,0x3E,0x8A,0xDF,0x00};
    // Build packet

  // NOTE Justin added this Byte reverse because the above definition is byte reversed from what the satellite needs.
  for(k=0;k<36;k++){
    GS_COMM_RF_OFF[k] = __bit_reverse_char(GS_COMM_RF_OFF[k]);
  }

    txBuffer[0] = 2;                                // Packet length
    txBuffer[1] = 0x01;                             // Packet address
    txBuffer[2] = (~TI_CC_SW_PxIFG << 1) & 0x02;    // Load switch inputs
    RFSendPacket(GS_COMM_RF_OFF,36);               // Send value over RF
    __delay_cycles(5000);                           // Switch debounce
  }
  TI_CC_SW_PxIFG &= ~(TI_CC_SW1);           // Clr flag that caused int
}

// The ISR assumes the interrupt came from GDO0. GDO0 fires indicating that
// CCxxxx received a packet
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    // if GDO fired
  if(TI_CC_GDO0_PxIFG & TI_CC_GDO0_PIN)
  {
    char len=2;                             // Len of pkt to be RXed (only addr
                                            // plus data; size byte not incl b/c
                                            // stripped away within RX function)
    if (RFReceivePacket(rxBuffer,&len))     // Fetch packet from CCxxxx
    TI_CC_LED_PxOUT ^= rxBuffer[1];         // Toggle LEDs according to pkt data
  }

  TI_CC_GDO0_PxIFG &= ~TI_CC_GDO0_PIN;      // After pkt RX, this flag is set.
}
