/*  Authors:
 *      Jose Carlos Paniagua Mendoza
 *      Andrés Apellaniz Solley
 *
 *  Date:
 *      05/12/18
 *
 *  Instructor:
 *      Omar Piña
 *
 *  Created with the help of the Mfrc522 library to simplify the manipulation of the sensor.
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"
#include "LIB/Mfrc522.h"
#include "inc\tm4c123gh6pm.h"

/* PIN Connections:
 * Used SSI2 (Module 2)
 *
 * SDA / CS / FSS ------------ PB5
 * SCK  / CLK     ------------ PB4
 * MOSI / TX      ------------ PB7
 * MISO /  RX     ------------ PB6
 * RST            ------------ PF0
 */

#define redLED          0x02    // PF1
#define blueLED         0x04    // PF2
#define greenLED        0x08    // PF3
#define chipSelectPin   0x20    // PB5
#define NRSTPD          0x01    //PF0
#define LOAD_0          8000000
#define CARD_LENGTH     10
#define ID_LENGTH       5
#define CARD_NUM        3


int dumpHex(unsigned char* buffer, int len); // Display the card data bytes in Hexadecimal in the console.
void InitSSI();
void InitConsole(); // Initialize the UART of the devices to communicate each other, to display information

uint8_t status;                                                 // gets the status of the sensor when reading
unsigned char str[MAX_LEN];
unsigned char cardID[CARD_LENGTH];                              // gets the card data
// define the valid CARDS for testing.
unsigned int masterID[CARD_NUM][ID_LENGTH] = {
                                              {0xe0, 0x47, 0x70, 0x18, 0xcf},
                                              {0x73, 0xd0, 0x02, 0x89, 0x28},
                                              {0xed, 0xd8, 0xce, 0x96, 0x6d}
};

volatile int state;                                             // aux for the FSM
volatile uint32_t time;                                         // gets the timer count value
volatile uint32_t i;

Mfrc522 Mfrc522(chipSelectPin, NRSTPD);                         //Library modified to work with CCS

void SSI2IntHandler(void){
    while(1)
    {
        time = TIMER0_TAR_R;
        switch(state){
        case 0:
            status = Mfrc522.Request(PICC_REQIDL, str);
            GPIO_PORTF_DATA_R  = blueLED; // LED in blue indicating that sensor is waiting for a input
            if(status == MI_OK){ // MI_OK status means that the sensor successfully read a card
                UARTprintf("\n----------------------------------------------------------------------------------------------\n\nTarjeta leida correctamente! \n");
            }
            status = Mfrc522.Anticoll(str); // read the card 4 bytes card serial number, the 5th byte is re-check byte
            memcpy(cardID, str, 10);
            if(status == MI_OK){
                UARTprintf("ID: \n");
                if (dumpHex((unsigned char*)cardID, CARD_LENGTH) == 1){
                    UARTprintf("\nTarjeta Invalida!!!\n");  // Put the LED in red if the card is incorrect
                    GPIO_PORTF_DATA_R  = redLED;
                    state = 1;
                }
                else{
                    UARTprintf("\nTarjeta Valida!!!\n");    // Put the LED in green if the card is incorrect
                    GPIO_PORTF_DATA_R  = greenLED;
                    state = 1;
                }
            }
            break;
        case 1:
            if(time == 0x00){
                state = 0;
            }
            break;
        } // end switch
        SSIIntClear(SSI2_BASE, SSI_TXFF); // Clear the interrupt for SSI2 from the vector table.
    } // end while
}

int dumpHex(unsigned char* buffer, int len){

    int i, j, flag;

    for(i=0; i < len; i++) {    // loop to display the card data with hexadecimal format
        if(buffer[i] < 0x10)
        {
            UARTprintf("0");
        }
        UARTprintf("%x ", buffer[i]);
    }

    flag = 0; // Initialize flag value in 0
    for(j = 0; j < CARD_NUM; j++){
        flag = 0;
        for(i=0; i < ID_LENGTH; i++){ // Loop through the card data, to compare each byte with the master ID
            if(masterID[j][i] != buffer[i]){
                flag = 1; // 1 if a value that doesn't match
                i = ID_LENGTH; // Break the loop
            }
        }
        if (flag == 0){
            j = CARD_NUM;
        }
    }

    return flag;
}

void InitConsole()
{
    SYSCTL_RCGCGPIO_R |= 0x01;  // Activate Port A

    GPIOPinConfigure(GPIO_PA0_U0RX); // Use pin A0 for Rx
    GPIOPinConfigure(GPIO_PA1_U0TX); // Use pin A1 for Tx

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Enable the UART 0 to communicate

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); // Select A0 and A1 as UART pin types

    UARTStdioConfig(0, 115200, 16000000); // UART port 0, with 115200 bauds(bit rate) and 16000000 as the frequency of the source clock
}

void InitSSI(){
    uint32_t junkAuxVar;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    SYSCTL_RCGCGPIO_R |= 0x2;   // Activate Port B for SDA
    SYSCTL_RCGCGPIO_R |= 0x8;   // Activate Port F for reset

    GPIOPinConfigure(GPIO_PB4_SSI2CLK); // Set PB4 for CLK data
    GPIOPinConfigure(GPIO_PB6_SSI2RX);  // Set PB6 for RX data
    GPIOPinConfigure(GPIO_PB7_SSI2TX);  // Set PB7 for TX data

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);  // The ports PB4, PB6 and PB7 will use SSI to communicate with the sensor

    GPIO_PORTB_DIR_R    |=   chipSelectPin; // Configure as an input
    GPIO_PORTB_DEN_R    |=   chipSelectPin; // Digital data enable
    GPIO_PORTF_DIR_R    |=   NRSTPD;        // Configure as an input
    GPIO_PORTF_DEN_R    |=   NRSTPD;        // Digital data enable

    // SSIConfigSetExpClk(uint32_t ui32Base, uint32_t ui32SSIClk, uint32_t ui32Protocol, uint32_t ui32Mode, uint32_t ui32BitRate, uint32_t ui32DataWidth);
    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 4000000, 8);

    //SSIIntClear(SSI2_BASE,SSI_TXEOT);
    //SSIIntRegister(SSI2_BASE, SSI2IntHandler);
    //SSIIntEnable(SSI2_BASE, SSI_TXEOT);

    SSIEnable(SSI2_BASE);   // Enable the SSI2 module

    while(SSIDataGetNonBlocking(SSI2_BASE, &junkAuxVar)){} // Validate that SSI2 works fine


    /*
    Interrupt explanation:

        The SSI2 interrupt gets activated when the Tx FIFO is full, after initializating the SSI2, the Tx FIFO is "full",
        that means that the SSI2, which communicates the sensor with the TIVA microcontroller,
        had successfully connected both of them, and it's ready for the next steps.
        In this case, the steps in general terms are :
        - Read the information that the sensor receives from the CARDS
        - Validate the ID from the read card.
     */
    SSIIntClear(SSI2_BASE, SSI_TXFF); // Clear the interrupt for SSI2 from the vector table.
    SSIIntRegister(SSI2_BASE, SSI2IntHandler); // Register the interrupt for SSI2, it calls the correspondent handler that activates the FSM
    SSIIntEnable(SSI2_BASE, SSI_TXFF);
}

int main(void) {

    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 40MHz

    InitConsole();

    // Timer
    SYSCTL_RCGCTIMER_R = 0x01;  // Enable clock  TIMER0

    TIMER0_CTL_R    &= ~0x1;    // Disable timer
    TIMER0_CFG_R    = 0;        // 32-bit timer
    TIMER0_TAMR_R   |= 0x12;    // Set: periodic mode, count up
    TIMER0_TAILR_R  = LOAD_0;   // top count up
    TIMER0_CTL_R    |= 0x1;     // Enable timer

    // LEDS
    SYSCTL_RCGCGPIO_R   |= 0x20;    // Activate Port F
    GPIO_PORTF_DIR_R    |= 0x02;    // PF1 output
    GPIO_PORTF_DEN_R    |= 0x02;    // Enable digital IO on PF1
    GPIO_PORTF_DIR_R    |= 0x04;    // PF2 output
    GPIO_PORTF_DEN_R    |= 0x04;    // Enable digital IO on PF2
    GPIO_PORTF_DIR_R    |= 0x08;    // PF3 output
    GPIO_PORTF_DEN_R    |= 0x08;    // Enable digital IO on PF3

    InitSSI();

    GPIO_PORTB_DATA_R   &=  ~chipSelectPin;
    GPIO_PORTF_DATA_R   |=  NRSTPD;

    Mfrc522.Init(); // Initialize the components for the RFID sensor.
} // end main
