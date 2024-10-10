#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

#define SW1    (1U << 0)  // PF0 (SW1)
#define SW2    (1U << 4)  // PF4 (SW2)

// Define LED pins
#define RED_LED    (1U << 1) // PF1
#define BLUE_LED   (1U << 2) // PF2
#define GREEN_LED  (1U << 3) // PF3

uint8_t receivedByte;
bool dataReceivedFlag = true;

void Uart2_send(void);
void UART2_Transmit(uint8_t data);

void PortF_Initialisation(void){

    // PORTF, PF7-PF0, PF4-SW1, PF3-green, PF2-blue, PF1-red, PF0-SW2
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF;   // Enable clock for Port F
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port F
    GPIO_PORTF_CR_R = 0x1f;                 // Commit changes,1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DEN_R = 0x1f;                // Digital function enable, 1-enable (PF7-PF0 = 00011111)
    GPIO_PORTF_DIR_R = 0x0e;                // Set output/input, 1-output (PF7-PF0 = 00001110)
    GPIO_PORTF_PUR_R = 0x11;                // Enable pull-up resistor, 1-enable (PF7-PF0 = 00010001)
    GPIO_PORTF_DATA_R = 0x00;               // Reset the data register (PF7-PF0 = 00000000)

}

void PortD_Initialisation(void) {

    // Enable the clock for UART2 and GPIO Port D

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOD;     // Enable GPIO Port D clock
    SYSCTL_RCGCUART_R |= SYSCTL_RCGC1_UART2;     // Enable UART2 clock
    // while ((SYSCTL_PRGPIO_R & (1U << 3)) == 0) {} // Wait for Port D to be ready

    // Configure PD6 (RX) and PD7 (TX)

    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;      // Unlock Port D
    GPIO_PORTD_CR_R = 0xc0;                 // Commit changes,1-enable (PD7-PD0 = 11000000)
    GPIO_PORTD_DEN_R = 0xc0;                // Digital enable PD6 and PD7
    GPIO_PORTD_AFSEL_R = 0xc0;              // Enable alternate function for PD6 and PD7
    GPIO_PORTD_AMSEL_R = 0x00;              // Turnoff analog function
    GPIO_PORTD_PCTL_R &= ~((0xF << 24) | (0xF << 28)); // Clear PCTL for PD6 and PD7
    GPIO_PORTD_PCTL_R |= ((0x1 << 24) | (0x1 << 28));  // Set PCTL to UART1 for PD6 Rx and PD7 Tx
}

void Uart2_Initialisation(void) {

    // Configure UART2 for 9600 baud rate, 8 data bits, odd parity
    UART2_CTL_R = 0x00;                     // Disable UART before configuration
    UART2_IBRD_R = 104;                     // Integer part of BRD = 16MHz / (16 * 9600) = 104
    UART2_FBRD_R = 11;                      // Fractional part of BRD = 0.16 * 64 + 0.5 = 11
    UART2_CC_R = 0x00;
    UART2_LCRH_R = 0x72;                    // 8 bits, odd parity
    UART2_CTL_R = 0x301;                    // Enable UART

    //NVIC_PRI7_R &= 0xFF3FFFFF;             // Prioritize and enable interrupts in NVIC
    //__asm("    cpsie i");                // Global interrupt enable
    // Enable RX interrupt
    //UART2_IM_R |= UART_IM_RXIM; // Enable receive interrupt
    //NVIC_EN0_R |= (1 << 23); // Enable interrupt for UART2 (IRQ 23)
}

/*void UART2_Handler(void) {
    GPIO_PORTF_DATA_R |= GREEN_LED;  // Turn off green LED
    GPIO_PORTF_DATA_R |= BLUE_LED;   // Turn off blue LED
    GPIO_PORTF_DATA_R |= RED_LED;     // Turn on red LED (error)

    if (UART2_MIS_R & UART_MIS_RXMIS) { // Check if receive interrupt occurred
        receivedByte = UART2_DR_R & 0xFF; // Read received data
        dataReceivedFlag = true; // Set flag to indicate data received
        UART2_ICR_R = UART_ICR_RXIC; // Clear the interrupt
    }
}*/

uint8_t UART2_ReceiveByte(void) {

    while ((UART2_FR_R & 0x10) != 0) // Wait until RXFE is 0
    {
        Uart2_send();
    }
    return UART2_DR_R; // Read data
//return 0;
}

void Uart2_Read(void){

    receivedByte =  UART2_ReceiveByte();

    if(dataReceivedFlag){
        // Check if a parity error occurred
        if (UART2_FR_R & 0x04) { // Check PE (Parity Error) bit
                    GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;   // Turn off blue LED
                    GPIO_PORTF_DATA_R |= RED_LED;     // Turn on red LED (error)
        } else {
            switch (receivedByte) {
                case 0xAA:
                    GPIO_PORTF_DATA_R |= GREEN_LED;  // Turn on green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;   // Turn off blue LED
                    GPIO_PORTF_DATA_R &= ~RED_LED;     // Turn off red LED (error
                    break;
                case 0xF0:
                    GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED
                    GPIO_PORTF_DATA_R |= BLUE_LED;   // Turn on blue LED
                    GPIO_PORTF_DATA_R &= ~RED_LED;     // Turn off red LED (error
                    break;
                default:
                    GPIO_PORTF_DATA_R &= ~GREEN_LED;  // Turn off green LED
                    GPIO_PORTF_DATA_R &= ~BLUE_LED;   // Turn off blue LED
                    GPIO_PORTF_DATA_R |= RED_LED;     // Turn on red LED (error)
                    break;
            }
        }
        //dataReceivedFlag = false; // Reset flag after handling
    }
}

void Uart2_send(void){

    if (!(GPIO_PORTF_DATA_R & SW1)) { // SW1 pressed
        UART2_Transmit(0xF0);
        while (!(GPIO_PORTF_DATA_R & SW1)); // Wait until released
    }
    if (!(GPIO_PORTF_DATA_R & SW2)) { // SW2 pressed
        UART2_Transmit(0xAA);
        while (!(GPIO_PORTF_DATA_R & SW2)); // Wait until released
    }
}

void UART2_Transmit(uint8_t data) {
    while (UART2_FR_R & UART_FR_TXFF);  // Wait until the transmit FIFO is not full
    UART2_DR_R = data;                  // Transmit data
}

int main(void) {
    PortF_Initialisation();
    PortD_Initialisation();
    Uart2_Initialisation();
    //__asm("    cpsie i");                // Global interrupt enable

    while (1) {

        Uart2_Read();
    }
}
