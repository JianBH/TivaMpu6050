#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

//uart
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include <string.h>
#include "utils/uartstdio.h"

	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	volatile uint8_t ui8Adjust;
	volatile uint8_t ui8AdjustB;

#define PWM_FREQUENCY 55// need  55 
//char
volatile char input;
//char Valid [] = "\tValid Input\n\r";
//char inValids [] = "\tInvalid Input\n\r";

void PWM0_Init(void){
	ui8Adjust = 83; //83 refer to zer0
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	
	//edit
	ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);//PD1
	ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1);
	
	ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0); //PD0
	ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);
	
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	
	ROM_GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	ROM_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	
	ui32PWMClock = SysCtlClockGet() / 64; //64, 32
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	
	PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);
	
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, ui32Load);
	
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);//1000, 500
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	
	ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);//1000, 500
	ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
	ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_1);
}
//create void functions for short the switch
void Movement(){

	switch(input){
				
				case 'F':
					ui8Adjust = -56;//10, 107, 55, 2  
					ui8AdjustB = 56;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);//1000, 500
					for(int i =0; i < strlen(Valid); i++){
					UARTCharPut(UART0_BASE, Valid[i]);
					}
				break;
				case 'B':
					ui8Adjust = 75;//10, 107, 55, 2  
					ui8AdjustB = -75;
				  ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);//1000, 500
					for(int i = 0; i < strlen(Valid); i++){
					UARTCharPut(UART0_BASE, Valid[i]);
					}
				break;
				case 'S': //this is corrects
					ui8Adjust = 0;
					ui8AdjustB = 0;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);//1000, 500
					for(int i =0; i < strlen(Valid); i++){
					UARTCharPut(UART0_BASE, Valid[i]);
					}
				break;
				case 'L': 
					ui8Adjust = -56;//10, 107, 55, 2  
					ui8AdjustB = 107;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);//1000, 500
					for(int i =0; i < strlen(Valid); i++){
					UARTCharPut(UART0_BASE, Valid[i]);
					}
					break;
				case 'R':
					ui8Adjust = 56;//10, 107, 55, 2  
					ui8AdjustB = -107;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);//1000, 500
					for(int i =0; i < strlen(Valid); i++){
					UARTCharPut(UART0_BASE, Valid[i]);
					}
					break;
					
				default:
					ui8Adjust = 107;//move opposite direction ?  
					ui8AdjustB = -107;
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui8Adjust * ui32Load / 1000);
					ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, ui8AdjustB * ui32Load / 1000);
				for(int i =0; i < strlen(inValids); i++){
					UARTCharPut(UART0_BASE, inValids[i]);
			}		
    }
	
}
void UARTIntHandler(void)
{

    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status

    UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts

   while(1){
		if (UARTCharsAvail(UART1_BASE)) // if char received on UART 1, transmit that char on UART 0
		//	UARTCharPut(UART0_BASE, UARTCharGet(UART1_BASE));
				input = UARTCharGet(UART1_BASE);
        			//UARTCharPutNonBlocking(UART1_BASE, input); //echo character
				//Movement();
		}
}
int main(void) {
		PWM0_Init();
	
		SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);		
		// Setting up GPIO for UART0
    		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
   		 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    		GPIOPinConfigure(GPIO_PA0_U0RX);
    		GPIOPinConfigure(GPIO_PA1_U0TX);
		GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		UARTStdioConfig(0, 115200,20000000); // (Port Num 0, 115200 Baud Rate, clock )
	
	
		SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
		SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
    // Setting up GPIO for UART1
		GPIOPinConfigure(GPIO_PC4_U1RX);
		GPIOPinConfigure(GPIO_PC5_U1TX); 
		GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

		UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
		UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    IntMasterEnable(); //enable processor interrupts
    IntEnable(INT_UART1); //enable the UART interrupt
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts
		
		
	while(1){
		//if (UARTCharsAvail(UART1_BASE)) // if char received on UART 1, transmit that char on UART 0
		//	UARTCharPut(UART0_BASE, UARTCharGet(UART1_BASE));
				//input = UARTCharGet(UART1_BASE);
        			//UARTCharPutNonBlocking(UART1_BASE, input); //echo character
		UARTprintf("%c", input);
				Movement();
		}

}
