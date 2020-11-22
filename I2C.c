#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"

// UART Libraries
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

//MPU6050 libraies
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
//#include "sensorlib/mpu6050.h"
#include "sensorlib/i2cm_drv.h"

#include <math.h>
#pragma import(__use_no_semihosting_swi)

uint32_t clockFreq;
int16_t avgAX, avgAY, avgAZ, stdX, stdY, stdZ;
int16_t AX, AY, AZ;
int16_t RawAx, RawAy, RawAz;
char msg[128];
char DataToSend;


void MpuCalibration(int samples, uint8_t *data );
char MovementDetection(int16_t x, int16_t y, int16_t z);

void CheckError(void);
void UART0Send(char *str);

uint8_t readI2C2( uint16_t Slave_address, uint8_t Slave_reg);
void readI2C2MultipleV2(uint8_t Slave_address, uint8_t Slave_reg, int count, uint8_t *data);
void writeI2C2(uint16_t device_address, uint8_t device_reg, uint8_t device_data);


//initialize I2C Base 2
void InitI2C2(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2); // enable the I2C module base 2
	
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C2); // reset I2C 2
	
	
	I2CMasterInitExpClk(I2C2_BASE, clockFreq, false); // Enable clock for I2C master module
																													// false - data rate 100kbps
																													// true - data rate 400 kbps
	// TPR = 0x13 = 19
	
	//HWREG(I2C2_BASE + I2C_O_FIFOCTL)= 80008000;
		
}
void GPIOEInit(void){
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	// enable GPIO peripheral to use I2C 2
	
	GPIOPinConfigure(GPIO_PE4_I2C2SCL); // Configure PE4 as I2C Clock line
	GPIOPinConfigure(GPIO_PE5_I2C2SDA); // Configure PE5 as I2C as the data line
	
	GPIOPinTypeI2CSCL(GPIO_PORTE_BASE, GPIO_PIN_4); //Configure PE4 & PE5 to use I2C function, also enable the digital function for PE4
	GPIOPinTypeI2C(GPIO_PORTE_BASE, GPIO_PIN_5); // Enables PE5(SDA Line) to be open drain & enables digital function for PE5
	
}
void InitUART0(void){
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);		// enabel UART 0
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);		// enable GPIO A peripheral 

  GPIOPinConfigure(GPIO_PA0_U0RX);		// Set PA0 as receiver
  GPIOPinConfigure(GPIO_PA1_U0TX);		// Set PA1 as transmitter
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); 	// configure GPIO PA0 & PA1 to use UART function
	
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // configure UART Clock 
	
	//UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	
	UARTStdioConfig(0, 115200,20000000); // (Port Num 0, 115200 Baud Rate, clock )
}
void InitUART1(void){
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);		// enabel UART 1
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);		// enable GPIO C peripheral 

  GPIOPinConfigure(GPIO_PC4_U1RX);		// Set PC4 as receiver
  GPIOPinConfigure(GPIO_PC5_U1TX);		// Set PC5 as transmitter
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5); 	// configure GPIO PA0 & PA1 to use UART function
	
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE)); // configure UART Clock 
	
	//UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	
	//UARTStdioConfig(0, 115200,20000000); // (Port Num 0, 115200 Baud Rate, clock )
}
void UART0Send(char *str){
	while(*str != '\0'){
		while(UARTBusy(UART0_BASE));
		UARTCharPut(UART0_BASE,*str++);
		}
}

void Delay(unsigned long counter){
	unsigned long i =0;
 	for(i =0; i < counter*1000;i++);
}

void readI2C2MultipleV2(uint8_t Slave_address, uint8_t Slave_reg, int count, uint8_t *data){
	
	for(int i = 0 ; i < count; i++){
		data[i] = readI2C2(Slave_address,Slave_reg+i);
	}
}
			
		
uint8_t readI2C2( uint16_t Slave_address, uint8_t Slave_reg){
	
	// specify we are writing a reg address to slave device
	I2CMasterSlaveAddrSet(I2C2_BASE, Slave_address, false); // false initiate Master to write, true initiate Master to read
	
	I2CMasterDataPut(I2C2_BASE, Slave_reg); // The register to be read
	
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND); // send control byte and reg address to slave device
	
	CheckError();
	
	while(I2CMasterBusy(I2C2_BASE)); // wait for send transaction to complete
	
	I2CMasterSlaveAddrSet(I2C2_BASE, Slave_address, true); // read from slave
	
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); 
	
	CheckError();
	
	while(I2CMasterBusy(I2C2_BASE));
	
	return (I2CMasterDataGet(I2C2_BASE)); //return the data from MCU register
	
}

void writeI2C2(uint16_t device_address, uint8_t device_reg, uint8_t device_data){
	
	I2CMasterSlaveAddrSet(I2C2_BASE, device_address, false); // false = transmit (0), true = receive(1)
	
	//send control byte and register address byte to slave device
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	
	CheckError();

	//register to be written
	I2CMasterDataPut(I2C2_BASE, device_reg);
	
	SysCtlDelay(20000);
	
	//specify data to be written to the above mentioned device_register
	I2CMasterDataPut(I2C2_BASE, device_data);
	
	//wait while checking for MCU to complete the transaction
	I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	
	//wait for MCU to finish transaction
	while(I2CMasterBusy(I2C2_BASE));
	
}	

void CheckError(void){
	if((I2C2_MCS_R & 0x02) == 0x01){
		UARTprintf("Error Detected! \n");
	}

	if((I2C2_MCS_R & 0x08) == 0x01){
		UARTprintf("Error on Data acknowledgement\n");
	}
	if((I2C2_MCS_R & 0x04) == 0x01){
		UARTprintf("Error on Address Acknowledgement\n");
	}
	if((I2C2_MCS_R & 0x40) == 0x01){
		UARTprintf("Clock timeout error!\n");
	}
}
		
void MPU6050init(void){
	writeI2C2(0x68, MPU6050_O_PWR_MGMT_1, 0x01);
	writeI2C2(0x68, MPU6050_O_SMPLRT_DIV, 0x07);
	writeI2C2(0x68, MPU6050_O_CONFIG, 0x01);
	writeI2C2(0x68, MPU6050_O_ACCEL_CONFIG,0x00);
	writeI2C2(0x68, MPU6050_O_GYRO_CONFIG, 0x01);
	writeI2C2(0x68, MPU6050_O_INT_ENABLE,0x01);
}	

void MpuCalibration(int samples, uint8_t *data ){
	UARTprintf("Beginning Calibration.......\n");
	int16_t *x ,*y, *z;
	int16_t sumX, sumY, sumZ;
	uint16_t SDx,SDy, SDz;
	int counter = samples;
	writeI2C2(0x68, MPU6050_O_PWR_MGMT_1, 0x80); // reset device
	Delay(1000);
	writeI2C2(0x68, MPU6050_O_PWR_MGMT_1, 0x01);
	
	for(int i =0; i < counter; i++){
		readI2C2MultipleV2(0x68, MPU6050_O_ACCEL_XOUT_H, 14, data);
		
		x[i] = ((data[0] << 8) | data[1]);
		y[i] = ((data[2] << 8) | data[3]);
		z[i] = ((data[4] << 8) | data[5]);
		
		sumX += x[i];
		sumY += y[i];
		sumZ += z[i];
	}
	avgAX = sumX / counter;
	avgAY = sumY / counter;
	avgAZ = sumZ /counter;
	
	for(int i =0; i < counter; i++){
		SDx += (uint16_t)pow(abs(x[i] - avgAX), 2);
		SDy += (uint16_t)pow(abs(y[i] - avgAY), 2);
		SDz += (uint16_t)pow(abs(z[i] - avgAZ), 2);
	}
		
		stdX = (int16_t)sqrt(SDx / counter);
		stdY = (int16_t)sqrt(SDy / counter);
		stdZ = (int16_t)sqrt(SDz / counter);	
	
	UARTprintf("Finish Calibration........\n");
	writeI2C2(0x68, MPU6050_O_PWR_MGMT_1, 0x64);
	}

char MovementDetection(int16_t x, int16_t y, int16_t z){
	char state = 'S';
	if(x < 172  && x  > 32){
		state = 'L'; // Left
	}
	if(x > -172 && x < -35 && (state != 'L')){
		state = 'R'; // right
	}
	if((z > 172 && z < 176) && state != 'R' && state != 'L'	) {
	//if(state != 'F' || state != 'R' || state != 'L' || state !='B'){
		state = 'S'; // stop
	}
	if( y > -135 && y < -50 && (state != 'L' || state != 'R')){
		state = 'F'; // forward
	}
	if(y > 14 && y < 110){
		state = 'B'; // backward
	}
	return state;
}
	
	

int main(void){
	SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); // 20MHZ clock
	
	uint8_t values[14];
	uint8_t calibrationVal[14];
	//clockFreq = SysCtlClockGet();
	InitUART0();
	InitUART1();
	GPIOEInit();
	InitI2C2();
	Delay(1000);
	MpuCalibration(100,calibrationVal);
	Delay(1000);
	MPU6050init();
	Delay(2000);
	/*
	int16_t slaveaddresscheck,regcheck;
	
	slaveaddresscheck = readI2C2(0x68,MPU6050_O_CONFIG);
	regcheck = readI2C2(0x68,MPU6050_O_SMPLRT_DIV);
	UARTprintf("Slave address = %d\n",slaveaddresscheck);
	UARTprintf("Slave address = %d\n",regcheck);
	*/

/*
for(int j = 0; j < 4; j++){
	values[j] = readI2C2(0x68,MPU6050_O_SMPLRT_DIV+j);
	UARTprintf(" Value stored in [%d] = %d\n",j,values[j]);
}
*/
	
	/*
		readI2C2MultipleV2(0x68, MPU6050_O_SMPLRT_DIV, 4, values);
		for(int i =0; i< 4; i++){
			UARTprintf(" Value stored in [%d] = %d\n",i,values[i]);
		}
		*/
		/*
		I2C2ReadMulti(0x68,MPU6050_O_SMPLRT_DIV,4,values);
		for(int i =0; i< 4; i++){
			UARTprintf(" Value stored in [%d] = %d\n",i,values[i]);
		}
		*/
		

	while(1){
		
		readI2C2MultipleV2(0x68, MPU6050_O_ACCEL_XOUT_H, 14, values);
/*
		for(int i =0; i< 14; i++){
			UARTprintf(" Value stored in [%d] = %d\n",i,values[i]);
		}
*/		
		
	
		RawAx = ((values[0] << 8) | values[1]);
		RawAy = ((values[2] << 8) | values[3]);
		RawAz = ((values[4] << 8) | values[5]);
		/*
		Temp =  (values[6] << 8) | values[7];
		RawGx = (int)((values[8] << 8) | values[9]);
		RawGy = (int)((values[10] << 8) | values[11]);
		RawGz = (int)((values[12] << 8) | values[13]);
		*/
	
		
		
	unsigned int AXM =  values[0];
	unsigned int AXL = values[1];
		
		AX = RawAx / (4*stdX);
		AY =  RawAy / (4*stdY);
		AZ =  RawAz / (4*stdZ);
		
		
	
		
		sprintf(msg, "Movement: %c",MovementDetection(AX,AY,AZ));
		UART0Send(msg);
		
		DataToSend = MovementDetection(AX, AY, AZ);
		
		while(UARTBusy(UART1_BASE));
		UARTCharPutNonBlocking(UART1_BASE, DataToSend);

		/*
		sprintf(msg, " RawAXM = %d \t",AXM);
		UART0Send(msg);	
		sprintf(msg, " RawAXL = %d \t",AXL);
		UART0Send(msg);	
		*/
		
		
		/*
		sprintf(msg, " RawAX = %d\t\t",AX);
		UART0Send(msg);		
		sprintf(msg, " RawAY = %d\t\t",AY);
		UART0Send(msg);	
		sprintf(msg, " RawAZ = %d\t\t",AZ);
		UART0Send(msg);	
	*/
		
	

		UARTprintf("\n");
		AX = 0;
		AY = 0;
		AZ = 0;
		SysCtlDelay(2000000);
	}

	}

