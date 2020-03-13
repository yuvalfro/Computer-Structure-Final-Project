/*
 * main implementation: use this 'C' sample to create your own application
 *
 */


//#include "derivative.h" /* include peripheral declarations */
#include "TFC.h"
#include<stdio.h>
#include <stdlib.h>
#include "derivative.h" 				// Include peripheral declarations 
#include "I2C.h"
#include "math.h"

/******************************************************************************
* Constants and macros
******************************************************************************/

//MMA8451Q Registers

#define STATUS_REG            0x00		// STATUS Register 

#define OUT_X_MSB_REG         0x01		// [7:0] are 8 MSBs of the 14-bit X-axis sample
#define OUT_X_LSB_REG         0x02		// [7:2] are the 6 LSB of 14-bit X-axis sample
#define OUT_Y_MSB_REG         0x03		// [7:0] are 8 MSBs of the 14-bit Y-axis sample
#define OUT_Y_LSB_REG         0x04		// [7:2] are the 6 LSB of 14-bit Y-axis sample
#define OUT_Z_MSB_REG         0x05		// [7:0] are 8 MSBs of the 14-bit Z-axis sample
#define OUT_Z_LSB_REG         0x06		// [7:2] are the 6 LSB of 14-bit Z-axis sample

#define F_SETUP_REG           0x09    	// F_SETUP FIFO Setup Register 
#define TRIG_CFG_REG          0x0A    	// TRIG_CFG Map of FIFO data capture events 
#define SYSMOD_REG            0x0B    	// SYSMOD System Mode Register 
#define INT_SOURCE_REG        0x0C    	// INT_SOURCE System Interrupt Status Register 
#define WHO_AM_I_REG          0x0D    	// WHO_AM_I Device ID Register 
#define XYZ_DATA_CFG_REG      0x0E    	// XYZ_DATA_CFG Sensor Data Configuration Register 
#define HP_FILTER_CUTOFF_REG  0x0F    	// HP_FILTER_CUTOFF High Pass Filter Register 

#define PL_STATUS_REG         0x10    	// PL_STATUS Portrait/Landscape Status Register 
#define PL_CFG_REG            0x11    	// PL_CFG Portrait/Landscape Configuration Register 
#define PL_COUNT_REG          0x12    	// PL_COUNT Portrait/Landscape Debounce Register 
#define PL_BF_ZCOMP_REG       0x13    	// PL_BF_ZCOMP Back/Front and Z Compensation Register 
#define P_L_THS_REG           0x14    	// P_L_THS Portrait to Landscape Threshold Register 

#define FF_MT_CFG_REG         0x15    	// FF_MT_CFG Freefall and Motion Configuration Register 
#define FF_MT_SRC_REG         0x16    	// FF_MT_SRC Freefall and Motion Source Register 
#define FT_MT_THS_REG         0x17    	// FF_MT_THS Freefall and Motion Threshold Register 
#define FF_MT_COUNT_REG       0x18    	// FF_MT_COUNT Freefall Motion Count Register 

#define TRANSIENT_CFG_REG     0x1D    	// TRANSIENT_CFG Transient Configuration Register 
#define TRANSIENT_SRC_REG     0x1E    	// TRANSIENT_SRC Transient Source Register 
#define TRANSIENT_THS_REG     0x1F    	// TRANSIENT_THS Transient Threshold Register 
#define TRANSIENT_COUNT_REG   0x20    	// TRANSIENT_COUNT Transient Debounce Counter Register 

#define PULSE_CFG_REG         0x21    	// PULSE_CFG Pulse Configuration Register 
#define PULSE_SRC_REG         0x22    	// PULSE_SRC Pulse Source Register 
#define PULSE_THSX_REG        0x23    	// PULSE_THS XYZ Pulse Threshold Registers 
#define PULSE_THSY_REG        0x24
#define PULSE_THSZ_REG        0x25
#define PULSE_TMLT_REG        0x26    	// PULSE_TMLT Pulse Time Window Register 
#define PULSE_LTCY_REG        0x27    	// PULSE_LTCY Pulse Latency Timer Register 
#define PULSE_WIND_REG        0x28    	// PULSE_WIND Second Pulse Time Window Register 

#define ASLP_COUNT_REG        0x29    	// ASLP_COUNT Auto Sleep Inactivity Timer Register 

#define CTRL_REG1             0x2A    	// CTRL_REG1 System Control 1 Register 
#define CTRL_REG2             0x2B    	// CTRL_REG2 System Control 2 Register 
#define CTRL_REG3             0x2C    	// CTRL_REG3 Interrupt Control Register 
#define CTRL_REG4             0x2D    	// CTRL_REG4 Interrupt Enable Register 
#define CTRL_REG5             0x2E    	// CTRL_REG5 Interrupt Configuration Register 

#define OFF_X_REG             0x2F    	// XYZ Offset Correction Registers 
#define OFF_Y_REG             0x30
#define OFF_Z_REG             0x31

//MMA8451Q 7-bit I2C address

#define MMA845x_I2C_ADDRESS   0x1D		// SA0 pin = 1 -> 7-bit I2C address is 0x1D 

//MMA8451Q Sensitivity at +/-2g

#define SENSITIVITY_2G		  4096

/******************************************************************************
* Global variables
******************************************************************************/

unsigned char AccData[6];
short Xout_14_bit, Yout_14_bit, Zout_14_bit;
float Xout_g, Yout_g, Zout_g;
char DataReadyUart = 0;
char DataReadyAcc = 0;
char DataReadyDis = 0;
char Xoffset, Yoffset, Zoffset;


volatile float Xstart, Ystart, Zstart;
volatile uint8_t gData;
char Xstr[13] = {'+','0' , '.' ,'0' ,'0' ,'0','0','\0'};
char Ystr[13] = {'+','0' , '.' ,'0' ,'0' ,'0','0','\0'};
char Zstr[13] = {'+','0' , '.' ,'0' ,'0' ,'0','0','\0'};
char disString[5] = {'0','0' ,'0','.','0'};
char pcmessage[40]; 			 // Array to receive string from PC
float distance;
volatile float captureR=0;		  //Input capture - rising
volatile float captureF=0;		  //Input capture - falling
volatile unsigned int captureFlag=0;	  //Input capture - Flag
int arridx = 0;   						 // UART0 interrupt: index to insert chars to array 
volatile unsigned int state=0;	  	//1-3D scan, 2-System movement mode, 3-Telemeter
volatile unsigned int rowsScanned=0;
volatile unsigned int scanningFlag = 0; //0 not scanning 1 scanning
const double pi = (4.0 * atan(1.0));
double roll;
double pitch;
int numOfRowsToScan = 0;

/******************************************************************************
* Functions
******************************************************************************/

void MCU_Init(void);
void Accelerometer_Init (void);
void Calibrate(void);
void InitPIT();
void InitTPM();

/******************************************************************************
* Main
******************************************************************************/  

int main(void){
	
	ClockSetup();
	InitGPIO();
	InitUARTs();
	InitTPM();
	MCU_Init();
    Accelerometer_Init();
  	Calibrate(); 
  	RGB_LED_OFF;

  	while(1){
  		if (DataReadyUart)
  		{
  			DataReadyUart = 0;
  			
			//---------------------------------------------------------------------------------------------------------------------
			// Telemeter
			// --------------------------------------------------------------------------------------------------------------------
			if (strncmp(pcmessage,"Telemeter", 9) == 0){
				memset(pcmessage,0,40);  //memset - clears the array
				TPM0_SC |= TPM_SC_CMOD(1);  // Start the TPM0 counter
				//TPM2_SC |= TPM_SC_CMOD(1);  // Start the TPM2 counter
  				int temp=0;
  				int j;
	
  				while (state == 3)
  				{
  					if (DataReadyDis)
  					{
  						DataReadyDis = 0;
  						temp = distance * 10; 						// Turn from decimal num to int (123.4 --> 1234)
  						disString[4] = temp%10 + 48; 
  						temp = temp / 10;
  						for(j=2; j>=0; j--){
  							disString[j] = temp%10 + 48; 				// +48 - Turn to ASCII (1+48=49, 49 is 1 in ASCII) 
  							temp = temp / 10;
  						}	
  					//	for (j=10000; j>0; j--);	 	  // Delay

  	  				UARTprintf(UART0_BASE_PTR,disString);
  	  				UARTprintf(UART0_BASE_PTR,"\r\n");

  					}
  				}
			}
			else{
				//---------------------------------------------------------------------------------------------------------------------
				// System movement
				// --------------------------------------------------------------------------------------------------------------------
				if (strncmp(pcmessage,"Sysmove", 7) == 0){
					memset(pcmessage,0,40);  //memset - clears the array
					I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06 
			        
					/*Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
					Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
					Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value
					
					Xout_g = ((float) Xout_14_bit) / SENSITIVITY_2G;		// Compute X-axis output value in g's
					Yout_g = ((float) Yout_14_bit) / SENSITIVITY_2G;		// Compute Y-axis output value in g's
					Zout_g = ((float) Zout_14_bit) / SENSITIVITY_2G;		// Compute Z-axis output value in g's
					
					Xstart = Xout_g;
					Ystart = Yout_g;
					Zstart = Zout_g;*/
					
				  	while(state == 2)
				    {
						if (DataReadyAcc)		// Is a new set of data ready? 
						{  		
							DataReadyAcc = 0;
																					
							I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06 
				            
							Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
							Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
							Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value
							
							Xout_g = ((float) Xout_14_bit) / SENSITIVITY_2G;		// Compute X-axis output value in g's
							Yout_g = ((float) Yout_14_bit) / SENSITIVITY_2G;		// Compute Y-axis output value in g's
							Zout_g = ((float) Zout_14_bit) / SENSITIVITY_2G;		// Compute Z-axis output value in g's
							
							/*------------------
							  Send Data with UART
							  -------------------*/
							int temp,j;
							switch(pcmessage[0]){
							case 'x': 
								temp = Xout_g * 10000; 						// Turn from decimal num to int (1.234 --> 1234)
								if(Xout_g<0){
									Xstr[0]='-';
									temp = 0-temp; 							// x= |x|
								}
								else
									Xstr[0]='+';
									for(j=6; j>=3; j--){
										Xstr[j] = temp%10 + 48; 				// +48 - Turn to ASCII (1+48=49, 49 is 1 in ASCII) 
										temp = temp / 10;
									}	
									Xstr[1] = temp%10+48;						// in the str[2] there is '.'
									UARTprintf(UART0_BASE_PTR,Xstr);
									UARTprintf(UART0_BASE_PTR,"\r\n");
							break;
							case 'y':
									temp = Yout_g * 10000; 						// Turn from decimal num to int (1.234 --> 1234)
									if(Yout_g<0){
										Ystr[0]='-';
										temp = 0-temp; 							// x= |x|
									}
									else
										Ystr[0]='+';
									for(j=6; j>=3; j--){
										Ystr[j] = temp%10 + 48; 				// +48 - Turn to ASCII (1+48=49, 49 is 1 in ASCII)  
										temp = temp / 10;
									}
									Ystr[1] = temp%10+48;						// in the str[2] there is '.'
									UARTprintf(UART0_BASE_PTR,Ystr);
									UARTprintf(UART0_BASE_PTR,"\r\n");
							break;
							case 'z':
									temp = Zout_g * 10000; 						// Turn from decimal num to int (1.234 --> 1234) 
									if(Zout_g<0){
										Zstr[0]='-';
										temp = 0-temp; 							// x= |x|
									}
									else
										Zstr[0]='+';
									for(j=6; j>=3; j--){
										Zstr[j] = temp%10 + 48; 				// +48 - Turn to ASCII (1+48=49, 49 is 1 in ASCII) 
										temp = temp / 10;
									}
									Zstr[1] = temp%10+48;						// in the str[2] there is '.'
									UARTprintf(UART0_BASE_PTR,Zstr);
									UARTprintf(UART0_BASE_PTR,"\r\n");
							break;
							} // end of switch case
						} // end of if data ready acc
				    } // end of while(sysmove)
				} // end of if strcmp sysmove
				else{
					//---------------------------------------------------------------------------------------------------------------------
					// System movement
					// --------------------------------------------------------------------------------------------------------------------
					if (strncmp(pcmessage,"3Dscan",6) == 0){
						numOfRowsToScan = pcmessage[6]-48;
						memset(pcmessage,0,40);  //memset - clears the array
						while((state==1) & (rowsScanned<numOfRowsToScan))
						{
								if (DataReadyAcc)		// Is a new set of data ready? 
								{  		
									DataReadyAcc = 0;
																							
									I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06 
									
									Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
									Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
									Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value
									
									Xout_g = ((float) Xout_14_bit) / SENSITIVITY_2G;		// Compute X-axis output value in g's
									Yout_g = ((float) Yout_14_bit) / SENSITIVITY_2G;		// Compute Y-axis output value in g's
									Zout_g = ((float) Zout_14_bit) / SENSITIVITY_2G;		// Compute Z-axis output value in g's
									
									roll = atan(Yout_g / sqrt(pow(Xout_g, 2) + pow(Zout_g, 2))) * 180 / pi;
									pitch = atan(-1 * Xout_g / sqrt(pow(Yout_g, 2) + pow(Zout_g, 2))) * 180 / pi;
									
									if ((roll <= -45) & (-10<=pitch) &(pitch<=10) & (scanningFlag == 0)){ //Start scan
											int j;
											scanningFlag ^= 1;
											BLUE_LED_ON;
											//for (j=1000000; j>0; j--);	 	  // Delay
											UARTprintf(UART0_BASE_PTR,"StartRowScan");
											UARTprintf(UART0_BASE_PTR,"\r\n");
											TPM0_SC |= TPM_SC_CMOD(1);  // Start the TPM0 counter
									}
									else{
										if ((roll >= 45) & (-10<=pitch) &(pitch<=10) & (scanningFlag == 1)){ //Stop scan
											scanningFlag ^= 1;
											TPM0_SC |= TPM_SC_CMOD(0);                      // Stop the TPM0 counter
											BLUE_LED_OFF;
											RED_LED_OFF;
											rowsScanned++;
											UARTprintf(UART0_BASE_PTR,"EndOfRow");
											UARTprintf(UART0_BASE_PTR,"\r\n");	
										}
										if (((pitch > 5) | (pitch < -5)) & (scanningFlag == 1)){
											BLUE_LED_OFF;
											RED_LED_ON;
										}
										if((pitch < 5) & (pitch > -5) & (scanningFlag == 1)){
											BLUE_LED_ON;
										    RED_LED_OFF;
										}
									}
									
                                    if(scanningFlag){
										int temp,j;
										if (DataReadyDis)
										{
											DataReadyDis = 0;
											temp = distance * 10; 		 // Turn from decimal num to int (123.4 --> 1234)
											disString[4] = temp%10 + 48; 
											temp = temp / 10;
											for(j=2; j>=0; j--){
											disString[j] = temp%10 + 48; // +48 - Turn to ASCII (1+48=49, 49 is 1 in ASCII) 
											temp = temp / 10;
											}	
										}
										if((strncmp(disString,"NaN",3)) != 0){
										UARTprintf(UART0_BASE_PTR,disString);
										UARTprintf(UART0_BASE_PTR,"\r\n");
										}
                                    }
								} // end of if data ready acc
						}//end while state 1\ all rows scanned
						if(rowsScanned>=numOfRowsToScan){
							UARTprintf(UART0_BASE_PTR,"EndOfScan");
							UARTprintf(UART0_BASE_PTR,"\r\n");	
							rowsScanned=0;
						}
					} // end of if strcmp 3Dscan
				}//end of else	
			}//end of else
  		} //end of if data ready uart
  	} //end of while 1
  	return 0;
}

//-----------------------------------------------------------------
//  FTM2 - ISR = Interrupt Service Routine - Echo
//-----------------------------------------------------------------
void FTM2_IRQHandler(){
	if (captureFlag ==0){
		captureR = TPM2_C0V;    // Time of rising edge in Echo pulse
		captureFlag = 1;
	}
	else{
		captureF = TPM2_C0V;   // Time of falling edge in Echo pulse
		captureFlag = 0;
		distance = (captureF - captureR)/43.3;  //Calculate distance from sensor (in room temp)
		DataReadyDis = 1;
		TPM2_SC |= TPM_SC_CMOD(0);  // Stop the TPM2 counter
		TPM2_CNT = 1;
	}
	//TPM2_SC |= TPM_SC_TOF_MASK;
	TPM2_C0SC |= TPM_CnSC_CHF_MASK; 				//Manual flag down of the timer
}

//-----------------------------------------------------------------
//  FTM0 - ISR = Interrupt Service Routine - Trigger
//-----------------------------------------------------------------
void FTM0_IRQHandler(){
	//TPM0_SC |= TPM_SC_TOF_MASK;
	TPM0_C1SC |= TPM_CnSC_CHF_MASK; 				//Manual flag down of the timer
	TPM2_SC |= TPM_SC_CMOD(1);  // Start the TPM2 counter
}

//-----------------------------------------------------------------
//  UART0 - ISR
//-----------------------------------------------------------------
void UART0_IRQHandler(){
		
	uint8_t Temp;
		
	if(UART0_S1 & UART_S1_RDRF_MASK){ // RX buffer is full and ready for reading
		
		Temp = UART0_D;
		if (Temp != '\r'){
			if (Temp != '@'){         //insert chars to array until pressing Enter
				pcmessage[arridx] = Temp;   
				arridx += 1;
			}
			else{
				if (strncmp(pcmessage,"Telemeter", 9) == 0){ 
					state = 3;
				}
				else{
					if (strncmp(pcmessage,"Sysmove", 7) == 0){
						state = 2;
						//UARTprintf(UART0_BASE_PTR,"ack");
						//UARTprintf(UART0_BASE_PTR,"\r\n");	
					}
					else{
						if (strncmp(pcmessage,"3Dscan", 6) == 0){
							state = 1;
						}
					}
				}
				
				arridx = 0;
				DataReadyUart = 1;
			}	
		}
	}
}


void MCU_Init(void)
{
	//I2C0 module initialization
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;		// Turn on clock to I2C0 module 
	SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;		// Turn on clock to Port E module 
	PORTE_PCR24 = PORT_PCR_MUX(5);			// PTE24 pin is I2C0 SCL line 
	PORTE_PCR25 = PORT_PCR_MUX(5);			// PTE25 pin is I2C0 SDA line 
	I2C0_F  = 0x14; 						// SDA hold time = 2.125us, SCL start hold time = 4.25us, SCL stop hold time = 5.125us *
	I2C0_C1 = I2C_C1_IICEN_MASK;    		// Enable I2C0 module 
	
	//Configure the PTA14 pin (connected to the INT1 of the MMA8451Q) for falling edge interrupts
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;		// Turn on clock to Port A module 
	PORTA_PCR14 |= (0|PORT_PCR_ISF_MASK|	// Clear the interrupt flag 
					  PORT_PCR_MUX(0x1)|	// PTA14 is configured as GPIO 
					  PORT_PCR_IRQC(0xA));	// PTA14 is configured for falling edge interrupts 
	
	//Enable PORTA interrupt on NVIC
	NVIC_ICPR |= 1 << ((INT_PORTA - 16)%32); 
	NVIC_ISER |= 1 << ((INT_PORTA - 16)%32); 
}

/******************************************************************************
* Accelerometer initialization function
******************************************************************************/ 

void Accelerometer_Init (void)
{
	unsigned char reg_val = 0;
	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x40);		// Reset all registers to POR values
	
	do		// Wait for the RST bit to clear 
	{
		reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, CTRL_REG2) & 0x40; 
	} 	while (reg_val);
	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, XYZ_DATA_CFG_REG, 0x00);		// +/-2g range -> 1g = 16384/4 = 4096 counts 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG2, 0x02);		// High Resolution mode
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x3D);	// ODR = 1.56Hz, Reduced noise, Active mode	
}

/******************************************************************************
* Simple offset calibration
******************************************************************************/ 

void Calibrate (void)
{
	unsigned char reg_val = 0;
	
	while (!reg_val)		// Wait for a first set of data		 
	{
		reg_val = I2C_ReadRegister(MMA845x_I2C_ADDRESS, STATUS_REG) & 0x08; 
	} 	
	  	
	I2C_ReadMultiRegisters(MMA845x_I2C_ADDRESS, OUT_X_MSB_REG, 6, AccData);		// Read data output registers 0x01-0x06  
	  						
	Xout_14_bit = ((short) (AccData[0]<<8 | AccData[1])) >> 2;		// Compute 14-bit X-axis output value
	Yout_14_bit = ((short) (AccData[2]<<8 | AccData[3])) >> 2;		// Compute 14-bit Y-axis output value
	Zout_14_bit = ((short) (AccData[4]<<8 | AccData[5])) >> 2;		// Compute 14-bit Z-axis output value
	  					
	Xoffset = Xout_14_bit / 8 * (-1);		// Compute X-axis offset correction value
	Yoffset = Yout_14_bit / 8 * (-1);		// Compute Y-axis offset correction value
	Zoffset = (Zout_14_bit - SENSITIVITY_2G) / 8 * (-1);		// Compute Z-axis offset correction value
	  					
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x00);		// Standby mode to allow writing to the offset registers	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_X_REG, Xoffset);		
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_Y_REG, Yoffset);	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, OFF_Z_REG, Zoffset);	
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG3, 0x00);		// Push-pull, active low interrupt 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG4, 0x01);		// Enable DRDY interrupt 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG5, 0x01);		// DRDY interrupt routed to INT1 - PTA14 
	I2C_WriteRegister(MMA845x_I2C_ADDRESS, CTRL_REG1, 0x2D);		// ODR = 1.56Hz, Reduced noise, Active mode	
}
/******************************************************************************
* PORT A Interrupt handler
******************************************************************************/ 

void PORTA_IRQHandler()
{
	PORTA_PCR14 |= PORT_PCR_ISF_MASK;			// Clear the interrupt flag 
	DataReadyAcc = 1;	
}

