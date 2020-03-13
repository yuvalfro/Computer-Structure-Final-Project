#include "TFC.h"
#include "mcg.h"

#define MUDULO_REGISTER  0x2EE0

// set I/O for switches and LEDs
void InitGPIO()
{
	//enable Clocks to all ports - page 206, enable clock to Ports
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	//GPIO Configuration - LEDs - Output
	PORTD_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;  //Blue
	GPIOD_PDDR |= BLUE_LED_LOC; //Setup as output pin
	
	PORTB_PCR18 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Red  
    PORTB_PCR19 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK; //Green
    GPIOB_PDDR |= RED_LED_LOC + GREEN_LED_LOC; //Setup as output pins
    
    //PORTD_PCR7 = PORT_PCR_MUX(1); // assign PTD7 as GPIO
    PORTC_PCR2 = PORT_PCR_MUX(4); // assign PTC2 as TPM0_CH1
    PORTB_PCR2 = PORT_PCR_MUX(3); // assign PTB2 as TPM2_CH0
    //GPIOD_PDDR &= ~PORT_LOC(7);  // PTD7 is Input
    //PORTD_PCR7 = PORT_PCR_IRQC(9); //PTD7 interrupt for rising edge
	//enable_irq(INT_PORTD-16); // Enable Interrupts 
	//set_irq_priority (INT_PORTD-16,0);  // Interrupt priority = 0 = max
}

//-----------------------------------------------------------------
// DipSwitch data reading
//-----------------------------------------------------------------
uint8_t TFC_GetDIP_Switch()
{
	uint8_t DIP_Val=0;
	
	DIP_Val = (GPIOC_PDIR>>4) & 0xF;

	return DIP_Val;
}

//-----------------------------------------------------------------
// TPMx - Initialization
//-----------------------------------------------------------------
void InitTPM(){
	//TPM0 Channel1 - Trigger - PTC2
	TPM0_SC = 0; // to ensure that the counter is not running
	TPM0_SC |= TPM_SC_PS(7);//+ TPM_SC_TOIE_MASK; //Prescaler =128, enable TOF (overflowed by MOD) interrupts
	TPM0_MOD = 0x2DC7; // PWM frequency of 16Hz = 24MHz/(128x11,719)
	TPM0_C1SC |= TPM_CnSC_MSB_MASK + TPM_CnSC_ELSB_MASK + TPM_CnSC_CHIE_MASK;//+ TPM_CnSC_CHF_MASK;; //Edge Aligned PWM
	TPM0_C1V = 0x07; //High level of trigger to ultrasonic is 37microsec
	TPM0_CONF |= TPM_CONF_DBGMODE(3); //LPTPM counter continues in debug mode
	enable_irq(INT_TPM0-16); // Enable Interrupts 
	set_irq_priority (INT_TPM0-16,0);  // Interrupt priority = 0 = max
	
	//TPM2 Channel0 - Echo - PTB2
	TPM2_SC = 0; // to ensure that the counter is not running
	TPM2_SC |= TPM_SC_PS(5) + TPM_SC_TOF_MASK;  //Prescaler =32, clear flag
	//24Mhz/32=0.75Mhz, T=1.333us*2^16=0.0873s= overflow time
	TPM2_MOD = 0xFFFF; // value of overflow 
	TPM2_C0SC |= TPM_CnSC_ELSB_MASK + TPM_CnSC_ELSA_MASK + TPM_CnSC_CHIE_MASK + TPM_CnSC_CHF_MASK;//input capture on both rising and falling
	TPM2_CONF |= TPM_CONF_DBGMODE(3); //LPTPM counter continues in debug mode
	enable_irq(INT_TPM2-16); // Enable Interrupts 
	set_irq_priority (INT_TPM2-16,0);  // Interrupt priority = 0 = max
	
}

//-----------------------------------------------------------------
// TPMx - Clock Setup
//-----------------------------------------------------------------
void ClockSetup(){
	    
	    pll_init(8000000, LOW_POWER, CRYSTAL,4,24,MCGOUT); //Core Clock is now at 48MHz using the 8MHZ Crystal
		
	    //Clock Setup for the TPM requires a couple steps.
	    //1st,  set the clock mux
	    //See Page 124 of f the KL25 Sub-Family Reference Manual
	    SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;// We Want MCGPLLCLK/2=24MHz (See Page 196 of the KL25 Sub-Family Reference Manual
	    SIM_SOPT2 &= ~(SIM_SOPT2_TPMSRC_MASK);
	    SIM_SOPT2 |= SIM_SOPT2_TPMSRC(1); //We want the MCGPLLCLK/2 (See Page 196 of the KL25 Sub-Family Reference Manual
		//Enable the Clock to the TPM0 and PIT Modules
		//See Page 207 of f the KL25 Sub-Family Reference Manual
		SIM_SCGC6 |= SIM_SCGC6_TPM0_MASK + SIM_SCGC6_TPM2_MASK;
	    // TPM_clock = 24MHz , PIT_clock = 48MHz
}

