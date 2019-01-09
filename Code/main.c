/* Obstacle Avoidance Smart Car
   - Avoids walls, slopes, and edges
*/
/* Ground Sonar: Power supply is Tiva board.
   - PA3 -> Trig
	 - PA2 -> Echo (2/3 Voltage Divider)
   Wall Sonar: Power supply is Tiva board.
   - PA5 -> Trig
	 - PA4 -> Echo (2/3 Voltage Divider)
*/
/* Left Servo: Power supply is battery pack(6V).
   - PWM -> PD0
	 Right Servo: Power supply is battery pack(6V).
	 -PWM -> PD1
	 Fanning Servo: Power supply is Tiva board.
	 -PWM -> PE4
*/

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_timer.h"
#include "inc/hw_uart.h"
#include "inc/hw_gpio.h"
#include "inc/hw_pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"

#include "utils/uartstdio.c"
#include <string.h>

#include "inc/tm4c123gh6pm.h"

//*****************************************************************************

//Constant for converting sonar pulse into distance value
const double temp = 1.0/80.0;

//Stores the pulse length of sonar sensor
volatile uint32_t pulse1=0;  // Ground sonar
volatile uint32_t pulse2=0;  // Wall sonar

//Load for servos
volatile uint32_t Load;

//Adjusts position of fanning servo
volatile uint8_t fan=50;

#define PWM_FREQ 50

//*****************************************************************************

void Interrupt_Init(void)
{
	IntPriorityGroupingSet(0x03);       // puts interrupt in Group 3
  IntEnable(INT_GPIOA);  							// enable interrupt (Port A)
	IntPrioritySet(INT_GPIOA, 0x00); 		// GPIOA interrupt priority is 0
	
	// Ground sonar
	GPIO_PORTA_IM_R |= 0x04;   					// arm interrupt on PA2
	GPIO_PORTA_IS_R &= ~0x04;     			// PA2 is edge-sensitive
  GPIO_PORTA_IBE_R |= 0x04;   				// PA2 both edge trigger
	
	// Wall sonar
	GPIO_PORTA_IM_R |= 0x10;   					// arm interrupt on PA4
	GPIO_PORTA_IS_R &= ~0x10;     			// PA4 is edge-sensitive
  GPIO_PORTA_IBE_R |= 0x10;   				// PA4 both edge trigger
	
	IntMasterEnable();        					// globally enable interrupt
}

void InitConsole(void)
{
	  // Enables UART to print out distance values
	
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlDelay(3);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);
}

void TimerInit(unsigned long period){
	//Set timer0 to be periodic
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER0_BASE, TIMER_A, period-1);
	IntPriorityGroupingSet(0x03);       // puts interrupt in Group 3
	IntPrioritySet(INT_TIMER0A, 0x01);  // Periodic Interrupt priority is 1
	IntEnable(INT_TIMER0A);             // enable interrupt
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
	
  //Set timer2 and timer5 to be periodic and count up
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  SysCtlDelay(3);
  TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC_UP);
  TimerEnable(TIMER2_BASE,TIMER_A);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
  SysCtlDelay(3);
  TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC_UP);
  TimerEnable(TIMER5_BASE,TIMER_A);
}

void GPIOPortA_Handler(void){
	//Clear interrupt flag
  GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_4);

	// Update pulse for ground sonar
	if(GPIO_INT_PIN_2 == GPIO_INT_PIN_2){
		//If it's a rising edge then set the timer to 0
		if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2){
		HWREG(TIMER2_BASE + TIMER_O_TAV) = 0; //Loads value 0 into the timer.
			TimerEnable(TIMER2_BASE,TIMER_A);
		}
		//If it's a falling edge, then get the value of the counter
		else{
			pulse1 = TimerValueGet(TIMER2_BASE,TIMER_A); //record value
			TimerDisable(TIMER2_BASE,TIMER_A);
			//Converts the counter value to cm.
      pulse1 =(uint32_t)(temp * pulse1);
      pulse1 = pulse1 / 58;
		}
	}
	
	// Update pulse for wall sonar
	if(GPIO_INT_PIN_4 == GPIO_INT_PIN_4){
		//If it's a rising edge then set the timer to 0
		if ( GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_4) == GPIO_PIN_4){
		HWREG(TIMER5_BASE + TIMER_O_TAV) = 0; //Loads value 0 into the timer.
			TimerEnable(TIMER5_BASE,TIMER_A);
		}
		//If it's a falling edge, then get the value of the counter
		else{
			pulse2 = TimerValueGet(TIMER5_BASE,TIMER_A); //record value
			TimerDisable(TIMER5_BASE,TIMER_A);
			//Converts the counter value to cm.
			pulse2 =(uint32_t)(temp * pulse2);
      pulse2 = pulse2 / 58;
		}
	}
	
}

void Timer0A_Handler(void){
	//Clear interrupt flag
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	
	//Scan with the servo
	switch(fan){
		case 50: fan = 85;
             PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, fan * Load / 1000);
		         break;
		case 85: fan = 116;
		          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, fan * Load / 1000);
		          break;
		default: fan = 50;
		         PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, fan * Load / 1000);
		         break;
	}

}

int main(void)
{
  volatile uint32_t PWMClock;
	volatile uint8_t Adjust;
	volatile int counter = 3000000;  // How long to make the car move back
	unsigned long period = 24000000; // 0.3s period for fanning servo
  Adjust = 75;
  
	// Set clock to 80 MHz
  SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	
	// Enable clocks
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  
	//Configures the UART
  InitConsole();
	
	//Configures the timer
  TimerInit(period);
	
	//Configure Trigger pin for Sonar
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlDelay(3);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlDelay(3);
  GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
	
	//Configure Echo pin for Sonar
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlDelay(3);
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
  GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);
  GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2,GPIO_BOTH_EDGES);
  GPIOIntRegister(GPIO_PORTA_BASE,GPIOPortA_Handler);
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlDelay(3);
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);
  GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_4);
  GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_4,GPIO_BOTH_EDGES);
  GPIOIntRegister(GPIO_PORTA_BASE,GPIOPortA_Handler);
	
	// Enable pins for servo PWMs
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
  GPIOPinConfigure(GPIO_PD0_M1PWM0);
	GPIOPinConfigure(GPIO_PD1_M1PWM1);
	GPIOPinConfigure(GPIO_PE4_M0PWM4);

  // Configure PWMs
  PWMClock = SysCtlClockGet() / 64;
  Load = (PWMClock / PWM_FREQ) - 1;
  PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, Load);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, Load);

  // PWM settings
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Adjust * Load / 1000);
  PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);
  PWMGenEnable(PWM1_BASE, PWM_GEN_0);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, Adjust * Load / 1000);
  PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);
  PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 50 * Load / 1000);
  PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
  PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	
	while(1)
	{
      // 10uS pulse to activate sonar sensors
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_PIN_3);
      SysCtlDelay(266);
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, ~GPIO_PIN_3);
			
			GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
      SysCtlDelay(266);
      GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, ~GPIO_PIN_5);

      //Prints out measured sonar distances to Tera Term
      UARTprintf("ground = %2dcm  wall = %2dcm \n" , pulse1, pulse2);
			
			if(((pulse1<10)&&(pulse1>5))||(pulse2 >10)||(pulse2 == 0)) //Forward
      {
        Adjust = 90;  // Rotate servo counter-clockwise
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Adjust * Load / 1000);
	  		Adjust = 50;   // Rotate servo clockwise
		  	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, Adjust * Load / 1000);
      }
		
	  	if((pulse1>6)||(pulse1<4)||(pulse2<5))  //Backward, turn left
      {
        Adjust = 65; // Rotate servo clockwise
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, Adjust * Load / 1000);
	  		Adjust = 110; // Rotate servo counter-clockwise 80
		  	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, Adjust * Load / 1000);
				
				do{  // Run for some time
					counter = counter-1;
				}while(counter > 1);
				counter = 2000000; // Reset counter
      }
      //wait about 15ms until the next reading.
      SysCtlDelay(400000);
		
	}
}
