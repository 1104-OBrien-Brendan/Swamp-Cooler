#include <dht.h>
dht DHT;
#include <Stepper.h>

#define RDA 0x80
#define TBE 0x20  
#define DHT11_PIN 7

//Declarations for Stepper Motor
const int stepsPerRevolution = 2038; // Total steps per revolution 
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

// UART Pointers
volatile unsigned char *myUCSR0A  = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B  = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C  = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0   = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0    = (unsigned char *)0x00C6;
// GPIO Pointers
volatile unsigned char *portA     = (unsigned char *) 0x22;
volatile unsigned char *portDDRA = (unsigned char *) 0x21;
volatile unsigned char *portE     = (unsigned char *) 0x2E;
volatile unsigned char *portDDRE  = (unsigned char *) 0x2D;

//water sensor pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//Button for Stepper Motor
int lastButtonState = 0; // last button state
int motorPosition = 0; //  track the motor position
volatile unsigned char* pinB = (unsigned char*) 0x23;
volatile unsigned char *ddrB =  (unsigned char*) 0x24;  
volatile unsigned char* portB = (unsigned char*) 0x25;

//pin 2 and 3 are chip inputs 5 is enabler and A10 is button input checking for ground input to set low 
// UART Pointers
volatile unsigned char* portK = (unsigned char*) 0x108;
volatile unsigned char* ddrK  = (unsigned char*) 0x107; 
volatile unsigned char* pinK  = (unsigned char*) 0x106; 


void ledLoop();

void waterSensorLoop();

//void LoopDHT11();

void openVent();
void closeVent();

void fanMotor();




void setup(){                
  Serial.begin(9600);

  // Start the UART
  U0Init(9600);

  //set up ADC:
  adc_init();

  // Set the speed of the stepper motor:
  myStepper.setSpeed(10);
  // Start serial communication at a baud rate of 9600:
  *ddrB &= 0x7F;
  *portB |= 0x80;
  
  *portDDRE |= 0x38;
  *portK |= 0x04;
  *ddrK &= 0x04;
  *portE |= 0x20;

}

void loop(){
  
ledLoop();

waterSensorLoop();

LoopDHT11();

//Stepper Motor
    int button = (*pinB & 0x80);
    Serial.println(button);
    if (!(*pinB & 0x80)) { 
        openVent();
        closeVent();
        delay(1000);
    }

fanMotor();

}

void U0Init(int U0baud){
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char kbhit(){
  return *myUCSR0A & RDA;
}
unsigned char getChar(){
  return *myUDR0;
}
void putChar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void ledLoop(){
  // set 22,23,24 to output for RED, YELLOW, GREEN BLUE LED’s
  *portDDRA |= 0x07;
  // set pin 22, HIGH RED
  *portA |= 0x01;
    delay(1500);
  //set pin ALL LED's LOW
  *portA = 0x00;
    delay(1500);
  // set pin 23, HIGH RED
  *portA |= 0x02;
    delay(1500);
  //set pin ALL LED's LOW
  *portA = 0x00;
    delay(1500);
  // set pin 24, HIGH RED
  *portA |= 0x04;
    delay(1500);
  //set pin ALL LED's LOW
  *portA = 0x00;
    delay(1500);
}
void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result ~ ADLAR=0
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num){
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7){
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void U0init(int U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}
unsigned char U0kbhit(){
  return *myUCSR0A & RDA;
}
unsigned char U0getchar(){
  return *myUDR0;
}
void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE)==0);
  *myUDR0 = U0pdata;
}

void waterSensorLoop(){
  unsigned int cs1 = adc_read(0);

  U0putchar(((cs1 % 1000) / 100) + '0');
  U0putchar(((cs1 % 100) / 10) + '0');
  U0putchar((cs1 % 10) + '0');
  U0putchar('\n');
}

void LoopDHT11(){
int chk = DHT.read11(DHT11_PIN);
  unsigned int temp = DHT.temperature;
  unsigned int temperature = DHT.temperature; //creates int variable with temperature value
  U0putchar('T');
  U0putchar('e');
  U0putchar('m');
  U0putchar('p');
  U0putchar(':');
  U0putchar(' ');
  U0putchar(((temp % 100) / 10) + '0'); 
  U0putchar((temp % 10) + '0'); 
  U0putchar('\n');

  unsigned int hum = DHT.humidity;
  unsigned int humidity = DHT.humidity;//creates int variable with humidity value

  U0putchar('H');
  U0putchar('u');
  U0putchar('m');
  U0putchar(':');
  U0putchar(' ');
   
  U0putchar(((hum % 100) / 10) + '0'); 
  U0putchar((hum % 10) + '0'); 
  U0putchar('\n');
  delay(1500);
}

void openVent(){
  if (motorPosition == 0){
    myStepper.step(stepsPerRevolution / 2); // Turn 180 degrees forwards
    motorPosition = 1; // Update the motor position
    lastButtonState = 1;
  }
}

void closeVent(){
  if (motorPosition == 1) {
    myStepper.step(-stepsPerRevolution / 2); // Return to original position
    motorPosition = 0; // Update the motor position
    lastButtonState = 0; // Update the lastButtonState
  }
}

void fanMotor(){
  if(*pinK & 0x04){
    *portE |= 0x28;
  }
  else{
    *portE &= 0xF7;//Sets pin 5 low when button is pressed
    Serial.print('low');
  }
}
