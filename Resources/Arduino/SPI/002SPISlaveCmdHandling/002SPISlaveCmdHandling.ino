/*

 * SPI pin numbers:
 * SCK   v  // RED //    Serial Clock.
 * MISO  12  // BLK //    Master In Slave Out.
 * MOSI  11  // YELLOW // Master Out Slave In.
 * SS    10  // GREEN //  Slave Select
 *

 */

#include <SPI.h>

const byte led = 8;           // Slave LED digital I/O pin.

boolean ledState = HIGH;      // LED state flag.

uint8_t dataBuff[255];

const uint8_t board_id[10] = "ARDUINOUNO";

#define NACK  0xA5
#define ACK   0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT_PULLUP);
  //make SPI as slave
  
  SPCR = (1<<SPE);              // enable SPI slave
  SPDR = 0xFF; // preload
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  while (!(SPSR & (1 << SPIF)));  // wait transfer complete
  return SPDR;                    // returns byte received on MOSI
}


void SPI_SlaveTransmit(uint8_t data)
{
  SPDR = data;                    // preload byte to shift out on MISO
  while (!(SPSR & (1 << SPIF)));  // wait until master clocks it out
  (void)SPDR;   
}
  

// The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(115200);
  
  // Initialize slave LED pin.
  pinMode(8, OUTPUT);
  
  digitalWrite(led,LOW);
  
  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}


byte checkData(byte cmd){
  if(cmd == COMMAND_LED_CTRL || cmd == COMMAND_SENSOR_READ) return ACK;
  return NACK;
}

// The loop function runs continuously after setup().
void loop() 
{
  byte data,command, pin,value,ackornack=NACK;
  
  //1. fist make sure that ss is low . so lets wait until ss is low 
  //Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS) == HIGH);   // wait selected
  
  //2. now lets wait until rx buffer has a byte
  command = SPI_SlaveReceive();

  ackornack = checkData(command);
  
  SPI_SlaveTransmit(ackornack);

  (void)SPI_SlaveReceive();         // consume dummy
  
if(command == COMMAND_LED_CTRL)
{
  pin = SPI_SlaveReceive();
  value = SPI_SlaveReceive();

  pinMode(pin, OUTPUT);                 // <-- ADD THIS
  digitalWrite(pin, value ? HIGH : LOW);
}
  
  else if ( command == COMMAND_SENSOR_READ)
  {
    //read analog pin number 
    uint16_t aread;
    uint8_t pin = SPI_SlaveReceive(); 
    //pinMode(pin+14, INPUT_PULLUP);
    uint8_t val;
    aread = analogRead(pin+14);
    val = map(aread, 0, 1023, 0, 255);
    
    SPI_SlaveTransmit(val);
  
    val = SPI_SlaveReceive(); //dummy read
    
    Serial.println("RCVD:COMMAND_SENSOR_READ");
  
    
  
  }
  while (digitalRead(SS) == LOW); // wait end of frame
  Serial.print("cmd=0x"); Serial.print(command, HEX);
Serial.print(" pin=");  Serial.print(pin);
Serial.print(" val=");  Serial.println(value);
  /*
  else if ( command == COMMAND_LED_READ)
  {
    uint8_t pin = SPI_SlaveReceive(); 
    uint8_t val = digitalRead(pin);
    SPI_SlaveTransmit(val);
    val = SPI_SlaveReceive(); //dummy read
    Serial.println("RCVD:COMMAND_LED_READ");
  
  }else if ( command == COMMAND_PRINT)
  {
    uint8_t len = SPI_SlaveReceive(); 
    for(int i=0 ; i < len ; i++)
    {
      dataBuff[i] = SPI_SlaveReceive();
    }
    Serial.println((char*)dataBuff);
    
    Serial.println("RCVD:COMMAND_PRINT");
  
  }else if ( command == COMMAND_ID_READ)
  {
    for(int i=0 ; i < strlen(board_id) ; i++)
    {
      SPI_SlaveTransmit(board_id[i]);
    }
      SPI_SlaveReceive();
    Serial.println("RCVD:COMMAND_ID_READ");
  }
 */

}
