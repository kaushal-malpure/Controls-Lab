/*
 * Arduino Pin Connections
 * SPI Chip Select Enc 0:   Pin  2
 * SPI Chip Select Enc 1:   Pin  3
 * SPI MOSI                 Pin 11
 * SPI MISO                 Pin 12
 * SPI SCLK:                Pin 13
 * 
 * 
 * AMT22 Pin Connections
 * Vdd (5V):                Pin  1
 * SPI SCLK:                Pin  2
 * SPI MOSI:                Pin  3
 * GND:                     Pin  4
 * SPI MISO:                Pin  5
 * SPI Chip Select:         Pin  6
 * 
*/

#include <SPI.h>

#define BAUDRATE        115200

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* SPI pins */
#define ENC_0            2 // Upar waala pin 2 encoder 0
#define ENC_1            5 // Neeche waala pin 3 encoder 1
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52

#define mPin1  11 // Input 1 = mPin1
#define mPin2  13 // Input 2 = mPin2

float prev_time, prev_angle0, prev_angle1 = 0.0;
float k1 = -1.9047;
float k2 = 105.4101;
float k3 = -2.1327;
float k4 = 15.8193;

void setup() 
{
  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_0, OUTPUT);
  pinMode(ENC_1, OUTPUT);
  
  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);

  //Get the CS line high which is the default inactive state
  digitalWrite(ENC_0, HIGH);
  digitalWrite(ENC_1, HIGH);

  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  
  //start SPI bus
  SPI.begin();
}

void loop() 
{  float angle0, angle1;
  //create a 16 bit variable to hold the encoders position
  uint16_t encoderPosition;
  //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
  uint8_t attempts;

  //if you want to set the zero position before beggining uncomment the following function call
  setZeroSPI(ENC_0);
  setZeroSPI(ENC_1);

  Serial.println("Set Arm");

  while(millis()<5000){}; // set arm position

  //once we enter this loop we will run forever
  while(1)
  {
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;

    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = getPositionSPI(ENC_0, RES14); 

    //if the position returned was 0xFFFF we know that there was an error calculating the checksum
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(ENC_0, RES14); //try again
    }

    if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
    {
      Serial.print("Encoder 0 error. Attempts: ");
      Serial.print(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
      Serial.write(NEWLINE);
    }
    else //position was good, print to serial stream
    {
            angle0 = map(encoderPosition,0,16562,0,360);
            angle0 = -3.1415+angle0*0.01745;
      Serial.print("Encoder 0: ");
      Serial.print(angle0, DEC); //print the position in decimal format
//      Serial.print(encoderPosition, DEC); //print the position in decimal format
      Serial.write(" ");
    }

    //////////again for second encoder//////////////////////////////
    
    //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;

    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = getPositionSPI(ENC_1, RES14); 

    //if the position returned was 0xFFFF we know that there was an error calculating the checksum
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(ENC_1, RES14); //try again
    }

    if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
    {
      Serial.print("Encoder 1 error. Attempts: ");
      Serial.print(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
      Serial.write(" ");
    }
    else //position was good, print to serial stream
    {
      angle1 = map(encoderPosition,0,16562,0,360);
      angle1 = angle1*0.01745 - 3.1415;
      Serial.print("Encoder 1: ");
//      Serial.print(encoderPosition, DEC); //print the position in decimal format
      Serial.print(angle1, DEC); 
      Serial.write(" ");
    }

    //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
    //delay() is in milliseconds  
    
    float current_time = millis();
    float d_angle0 = (angle0-prev_angle0)*1000/(current_time-prev_time);
    float d_angle1 = (angle1-prev_angle1)*1000/(current_time-prev_time);

    float motor_vol = (-k1*angle1 - k2*angle0 - k3*d_angle1 - k4*d_angle0);
    float duty_cycle = motor_vol / 12;

    Serial.println(motor_vol);
    Serial.write(" ");
    

    if(duty_cycle >= 0.75) { duty_cycle = 0.75;}
    else if (duty_cycle <= -0.75) { duty_cycle = -0.75;}

    if (duty_cycle >0) {
      analogWrite(mPin2, duty_cycle*255);
      analogWrite(mPin1, 0);
    }
    else {
      analogWrite(mPin1, -duty_cycle*255);
      analogWrite(mPin2, 0);
     }  
      
    Serial.println(duty_cycle);
    Serial.write(" ");
    prev_time = current_time;
    prev_angle0 = angle0;
    prev_angle1 = angle1;
    delay(1);
  }
}

/*
 * This function gets the absolute position from the AMT22 encoder using the SPI bus. The AMT22 position includes 2 checkbits to use
 * for position verification. Both 12-bit and 14-bit encoders transfer position via two bytes, giving 16-bits regardless of resolution.
 * For 12-bit encoders the position is left-shifted two bits, leaving the right two bits as zeros. This gives the impression that the encoder
 * is actually sending 14-bits, when it is actually sending 12-bit values, where every number is multiplied by 4. 
 * This function takes the pin number of the desired device as an input
 * This funciton expects res12 or res14 to properly format position responses.
 * Error values are returned as 0xFFFF
 */
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit. 
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.  
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere 
 * This function takes the pin number of the desired device as an input
 */
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the 
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3); 
  
  spiWriteRead(AMT22_RESET, encoder, true);
  
  delay(250); //250 second delay to allow the encoder to start back up
}
