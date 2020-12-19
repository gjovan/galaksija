#define SHIFT_DATA 25
#define SHIFT_CLK 27
#define SHIFT_LATCH 26
#define WRITE_EN 12
#define OUT_EN 13



/* 2K */

#define BYTES 2048
#include "char_rom.h"


const int buttonPin = 14;
int data_pin[]= { 15, 16, 17, 18, 23, 19, 21, 22 };

char buf[80];
int buttonState = 0; 
int state = 0;
int addr = 0; 

int topAddr= BYTES; /* treba BYTES */ 

byte dat[BYTES];

/*
 * Output the address bits and outputEnable signal using shift registers.
 */
void setAddress(int address, bool outputEnable) {
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, (address >> 8) | (outputEnable ? 0x00 : 0x80));
  //shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address >> 8);
  shiftOut(SHIFT_DATA, SHIFT_CLK, MSBFIRST, address);

  digitalWrite(SHIFT_LATCH, LOW);
  digitalWrite(SHIFT_LATCH, HIGH);
  digitalWrite(SHIFT_LATCH, LOW);
}

void setData( byte dat ) {
  for (int sbit = 0; sbit < 8; sbit += 1) {
    digitalWrite(data_pin[sbit], dat & 1);
    dat = dat >> 1;
  }
}

/*
 * Read a byte from the EPROM at the specified address.
 */
byte readEPROM(int address) {
  digitalWrite(OUT_EN,HIGH);
    digitalWrite(WRITE_EN,HIGH);
  setAddress(address, /*outputEnable*/ true); 
  delay(1);
  byte data = 0;

  delay(1);
  digitalWrite(OUT_EN,LOW);
    digitalWrite(WRITE_EN,LOW);
  delay(2);
    
  for (int sbit = 7; sbit >= 0; sbit -= 1) {
    data = (data << 1) + digitalRead(data_pin[sbit]);
  }
  //setAddress(/*address*/ 0, /*outputEnable*/ false);
  return data;
}

/*
 * Write a byte to the EEPROM at the specified address. pulse LOW
 */
void writeEPROM0(int address, byte data) {
  digitalWrite(WRITE_EN,HIGH);
  digitalWrite(OUT_EN, LOW);

  setAddress(address, /*outputEnable*/ false); 
  setData( data );
  delayMicroseconds(4);
  digitalWrite(OUT_EN,HIGH);
  
  //Vpp pulse
  delayMicroseconds(4);
  digitalWrite(WRITE_EN,LOW);


  //for (int i = 0; i <5; i += 1)
      delayMicroseconds(10000);
/*
  for (int i = 0; i <3; i += 1)
 delayMicroseconds(9000);
 */
 digitalWrite(WRITE_EN,HIGH);
  delayMicroseconds(4);
  digitalWrite(OUT_EN,LOW);
}



/*
 * Write a byte to the EEPROM at the specified address. pulse HIGH
 */
void writeEPROM(int address, byte data) {
  digitalWrite(WRITE_EN,LOW);
  digitalWrite(OUT_EN, LOW);

  setAddress(address, /*outputEnable*/ false);
  setData( data );
  delayMicroseconds(4);
  digitalWrite(OUT_EN,HIGH);
  
  //Vpp pulse
  delayMicroseconds(4);
  digitalWrite(WRITE_EN,HIGH);

/*
  for (int i = 0; i <5; i += 1)
      delayMicroseconds(10000);
*/
delayMicroseconds(1000); 
 digitalWrite(WRITE_EN,LOW);
   delayMicroseconds(4);
  digitalWrite(WRITE_EN,HIGH);
  
  for (int i = 0; i <4; i += 1)
 delayMicroseconds(9000);
 
 digitalWrite(WRITE_EN,LOW);
  delayMicroseconds(4);
  digitalWrite(OUT_EN,LOW);
}



/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void printContents() {
    digitalWrite(WRITE_EN, LOW);   /*read */
  for (int sbit = 0; sbit < 8; sbit += 1) {
    pinMode(data_pin[sbit], INPUT);
  }
  for (int base = 0; base < BYTES; base += 16) {
    byte data[16];
    for (int offset = 0; offset <= 15; offset += 1) {
      data[offset] = readEPROM(base + offset);
    }

    char buf[80];
    sprintf(buf, "%04x:  %02x %02x %02x %02x %02x %02x %02x %02x   %02x %02x %02x %02x %02x %02x %02x %02x",
            base, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
            data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    Serial.println(buf);
  }
}

/*
 * Read the contents of the EPROM and compare with buffer.
 */
void compareContents() {
   char buf[80];
   digitalWrite(WRITE_EN, LOW);   /* read */
   Serial.println("Compare ...");
   Serial.println("ADDR  ROM BUF Result");
   Serial.println("-----------------------");
      
   for (int sbit = 0; sbit < 8; sbit += 1) {
    pinMode(data_pin[sbit], INPUT);
   }
   
   for (int base = 0; base < topAddr; base += 1) {
    byte rdata;
    int rt = 0;
     for (rt = 0; rt < 4; rt += 1) {
      rdata = readEPROM(base);
      if( rom[base] == rdata )
       rt= 4;
     }
      
     if( rom[base] == rdata )
       sprintf(buf, "%04x:  %02x %02x OK ", base, rdata, rom[base]);
     else
       sprintf(buf, "%04x:  %02x %02x ERROR ", base, rdata, rom[base]);

    Serial.println(buf);
   }
  Serial.println(".");
  Serial.println(" done");
}

/*
 * Read the contents of the EEPROM and print them to the serial monitor.
 */
void writeContents() {
    digitalWrite(WRITE_EN, HIGH);   /* write */
  for (int sbit = 0; sbit < 8; sbit += 1) {
    pinMode(data_pin[sbit], OUTPUT);
  }
   setData( 0 );
  setAddress(/*address*/ 0, /*outputEnable*/ false);
  delay(60);
  // Program data bytes
  Serial.print("Programming EPROM");
  for (int base = 0; base < topAddr ; base += 1) {
    writeEPROM(base, rom[base]);

    if (base % 64 == 0) {
      Serial.print(".");
    }
  }
  Serial.println(".");
  Serial.println(" done");

}


void setup() {
  memset(dat, 0, BYTES);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, LOW);
  // put your setup code here, to run once:
  pinMode(SHIFT_DATA, OUTPUT);
  pinMode(SHIFT_CLK, OUTPUT);
  pinMode(SHIFT_LATCH, OUTPUT);

  digitalWrite(WRITE_EN, LOW);
  pinMode(WRITE_EN, OUTPUT);

    digitalWrite(OUT_EN, LOW);
  pinMode(OUT_EN, OUTPUT);
  
  Serial.begin(57600);
  Serial.println("Ready!");


}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    if(state==0){
       writeContents();
       state +=1;
    }  
    else 
     if(state < 5)
      compareContents();
      else
      printContents();
    delay(300);
  }
}
