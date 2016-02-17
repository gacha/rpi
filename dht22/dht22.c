#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define DHT_PIN 7
#define DHT_TIMEOUT 200

/*
 DHT22 sensor example for RaspberryPi

 Download and install wiringPi - http://wiringpi.com/download-and-install/
 Check out manual - http://www.dfrobot.com/image/data/SEN0137/AM2302_manual.pdf

 Code inspired from:
  - https://github.com/sweetpi/rpi_dht22/blob/master/dht22.c
  - http://git.drogon.net/?p=wiringPi;a=blob;f=devLib/maxdetect.c
  - http://playground.arduino.cc/Main/DHTLib

  Author: Gatis Tomsons
*/

// It sleep's while the given pin is at given state
// and then return true if state changes or false if timeout is reached
bool sleepWhile(int pin, uint8_t state){
  uint32_t timeout = micros() + DHT_TIMEOUT;
  while(digitalRead(pin) == state){
    if(micros() > timeout)
      return false;
  }
  return true;
}

// read data from the sensor
void readData(){

    uint8_t data[5] = {0,0,0,0,0};

    // MCU says hello, see manual for bus communication timing
    pinMode(DHT_PIN, OUTPUT);
    digitalWrite(DHT_PIN, HIGH);
    delay(5);
    digitalWrite(DHT_PIN, LOW);
    delay(18);
    digitalWrite(DHT_PIN, HIGH);
    delayMicroseconds(30);
    pinMode(DHT_PIN, INPUT);
    if(!sleepWhile(DHT_PIN, HIGH)) return;

    // DHT responds with 80us LOW and 80us HIGH
    if(!sleepWhile(DHT_PIN, LOW)) return;
    if(!sleepWhile(DHT_PIN, HIGH)) return;

    // Read 40 bits of data
    for(int i=0; i<40; i++){

      // LOW 50us before each bit
      if(!sleepWhile(DHT_PIN, LOW)) return;
      uint32_t t = micros();
      // This is the real bit signal
      if(!sleepWhile(DHT_PIN, HIGH)) return;

      int diff = micros() - t;
      // move everything to the left, so basically we are adding a zero
      // to the current byte till all 8 are stored
      data[i / 8] <<= 1;
      // If HIGH for more than 68us then it's a HIGH bit
      if(diff >= 68){
        data[i / 8] |= 1; // turn on current bit
      }
    }

    // compares only 8 bits of the sum, because it could be more than that
    // 0xFF == b11111111 - so it kinda takes first 8 bits
    if(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)){
      float hum = ((float)data[0] * 256 + (float)data[1]) / 10.0;
      // takes only first 7 bits, because last bit is indicator for posetive/
      // negative value, 0x7F ==  b1111111
      float temp = ((float)(data[2] & 0x7F) * 256 + (float)data[3]) / 10.0;
      // check if bit 8 is HIGH, if yes then temp is negative
      // it works, I checked in freezer
      if(data[2] & (1 << 7)){
        temp *= -1;
      }

      printf("Humidity: %.1f %%\n", hum);
      printf("Temp: %.1f C\n\n", temp);
    }
}

int main(int argc, const char* argv[]){
  wiringPiSetup();
  do {
    readData();
    delay(3000);
  } while(true);
}
