
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


RF24 radio(8, 10);

const byte address[6] = "00001";

struct JoystickPacket {
  int16_t x;
  int16_t y;

};


JoystickPacket data;

int vrX = A0;
int vrY = A1;



void setup() {
  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();


  



}

void loop() {

  int rawX = analogRead(vrX);
  int rawY = analogRead(vrY);

  data.x = rawX - 512;
  data.y = rawY - 512;


  radio.write(&data, sizeof(data));
  delay(10);
  


}
