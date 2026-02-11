#include <Arduino.h>

#include "RAK3172.h"

RAK3172 rak;

void setup() {
  // put your setup code here, to run once:
	rak.init(UART_NUM_1, 39, 38); //Initiate RAK communication
	delay(100);

	rak.setRegion(LoraRegion_t::AU915); //Set LoRaWAN region to AU915
	delay(100);
	
	rak.setClass(LoraClass_t::CLASS_C); //Set LoRaWAN class to class C
	delay(100);
	rak.setMode(LoraMode_t::OTAA); //Set LoRaWAN mode to OTAA
	delay(100);
	rak.setSubBand(1);//Select the subband
	delay(100);

	
	rak.setDevEUI("0102030405060708"); //Configure the DEVEUI as 0x0102030405060708
	delay(100);
	rak.setAppKey("01020304050607080102030405060708"); //Configure the APPKEY as 0x01020304050607080102030405060708
	delay(100);

    //Configure the downlink callback
	rak.setDownlinkCallback([](const char* message) {
		DownlinkMessage data;
		data = rak.getLastDownlink();
		Serial.println(data.data); //Print the downlink message
	});

	rak.join(0, 8, 5);

  rak.send(1, "010203");
  }

void loop() {
  // put your main code here, to run repeatedly:
}
