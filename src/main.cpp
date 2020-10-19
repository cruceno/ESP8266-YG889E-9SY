/* https://www.cooking-hacks.com/documentation/tutorials/modbus-module-shield-tutorial-for-arduino-raspberry-pi-intel-galileo/
 *
 *  This sketch shows the use of the Modbus communication protocol over
 *  RS-232 standard, and the use of the main functions of the library.
 *. Modbus allows for communication between many devices connected
 *  to the same network. There are many variants of Modbus protocols,
 *  but Arduino implements the RTU format. Modbus RTU is the most
 *  common implementation available for Modbus.
 *
 *  This example shows the use of the function readCoils. This function
 *  requests the ON/OFF status of discrete coils from the slave device.
 *
 *  Copyright (C) 2014 Libelium Comunicaciones Distribuidas S.L.
 *  http://www.libelium.com
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Version:          0.1
 *  Implementation:   Ahmad Saad
 *  
 *  
 *  Modified by PDAControl
 *   Settings for libraries for ESP8266
 *  - Function  crc16 
 *  - Function  makeWord
 *  More info      : http://pdacontrolen.com
 *  Mas informacion: http://pdacontroles.com
 *  Channel  Youtube https://www.youtube.com/c/JhonValenciaPDAcontrol/videos   
 *
 *   
 *  Modified by Trialcommand
 *   More Tutorials: 
 *   Website http://trialcommand.com
 *	 In English: http://en.trialcommand.com
 *	 En Español: http://en.trialcommand.com  
 *   
 */
 #include <ESP8266WiFi.h>
#include <ModbusMaster232.h> 
#include <SoftwareSerial.h>  // Modbus RTU pins   D7(13),D8(15)   RX,TX

// Instantiate ModbusMaster object as slave ID 1
  ModbusMaster232 node(1, 2);

void setup()
{

  Serial.begin(9600);
  delay(100);  
  node.begin(9600);  // Modbus RTU
  delay(100);
  Serial.println("Connected "); 
  Serial.println("Modbus RTU Master Online");

  
}


void loop()
{
//Website http://trialcommand.com
///////// Holding Register [0]  A [9]   = 10 Holding Registers Escritura
///////// Holding Register [0] A [9] = 10 Holding Registers Writing
uint8_t result;
int Mdelay = 5; 

result = node.readHoldingRegisters(14, 1); 
Serial.printf_P("Resgistro [14], Resultado: %d\n", result);
Serial.print(node.getResponseBuffer(0));
node.clearResponseBuffer();
delay(Mdelay);

 

}