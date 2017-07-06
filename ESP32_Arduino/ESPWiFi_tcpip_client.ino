/*
  WiFi TCPIP Client To Serial
*/
#include <WiFi.h>
#include <WiFiMulti.h>
#include <SPI.h>

WiFiMulti wifiMulti;

int status = WL_IDLE_STATUS;
IPAddress server(192,168,0,186);  // Crazyflie-ROS VM

WiFiClient client;

const char* ssid = "FreeWifi";
const char* password = "748748748";

HardwareSerial Serial1(2);  // UART1/Serial1 pins 16,17

void setup() {
  Serial.begin(115200);
  Serial.println("\nConnecting");

  wifiMulti.addAP(ssid, password);
  wifiMulti.addAP("ssid_from_AP_2", "your_password_for_AP_2");
  wifiMulti.addAP("ssid_from_AP_3", "your_password_for_AP_3");

  Serial.println("Connecting Wifi ");
  for (int loops = 10; loops > 0; loops--) {
    if (wifiMulti.run() == WL_CONNECTED) {
      Serial.println("");
      Serial.print("WiFi connected ");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      break;
    }
    else {
      Serial.println(loops);
      delay(1000);
    }
  }
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi connect failed");
    delay(1000);
    ESP.restart();
  }

  //start UART and the server
  Serial1.begin(115200);
  // if you get a connection, report back via serial:
  Serial.println("Connecting to server");
  if (client.connect(server, 8888)) {
    Serial.println("Connected to server");
  }
}

void loop() {
  uint8_t i;
  if (wifiMulti.run() == WL_CONNECTED) {
    //check server for data
    if (client.connected()){
//        size_t startlen = client.available();
//        uint8_t startbuf[startlen];
//        client.readBytes(startbuf, startlen); //start-byte
//        size_t datalen = client.available();
//        uint8_t databuf[datalen];
//        client.readBytes(databuf, datalen); //data
//        size_t checklen = client.available();
//        uint8_t checkbuf[checklen];
//        client.readBytes(checkbuf, checklen); //checksum
//        Serial.println("Data received from Crazyflie-ROS");
//        Serial1.write(startbuf, startlen);
//        Serial1.write(databuf, datalen);
//        Serial1.write(checkbuf, checklen);
//        Serial.println("Data written to serial 1"); 
        
        size_t len = client.available();
        uint8_t sbuf[len];
        client.readBytes(sbuf, len); //start-byte
        Serial.println("Data received from Crazyflie-ROS");
        Serial1.write(sbuf, len);
        Serial.println("Data written to serial 1");

  
        //check UART for data

        len = Serial1.available();
        if(len){
            //Serial.println("Receiving data");
                      
            uint8_t sbuf[len];
            //Serial.println("Sending data");
            Serial1.readBytes(sbuf, len);  
            Serial.printf("Data received from serial 1: %d\n",len);      
            client.write(sbuf,len);
            delay(1);
            Serial.printf("Data sent to Crazyflie-ROS: %d\n",sbuf);

       }
      else {
            //Serial.println("Stopping client");       
            //client.stop();
      }
    }
  }
}


//    //check UART for data
//    if(Serial1.available()){
//      size_t len = Serial1.available();
//      uint8_t sbuf[len];
//      Serial1.readBytes(sbuf, len);
//      //push UART data to all connected telnet clients
//      for(i = 0; i < MAX_SRV_CLIENTS; i++){
//        if (serverClients[i] && serverClients[i].connected()){
//          serverClients[i].write(sbuf, len);
//          delay(1);
//        }
//      }
//      //print the pushed UART data
//      Serial.printf("Data sent: %d",sbuf);
//  }
//  else {
//    Serial.println("WiFi not connected!");
//    for(i = 0; i < MAX_SRV_CLIENTS; i++) {
//      if (serverClients[i]) serverClients[i].stop();
//    }
//    delay(1000);
//  }
//}
