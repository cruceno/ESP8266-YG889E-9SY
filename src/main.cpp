/*
MQTT  Topics
Control de delay              H7FOLHi8GcHSiTF/delay
Tension de fase A             H7FOLHi8GcHSiTF/TTxrFaLgyyoXtFF
Tension de fase B             H7FOLHi8GcHSiTF/lCM8NXj3n2nGQQd
Tension de fase C             H7FOLHi8GcHSiTF/9hVYufhgmw6KRnk
Tension de linea AB           H7FOLHi8GcHSiTF/DZod8mXMHuRyODi
Tension de linea BC           H7FOLHi8GcHSiTF/iwoJ7cQ9ooSWgsH
Tension de linea CA           H7FOLHi8GcHSiTF/WpiH57eIXTKAp71
Corriente de fase A           H7FOLHi8GcHSiTF/QBYTumaIba4Y3fc
Corriente de fase B           H7FOLHi8GcHSiTF/e4gCp0slDCZN8Ku
Corriente de fase C           H7FOLHi8GcHSiTF/xzUH3usAPF1FhuJ
Potencia activa fase A        H7FOLHi8GcHSiTF/dOTlwXRRU3ujCPt
Potencia activa fase B        H7FOLHi8GcHSiTF/X6UBIF1uWQbnG44
Potencia activa fase C        H7FOLHi8GcHSiTF/bN7Bqz8PaMnaoJN
Potencia activa total         H7FOLHi8GcHSiTF/Hd8AIKXXzGWupBs
Factor de potencia fase A     H7FOLHi8GcHSiTF/r6dZcZ8uS7D3O2C
Factor de potencia fase B     H7FOLHi8GcHSiTF/K5OXvutli45BJqW
Factor de potencia fase C     H7FOLHi8GcHSiTF/07W3ILLfrSTM3Mc
Factor de potencia total      H7FOLHi8GcHSiTF/xiwbHigeDIwDY05 
Potencia reactiva fase A      H7FOLHi8GcHSiTF/1nFF53RvOntt1S4 
Potencia reactiva fase B      H7FOLHi8GcHSiTF/ZgxqjdjqXzlwyYx 
Potencia reactiva fase C      H7FOLHi8GcHSiTF/iyKK28s7OpxArIK 
Potencia reactiva total       H7FOLHi8GcHSiTF/tiiKHVQdTPUuHAK 
Potencia aparente fase A      H7FOLHi8GcHSiTF/922XhH8DaFkbvEa 
Potencia aparente fase B      H7FOLHi8GcHSiTF/3dIng3XhS4ap3IP 
Potencia aparente fase C      H7FOLHi8GcHSiTF/KKrsJsAOanq7YvQ 
Potencia aparente fase total	H7FOLHi8GcHSiTF/gpjLYjBj2I9S7im
Frecuencia de linea           H7FOLHi8GcHSiTF/Lh37OZgKxub3izR

*/

#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>

#include "FS.h" // SPIFFS is declared
//#include "LittleFS.h" // LittleFS is declared
#include <Ticker.h>
#include <AsyncMqttClient.h>

#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <ArduinoJson.h>  
#include <ESPAsyncWiFiManager.h>  

// TODO: Modificar segun dispositivo agregando los topics que sean necesarios
char  mqtt_server[50]; 
char  mqtt_port[5];
char  control_topic[20];

char  log_topic[21] ;
//TODO: Aadd sensor topocs here
char  Ua1_t[16], Ub1_t[16], Uc1_t[16],
      Ia1_t[16], Ib1_t[16], Ic1_t[16],
      PFa1_t[16], PFb1_t[16], PFc1_t[16],
      Ua2_t[16], Ub2_t[16], Uc2_t[16],
      Ia2_t[16], Ib2_t[16], Ic2_t[16],
      PFa2_t[16], PFb2_t[16], PFc2_t[16];

// Ip settings 
char use_static[2] = "0";
char static_ip[16] = "";
char static_gw[16] = "";
char static_sn[16] = "";
char static_dns1[16] = "";
char static_dns2[16] = "";
unsigned int interval = 60;
unsigned long live_since =0, check_wifi = 60;
float value;

/*ESP AsyncWebSever config*/
  AsyncWebServer server(80);
  DNSServer dns;

/*  ESP Async Wifi Manager Config */
  AsyncWiFiManager wifiManager(&server,&dns);
  //flag for saving data
  bool shouldSaveConfig = false;

  //callback notifying us of the need to save config
  void saveConfigCallback () {
   // Serial.println("Should save config");
    shouldSaveConfig = true;
  }

  void configModeCallback (AsyncWiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    //if you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());
  }

  /* FileSystem config  */
  void loadConfigFromFS(){
    // TODO: Modificar segun dispositivo agregando los topics que sean necesarios

    //read configuration from FS json
    Serial.println("mounting FS...");
    if (SPIFFS.begin()) {
      Serial.println("mounted file system");
      if (SPIFFS.exists("/config.json")) {
        //file exists, reading and loading
        Serial.println("reading config file");
        File configFile = SPIFFS.open("/config.json", "r");
        if (configFile) {
          Serial.println("opened config file");

          const size_t capacity = JSON_OBJECT_SIZE(29)+ 1080;
          DynamicJsonDocument doc(capacity);
          DeserializationError error = deserializeJson(doc, configFile);
            // Test if parsing succeeds.
          if (error) {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println("failed to load json config");
            Serial.println(error.f_str());
          }
          else{
            serializeJsonPretty(doc, Serial);
            Serial.println();
            Serial.println("----------------------------------");
            strcpy(mqtt_server, doc["mqtt_server"]);
            Serial.println(mqtt_server);
            strcpy(mqtt_port, doc["mqtt_port"]);
            Serial.println(mqtt_port);
            strcpy(control_topic, doc["device_topic"]);
            Serial.println(control_topic);
            // Aadd sensor topics here
            
                strcpy(Ua1_t,  doc["device1-ua"]    );
                Serial.println(Ua1_t);
                strcpy(Ub1_t,  doc["device1-ub"]    );
                Serial.println(Ub1_t);
                strcpy(Uc1_t,  doc["device1-uc"]    );
                Serial.println(Uc1_t);
                strcpy(Ia1_t,  doc["device1-ia"]    );
                Serial.println(Ia1_t);
                strcpy(Ib1_t,  doc["device1-ib"]    );
                Serial.println(Ib1_t);
                strcpy(Ic1_t,  doc["device1-ic"]    );
                Serial.println(Ic1_t);
                strcpy(PFa1_t, doc["device1-pfa"]   );
                Serial.println(PFa1_t);
                strcpy(PFb1_t, doc["device1-pfb"]   );
                Serial.println(PFb1_t);
                strcpy(PFc1_t, doc["device1-pfc"]   );
                Serial.println(PFc1_t);
                strcpy(Ua2_t,  doc["device2-ua"] );
                Serial.println(Ua2_t);
                strcpy(Ub2_t,  doc["device2-ub"] );
                Serial.println(Ub2_t);
                strcpy(Uc2_t,  doc["device2-uc"] );
                Serial.println(Uc2_t);
                strcpy(Ia2_t,  doc["device2-ia"] );
                Serial.println(Ia2_t);
                strcpy(Ib2_t,  doc["device2-ib"] );
                Serial.println(Ib2_t);
                strcpy(Ic2_t,  doc["device2-ic"] );
                Serial.println(Ic2_t);
                strcpy(PFa2_t, doc["device2-pfa"]);
                Serial.println(PFa2_t);
                strcpy(PFb2_t, doc["device2-pfb"]);
                Serial.println(PFb2_t);
                strcpy(PFc2_t, doc["device2-pfc"]);
                Serial.println(PFc2_t);
            //strcpy(temperature_topic, doc["temperature_topic"]);

            //--------------------------
            interval = doc["delay"];
            Serial.println(interval);

            strcpy(static_ip, doc["static_ip"]);
            Serial.println(static_ip);
            strcpy(static_gw, doc["static_gw"]);
            Serial.println(static_gw);
            strcpy(static_sn, doc["static_sn"]);
            Serial.println(static_sn);
            strcpy(static_dns1, doc["static_dns1"]);
            Serial.println(static_dns1);
            strcpy(static_dns2, doc["static_dns2"]);
            Serial.println(static_dns2);

            Serial.println("----------------------------------");
          } 

        }
        configFile.close();
      }
    }
    else {
      Serial.println("failed to mount FS");
    }
  }

  void saveConfigToFS(){
    // TODO: Modificar segun dispositivo agregando los topics que sean necesarios

    if (SPIFFS.begin()){
      //Serial.println("saving config");
      const size_t capacity = JSON_OBJECT_SIZE(29)+ 1080;
      DynamicJsonDocument doc(capacity);

      doc["mqtt_server"] = mqtt_server;
      doc["mqtt_port"] = mqtt_port;
      doc["device_topic"] = control_topic;
      //TODO: Aadd sensor topics here

      //doc["temperature_topic"] = temperature_topic;
      doc["device1-ua"]    = Ua1_t; 
      doc["device1-ub"]    = Ub1_t; 
      doc["device1-uc"]    = Uc1_t; 
      doc["device1-ia"]    = Ia1_t; 
      doc["device1-ib"]    = Ib1_t; 
      doc["device1-ic"]    = Ic1_t; 
      doc["device1-pfa"]   = PFa1_t;
      doc["device1-pfb"]   = PFb1_t;
      doc["device1-pfc"]   = PFc1_t;

      doc["device2-ua"] = Ua2_t; 
      doc["device2-ub"] = Ub2_t; 
      doc["device2-uc"] = Uc2_t; 
      doc["device2-ia"] = Ia2_t; 
      doc["device2-ib"] = Ib2_t; 
      doc["device2-ic"] = Ic2_t; 
      doc["device2-pfa"]= PFa2_t;
      doc["device2-pfb"]= PFb2_t;
      doc["device2-pfc"]= PFc2_t;

      //------------------------
      doc["delay"] = interval;
      doc["static_ip"] = static_ip;
      doc["static_gw"] = static_gw;
      doc["static_sn"] = static_sn;
      doc["static_dns1"] = static_dns1;
      doc["static_dns2"] = static_dns2;

      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
      serializeJsonPretty(doc, Serial);
      serializeJsonPretty(doc, configFile);
      configFile.close();
      Serial.println("\n--------------------------------");
      Serial.println("|Update configuration complete.|");
      Serial.println("--------------------------------");
    }
  }


AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;
/*  MQTT Payload config */
  #define MSG_BUFFER_SIZE	(10)

  #define LOG_BUFFER_SIZE (250)
  char msg[MSG_BUFFER_SIZE];
  char log_msg[LOG_BUFFER_SIZE];
  unsigned long lastMsg = 0;

 /*
 TODO: General function to submit system logs
void publish_log(char *payload){
    snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \"%s\"}", payload);
    mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);
    strcpy(log_msg, "");
}
*/
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi function ... ");
  // TODO: WIfi AP name and password from config file
  if (!wifiManager.autoConnect("ESP866-MODBUS", "123456789")) {
    Serial.println("Fallo al conectar al Wifi y timeout alcanzado");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.restart();
  }
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
 /* Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  Serial.println(control_topic);
  //TODO: Aadd sensor topics here
  Serial.println(temperature_topic);*/
  mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  delay(1000);
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  mqttClient.subscribe(control_topic, 0);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  
  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("Payload:");
  Serial.println(payload);

  const size_t capacity = JSON_OBJECT_SIZE(5) + 196 ;
  DynamicJsonDocument doc(capacity);
  DeserializationError error = deserializeJson(doc, payload);

  if (error){  
     snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \" %s - BAD JSON  \"}", payload);
     mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);

  }
  else{
    // Compando para settear el delay solo si es mayor a 5 segundos
    int delay_value = doc["delay"];

    if (delay_value && delay_value > 5) {
      Serial.println("seteando nuevo delay");
      interval = delay_value;
      saveConfigToFS();
    }
    else if(delay_value){
      // Si es menor a 5 segundos no se actualiza y se manda un log de valo invalido
      snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \" %s - INVALID VALUE \"}", payload);
      mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);


    }
    else{
      snprintf(log_msg, LOG_BUFFER_SIZE, "{\"log\": \" %s - INVALID CMD  \"}", payload);
      mqttClient.publish(log_topic, 0, false, log_msg, LOG_BUFFER_SIZE);

    }
  }
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/
#define MAX485_DE      4
#define MAX485_RE_NEG  4

// instantiate ModbusMaster object
ModbusMaster device_1, device_2;
SoftwareSerial swSer;

void preTransmission()
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission()
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void read_and_publish_sensors(){
    /* Toma de lecturas y envio de datos*/
  uint8_t result;
  //uint16_t data[7];
  Serial.println("Publish message device 1: ");
  // Read 16 registers starting at 0x3100)
  result = device_1.readInputRegisters(0x25, 26);

  if (result == device_1.ku8MBSuccess)
  {
   // # Voltage de fases
   //'Ua': 0x25,  #0x00
    value = device_1.getResponseBuffer(0x00)/10.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
    Serial.print(Ua1_t);Serial.print(": ");
    Serial.println(msg);
    mqttClient.publish(Ua1_t, 0, false, msg, MSG_BUFFER_SIZE);
   //'Ub': 0x26,  #0x01
    value = device_1.getResponseBuffer(0x01)/10.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.print(Ub1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(Ub1_t, 0, false, msg, MSG_BUFFER_SIZE);
   //'Uc': 0x27,  #0x02 
    value = device_1.getResponseBuffer(0x02)/10.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.print(Uc1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(Uc1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // # Tension de linea
   // 'Uab': 0x28, #0x03
   // 'Ubc': 0x29, #0x04
   // 'Uca': 0x2A, #0x05
   // # Corrientes de fase
   // 'Ia': 0x2B, #0x06
    value = device_1.getResponseBuffer(0x06)/100.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.print(Ia1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(Ia1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'Ib': 0x2C, #0x07
    value = device_1.getResponseBuffer(0x07)/100.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.print(Ib1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(Ib1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'Ic': 0x2D, #0x08
    value = device_1.getResponseBuffer(0x08)/100.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.print(Ic1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(Ic1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // # Potencia ctiva
   // 'Pa': 0x2E, #0x09
   // 'Pb': 0x2F, #0x0A
   // 'Pc': 0x30, #0x0B
   // 'Ps': 0x31, #0x0C
   // # Reactiva
   // 'Qa': 0x32, #0x0D
   // 'Qb': 0x33, #0x0E
   // 'Qc': 0x34, #0x0F
   // 'Qs': 0x35, #0x10  
   // # Factor de potencia
   // 'PFa': 0x36, #0x11
    value = device_1.getResponseBuffer(0x11)/1000.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.print(PFa1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(PFa1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'PFb': 0x37, #0x12
    value = device_1.getResponseBuffer(0x12)/1000.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.print(PFb1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(PFb1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'PFc': 0x38, #0x13
    value = device_1.getResponseBuffer(0x13)/1000.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.print(PFc1_t);Serial.print(": ");
        Serial.println(msg);

    mqttClient.publish(PFc1_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'PFs': 0x39, #0x14
   //  # Potencia aparente
   //  'Sa': 0x3A, #0x15
   //  'Sb': 0x3B, #0x16
   //  'Sc': 0x3C, #0x17
   //  'Ss': 0x3D, #0x18
   //  #Frecuencia
   //  'F': 0x3E #0x19
   //  */ 
  }
  else{
    Serial.println("Error al conectar con el dispositivo 1");
  }
  Serial.println("Publish message device 2: ");
  delay(2000);
  result = device_2.readInputRegisters(0x25, 26);
  
  if (result == device_2.ku8MBSuccess)
  {
   // # Voltage de fases
   //'Ua': 0x25,  #0x00    value = device_1.getResponseBuffer(0x02)/100.0f;

    value = device_2.getResponseBuffer(0x00)/10.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.println(msg);

    mqttClient.publish(Ua2_t, 0, false, msg, MSG_BUFFER_SIZE);
   //'Ub': 0x26,  #0x01
    value = device_2.getResponseBuffer(0x01)/10.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.println(msg);

    mqttClient.publish(Ub2_t, 0, false, msg, MSG_BUFFER_SIZE);
   //'Uc': 0x27,  #0x02 
    value = device_2.getResponseBuffer(0x02)/10.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.2f", value);
        Serial.println(msg);

    mqttClient.publish(Uc2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // # Tension de linea
   // 'Uab': 0x28, #0x03
   // 'Ubc': 0x29, #0x04
   // 'Uca': 0x2A, #0x05
   // # Corrientes de fase
   // 'Ia': 0x2B, #0x06
    value = device_2.getResponseBuffer(0x06)/100.0f;
    snprintf (msg, 10, "%.2f", value);
        Serial.println(msg);

    mqttClient.publish(Ia2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'Ib': 0x2C, #0x07
    value = device_2.getResponseBuffer(0x07)/100.0f;
    snprintf (msg, 10, "%.2f", value);
        Serial.println(msg);
    mqttClient.publish(Ib2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'Ic': 0x2D, #0x08
    value = device_2.getResponseBuffer(0x08)/100.0f;
    snprintf (msg, 10, "%.2f", value);
        Serial.println(msg);

    mqttClient.publish(Ic2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // # Potencia activa
   // 'Pa': 0x2E, #0x09
   // 'Pb': 0x2F, #0x0A
   // 'Pc': 0x30, #0x0B
   // 'Ps': 0x31, #0x0C
   // # Reactiva
   // 'Qa': 0x32, #0x0D
   // 'Qb': 0x33, #0x0E
   // 'Qc': 0x34, #0x0F
   // 'Qs': 0x35, #0x10  
   // # Factor de potencia
   // 'PFa': 0x36, #0x11
    value = device_2.getResponseBuffer(0x11)/1000.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.println(msg);

    mqttClient.publish(PFa2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'PFb': 0x37, #0x12
    value = device_2.getResponseBuffer(0x12)/1000.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.println(msg);

    mqttClient.publish(PFb2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'PFc': 0x38, #0x13
    value = device_2.getResponseBuffer(0x13)/1000.0f;
    snprintf (msg, MSG_BUFFER_SIZE, "%.3f", value);
        Serial.println(msg);

    mqttClient.publish(PFc2_t, 0, false, msg, MSG_BUFFER_SIZE);
   // 'PFs': 0x39, #0x14
   //  # Potencia aparente
   //  'Sa': 0x3A, #0x15
   //  'Sb': 0x3B, #0x16
   //  'Sc': 0x3C, #0x17
   //  'Ss': 0x3D, #0x18
   //  #Frecuencia
   //  'F': 0x3E #0x19
   //  */ 
  }
  else {
    Serial.println("Error al comunicarse con el dispositivo 2");
  }
}
void setup()
{
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  Serial.begin(115200);
  swSer.begin(9600, SWSERIAL_8N1, 14, 12, false, 256, 11);

  // Modbus slave ID 1
  device_1.begin(1, swSer);
  device_2.begin(2, swSer);

  // Cargando archivo de configuracion
  loadConfigFromFS();
  delay(1000);

  
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

  AsyncWiFiManagerParameter config_mqtt_server("server", "MQTT server", mqtt_server, 40);
  AsyncWiFiManagerParameter config_mqtt_port("port", "MQTT port", mqtt_port, 5);
  AsyncWiFiManagerParameter config_control_topic("control_topic", "Topic de Control", control_topic, 20);
  //TODO: Add sensor topics here

  //AsyncWiFiManagerParameter config_temperature_topic("temperature_topic", "Topic Temperatura", temperature_topic, 16);
  

  AsyncWiFiManagerParameter config_use_static("use_static", "Usar Ip estatica", use_static, 2);

  AsyncWiFiManagerParameter config_static_ip("ip", "IP del dispositivo", static_ip, 16);
  AsyncWiFiManagerParameter config_static_gw("gw", "Puesta de enlace", static_gw, 16);
  AsyncWiFiManagerParameter config_static_sn("sn", "Mascara de red", static_sn, 16);
  AsyncWiFiManagerParameter config_static_dns1("dns1", "Servidor DNS 1", static_dns1, 16);
  AsyncWiFiManagerParameter config_static_dns2("dns2", "servidor DNS 2", static_dns2, 16);

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setDebugOutput(false);
  //add all your parameters here
  wifiManager.addParameter(&config_mqtt_server);
  wifiManager.addParameter(&config_mqtt_port);
  wifiManager.addParameter(&config_control_topic);
  // TODO: Add sensor topics here
  //wifiManager.addParameter(&config_temperature_topic);

  wifiManager.addParameter(&config_static_ip);
  wifiManager.addParameter(&config_static_gw);
  wifiManager.addParameter(&config_static_sn);
  wifiManager.addParameter(&config_static_dns1);
  wifiManager.addParameter(&config_static_dns2);

  //set static ip or dhcp
   if (!*static_ip) {

      Serial.println("Usar DHCP");
    }

    else {
       //set static ip
      //the commented bit only works for ESP8266 core 2.1.0 or newer
      IPAddress _ip,_gw,_sn, _dns1, _dns2;
      _ip.fromString(static_ip);
      _gw.fromString(static_gw);
      _sn.fromString(static_sn);
      _dns1.fromString(static_dns1);
      _dns2.fromString(static_dns2);
      wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn, _dns1, _dns2);
      Serial.println("Usar IP Estatica");
      
    }
  //

  wifiManager.setTimeout(120);

  Serial.println("Set Up MQTT Service");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer( mqtt_server, atoi(mqtt_port));
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  // Callbacks allow us to configure the RS485 transceiver correctly
   connectToWifi();
    
  
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  //if you get here you have connected to the WiFi
  //Serial.println("connected...yeey :)");

  //save the custom parameters to FS
  if (shouldSaveConfig) {    
    Serial.println("\n---------------------");
    Serial.println("| Saving new config |\n---------------------\n");

    //read updated parameters
    strcpy(mqtt_server, config_mqtt_server.getValue());
    strcpy(mqtt_port, config_mqtt_port.getValue());
    strcpy(control_topic, config_control_topic.getValue());


    // TODO: Add sensor topics here
    //strcpy(temperature_topic, config_temperature_topic.getValue());


    //------------------------------------------


    strcpy(static_ip, config_static_ip.getValue());
    strcpy(static_gw, config_static_gw.getValue());
    strcpy(static_sn, config_static_sn.getValue());
    strcpy(static_dns1, config_static_dns1.getValue());
    strcpy(static_dns2, config_static_dns2.getValue());
    
    saveConfigToFS();    //end save
    shouldSaveConfig=false;
    ESP.restart();
  }

  strcpy(log_topic, control_topic);
  strcat(log_topic, "/log");

  Serial.print("\nlocal ip: ");
  Serial.println(WiFi.localIP());
  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    //TODO: index page for control and info
    request->send(200, "text/plain", "Yo soy el medidor de tension.");
  });
  server.on("/factoryReset", HTTP_GET, [](AsyncWebServerRequest *request) {
    //TODO: index page for control and info
    wifiManager.resetSettings();
    ESP.restart();

  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();

  //publish_log("Equipo iniciado con exito !");
  
  device_1.preTransmission(preTransmission);
  device_1.postTransmission(postTransmission);
  device_2.preTransmission(preTransmission);
  device_2.postTransmission(postTransmission);
}

bool state = true;

void loop()
{

  AsyncElegantOTA.loop();
  unsigned long now = millis();
  if (now - lastMsg > interval * 1000) {
      lastMsg = now; 
      read_and_publish_sensors();
  }

  if (now - live_since > interval * 1000 * 10) {
      ESP.restart();
  }
  
}