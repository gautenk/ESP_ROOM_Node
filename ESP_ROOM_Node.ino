//	ESP Roomsensor V1.1
//
//	This MQTT client will connect over Wifi to the MQTT broker and controls a digital output (LED, relay):
//	- toggle output and send status message on local button press
//	- receive messages from the MQTT broker to control output, change settings and query state
//	- periodically send status messages
//
//	Several nodes can operate within the same network; each node has a unique node ID.
//	On startup the node operates with default values, as set during compilation.
//	Hardware used is a ESP8266 WiFi module that connects directly to the MQTT broker.
//
//	Message structure is equal to the RFM69-based gateway/node sytem by the same author.
//	This means both type of gateway/nodes can be used in a single Openhab system.
//
//	The MQTT topic is home/esp_gw/direction/nodeid/devid
//	where direction is "sb" towards the node and "nb" towards the MQTT broker.
//  A will is published in topic home/esp_gw/disconnected to indicate disconnection.
//
//	Defined devices are:
//	0	uptime:		read uptime in minutes
//	1	interval:	read/set transmission interval for push messages
//	3	version:	read software version
//	2	RSSI:		read radio signal strength
//	5	ACK:		read/set acknowledge message after a 'set' request
//	10	IP:			Read IP address
//  32  Red LED:      read/set red LED
//  33  Green LED:    read/set green LED
//  34  Blue LED:     read/set blue LED
//  35  RGB LED:      set-only rgb LED. read will send 32,33,34
//  40  Movement:     read PIR
//  41  REED Switch:  read REEDSWITCH
//  42  Water sensor: read Watersensor
//  48  temperature:  read temperature
//  49  humidity:     read humidity
//  51  OW-temp     read onewire temperature values
//  64  Light     read ambient light
//	92	error:		tx only: device not supported
//	91	error:		tx only: syntax error
//	99	wakeup:		tx only: first message sent on node startup
//
//	version 1.0 by computourist@gmail.com May 2016
//	version 1.1 May 2016
//		- some minor bug fixes and code improvements
//	version 1.2 June 2016
//		- code improvements bij Gaute Korsnes: send IP address & RSSI
//
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#define VERSION "ESP Roomsensor V1.1"						// this value can be queried as device 3
#define wifi_ssid "xxx"						// wifi station name
#define wifi_password "xxxxxxxx"				// wifi password
#define mqtt_server "192.168.xxx.xxx"			// mqtt server IP
#define nodeId 1								// node ID
//#define DEBUG
#define RANDOM //enable to make the chip to wait 0-30 sec before startup. useful if you have multiple sensors running on same powersupply.

//Enable sensors
#define REEDENABLE
#define WATERENABLE
#define OWENABLE

//	sensor setting

#define LEDRPIN 12
#define LEDGPIN 13
#define LEDBPIN 14
#define PIRPIN 15
#define DHTPIN 4
#define DHTTYPE DHT22         // type of sensor
#define DHTTEMPCORRECTION 0
#define OWPIN 5
#define LIGHTPIN A0
#define REEDPIN 2
#define WATERPIN 16
#define SERIAL_BAUD 115200
#define HOLDOFF 1000							// blocking period between triggered messages

//	STARTUP DEFAULTS

long 	TXinterval = 60;						// periodic transmission interval in seconds

//	VARIABLES

int		DID;									// Device ID
int		error;									// Syntax error code
int LEDRState;
int LEDGState;
int LEDBState;
int LightState;          // temperature
int   signalStrength;             // radio signal strength
long LEDState;
long  lastPeriod = -1;            // timestamp last transmission
long  lastMinute = -1;            // timestamp last minute
long  lastPIRPress = -1;        // timestamp last PIRCHANGE
long  upTime = 0;               // uptime in minutes
float hum, temp;          // humidity, temperature
bool  ackPIR = true;          // flag for message on PIR trigger
bool  curPIR = true;        // current PIR state
bool  lastPIR = true;       // last PIR state
bool  mqttCon = false;            // MQTT broker connection flag
bool  setAck = true;          // send ACK message on 'SET' request
bool	wakeUp = true;							// wakeup indicator
bool	msgBlock = false;						// flag to hold button message
bool	readAction;								// indicates read / set a value
bool	send0, send1, send2, send3, send5, send6, send7;
bool  send32, send33, send34, send40, send48, send49;
bool	send10, send99, send64;
String	IP;										// IPaddress of ESP
char	buff_topic[30];							// mqtt topic
char	buff_msg[32];							// mqtt message
char  clientName[10];             // Mqtt client name

#ifdef REEDENABLE
bool  ackREED = true;          // flag for message on PIR trigger
bool  send41;
bool  curREED = true;        // current PIR state
bool  lastREED = true;       // last PIR state
long  lastREEDPress = -1;        // timestamp last PIRCHANGE
#endif

#ifdef WATERENABLE
bool  ackWATER = true;          // flag for message on PIR trigger
bool  send42;
bool  curWATER = true;        // current PIR state
bool  lastWATER = true;       // last PIR state
long  lastWATERPress = -1;        // timestamp last PIRCHANGE
#endif

#ifdef OWENABLE
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(OWPIN);
DallasTemperature sensors(&oneWire);
float  owTemp;          // temperature
bool  send51;
#endif


void mqttSubs(char* topic, byte* payload, unsigned int length);

WiFiClient espClient;
PubSubClient client(mqtt_server, 1883, mqttSubs, espClient); // instantiate MQTT client
DHT dht(DHTPIN, DHTTYPE, 3);      // initialise temp/humidity sensor for 3.3 Volt arduino
	
//	FUNCTIONS

//===============================================================================================

void pubMQTT(String topic, String topic_val){ // publish MQTT message to broker
  Serial.print("topic " + topic + " value:");
	Serial.println(String(topic_val).c_str());
	client.publish(topic.c_str(), String(topic_val).c_str(), false);
	}

void mqttSubs(char* topic, byte* payload, unsigned int length) {	// receive and handle MQTT messages
	int i;
	error = 4; 										// assume invalid device until proven otherwise
	Serial.print("Message arrived [");
	Serial.print(topic);
	Serial.print("] ");
	for (int i = 0; i < length; i++) {
		Serial.print((char)payload[i]);
		}
	Serial.println();
	if (strlen(topic) == 27) {						// correct topic length ?
		DID = (topic[25]-'0')*10 + topic[26]-'0';	// extract device ID from MQTT topic
		payload[length] = '\0';						// terminate string with '0'
		String strPayload = String((char*)payload);	// convert to string
	readAction = (strPayload == "READ");			// 'READ' or 'SET' value
	if (length == 0) {error = 2;}					// no payload sent
	else {
		if (DID ==0) {								// uptime 
			if (readAction) {
				send0 = true;
				error = 0;
			} else error = 3;						// invalid payload; do not process
		}
		if (DID==1) {								// transmission interval
			error = 0;
			if (readAction) {
				send1 = true;
			} else {								// set transmission interval
				TXinterval = strPayload.toInt();
				if (TXinterval <10 && TXinterval !=0) TXinterval = 10;	// minimum interval is 10 seconds
			}
		}
		if (DID ==2) {								// RSSI
			if (readAction) {
				send2 = true;
				error = 0;
			} else error = 3;						// invalid payload; do not process
		}
		if (DID==3) {								// version
			if (readAction) {
				send3 = true;
				error = 0;
			} else error = 3;						// invalid payload; do not process
		}
		if (DID==5) {								// ACK
			if (readAction) {
				send5 = true;
				error = 0;
			} else if (strPayload == "ON") {
					setAck = true;
					if (setAck) send5 = true;
					error = 0;
			}	else if (strPayload == "OFF") {
					setAck = false;
					if (setAck) send5 = true;
					error = 0;
			}	else error = 3;
		}
		if (DID ==10) {								// IP address 
			if (readAction) {
				send10 = true;
				error = 0;
			} else error = 3;						// invalid payload; do not process
		}
		if (DID==32) {                // transmission interval
      if (readAction) {
        send32 = true;
        error = 0;
      } else {                // set transmission interval
        LEDRState = strPayload.toInt();
        analogWrite(LEDRPIN,LEDRState);
        if (setAck) send32 = true;
        Serial.print("Set Red LED State to: ");
        Serial.println(LEDRState);
        error = 0;
      }
    }
    if (DID==33) {                // transmission interval
      if (readAction) {
        send33 = true;
        error = 0;
      } else {                // set transmission interval
        LEDGState = strPayload.toInt();
        analogWrite(LEDGPIN,LEDGState);
        if (setAck) send33 = true;
        Serial.print("Set Green LED State to: ");
        Serial.println(LEDGState);
        error = 0;
      }
    }
    if (DID==34) {                // transmission interval
      if (readAction) {
        send34 = true;
        error = 0;
      } else {                // set transmission interval
        LEDBState = strPayload.toInt();
        analogWrite(LEDBPIN,LEDBState);
        if (setAck) send34 = true;
        Serial.print("Set Blue LED State to: ");
        Serial.println(LEDBState);
        error = 0;
      }
    }
    if (DID==35) {                // transmission interval
      if (readAction) {
        error = 3;
      } else {                // set transmission interval
        LEDState = strPayload.toInt();
        LEDRState = LEDState/1000000;
        LEDGState = (LEDState/1000)-(LEDRState*1000);
        LEDBState = LEDState-(LEDRState*1000000)-(LEDGState*1000);
        analogWrite(LEDRPIN,LEDRState);
        analogWrite(LEDGPIN,LEDGState);
        analogWrite(LEDBPIN,LEDBState);
        if (setAck){
          send32 = true;      // acknowledge message ?
          send33 = true;      // acknowledge message ?
          send34 = true;      // acknowledge message ?
        }
        Serial.print("Set Red LED State to: ");
        Serial.println(LEDRState);
        Serial.print("Set Green LED State to: ");
        Serial.println(LEDGState);
        Serial.print("Set Blue LED State to: ");
        Serial.println(LEDBState);
        error = 0;
      }
    }
    if (DID ==40) {                // IP address 
      if (readAction) {
        send40 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
    #ifdef REEDENABLE
    if (DID ==41) {                // IP address 
      if (readAction) {
        send41 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
    #endif
    #ifdef WATERENABLE
    if (DID ==42) {                // IP address 
      if (readAction) {
        send42 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
    #endif
    if (DID ==48) {                // IP address 
      if (readAction) {
        send48 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
    if (DID ==49) {                // IP address 
      if (readAction) {
        send49 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
    #ifdef OWENABLE
    if (DID ==51) {                // IP address 
      if (readAction) {
        send51 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
    #endif
    if (DID ==64) {                // IP address 
      if (readAction) {
        send64 = true;
        error = 0;
      } else error = 3;           // invalid payload; do not process
    }
	}
		} else error =1;
		if (error !=0) {							// send error message
				sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev91", nodeId);
				sprintf(buff_msg, "syntax error %d", error);
				pubMQTT(buff_topic, buff_msg);
			}
		}

	void reconnect() {								// reconnect to mqtt broker
		sprintf(buff_topic, "home/esp_gw/sb/node%02d/#", nodeId);
    sprintf(clientName, "ESP_%02d", nodeId);
    mqttCon = true;
		while (!client.connected()) {
			Serial.print("Connect to MQTT broker...");
			if (client.connect(clientName,"home/esp_gw/disconnected",0,true,clientName)) {
        mqttCon = false;
				client.subscribe(buff_topic);
				Serial.println("connected");
			} else {
				Serial.println("Failed, try again in 5 seconds");
				delay(5000);
				}
			}
		}

	void sendMsg() {								// send any outstanding messages
	int i;
	if (wakeUp) {									// send wakeup message
		wakeUp = false;
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev99", nodeId);
		sprintf(buff_msg, "NODE %d WAKEUP: %s",  nodeId, clientName);
		send99 = false;
		pubMQTT(buff_topic, buff_msg);
		}
	if (send0) {									// send uptime
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev00", nodeId);
		sprintf(buff_msg, "%d", upTime);
		send0 = false;
		pubMQTT(buff_topic, buff_msg);
		}

	if (send1) {									// send transmission interval
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev01", nodeId);
		sprintf(buff_msg, "%d", TXinterval);
		send1 = false;
		pubMQTT(buff_topic, buff_msg);
		}

	if (send2) {									// send transmission interval
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev02", nodeId);
		signalStrength = WiFi.RSSI();
		sprintf(buff_msg, "%d", signalStrength);
		send2 = false;
		pubMQTT(buff_topic, buff_msg);
		}

	if (send3) {									// send software version
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev03", nodeId);
		for (i=0; i<sizeof(VERSION); i++) {
			buff_msg[i] = VERSION[i];}
		buff_msg[i] = '\0';
		send3 = false;
		pubMQTT(buff_topic, buff_msg);
		}

	if (send5) {									// send ACK state
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev05", nodeId);
		if (!setAck) sprintf(buff_msg, "OFF");
		else sprintf(buff_msg, "ON");
		pubMQTT(buff_topic, buff_msg);
		send5 = false;
	}
	if (send10) {								// send IP address
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev10", nodeId);
		for (i=0; i<16; i++) {
			buff_msg[i] = IP[i];}
		buff_msg[i] = '\0';
		pubMQTT(buff_topic, buff_msg);
		send10 = false;
	}
  if (send32) {                  // send button pressed message
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev32", nodeId);
    sprintf(buff_msg, "%d", LEDRState);
    pubMQTT(buff_topic, buff_msg);
    send32 = false;
  }
  if (send33) {                  // send button pressed message
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev33", nodeId);
    sprintf(buff_msg, "%d", LEDGState);
    pubMQTT(buff_topic, buff_msg);
    send33 = false;
  }
  if (send34) {                  // send button pressed message
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev34", nodeId);
    sprintf(buff_msg, "%d", LEDBState);
    pubMQTT(buff_topic, buff_msg);
    send34 = false;
  }
	if (send40) {									// send button pressed message
		sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev40", nodeId);
		if (curPIR==HIGH) sprintf(buff_msg, "OFF");
		if (curPIR==LOW) sprintf(buff_msg, "ON");
		pubMQTT(buff_topic, buff_msg);
		send40 = false;
	}
  #ifdef REEDENABLE
  if (send41) {                 // send button pressed message
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev41", nodeId);
    if (curREED==HIGH) sprintf(buff_msg, "OFF");
    if (curREED==LOW) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send41 = false;
  }
	#endif
  #ifdef WATERENABLE
  if (send42) {                 // send button pressed message
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev42", nodeId);
    if (curWATER==HIGH) sprintf(buff_msg, "OFF");
    if (curWATER==LOW) sprintf(buff_msg, "ON");
    pubMQTT(buff_topic, buff_msg);
    send42 = false;
  }
  #endif
  if (send48) {                  // send uptime
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev48", nodeId);
    temp = dht.readTemperature() + DHTTEMPCORRECTION;
    sprintf(buff_msg, "%d", temp);
    send48 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  if (send49) {                  // send uptime
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev49", nodeId);
    hum = dht.readHumidity();
    sprintf(buff_msg, "%d", hum);
    send49 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  #ifdef OWENABLE
  if (send51) {                  // send uptime
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev51", nodeId);
    owTemp = sensors.getTempCByIndex(0);
    sprintf(buff_msg, "%d", owTemp);
    send51 = false;
    pubMQTT(buff_topic, buff_msg);
  }
  #endif
  if (send64) {                  // send button pressed message
    sprintf(buff_topic, "home/esp_gw/nb/node%02d/dev64", nodeId);
    LightState = analogRead(LIGHTPIN);
    sprintf(buff_msg, "%d", LightState);
    pubMQTT(buff_topic, buff_msg);
    send64 = false;
  }
}

	//	SETUP

//===============================================================================================
void setup() {									// set up serial, output and wifi connection
  dht.begin();            // initialise temp/hum sensor
  pinMode(LEDRPIN, OUTPUT);
  pinMode(LEDGPIN, OUTPUT);
  pinMode(LEDBPIN, OUTPUT);
  LEDRState = 255;
  LEDGState = 125;
  LEDBState = 0;
  analogWrite(LEDRPIN, LEDRState);
  analogWrite(LEDGPIN, LEDGState);
  analogWrite(LEDBPIN, LEDBState);
  delay(100);
  pinMode(PIRPIN, INPUT);
  delay(100);
  #ifdef REEDENABLE
  pinMode(REEDPIN, INPUT);
  delay(100);
  #endif
  #ifdef WATERENABLE
  pinMode(WATERPIN, INPUT);
  delay(100);
  #endif
  #ifdef OWENABLE
  sensors.begin(); // Onewire: Start up the library
  delay(100);
  #endif
	send0 = false;
  send1 = false;
  send2 = false;
	send3 = false;
	send5 = false;
	send10 = true;
	send32 = false;
  send33 = false;
  send34 = false;
	send40 = false;
  #ifdef REEDENABLE
  send41 = false;
  #endif
  #ifdef WATERENABLE
  send42 = false;
  #endif
  send48 = false;
  send49 = false;
  #ifdef OWENABLE
  send51 = false;
  #endif
  send64 = false;
  Serial.begin(SERIAL_BAUD);
  Serial.println();							// connect to WIFI
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  IP = WiFi.localIP().toString();
  Serial.println(IP);
  LEDRState = 0;
  LEDGState = 0;
  LEDBState = 0;
  #ifdef RANDOM
  delay(random(30000));
  #endif
  analogWrite(LEDRPIN, LEDRState);
  analogWrite(LEDGPIN, LEDGState);
  analogWrite(LEDBPIN, LEDBState);
  delay(500);
  analogWrite(LEDRPIN, 255);
  delay(50);
  analogWrite(LEDRPIN, 0);
  delay(200);
  analogWrite(LEDRPIN, 255);
  delay(50);
  analogWrite(LEDRPIN, 0);
  delay(200);
  analogWrite(LEDRPIN, 255);
  delay(50);
  analogWrite(LEDRPIN, 0);
}

	//	LOOP

//===============================================================================================
	void loop() {									// Main program loop
	if (!client.connected()) {
		reconnect();
		}
	client.loop();

		// DETECT INPUT CHANGE
  curPIR = digitalRead(PIRPIN);
  msgBlock = ((millis() - lastPIRPress) < HOLDOFF);    // hold-off time for additional button messages
  if (!msgBlock &&  (curPIR != lastPIR)) {      // input changed ?
    delay(5);
    lastPIRPress = millis();              // take timestamp
    send40 = true;                    // set button message flag
  lastPIR = curPIR;
  }

  #ifdef REEDENABLE
  curREED = digitalRead(REEDPIN);
  msgBlock = ((millis() - lastREEDPress) < HOLDOFF);    // hold-off time for additional button messages
  if (!msgBlock &&  (curREED != lastREED)) {      // input changed ?
    delay(5);
    lastREEDPress = millis();              // take timestamp
    send41 = true;                    // set button message flag
  lastREED = curREED;
  }
  #endif

  #ifdef WATERENABLE
  curWATER = digitalRead(WATERPIN);
  msgBlock = ((millis() - lastWATERPress) < HOLDOFF);    // hold-off time for additional button messages
  if (!msgBlock &&  (curWATER != lastWATER)) {      // input changed ?
    delay(5);
    lastWATERPress = millis();              // take timestamp
    send42 = true;                    // set button message flag
  lastWATER = curWATER;
  }
  #endif
  
  #ifdef OWENABLE
  sensors.requestTemperatures(); // Send command to get temperatures
  
  #endif

		// INCREASE UPTIME 

	if (lastMinute != (millis()/60000)) {						// another minute passed ?
		lastMinute = millis()/60000;
		upTime++;
		}

		// PERIODIC TRANSMISSION


	if (TXinterval > 0){
		int currPeriod = millis()/(TXinterval*1000);
		if (currPeriod != lastPeriod) {							// interval elapsed ?
			lastPeriod = currPeriod;

			// list of sensordata to be sent periodically..
			// remove comment to include parameter in transmission
 
			send0 = true;										// send uptime
			send1 = true;                   // send interval
      send2 = true;										// send RSSI
			send3 = true;										// send version
      send5 = true;                   // send ack
      send10 = true;                   // send uptime
			send32 = true;										// red LED
      send33 = true;                    // green LED
      send34 = true;                    // blue LED
      send40 = true;                    // Pir state
      #ifdef REEDENABLE
      send41 = true;                    // REED state
      #endif
      #ifdef WATERENABLE
      send42 = true;                    // WATER state
      #endif
      send48 = true;                    // temperature
      send49 = true;                    // humidity
      #ifdef OWENABLE
      send51 = true;                    // onewire temperature
      #endif
      send64 = true;                    // light level
			}
		}

	sendMsg();													// send any mqtt messages

}		// end loop
    

