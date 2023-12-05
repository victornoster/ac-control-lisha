/*MQTT implementation for ESP8266 with AT commands*/
#include "usart.h"
#include <string>
using namespace std;

class MQTT {
public:

	enum MQTT_COMMANDS{
		MQTTUSERCFG = 0,
		MQTTCONN,
		MQTTPUBSTRING,
		MQTTPUBRAW,
		MQTTSUB,
	};
	const char * wifi_command = "AT+CWJAP=";
	const char * mqtt_commands[5] = {
		 "AT+MQTTUSERCFG=",
		 "AT+MQTTCONN=",
		 "AT+MQTTPUB=",
		 "AT+MQTTPUBRAW=",
		 "AT+MQTTSUB="
	};

	MQTT(const char* bbroker, uint16_t pport, const char* sssid, const char* ppsw); //implements the mqtt connection with broker and wi-fi network

	void setSSID(const char* _ssid);
	void setPSW(const char* _psw);
	void setMQTTServer(const char* _broker);
	void setMQTTPort(uint16_t _port);
	string Subscribe(const char* _topic_in, uint8_t length); //return a payload of the topic subscribed
	void Publish(const char* _topic_out, string _payload); //send a payload to the topic published
	void Connect(const char* _broker, uint16_t _port, uint8_t reconnect);
private:
	const char* topic_in;
	const char* topic_out;
	const char* ssid;
	const char* psw;
	const char* mqqt_server;
	uint16_t mqtt_port;

};
