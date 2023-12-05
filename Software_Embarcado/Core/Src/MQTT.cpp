/*MQTT implementation*/

#include "MQTT.hpp"

MQTT::MQTT(const char* bbroker, uint16_t pport, const char* sssid, const char* ppsw){

	setMQTTServer(bbroker);
	setMQTTPort(pport);
	setSSID(sssid);
	setPSW(ppsw);

	sendATcommand(wifi_command, ssid, sizeof(ssid));
	sendCommand(psw, sizeof(psw));

//AT+MQTTUSERCFG=<LinkID>,<scheme>,<"client_id">,<"username">,<"password">,<cert_key_ID>,<CA_ID>,<"path">
	sendATcommand(mqtt_commands[MQTTUSERCFG], 0, 1); //LinkID
	sendCommand((const char*)1, 1);//scheme
	sendCommand("1", 1);//client id
	sendCommand("user", 4); //user
	sendCommand("pass", 4); //pass
	sendCommand(0, 1);
	sendCommand("" "", 3);
	sendCommand("\r", 1);
}

void MQTT::setSSID(const char* _ssid){
	ssid = _ssid;
}

void MQTT::setPSW(const char* _psw){
	psw=_psw;
}

void MQTT::setMQTTServer(const char* _broker){

	mqqt_server = _broker;

}

void MQTT::setMQTTPort(uint16_t _port){
	mqtt_port = _port;
}

string MQTT::Subscribe(const char* _topic_in, uint8_t length){	//return a payload of the topic subscribed
	string msg;
	uint8_t aux[length];
	topic_in = _topic_in;
	sendATcommand(mqtt_commands[MQTTSUB], 0, 1); //LinkId
	sendCommand(topic_in, sizeof(topic_in));

	for(int i = 0; i < length; i++)
	    {
			aux[i] = receiveCommand();
			char c = (char)aux[i];
	        msg += c;
	    }
	return msg;
}

void MQTT::Publish(const char* _topic_out, string _payload){ //send a payload to the topic published
	topic_out = _topic_out;
	sendATcommand(mqtt_commands[MQTTPUBSTRING], 0, 1); //linkID
	sendCommand(topic_out, sizeof(topic_out));
}

void MQTT::Connect(const char* _broker, uint16_t _port, uint8_t reconnect){
	sendATcommand(mqtt_commands[MQTTCONN], 0, 1); //LINKID
	sendCommand(_broker, sizeof(_broker));
	sendCommand((const char*)_port, sizeof(_port));
	sendCommand((const char*)reconnect, 1);

}
