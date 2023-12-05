//Programa: NodeMCU e MQTT - Controle e Monitoramento IoT
//Autor: Pedro Bertoleti
 
#include <ESP8266WiFi.h> // Importa a Biblioteca ESP8266WiFi
#include <WiFiClient.h>
#include <PubSubClient.h> // Importa a Biblioteca PubSubClient
#include <ESP8266WebServer.h>

//defines:
//defines de id mqtt e tópicos para publicação e subscribe
#define TOPICO_SUBSCRIBE "teste/AC_control/Sub"     //tópico MQTT de escuta
#define TOPICO_PUBLISH   "teste/AC_control/Pub"    //tópico MQTT de envio de informações para Broker
                                                   //IMPORTANTE: recomendamos fortemente alterar os nomes
                                                   //            desses tópicos. Caso contrário, há grandes
                                                   //            chances de você controlar e monitorar o NodeMCU
                                                   //            de outra pessoa.
#define ID_MQTT  "AC_Control"     //id mqtt (para identificação de sessão)
                               //IMPORTANTE: este deve ser único no broker (ou seja, 
                               //            se um client MQTT tentar entrar com o mesmo 
                               //            id de outro já conectado ao broker, o broker 
                               //            irá fechar a conexão de um deles).
#define DATA_SIZE 20                                

 
// WIFI
const char* SSID = "LISHA_WIFI"; // SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "l1sh4_2022"; // Senha da rede WI-FI que deseja se conectar
//MQTT
const char* BROKER_MQTT = "mqtt.eclipseprojects.io";
int BROKER_PORT = 1883; // Porta do Broker MQTT
 
 
//Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient
char data[DATA_SIZE];
int sizeofData;

//Prototypes
void initSerial();
void initWiFi();
void initMQTT();
void reconnectWiFi(); 
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void VerificaConexoesWiFIEMQTT(void);
void InitOutput(void);
/* 
 *  Implementações das funções
 */
void setup() {
  Serial.begin(115200);
  //inicializações:
  initWiFi();
  initMQTT();
}
 
//Função: inicializa e conecta-se na rede WI-FI desejada
//Parâmetros: nenhum
//Retorno: nenhum
void initWiFi() 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
     
    reconnectWiFi();
}
  
//Função: inicializa parâmetros de conexão MQTT(endereço do 
//        broker, porta e seta função de callback)
//Parâmetros: nenhum
//Retorno: nenhum
void initMQTT() 
{
    MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
  
//Função: função de callback 
//        esta função é chamada toda vez que uma informação de 
//        um dos tópicos subescritos chega)
//Parâmetros: nenhum
//Retorno: nenhum
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    char msg[length];
    
    //Serial.print("Mensagem recebida no MQTT: ");
    
    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
       msg[i] = (char)payload[i];
       Serial.print(msg[i]);
    }
    delay(10);
    //Serial.println();
    //toma ação dependendo da string recebida:
    //verifica se deve colocar nivel alto de tensão na saída D0:
    //IMPORTANTE: o Led já contido na placa é acionado com lógica invertida (ou seja,
    //enviar HIGH para o output faz o Led apagar / enviar LOW faz o Led acender)
    
}
  
//Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
//        em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
//Parâmetros: nenhum
//Retorno: nenhum
void reconnectMQTT() 
{
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE); 
        } 
        else
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            delay(2000);
        }
    }
}
  
//Função: reconecta-se ao WiFi
//Parâmetros: nenhum
//Retorno: nenhum
void reconnectWiFi() 
{
    //se já está conectado a rede WI-FI, nada é feito. 
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
        
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI

    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
   
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}
 
//Função: verifica o estado das conexões WiFI e ao broker MQTT. 
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFIEMQTT(void)
{
    if (!MQTT.connected()) 
        reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
     
     reconnectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}
 
//Função: envia ao Broker o estado atual do output 
//Parâmetros: nenhum
//Retorno: nenhum
void sendDataToMQTT(void) {
  //Serial.print("Mensagem enviada ao BROKER: ");
  MQTT.publish(TOPICO_PUBLISH, (const uint8_t*)data, 6);
  
  /*for (int i = 0; i < sizeofData; i++) {
    Serial.print(data[i]);
  }*/
  //Serial.println();
  delay(20);
}

void readFromSTM(){
if (Serial.available() > 0) {
  delay(500);
 // read the incoming byte:
 //Serial.print("Chegou x bytes: ");
 //Serial.println(Serial.available());
 int i = 0;
    while(Serial.available() > 0 && i < DATA_SIZE-1) {
      //Serial.print("Tem x bytes: ");
      //Serial.println(Serial.available());
      data[i] = char(Serial.read());
      //Serial.print(data[i]);
      ++i;
    }
    data[i] = 0;
    sizeofData = i;
    //Serial.println(sizeofData);
    if(data[0]=='[' && data[5]==']') sendDataToMQTT(); 
  }
}

//programa principal
void loop() 
{   
    //garante funcionamento das conexões WiFi e ao broker MQTT
    VerificaConexoesWiFIEMQTT();
  
    //envia o status de todos os outputs para o Broker no protocolo esperado
    readFromSTM();
 
    //keep-alive da comunicação com broker MQTT
    MQTT.loop();
}