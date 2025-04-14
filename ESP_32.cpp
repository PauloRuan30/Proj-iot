/*
  ESP32 - Sistema de Monitoramento Distribuído de Ambientes (SMDA)
  
  Funcionalidades implementadas:
  - Leitura de sensores (temperatura, umidade, som, luminosidade e presença)
  - Comunicação com outros ESP32 via ESP-NOW
  - Envio de dados para servidor central via MQTT
  - Mecanismos de resiliência e detecção de falhas
  - Modo de baixo consumo de energia
  
  Hardware necessário:
  - ESP32
  - Sensor DHT22 (temperatura e umidade)
  - Sensor KY-038 (detecção de som)
  - Sensor LDR (luminosidade)
  - Sensor PIR HC-SR501 (detecção de movimento)
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ========== CONFIGURAÇÃO GERAL ==========
#define NODE_ID 1  // ID único deste ESP32 (1, 2 ou 3)
#define NODE_NAME "Sala"  // Nome do ambiente monitorado

// ========== CONFIGURAÇÃO DOS PINOS ==========
#define DHT_PIN 4        // DHT22 - Temperatura e umidade
#define SOUND_PIN 36     // KY-038 - Sensor de som (analógico)
#define SOUND_DIGITAL 39 // KY-038 - Detecção de som alto (digital)
#define LDR_PIN 34       // LDR - Sensor de luminosidade
#define PIR_PIN 5        // PIR - Sensor de movimento
#define LED_PIN 2        // LED embutido

// ========== CONFIGURAÇÃO DE SENSORES ==========
#define DHT_TYPE DHT22
#define SAMPLE_INTERVAL 10000  // Intervalo de amostragem em ms (10 segundos)
#define MQTT_PUBLISH_INTERVAL 30000  // Intervalo de publicação MQTT em ms (30 segundos)
#define PRESENCE_TIMEOUT 60000 // Tempo antes de considerar ambiente sem presença (60 segundos)

// ========== CONFIGURAÇÃO WIFI ==========
const char* ssid = "SeuWiFi";
const char* password = "SuaSenha";

// ========== CONFIGURAÇÃO MQTT ==========
const char* mqtt_server = "192.168.1.100";  // Endereço IP do servidor MQTT
const int mqtt_port = 1883;
const char* mqtt_user = "smda_user";
const char* mqtt_password = "smda_password";
const char* mqtt_client_id = "ESP32_Node1";  // Alterar conforme NODE_ID

// Tópicos MQTT
String baseTopic = "smda/";
String dataTopic = baseTopic + "data/" + NODE_ID;
String statusTopic = baseTopic + "status/" + NODE_ID;
String alertTopic = baseTopic + "alert/" + NODE_ID;
String controlTopic = baseTopic + "control/" + NODE_ID;
String heartbeatTopic = baseTopic + "heartbeat";

// ========== CONFIGURAÇÃO ESP-NOW ==========
// MAC addresses dos outros ESP32 na rede (substituir pelos MACs reais)
uint8_t broadcastAddress1[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // MAC do ESP32 #2
uint8_t broadcastAddress2[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // MAC do ESP32 #3

// Estrutura para enviar dados via ESP-NOW
typedef struct sensor_data {
  int nodeId;
  float temperature;
  float humidity;
  int soundLevel;
  int lightLevel;
  bool presence;
  unsigned long timestamp;
} sensor_data;

// ========== VARIÁVEIS GLOBAIS ==========
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;

// Dados dos sensores
float temperature = 0.0;
float humidity = 0.0;
int soundLevel = 0;
int lightLevel = 0;
bool presence = false;
unsigned long lastMovementTime = 0;

// Dados dos outros nós
sensor_data receivedData[3];  // Armazena dados dos outros nós (incluindo este)

// Controle de tempo
unsigned long lastSampleTime = 0;
unsigned long lastPublishTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastEspNowTime = 0;
unsigned long lastReconnectAttempt = 0;

// Contadores de falhas
int wifiReconnectAttempts = 0;
int mqttReconnectAttempts = 0;
int espNowFailures = 0;

// Estado de outros nós
bool nodeActive[3] = {false, false, false};

// ========== FUNÇÕES DE INICIALIZAÇÃO ==========

void setupWiFi() {
  Serial.println("Conectando à rede WiFi...");
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) {
    delay(500);
    Serial.print(".");
    attempt++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFalha ao conectar WiFi. Operando em modo limitado.");
  }
}

void setupMQTT() {
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  
  Serial.println("Configuração MQTT concluída");
}

void setupESPNow() {
  // Inicializa ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }
  
  // Registra função de callback
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  
  // Registra peers
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  
  // Adiciona peer 1
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar peer 1");
  }
  
  // Adiciona peer 2
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar peer 2");
  }
  
  Serial.println("ESP-NOW inicializado com sucesso");
}

void setupSensors() {
  pinMode(SOUND_PIN, INPUT);
  pinMode(SOUND_DIGITAL, INPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  dht.begin();
  
  Serial.println("Sensores inicializados");
}

// ========== FUNÇÕES DE COMUNICAÇÃO MQTT ==========

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida [");
  Serial.print(topic);
  Serial.print("] ");
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.println(message);
  
  // Processar comandos recebidos
  if (String(topic) == controlTopic) {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, message);
    
    // Exemplo de comando para ajustar intervalo de amostragem
    if (doc.containsKey("sampleInterval")) {
      int newInterval = doc["sampleInterval"];
      if (newInterval >= 1000) {  // Mínimo 1 segundo
        preferences.begin("smda", false);
        preferences.putInt("sampleInterval", newInterval);
        preferences.end();
        Serial.println("Novo intervalo de amostragem: " + String(newInterval) + "ms");
      }
    }
    
    // Comando para reiniciar o dispositivo
    if (doc.containsKey("restart") && doc["restart"]) {
      Serial.println("Reiniciando o dispositivo...");
      ESP.restart();
    }
  }
}

boolean reconnectMQTT() {
  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_password)) {
    Serial.println("Conectado ao servidor MQTT");
    
    // Inscrever em tópicos relevantes
    mqttClient.subscribe(controlTopic.c_str());
    mqttClient.subscribe(heartbeatTopic.c_str());
    
    // Publicar mensagem de status online
    DynamicJsonDocument statusDoc(256);
    statusDoc["nodeId"] = NODE_ID;
    statusDoc["name"] = NODE_NAME;
    statusDoc["status"] = "online";
    statusDoc["ip"] = WiFi.localIP().toString();
    statusDoc["rssi"] = WiFi.RSSI();
    
    String statusMsg;
    serializeJson(statusDoc, statusMsg);
    mqttClient.publish(statusTopic.c_str(), statusMsg.c_str(), true);
    
    mqttReconnectAttempts = 0;
    return true;
  } else {
    mqttReconnectAttempts++;
    Serial.print("Falha na conexão MQTT, rc=");
    Serial.print(mqttClient.state());
    Serial.println(" Tentativa: " + String(mqttReconnectAttempts));
    
    // Se falhar muitas vezes, reinicia o WiFi
    if (mqttReconnectAttempts > 5) {
      Serial.println("Muitas falhas MQTT. Reiniciando WiFi...");
      WiFi.disconnect();
      delay(1000);
      setupWiFi();
      mqttReconnectAttempts = 0;
    }
    return false;
  }
}

void publishData() {
  if (!mqttClient.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(512);
  doc["nodeId"] = NODE_ID;
  doc["name"] = NODE_NAME;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["soundLevel"] = soundLevel;
  doc["lightLevel"] = lightLevel;
  doc["presence"] = presence;
  doc["timestamp"] = millis();
  
  String message;
  serializeJson(doc, message);
  
  if (mqttClient.publish(dataTopic.c_str(), message.c_str())) {
    Serial.println("Dados publicados no MQTT");
  } else {
    Serial.println("Falha ao publicar dados no MQTT");
  }
}

void publishHeartbeat() {
  if (!mqttClient.connected()) {
    return;
  }
  
  DynamicJsonDocument doc(128);
  doc["nodeId"] = NODE_ID;
  doc["uptime"] = millis() / 1000;
  doc["freeHeap"] = ESP.getFreeHeap();
  
  String message;
  serializeJson(doc, message);
  
  mqttClient.publish(heartbeatTopic.c_str(), message.c_str());
}

// ========== FUNÇÕES DE COMUNICAÇÃO ESP-NOW ==========

void sendESPNowData() {
  sensor_data dataToSend;
  
  dataToSend.nodeId = NODE_ID;
  dataToSend.temperature = temperature;
  dataToSend.humidity = humidity;
  dataToSend.soundLevel = soundLevel;
  dataToSend.lightLevel = lightLevel;
  dataToSend.presence = presence;
  dataToSend.timestamp = millis();
  
  // Salva os dados locais no array
  receivedData[NODE_ID - 1] = dataToSend;
  
  // Envia para os outros ESPs
  esp_err_t result1 = esp_now_send(broadcastAddress1, (uint8_t *)&dataToSend, sizeof(dataToSend));
  esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *)&dataToSend, sizeof(dataToSend));
  
  if (result1 == ESP_OK && result2 == ESP_OK) {
    Serial.println("Dados enviados via ESP-NOW com sucesso");
    espNowFailures = 0;
  } else {
    Serial.println("Erro ao enviar dados via ESP-NOW");
    espNowFailures++;
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("Falha na entrega ESP-NOW");
  }
}

void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (data_len == sizeof(sensor_data)) {
    sensor_data *receivedDataStruct = (sensor_data *)data;
    
    int nodeIndex = receivedDataStruct->nodeId - 1;
    if (nodeIndex >= 0 && nodeIndex < 3 && nodeIndex != (NODE_ID - 1)) {
      // Armazena os dados recebidos
      receivedData[nodeIndex] = *receivedDataStruct;
      nodeActive[nodeIndex] = true;
      
      Serial.print("Dados recebidos do nó ");
      Serial.println(receivedDataStruct->nodeId);
      Serial.print("Temperatura: ");
      Serial.println(receivedDataStruct->temperature);
      Serial.print("Umidade: ");
      Serial.println(receivedDataStruct->humidity);
      
      // Se o MQTT estiver conectado, publica os dados recebidos
      if (mqttClient.connected()) {
        String forwardTopic = baseTopic + "forward/" + String(receivedDataStruct->nodeId);
        
        DynamicJsonDocument doc(512);
        doc["nodeId"] = receivedDataStruct->nodeId;
        doc["forwardedBy"] = NODE_ID;
        doc["temperature"] = receivedDataStruct->temperature;
        doc["humidity"] = receivedDataStruct->humidity;
        doc["soundLevel"] = receivedDataStruct->soundLevel;
        doc["lightLevel"] = receivedDataStruct->lightLevel;
        doc["presence"] = receivedDataStruct->presence;
        doc["timestamp"] = receivedDataStruct->timestamp;
        
        String message;
        serializeJson(doc, message);
        
        mqttClient.publish(forwardTopic.c_str(), message.c_str());
      }
    }
  }
}

// ========== FUNÇÕES DE LEITURA DE SENSORES ==========

void readSensors() {
  // Leitura do sensor DHT (temperatura e umidade)
  float newTemp = dht.readTemperature();
  float newHum = dht.readHumidity();
  
  // Verifica se as leituras são números válidos
  if (!isnan(newTemp)) {
    temperature = newTemp;
  }
  
  if (!isnan(newHum)) {
    humidity = newHum;
  }
  
  // Leitura do sensor de som
  int soundValue = analogRead(SOUND_PIN);
  int soundDigital = digitalRead(SOUND_DIGITAL);
  
  soundLevel = map(soundValue, 0, 4095, 0, 100);  // Mapeia para percentual
  
  if (soundDigital == LOW) {  // LOW é som detectado em muitos sensores KY-038
    Serial.println("Som alto detectado!");
    // Opcional: publicar alerta de som alto
  }
  
  // Leitura do sensor de luminosidade
  int ldrValue = analogRead(LDR_PIN);
  lightLevel = map(ldrValue, 0, 4095, 0, 100);  // Mapeia para percentual
  
  // Leitura do sensor de movimento (PIR)
  int pirValue = digitalRead(PIR_PIN);
  if (pirValue == HIGH) {
    lastMovementTime = millis();
    presence = true;
    digitalWrite(LED_PIN, HIGH);  // Acende LED integrado
  } else {
    digitalWrite(LED_PIN, LOW);   // Apaga LED
    
    // Verifica se passou tempo suficiente para considerar ausência
    if (millis() - lastMovementTime > PRESENCE_TIMEOUT) {
      presence = false;
    }
  }
  
  // Log dos valores lidos
  Serial.println("Leituras dos sensores:");
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.println("°C");
  
  Serial.print("Umidade: ");
  Serial.print(humidity);
  Serial.println("%");
  
  Serial.print("Nível de som: ");
  Serial.print(soundLevel);
  Serial.println("%");
  
  Serial.print("Luminosidade: ");
  Serial.print(lightLevel);
  Serial.println("%");
  
  Serial.print("Presença: ");
  Serial.println(presence ? "Sim" : "Não");
}

// ========== CHECAGEM DE ALERTAS ==========

void checkAlerts() {
  // Verifica se há alguma condição para gerar alertas
  bool hasAlert = false;
  DynamicJsonDocument alertDoc(256);
  alertDoc["nodeId"] = NODE_ID;
  alertDoc["name"] = NODE_NAME;
  
  // Exemplo: Alerta de temperatura alta
  if (temperature > 30.0) {
    alertDoc["temperatureHigh"] = temperature;
    hasAlert = true;
  }
  
  // Exemplo: Alerta de umidade baixa
  if (humidity < 30.0) {
    alertDoc["humidityLow"] = humidity;
    hasAlert = true;
  }
  
  // Exemplo: Alerta de som alto
  if (soundLevel > 80) {
    alertDoc["noiseLevelHigh"] = soundLevel;
    hasAlert = true;
  }
  
  // Se houver alertas e estiver conectado ao MQTT, publica
  if (hasAlert && mqttClient.connected()) {
    String alertMsg;
    serializeJson(alertDoc, alertMsg);
    mqttClient.publish(alertTopic.c_str(), alertMsg.c_str());
    
    Serial.println("Alerta publicado!");
  }
}

// ========== VERIFICAÇÃO DE OUTROS NÓS ==========

void checkOtherNodes() {
  unsigned long currentTime = millis();
  
  // Verifica se algum nó está inativo por muito tempo (2 minutos)
  for (int i = 0; i < 3; i++) {
    if (i != (NODE_ID - 1) && nodeActive[i]) {
      // Se não recebeu dados recentes deste nó
      if (currentTime - receivedData[i].timestamp > 120000) {
        nodeActive[i] = false;
        
        // Reporta nó inativo
        if (mqttClient.connected()) {
          DynamicJsonDocument statusDoc(128);
          statusDoc["nodeId"] = NODE_ID;
          statusDoc["reportingNodeOffline"] = i + 1;
          
          String statusMsg;
          serializeJson(statusDoc, statusMsg);
          mqttClient.publish(statusTopic.c_str(), statusMsg.c_str());
        }
      }
    }
  }
}

// ========== FUNÇÃO PRINCIPAL ==========

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n===== Sistema de Monitoramento Distribuído de Ambientes =====");
  Serial.println("Node ID: " + String(NODE_ID) + " - " + NODE_NAME);
  
  // Carrega configurações salvas
  preferences.begin("smda", true);
  int savedInterval = preferences.getInt("sampleInterval", SAMPLE_INTERVAL);
  preferences.end();
  
  // Inicializações
  setupSensors();
  setupWiFi();
  setupMQTT();
  setupESPNow();
  
  lastSampleTime = millis() - SAMPLE_INTERVAL;  // Para iniciar leitura rápida
  
  Serial.println("Sistema inicializado e pronto!");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Reconecta WiFi se necessário
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Conexão WiFi perdida. Reconectando...");
    setupWiFi();
  }
  
  // Reconecta MQTT se necessário
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      
      if (reconnectMQTT()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    mqttClient.loop();
  }
  
  // Leitura periódica dos sensores
  if (currentMillis - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentMillis;
    readSensors();
    checkAlerts();
  }
  
  // Envio de dados via MQTT
  if (currentMillis - lastPublishTime >= MQTT_PUBLISH_INTERVAL) {
    lastPublishTime = currentMillis;
    publishData();
  }
  
  // Envio de dados via ESP-NOW
  if (currentMillis - lastEspNowTime >= 15000) {  // A cada 15 segundos
    lastEspNowTime = currentMillis;
    sendESPNowData();
    checkOtherNodes();
  }
  
  // Envio periódico de heartbeat
  if (currentMillis - lastHeartbeatTime >= 60000) {  // A cada minuto
    lastHeartbeatTime = currentMillis;
    publishHeartbeat();
  }
  
  // Pequeno delay para não sobrecarregar o ESP32
  delay(50);
}
