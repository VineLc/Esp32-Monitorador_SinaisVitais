#include "arduino_secrets.h"
#include "thingProperties.h"  // Inclui a biblioteca responsável pelas propriedades do Thing (IoT)
#include <MAX3010x.h>  // Inclui a biblioteca para o sensor MAX3010x (oxímetro de pulso)
#include "filters.h"  // Inclui a biblioteca para filtros de sinal (usado no processamento de batimentos cardíacos)
#include <OneWire.h>  // Inclui a biblioteca para comunicação OneWire (usada no sensor de temperatura)
#include <DallasTemperature.h>  // Inclui a biblioteca para sensores de temperatura Dallas

// Definir o pino para o sensor de temperatura
const int ONE_WIRE_BUS = 4;
OneWire oneWire(ONE_WIRE_BUS);  // Inicializa o barramento OneWire no pino 4
DallasTemperature sensors(&oneWire);  // Inicializa a instância de sensor de temperatura com o barramento OneWire

MAX30105 sensor;  // Instância do sensor MAX30105 (oxímetro de pulso)
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;  // Define a taxa de amostragem para o sensor MAX30105 (400 amostras por segundo)
const float kSamplingFrequency = 400.0;  // Define a frequência de amostragem para os filtros (400 Hz)

// Limiares e constantes
const unsigned long kFingerThreshold = 10000;  // Limiar para detectar a presença de um dedo (leitura acima de 10000)
const unsigned int kFingerCooldownMs = 500;  // Tempo de espera (em milissegundos) para evitar leituras consecutivas de batimento
const float kEdgeThreshold = -2000.0;  // Limiar para detectar a transição de um batimento cardíaco
const float kLowPassCutoff = 5.0;  // Frequência de corte do filtro passa-baixa
const float kHighPassCutoff = 0.5;  // Frequência de corte do filtro passa-alta
const bool kEnableAveraging = false;  // Habilitar ou desabilitar a média de múltiplas leituras
const int kAveragingSamples = 5;  // Número de amostras a serem usadas para o cálculo da média
const int kSampleThreshold = 5;  // Número mínimo de amostras para considerações estatísticas

void setup() {
  Serial.begin(9600);  // Inicializa a comunicação serial com uma taxa de 9600 bps
  delay(2000);  // Aguarda 1,5 segundos para permitir que o sensor inicialize
  initProperties();  // Inicializa as propriedades do Thing para comunicação com a nuvem
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);  // Inicializa a conexão com a plataforma Arduino Cloud

  // Inicializar sensor de temperatura
  sensors.begin();  // Inicializa o sensor de temperatura DS18B20 no barramento OneWire
  
  // Inicializar sensor MAX30105
  if(sensor.begin() && sensor.setSamplingRate(kSamplingRate)) { 
    Serial.println("Sensor MAX30105 initialized");  // Confirma a inicialização bem-sucedida do sensor
  }
  else {
    Serial.println("Sensor MAX30105 not found");  // Informa se o sensor não foi encontrado
    while(1);  // Entra em loop infinito se o sensor não for encontrado
  }
  
  setDebugMessageLevel(2);  // Define o nível de mensagens de depuração
  ArduinoCloud.printDebugInfo();  // Imprime informações de depuração da conexão com a nuvem
}

LowPassFilter low_pass_filter_red(kLowPassCutoff, kSamplingFrequency);  // Filtro passa-baixa para o sinal vermelho do MAX30105
LowPassFilter low_pass_filter_ir(kLowPassCutoff, kSamplingFrequency);  // Filtro passa-baixa para o sinal infravermelho do MAX30105
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);  // Filtro passa-alta para o sinal do MAX30105
Differentiator differentiator(kSamplingFrequency);  // Diferenciador para detectar mudanças rápidas no sinal (batimentos)
MovingAverageFilter<kAveragingSamples> averager_bpm;  // Filtro de média móvel para o cálculo da frequência cardíaca
MovingAverageFilter<kAveragingSamples> averager_r;  // Filtro de média móvel para o cálculo da relação R (para o SpO2)
MovingAverageFilter<kAveragingSamples> averager_spo2;  // Filtro de média móvel para o cálculo do SpO2

MinMaxAvgStatistic stat_red;  // Estatísticas de valor mínimo, máximo e médio para o sinal vermelho
MinMaxAvgStatistic stat_ir;  // Estatísticas de valor mínimo, máximo e médio para o sinal infravermelho

// Constantes usadas no cálculo do SpO2
float kSpO2_A = 1.5958422;
float kSpO2_B = -34.6596622;
float kSpO2_C = 112.6898759;

long last_heartbeat = 0;  // Armazena o tempo do último batimento cardíaco detectado
long finger_timestamp = 0;  // Armazena o tempo em que o dedo foi detectado
bool finger_detected = false;  // Indica se um dedo foi detectado no sensor
float last_diff = NAN;  // Armazena a última diferença do sinal (usado para detectar cruzamento de batimento)
bool crossed = false;  // Indica se houve um cruzamento no sinal de batimento cardíaco
long crossed_time = 0;  // Armazena o tempo do cruzamento do batimento cardíaco

void loop() {   

  auto sample = sensor.readSample(1000);
  float current_value_red = sample.red;
  float current_value_ir = sample.ir;
  
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager_bpm.reset();
    averager_r.reset();
    averager_spo2.reset();
    low_pass_filter_red.reset();
    low_pass_filter_ir.reset();
    high_pass_filter.reset();
    stat_red.reset();
    stat_ir.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value_red = low_pass_filter_red.process(current_value_red);
    current_value_ir = low_pass_filter_ir.process(current_value_ir);

    // Statistics for pulse oximetry
    stat_red.process(current_value_red);
    stat_ir.process(current_value_ir);

    // Heart beat detection using value for red LED
    float current_value = high_pass_filter.process(current_value_red);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat - Falling Edge Threshold
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 60000/(crossed_time - last_heartbeat);
          float rred = (stat_red.maximum()-stat_red.minimum())/stat_red.average();
          float rir = (stat_ir.maximum()-stat_ir.minimum())/stat_ir.average();
          float r = rred/rir;
          float spo2 = kSpO2_A * r * r + kSpO2_B * r + kSpO2_C;

          sensors.requestTemperatures();  // Solicita a leitura da temperatura do sensor DS18B20
          if(temp == DEVICE_DISCONNECTED_C) {
                Serial.println("Erro: Sensor de temperatura desconectado");  // Exibe mensagem de erro se o sensor estiver desconectado
            } 

          temp = sensors.getTempCByIndex(0);  // Atualiza a variável temp com a temperatura medida 
          btm = bpm;
          oxi = spo2;
          
          if(bpm > 50 && bpm < 250) {
            // Average?
            if(kEnableAveraging) {
              int average_bpm = averager_bpm.process(bpm);
              int average_r = averager_r.process(r);
              int average_spo2 = averager_spo2.process(spo2);
  
              // Show if enough samples have been collected
              if(averager_bpm.count() >= kSampleThreshold) {
                //Serial.print("Time (ms): ");
                //Serial.println(millis()); 
                Serial.println("---------------------------------------------------------");
                Serial.print("Media de batimento por minuto: ");
                Serial.println(average_bpm);
                Serial.print("Amplitudes dos sinais de luz: ");
                Serial.println(average_r);  
                Serial.print("% Saturação de oxigênio no sangue: ");
                Serial.println(average_spo2);  
                Serial.print("Temperatura C°: ");
                Serial.println(temp); 
              }
            }
            else {
              //Serial.print("Time (ms): ");
              //Serial.println(millis()); 
              Serial.println("---------------------------------------------------------");
              Serial.print("Media de batimento por minuto: ");
              Serial.println(bpm);  
              Serial.print("Amplitudes dos sinais de luz: ");
              Serial.println(r);
              Serial.print("% Saturação de oxigênio no sangue: ");
              Serial.println(spo2);   
              Serial.print("Temperatura C°: ");
              Serial.println(temp); 
            }            
          }

          // Reset statistic
          stat_red.reset();
          stat_ir.reset();
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
  ArduinoCloud.update();
}