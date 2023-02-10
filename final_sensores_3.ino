#include <stdio.h>
#include <math.h>
#include "Arduino.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "DHT.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_task_wdt.h"
 
// Imagens em bitmap
#include "logos/engauto.h" //Logotipo automovel
#include "logos/logo_temp_motor.h" //Logotipo temp_motor
#include "logos/logo_volts_motor.h" //Logotipo volts_motor
#include "logos/logo_temp_exterior.h" //Logotipo temp_exterior
#include "logos/logo_humidade.h" //Logotipo logo_humidade
#include "logos/logo_tps.h" //Logotipo logo_tps
#include "logos/logo_rpm.h" //Logotipo logo_rpm
 
// Constantes Global
const uint8_t DEBUG = 0; // Ativar esta opcao para netrar em modo "DEBUG"
 
// Definicoes ADC
#define ADCres 12 //Definicao de ADC
 
// Variaveis Globais
bool ESTADODISPLAY = false; //Registo de que os logotipos foram escritos no Display
 
//Definir os pinos de input
#define TFT_DC 12
#define TFT_CS 13
#define TFT_MOSI 14
#define TFT_CLK 27
#define TFT_RST 0
#define DHT_PIN 5
#define Volts_PIN 2
#define LM35_PIN 36
#define Hall_PIN 19
#define Pot_PIN 33
#define Motor_PIN 4
#define Ventoinha_PIN 21
#define Botao_PIN 32
 
// Definição de opções de espera do botao
#define botaoEspera 300
 
// Definicoes do display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST);
 
// Definicoes do sensor DHT11
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
 
// Definição de estruturas
typedef struct {
  int humidade;
  float temp_exterior;
} temphumidade_para_display;
 
// Definicoes de prototipos das tarefas
void vDisplay( void * pvParameters);
void vLerTempHumidade( void * pvParameters);
void vLerTempMotor (void * pvparameters);
void vLerVoltMotor (void * pvparameters);
void vLerBotao (void * pvParameters);
void vMotor (void * pvparameters);
void vVentoinha (void * pvparameters);
void vLerHall(void * pvParameters);
void vRPM(void * pvParameters);
 
// Handler de tarefas
TaskHandle_t xLerHallHandle;
TaskHandle_t xConverterRPMHandle;
 
// Definicoes dos prototipos das interrupcoes
void IRAM_ATTR ventoinhaBotao();
void IRAM_ATTR contagemRevolucao();
 
// Queues 
QueueHandle_t temphumidadeQueue;
QueueHandle_t tempmotorQueue;
QueueHandle_t voltMotorQueue;
QueueHandle_t botaoQueue;
QueueHandle_t rodaQueue;
QueueHandle_t velocidadeRodaQueue;
 
// Protoripos de semaforos para sincronizacao de interrupcoees
SemaphoreHandle_t botaoSemafro;
SemaphoreHandle_t resetContagemSemafro; //Semaforo para reset da contagem de passagens nos imanes (RPM)
SemaphoreHandle_t rodaSemafro;
 
void setup()
{
  // Inicializacao do sensor DHT11
  dht.begin();
  
  // Inicializacao do Display
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
 
  // Inputs de sensores
  pinMode(DHT_PIN, INPUT_PULLUP);
  pinMode(Volts_PIN, INPUT_PULLUP);
  pinMode(LM35_PIN, INPUT);
  pinMode(Hall_PIN, INPUT);
  
  // Inputs do utilizador
  pinMode(Botao_PIN, INPUT_PULLUP);
 
  // Outputs
  pinMode(Motor_PIN, OUTPUT);
  pinMode(Ventoinha_PIN, OUTPUT);
 
  // Criar Semaphores
  vSemaphoreCreateBinary(botaoSemafro);
  vSemaphoreCreateBinary(resetContagemSemafro);
  vSemaphoreCreateBinary(rodaSemafro);
  
  // Interrupcao para o botao
  attachInterrupt(digitalPinToInterrupt(Botao_PIN), ventoinhaBotao, FALLING);
  interrupts(); //Permite interrupcoes
  
  analogReadResolution(ADCres);
  Serial.begin(115200); //Inicia a porta serie
 
  // Criar queues
  temphumidadeQueue = xQueueCreate(3, sizeof(temphumidade_para_display));
  tempmotorQueue = xQueueCreate(3, sizeof(float));
  voltMotorQueue = xQueueCreate(3, sizeof(float));
  botaoQueue = xQueueCreate(3, sizeof(short int));
  rodaQueue = xQueueCreate(1, sizeof(int));
  velocidadeRodaQueue = xQueueCreate(3, sizeof(short int));
 
  vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
 
  // Criar tarefas
  xTaskCreatePinnedToCore(vDisplay, "escrever imagens e valores no display", 16192, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(vLerTempHumidade, "Temperatura e Humidade", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vLerTempMotor, "Temperatura Motor", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vLerVoltMotor, "DDP do Motor", 1024, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(vLerBotao, "Pressionar botao para ligar ventoinha", 1024, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(vRPM, "Contagem RPM", 1024, NULL, 1, NULL, 0);
 
  xTaskCreatePinnedToCore(vMotor, "Tarefa do motor de alta prioridade num outro core", 1024, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(vVentoinha, "Tarefa da ventoinha de baixa prioridade num outro core", 1024, NULL, 2, NULL, 1);
 
}
 
// Task para fazer a leitura do sensor DHT11
void vLerTempHumidade (void*pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
 
  temphumidade_para_display enviar_dados_dht11;
 
  for (;;)
  {
    enviar_dados_dht11.humidade = dht.readHumidity();
    enviar_dados_dht11.temp_exterior = dht.readTemperature();
 
    if (isnan(enviar_dados_dht11.humidade) || isnan(enviar_dados_dht11.temp_exterior))
    {
      Serial.println("Erro: Nao foi possivel ler o sensor DHT11!");
    } else {
      xQueueSend(temphumidadeQueue, (void *) &enviar_dados_dht11, 0);
      if (DEBUG == 1) {
        Serial.print("Humidity: ");
        Serial.print(enviar_dados_dht11.humidade);
        Serial.print("%  Temperature: ");
        Serial.print(enviar_dados_dht11.temp_exterior);
        Serial.println("°C ");
      }
    }
    
    Serial.println("  Tarefa: vLerTempHumidade");
 
    vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
  }
}
 
// Task para fazer a leitura do sensor LM35 agregado ao motor
void vLerTempMotor (void * pvparameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float tempmotor = 0, valorBruto = 0, ddP = 0;
  for (;;)
  {
    int valorBruto = analogRead(LM35_PIN);
    //Converter o valor ADC para diferenca de potencial em milivolts
    float ddP = valorBruto * (3300.0 / 4096.0);
    //Converter a diferenca de potencial para temperature em gruas celciusC
    float tempmotor = (ddP * 0.1)+10;
    Serial.println(valorBruto);
 
    xQueueSendToBack(tempmotorQueue, &tempmotor, 0);
 
    if (DEBUG == 1)
    {
      Serial.print("Temperatura do Motor: ");
      Serial.print(tempmotor);
      Serial.println("ºC");
    }
    
    Serial.println("  Tarefa: vLerTempMotor");
 
    vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
  }
}
 
// Task para fazer a leitura do sensor voltimetro que le a diferenca de potencial do motor
void vLerVoltMotor (void * pvparameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float voltOut = 0, voltIn = 0, valorBruto = 0, ddP = 0;
  float R1 = 30000.0, R2 = 7500.0;
  for (;;)
  {
    valorBruto = analogRead(Volts_PIN);
    voltOut = (valorBruto * 5.0) / 1024.0;
    voltIn = voltOut / (R2 / (R1 + R2));
 
    xQueueSendToFront(voltMotorQueue, &voltOut, 0);
 
    if (DEBUG == 1)
    {
      Serial.print("Volts do Motor: ");
      Serial.print(voltOut);
      Serial.println(" V");
    }
    
    Serial.println("  Tarefa: vLerVoltMotor");
 
    vTaskDelayUntil(&xLastWakeTime, (250 / portTICK_PERIOD_MS));
  }
}
 
// Task para fazer a aplicar a diferenca de potencial pretendida no motor apartir do potenciometro
void vMotor (void * pvparameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int potenciometro = 0, potenciometro2PWM = 0;
 
  for (;;)
  {
    potenciometro = analogRead(Pot_PIN);
    potenciometro = (potenciometro / 16.61);
 
    if (DEBUG == 1)
    {
      Serial.print("Sinal do Motor: ");
      Serial.println(potenciometro);
    }
 
    analogWrite(Motor_PIN, potenciometro);
    
    Serial.println("  Tarefa: vMotor");
 
    vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS));
  }
}
 
// Task de acionamento da ventoinha apartir de um botao 
void vVentoinha (void * pvparameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int estado = 0, botao = 0;
 
  for (;;)
  {
    xQueueReceive(botaoQueue, &botao, 0);
    if (botao == 5) {
      if (estado == 0) {
        digitalWrite(Ventoinha_PIN, HIGH);
        estado = 1;
      } else if (estado == 1) {
        digitalWrite(Ventoinha_PIN, LOW);
        estado = 0;
      }
    }
    botao = 0;
 
    if (DEBUG == 1)
    {
      Serial.print("Estado da Ventoinha (1-ON/0-OFF): ");
      Serial.println(estado);
    }
    
    Serial.println("  Tarefa: vVentoinha");
 
    vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS));
  }
}
 
// Task para fazer a leitura do sensor de Hall
void vLerHall (void * pvParamenters)
{
  xSemaphoreTake(resetContagemSemafro, 0);
  xSemaphoreTake(rodaSemafro, 0);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned short int contagemIman = 0;
 
  for (;;)
  {
    if (xSemaphoreTake(resetContagemSemafro, 0) == pdTRUE )
    {
      xQueueSendToBack(rodaQueue, &contagemIman, 0);
      contagemIman = 0;
      //Serial.println("  reset");
    }
 
    if (xSemaphoreTake(rodaSemafro, 0) == pdPASS)
    {
      contagemIman++;
      xQueueOverwrite(rodaQueue, &contagemIman);
    }
 
    if (DEBUG == 1)
    {
      Serial.print("Contagem do iman: ");
      Serial.println(contagemIman);
    }
    
    Serial.println("  Tarefa: vLerHall");
 
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
 
}
 
// Task para fazer a conversao da leitura do sensor de Hall em RPM
void vRPM (void * pvparameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  int estado = 0, botao = 0;
  unsigned short int  contagemIman = 0, velociadeRPM = 0, numImans = 1;
 
  attachInterrupt(digitalPinToInterrupt(Hall_PIN), contagemRevolucao, RISING);
  for (;;)
  {
    xTaskCreatePinnedToCore(vLerHall, "Debouncer for hall sensor", 1024, NULL, 1, &xLerHallHandle, 0);
    xSemaphoreGive(resetContagemSemafro);
 
    vTaskDelayUntil(&xLastWakeTime, (1000/ portTICK_PERIOD_MS));
    xSemaphoreTake(resetContagemSemafro, 0);
    vTaskDelete(xLerHallHandle);
 
    xQueueReceive(rodaQueue, &contagemIman, 0);
    xQueueReset(rodaQueue);
 
    velociadeRPM = (contagemIman * 60) / 2;
    if(velociadeRPM < 100){
      velociadeRPM == 0;
    }
    xQueueSend(velocidadeRodaQueue, &velociadeRPM, NULL);
 
    if (DEBUG == 1)
    {
      Serial.print("RPM do Motor: ");
      Serial.println(velociadeRPM);
    }
 
    Serial.println("  Tarefa: vRPM");
  }
}
 
// Task de acionamento do botao
void vLerBotao (void * pvParameters)
{
  xSemaphoreTake(botaoSemafro, 0);//Tira semaforo devido a iniciar programa com semáforo
  int botao = 5;// valor de identificacao de pressionamneto do botao
 
  for (;;)
  {
    xSemaphoreTake(botaoSemafro, portMAX_DELAY);
    xQueueSendToBack(botaoQueue, &botao, 0);
    vTaskDelay(botaoEspera / portTICK_PERIOD_MS);
    xSemaphoreTake(botaoSemafro, portMAX_DELAY);//Redundancia
    //apos periodo de espera garante que a tarefa nao corre acidentalmente.
  }
 
  if (DEBUG == 1)
    {
      Serial.print("Botao da ventoinha acionado");
    }
    
    Serial.println("  Tarefa: vLerBotao");
}
 
// Task do Display
void vDisplay(void *pvParameters)
{
  temphumidade_para_display receber_dados_dht11;
  float potenciometro = 0, tempmotor = 0, voltmotor = 0;
  int tps = 0, velociadeRPM = 0;
 
  TickType_t xLastWakeTime = xTaskGetTickCount();
 
  for (;;)
  {
    if (ESTADODISPLAY == false)
    {
      ESTADODISPLAY = true;
      //  LogoEAU
      int altura = 17, largura = 160;   // Definicao da dimensao da imagem
      int linha, coluna, inicio_x = 0; // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna, linha, pgm_read_word(engauto + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        }
      }
 
      // RPM
      altura = 20;
      largura = 38;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 110, linha + 80, pgm_read_word(logo_rpm + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }
 
      // TPS
      altura = 20;
      largura = 31;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 62, linha + 80, pgm_read_word(logo_tps + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }
 
      // Humidade
      altura = 20;
      largura = 15;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 19, linha + 80, pgm_read_word(logo_humidade + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }
 
      // Temp. Exterior
      altura = 20;
      largura = 16;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 120, linha + 25, pgm_read_word(logo_temp_exterior + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }
 
      // Volts do Motor
      altura = 21;
      largura = 21;   // Definicao da dimensao da imagem
      inicio_x = 0;  // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 70, linha + 25, pgm_read_word(logo_temp_ecu + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }
 
      // Temp. Motor
      altura = 20;    // Definicao da dimensao da imagem
      largura = 23;   // Definicao da dimensao da imagem
      inicio_x = 0;   // Definicao de ints temporarios
      // Ler a linha do bitmap
      for (linha = 0; linha < altura; linha++)
      {
        // Ler coluna do bitmap
        for (coluna = 0; coluna < largura; coluna++)
        {
          tft.drawPixel(coluna + 15, linha + 25, pgm_read_word(logo_temp_motor + inicio_x)); // Desenhar o pixel no sitio correto
          inicio_x++;
        } // end pixel
      }
    }
 
    xQueueReceive(velocidadeRodaQueue, &velociadeRPM, 0);
    xQueueReceive(temphumidadeQueue, &receber_dados_dht11, 0);
    xQueueReceive(tempmotorQueue, &tempmotor, 0);
    xQueueReceive(voltMotorQueue, &voltmotor, 0);
 
    potenciometro = analogRead(Pot_PIN);
    potenciometro = potenciometro / 40.95;
    tps = (int)potenciometro;
 
    if (DEBUG == 1) {
      Serial.println("Dados escrever display: ");
      Serial.println(tempmotor);
      Serial.println(voltmotor);
      Serial.println(receber_dados_dht11.temp_exterior);
      Serial.println(receber_dados_dht11.humidade);
      Serial.println(potenciometro);
      Serial.println(velociadeRPM);
    }
 
    tft.setCursor(10, 50);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE);
    tft.fillRect(10, 50, 42, 7, ST77XX_BLACK); //Apagar valores
    tft.print(tempmotor);
    tft.print((char)167);
    tft.print("C");
    tft.setCursor(60, 50);
    tft.fillRect(50, 50, 52, 7, ST77XX_BLACK); //Apagar valores
    tft.print(voltmotor);
    tft.print("V");
    tft.setCursor(110, 50);
    tft.fillRect(110, 50, 42, 7, ST77XX_BLACK); //Apagar valores
    tft.print(receber_dados_dht11.temp_exterior);
    tft.print((char)167);
    tft.print("C");
    tft.setCursor(16, 110);
    tft.fillRect(16, 110, 18, 7, ST77XX_BLACK); //Apagar valores
    tft.print(receber_dados_dht11.humidade);
    tft.print("%");
    tft.setCursor(68, 110);
    tft.fillRect(68, 110, 30, 7, ST77XX_BLACK); //Apagar valores
    tft.print(tps);
    tft.print("%");
    tft.setCursor(122, 110);
    tft.fillRect(118, 110, 26, 7, ST77XX_BLACK); //Apagar valores
    tft.print(velociadeRPM);
 
    Serial.println("  Tarefa: Update de valores vDisplay");
      
    vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_PERIOD_MS));
  }
}
 
void ventoinhaBotao()
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xSemaphoreGiveFromISR(botaoSemafro, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);
}
 
void  contagemRevolucao()
{
  static signed portBASE_TYPE xHigherPriorityTaskWoken;
  xSemaphoreGiveFromISR(rodaSemafro, (signed portBASE_TYPE *)&xHigherPriorityTaskWoken);
 
}
 
void loop()
{
  vTaskDelete(NULL);
}
