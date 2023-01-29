#include <Arduino.h>
#include <BluetoothSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>

LiquidCrystal lcd(13, 12, 14, 27, 26, 25);
BluetoothSerial SerialBT;
// GPIO donde el sensor DS18B20 está conectado
const int oneWireBus = 4;
const int sensorNivel = 32;
const int botonManual = 23;
const int botonAutomatico = 34;
const int bomba = 5;
const int ledAuto = 18;
const int ledManual = 19;
const int pHpin = 35;
int encendido = 0;
float Nivel_Agua;
float temperatura;
float PH;

unsigned long currentTime = millis();// Current time
unsigned long previousTime = 0; // Previous time
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
OneWire oneWire(oneWireBus); // configura la comunicacion onewire
DallasTemperature sensors(&oneWire);// para el onewire a la libreia de DallasTemerature

void TaskBlink1(void *pvParameters) {
  
  for (;;) { // A Task shall never return or exit.
   
     // Condiones de Notificacion
  String Alarma = "";
  String condicion ="F";
  if (Nivel_Agua>2.5 || Nivel_Agua<0){
    Alarma=Alarma+"Revisar Nivel de agua";
    condicion="V";
  }

  if (temperatura>28 || temperatura<20)
  { Alarma=Alarma+"Revisar Temperatura";
    condicion="V";
  }

  if (PH>9 || PH<6)//ingresar condicion para envio de mensaje
  { Alarma=Alarma+"Revisar PH";
    condicion="V";
  }
    // Mostrar en conexion serial - Tambien lo usamos para la APP
  SerialBT.print(String(temperatura));
  SerialBT.print(String("|"));
  SerialBT.print(PH);
  SerialBT.print(String("|"));
  SerialBT.print(Nivel_Agua);
  SerialBT.print(String("|"));
  SerialBT.print(Alarma);
  SerialBT.print(String("|"));
  SerialBT.print(condicion);
  SerialBT.print(String("|"));
   
  vTaskDelay(5000 / portTICK_PERIOD_MS );
    
  }
}

void setup(){
  lcd.begin(16, 2);
  Serial.begin(115200);
  SerialBT.begin("Monitoreo_ESP32"); // nombre del dispositivo 

  // inicia el sensor DS18B20
  sensors.begin();
  pinMode(bomba, OUTPUT);
  pinMode(ledAuto, OUTPUT);
  pinMode(ledManual, OUTPUT);
  pinMode(botonManual, INPUT);
  pinMode(botonAutomatico, INPUT);


  xTaskCreate( //creacion de la tarea
  TaskBlink1, //nombre de la función
  "Blink1", // nombre de la tarea
  2048, //numero de para su uso como pila. 
  NULL,
  1, //prioridad 1
  NULL
  );
}

void loop(){
  // Nivel de agua
  int valorSensorNivel = analogRead(sensorNivel); // lectura en bit
  if (valorSensorNivel < 1000){
    Nivel_Agua = 0.0;
  }
  else{
    Nivel_Agua = (4.0 / 1000.0) * (double(valorSensorNivel) - 1000.0);
  }
  if (digitalRead(botonAutomatico) == HIGH){
    delay(600);
    encendido = !encendido;
  }
  digitalWrite(ledAuto, encendido);
  digitalWrite(ledManual, !encendido);

  if (digitalRead(botonManual) == HIGH && encendido == 0){

    if (valorSensorNivel <= 2000)
    {
      digitalWrite(bomba, HIGH);
    }
    else
    {
      digitalWrite(bomba, LOW);
    }
  }
  else{
    digitalWrite(bomba, LOW);
  }

  if (encendido == 1){//control automatico de la bomba
    if (valorSensorNivel < 1600)
    {
      digitalWrite(bomba, HIGH);
    }
    else{
      digitalWrite(bomba, LOW);
    }
  }
  // Lectura del sensor PH
  PH = (14.0*analogRead(pHpin)/4095.0); // Lee y escala los valores ente 0-14.
  sensors.requestTemperatures(); // temperatura
  temperatura = sensors.getTempCByIndex(0); // temperatura en °C
  
  // MOSTAR EL EL LCD
  lcd.setCursor(0, 0);
  lcd.print("T(C):" + String(temperatura) + "");
  // delay(500);
  lcd.setCursor(0, 1);
  lcd.print("H:" + String(Nivel_Agua) + " PH:" + String(PH) + "");
  void TaskBlink1(void *pvParameters);
}

