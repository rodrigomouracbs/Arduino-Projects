#include <SPI.h> // datalogger library
#include <SD.h> // datalogger library
#include <Wire.h> // DHT library
#include <DS3231.h> //clock library
#include <SimpleTimer.h>
#include <LinkedList.h>
#include <Gaussian.h>
#include <GaussianAverage.h> //useful for stabilizing pressure transducer readings

//The entire code was developed for the automation of eight irrigation sectors as shown in: http://www.ambi-agua.net/seer/index.php/ambi-agua/article/view/2245
//First group of averages for automation program
GaussianAverage T1S1 (10);
GaussianAverage T2S1 (10);
GaussianAverage T3S1 (10);
GaussianAverage T4S1 (10);
GaussianAverage T1S2 (10);
GaussianAverage T2S2 (10);
GaussianAverage T3S2 (10);
GaussianAverage T4S2 (10);
//Second group of averages for DataLog program
GaussianAverage T1S1r (10);
GaussianAverage T2S1r (10);
GaussianAverage T3S1r (10);
GaussianAverage T4S1r (10);
GaussianAverage T1S2r (10);
GaussianAverage T2S2r (10);
GaussianAverage T3S2r (10);
GaussianAverage T4S2r (10);

//Variables for solenoid valves and pump irrigation activation
int rele1 = 43; //T4S1
int rele2 = 41; //T3S1
int rele3 = 39; //T2S1
int rele4 = 37; //T1S1
int rele5 = 42; //T4S2
int rele6 = 40; //T3S2
int rele7 = 38; //T2S2
int rele8 = 36; //T1S2
int releBomba = 27; //pump
const int chipSelect = 53; //datalogger CS (Chip Select)
File dataFile;

// the timer object
DS3231 clock; 
RTCDateTime dt; 
SimpleTimer irriga;

//Variables for time irrigation programming (seconds)
int tempoIrriga1 = 0; //T4S1
int tempoIrriga2 = 0; //T3S1
int tempoIrriga3 = 0; //T2S1
int tempoIrriga4 = 0; //T1S1
int tempoIrriga5 = 0; //T4S2
int tempoIrriga6 = 0; //T3S2
int tempoIrriga7 = 0; //T2S2
int tempoIrriga8 = 0; //T1S2
void ligaIrriga();
void grava();
void espera();
unsigned long tempoA = 0;
unsigned long tempoB = 0;
bool acionarA = false;
bool acionarB = false;

void setup() {
  Serial.begin(9600); /*Inicia a comunicação com a Serial em 9600*/
  Wire.begin();
  irriga.setInterval(6000, ligaIrriga);
  Serial.println("Starting Reading");
  clock.begin();
  analogReference(DEFAULT);
  clock.setDateTime(__DATE__, __TIME__);
  dt = clock.getDateTime();
  SD.begin(chipSelect);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(rele1, OUTPUT);
  pinMode(rele2, OUTPUT);
  pinMode(rele3, OUTPUT);
  pinMode(rele4, OUTPUT);
  pinMode(rele5, OUTPUT);
  pinMode(rele6, OUTPUT);
  pinMode(rele7, OUTPUT);
  pinMode(rele8, OUTPUT);
  pinMode(releBomba, OUTPUT);
  digitalWrite(rele1, HIGH);
  digitalWrite(rele2, HIGH);
  digitalWrite(rele3, HIGH);
  digitalWrite(rele4, HIGH);
  digitalWrite(rele5, HIGH);
  digitalWrite(rele6, HIGH);
  digitalWrite(rele7, HIGH);
  digitalWrite(rele8, HIGH);
  digitalWrite(releBomba, HIGH);
}
void loop() {
  irriga.run();/*Função que inicia o processo da biblioteca SimpleTimer*/
  grava();
  espera();
}
//Variables for critical matric potentials used for irrigation automation
const int VT4S1 = 1;
const int VT3S1 = 1;
const int VT2S1 = 1;
const int VT1S1 = 1;
const int VT1S2 = 1;
const int VT2S2 = 1;
const int VT3S2 = 1;
const int VT4S2 = 1;

//Automation Program
void ligaIrriga() {
  if (acionarA) {
    int  v1 = analogRead(A0);
    delay(300);
    int  v2 = analogRead(A1);
    delay(300);
    int  v3 = analogRead(A2);
    delay(300);
    int  v4 = analogRead(A8);
    delay(300);
    int  v5 = analogRead(A9);
    delay(300);
    int  v6 = analogRead(A5);
    delay(300);
    int  v7 = analogRead(A6);
    delay(300);
    int  v8 = analogRead(A7);
    delay(300);
    float value1 = (((v1 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value2 = (((v2 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value3 = (((v3 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value4 = (((v4 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value5 = (((v5 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value6 = (((v6 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value7 = (((v7 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    float value8 = (((v8 / 1023.0) - 0.04) / 0.0012585);
    delay(300);
    T1S1 += value1;
    T1S1.process();
    T2S1 += value2;
    T2S1.process();
    T3S1 += value3;
    T3S1.process();
    T4S1 += value4;
    T4S1.process();
    T1S2 += value5;
    T1S2.process();
    T2S2 += value6;
    T2S2.process();
    T3S2 += value7;
    T3S2.process();
    T4S2 += value8;
    T4S2.process();
    /*####### SECTOR 1 T4S1 #######*/
    if (T4S1.mean > VT4S1) {
      digitalWrite(rele1, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga1; i++) {
        delay(1000);
      }
      digitalWrite(rele1, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 2 T3S1 #######
    if (T3S1.mean > VT3S1) {
      digitalWrite(rele2, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga2; i++) {
        delay(1000);
      }
      digitalWrite(rele2, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 3 T2S1 #######
    if (T2S1.mean > VT2S1) {
      digitalWrite(rele3, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga3; i++) {
        delay(1000);
      }
      digitalWrite(rele3, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 4 T1S1 #######
    if (T1S1.mean > VT1S1) {
      digitalWrite(rele4, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga4; i++) {
        delay(1000);
      }
      digitalWrite(rele4, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 5 T4S2 #######
    if (T4S2.mean > VT4S2) {
      digitalWrite(rele5, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga5; i++) {
        delay(1000);
      }
      digitalWrite(rele5, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 6 T3S2 #######
    if (T3S2.mean > VT3S2) {
      digitalWrite(rele6, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga6; i++) {
        delay(1000);
      }
      digitalWrite(rele6, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 7 T2S2 #######
    if (T2S2.mean > VT2S2) {
      digitalWrite(rele7, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga7; i++) {
        delay(1000);
      }
      digitalWrite(rele7, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    //####### SECTOR 8 T1S2 #######
    if (T1S2.mean > VT1S2) {
      digitalWrite(rele8, LOW);
      digitalWrite(releBomba, LOW);
      for (int i = 0; i < tempoIrriga8; i++) {
        delay(1000);
      }
      digitalWrite(rele8, HIGH);
      digitalWrite(releBomba, HIGH);
    }
    tempoA = millis();
    acionarA = false;
  }
}// end of automation program

//DataLog Program
void grava() {
  if (acionarB) {
    if (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      return;
    }
    float  v1 = analogRead(A0);
    delay(300);
    float  v2 = analogRead(A1);
    delay(300);
    float  v3 = analogRead(A2);
    delay(300);
    float  v4 = analogRead(A8);
    delay(300);
    float  v5 = analogRead(A9);
    delay(300);
    float  v6 = analogRead(A5);
    delay(300);
    float  v7 = analogRead(A6);
    delay(300);
    float  v8 = analogRead(A7);
    delay(300);
    //Application of calibration equation for converting pressure transducer readings in kPa
    float value1a = (((v1 / 1023.0) - 0.04) / 0.0012585);
    float value1 =((0.9995*value1a)+2.9885);
    delay(300);
    float value2a = (((v2 / 1023.0) - 0.04) / 0.0012585);
    float value2 =((0.9995*value2a)+2.9885);
    delay(300);
    float value3a = (((v3 / 1023.0) - 0.04) / 0.0012585);
    float value3 =((0.9995*value3a)+2.9885);
    delay(300);
    float value4a = (((v4 / 1023.0) - 0.04) / 0.0012585);
    float value4 =((0.9995*value4a)+2.9885);
    delay(300);
    float value5a = (((v5 / 1023.0) - 0.04) / 0.0012585);
    float value5 =((0.9995*value5a)+2.9885);
    delay(300);
    float value6a = (((v6 / 1023.0) - 0.04) / 0.0012585);
    float value6 =((0.9995*value6a)+2.9885);
    delay(300);
    float value7a = (((v7 / 1023.0) - 0.04) / 0.0012585);
    float value7 =((0.9995*value7a)+2.9885);
    delay(300);
    float value8a = (((v8 / 1023.0) - 0.04) / 0.0012585);
    float value8 =((0.9995*value8a)+2.9885);
    delay(300);
    T1S1r += value1;
    T1S1r.process();
    delay(300);
    T2S1r += value2;
    T2S1r.process();
    delay(300);
    T3S1r += value3;
    T3S1r.process();
    delay(300);
    T4S1r += value4;
    T4S1r.process();
    delay(300);
    T1S2r += value5;
    T1S2r.process();
    delay(300);
    T2S2r += value6;
    T2S2r.process();
    delay(300);
    T3S2r += value7;
    T3S2r.process();
    delay(300);
    T4S2r += value8;
    T4S2r.process();
    delay(300);
    dt = clock.getDateTime();
    String dataString = "";
    dataString += (dt.year); 
    dataString += ("/");
    dataString += (dt.month);
    dataString += ("/");
    dataString += (dt.day);
    dataString += ('\t');
    dataString += (dt.hour);
    dataString += (":");
    dataString += (dt.minute);
    dataString += (":");
    dataString += (dt.second);
    dataString += ('\t');
    dataString += (" Temp: ");
    int sensor = clock.readTemperature();
    dataString += String(sensor);
    dataString += ('\t');
    dataString += ("T1S1: ");
    dataString += String(T1S1r.mean);
    dataString += ('\t');
    dataString += ("T2S1: ");
    dataString += String(T2S1r.mean);
    dataString += ('\t');
    dataString += ("T3S1: ");
    dataString += String(T3S1r.mean);
    dataString += ('\t');
    dataString += ("T4S1: ");
    dataString += String(T4S1r.mean);
    dataString += ('\t');
    dataString += ("T1S2: ");
    dataString += String(T1S2r.mean);
    dataString += ('\t');
    dataString += ("T2S2: ");
    dataString += String(T2S2r.mean);
    dataString += ('\t');
    dataString += ("T3S2: ");
    dataString += String(T3S2r.mean);
    dataString += ('\t');
    dataString += ("T4S2: ");
    dataString += String(T4S2r.mean);
    dataFile = SD.open("DATALOG.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString); 
      dataFile.close();
    }
    Serial.println(dataString);
    tempoB = millis();
    acionarB = false;
  }
}//end of DataLog Program

//Define the time span execution for automation and DataLog programs (milliseconds)
void espera() {
  if (!acionarA && millis() - tempoA >= 3000000000) {
    acionarA = true;//40 minutos.
  }
  if (!acionarB && millis() - tempoB >= 15000) {
    acionarB = true; //10 segundos.
  }
}
