/*
Алгоритм программы обеспечения эксперимента метелица.
*/

#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>
#include <TroykaIMU.h>

int count;

File myFile;
int sound_flag=0;
int camera_counter=0;
int i=0;
int exit_time=2*60*60;
Barometer barometer;

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);

  pinMode(7, OUTPUT);
  digitalWrite(7,LOW);
  
  while (!SD.begin(8));//Ждём карту
  SD.remove("LOG.txt");
  
  barometer.begin();

  
  delay(300);
  digitalWrite(7,HIGH);
  count=0;
  Serial.println("ok");
 //Если всё ок, то будет 5В на 7 ноге
}

void loop() // run over and over
{ 
  String temp; 
  if (Serial1.available()>0) { 
    Serial1.setTimeout(5);
    temp=Serial1.readString();
    myFile = SD.open("LOG.txt", FILE_WRITE); 
    if (myFile) {
      myFile.print(temp);
      if(count%4){
        // создаём переменную и присваиваем ей значения абсолютного давления
        float pressure = barometer.readPressureMillibars();
        // создаём переменную и присваиваем ей значения высоты над уровнем море
        float altitude = barometer.pressureToAltitudeMeters(pressure);
        // создаём переменную и присваиваем ей температуру окружающей среды
        float temperature = barometer.readTemperatureC();

        myFile.println("pres="+String(pressure)+", alt="+String(altitude)+", tmp="+String(temperature));
        count=0;
      } else
        myFile.println(" ");
      myFile.close();
    }
  }
  delay(500);
      
}

