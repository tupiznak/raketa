/*
Пакет прошивок для эксперимента "Метелица": прошивка МК1.
*/

// БИБЛИОТЕКИ ------------------------------------------------------
// Для датчика бибилиотека I2C
#include <Wire.h>

// Библиотеки датчика GY-86
#include <MPU6050.h>     // Процессор
#include <HMC5883L.h>    // Компас
#include <MS5611.h>      // Барометр

// Программный UART (резерв)
#include <SoftwareSerial.h>

// ДЕФАЙНЫ ---------------------------------------------------------
#define DETECTOR  16        // Пин к детектору
#define LIGHT     7         // Пин мощного светодиода
#define ENGINE_1  5         // Пин к реле двигателя
#define ENGINE_2  6         // Пин к пережигателю подвеса
#define PARACHUTE 10        // Пин к реле парашюта

// ПЕРЕМЕННЫЕ ЭКСПЕРИМЕНТА -----------------------------------------
// 1) Датчики
MPU6050  mpu       ;        // Процессор GY-86
HMC5883L compass   ;        // Компасс GY-86
MS5611   ms5611    ;        // Барометр GY-86
    
Vector   vNormGyro ;        // Данные с ДУС, нормированные
Vector   vNormAccel;        // Данные с акселерометра, нормированные
int      nTemp     ;        // Данные с датчика температуры
Vector   vMag      ;        // Данные с компаса
long     lnAtmPRSS ;        // Данные с барометра
int      nAbsALT   ;        // Данные о высоте
int      nStartALT ;        // Высота старта
int      nTankPRSS ;        // Данные с датчика давления в баке

// ПЕРЕМЕННЫЕ СРЕДЫ ----------------------------------------------------------------------------------------------------------------------------------
// 1) Программные последовательные порты
SoftwareSerial mySerial(9, 8); // RX, TX

// 3) Таймеры и счетчики
float fTimer            =0;    // Таймер отсчета времени, для лога времени и нижнего предела
float fBackup_timer     =0;    // Таймер резервного включения двигателя, в случае отказа датчика (верхний предел)
float fTelem_timer      =0;    // Таймер длительности записи телеметрии
float fPAR_timer        =0;    // Таймер задержки раскрытия парашюта
float fENG_ON_timer     =0;    // Таймер отключения двигателя
float fENG_DELAY_timer  =0;    // Таймер задержки включения двигателя

float fFreq             =200;  // Счетчик циклов, время цикла ОСРВ, мс
int   nLight_count      =0;    // Счетчик для мигания мощным светодиодом
int   nDelay_Data_count =0;    // Счетчик отсчета для отправки данных давления.                        
int   nMSG              =0;    // Счетчик для печати слов поясняющих этап миссии в телеметрию
int   nCount            =0;    // Счетчик номера замера данных с датчиков
unsigned long buf       =0;    // Буфер для контроля времени цикла


// 4)Временные контрольные точки миссии
// ДЛЯ ОТЛАДКИ:
const int   TT             =5;    // Время записи телеметрии после раскрытия парашюта, с
const int   LL             =5;    // Время задержки, для анализа высоты по барометру,  с
const int   UL             =5;    // Время задержки, если барометр не отвечает,        с
const int   PAR_DELAY      =2;    // Задержка для парашюта,                            с         
const int   ENG_ALT        =1;    // Высота включения двигателя,                       м
const int   ENG_ON_TIME    =3;    // Задержка отключения двигла в камере,              c
const int   ENG_DELAY_TIME =5;    // Задержка включения двигла в камере,               c


/*
// 4)Временные контрольные точки миссии
// ДЛЯ БАРОКАМЕРЫ:
const int   TT             =15;   // Время записи телеметрии после раскрытия парашюта, с
const int   LL             =5*60; // Время задержки, для анализа высоты по барометру,  с
const int   UL             =5;    // Время задержки, если барометр не отвечает,        с
const int   PAR_DELAY      =10;   // Задержка для парашюта,                            с         
const int   ENG_ALT        =25000;// Высота включения двигателя,                       м
const int   ENG_ON_TIME    =15;   // Задержка отключения двигла в камере,              c
const int   ENG_DELAY_TIME =60;   // Задержка включения двигла в камере,               c
*/

/*
// 4)Временные контрольные точки миссии !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                                                             
// ДЛЯ РАБОТЫ:
const int   TT              =10;   // Время работы нагревателя парашюта,                с 
const int   LL              =20*60;// Время задержки, для анализа высоты по барометру,  с
const int   UL              =5;    // Время задержки, если барометр не отвечает,        с
const int   PAR_DELAY       =2;    // Задержка для парашюта,                            с         
const int   ENG_ALT         =25000;// Высота включения двигателя,                       м
const int   ENG_ON_TIME     =20;   // Время работы двигла в камере,                     c
const int   ENG_DELAY_TIME  =60;   // Задержка включения двигла в камере,               c
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/

// 5) Флаги
boolean     LTL=false        ;      // Фиксация готовности по таймеру
boolean     EXP_ALT_DT=false ;      // Фиксация высоты эксперимента
boolean     ENG_READY=false  ;      // Готовность к запуску движка
boolean     DEPLOY_DT=false  ;      // Фиксация отделения
boolean     EXPMT=false      ;      // Эксперимент
boolean     FNL=false        ;      // Финал миссии
boolean     ENG_DONE=false   ;      // Двигатель можно выключить
boolean     PAR_DONE=false   ;      // Парашют отработал
boolean     BLNK=false       ;      // Когда мигать
void (* resetFunc)  (void) =0;      //Перезагрузка с планшета
//***************************************************************************************************************************************************

void setup() {
// 1) Инициализация последовательных портов  
 Serial1.begin(9600);
 mySerial.begin(9600);

// 2) Инициализация датчиков
// 2.1) Инициализация гироскопа и акселерометра (процессора MPU6050)
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G)) {
   printMultiOut("Could not find a valid MPU6050 sensor, check wiring!\n");
   delay(500);
  }
  
// 2.2) Калибровка гироскопа. Калибровка должна проходить в покое.
  mpu.calibrateGyro();

// 2.3) Установка чувствительности. По умолчанию 3.
  mpu.setThreshold(3);  
      
// 2.4) Инициализация компаса
// Включение мастера для компаса(bypass mode)
  bypass_mode(true);
  while (!compass.begin()) {
    delay(500);
  }
  compass.setRange(HMC5883L_RANGE_8_1GA);         // Установка пределов измерения
  compass.setMeasurementMode(HMC5883L_CONTINOUS); // Режим измерения
  compass.setDataRate(HMC5883L_DATARATE_30HZ);    // Частота данных
  compass.setSamples(HMC5883L_SAMPLES_8);         // Set number of samples averaged   

// 2.5) Инициализация барометра
  while(!ms5611.begin()) {
   printMultiOut("Could not find a valid MS5611 sensor, check wiring!\n");
   delay(500);
  }    

// 3) Инициализация пинов
  pinMode(LIGHT, OUTPUT);             
  pinMode(ENGINE_1, OUTPUT);
  pinMode(ENGINE_2, OUTPUT); 
  pinMode(PARACHUTE, OUTPUT);
  pinMode(DETECTOR, INPUT); 
  digitalWrite(LIGHT, LOW);     // Мощный светодиод, транзистор Дарлингтона      
  digitalWrite(ENGINE_1, LOW);  // Реле 1 двигателя          
  digitalWrite(ENGINE_2, LOW);  // Реле 2 двигателя         
  digitalWrite(PARACHUTE, LOW); // Реле парашюта          
  digitalWrite(DETECTOR, HIGH); // Вход детектора отцепа
  
  delay(2000);                  // Задержка для человека за планшетом
  // 4) Отладочные строчки
  while(!mySerial.available());
  printMultiOut("> System ready > DO YOU WANT TO START [Y/N]?: \n");
  
  // Пока не придет команда "y" программа дальше не идет:
  String tmp="";
  while (find_text("y",tmp)==-1) {
    tmp = mySerial.readString();
  }
  printMultiOut("> Started. \n");
       
  // Отметить высоту старта (торировка):
  nStartALT=ms5611.getAltitude(ms5611.readPressure());
  
  printMultiOut("> TIMER STARTED > Powerlight BLINK > WAIT ["+String(LL)+" sec]\n");
}
  //.......................................................................................................................
  
void loop() {// ОСНОВНОЙ ЦИКЛ ПРОГРАММЫ
  // 0) Контроль времени цикла
  buf = millis();
  
  // УСТАНОВКА ФЛАГОВ МИССИИ-----------------------------------------------------------------------
  // 1) флаг начала анализа данных высоты
  if (fTimer>LL&&FNL==false) {                //  Если таймер (fTimer) отсчитал время LL, то необходимо
    LTL=true;EXPMT=true;                  //  запустить программу эксперимента метелица
  }                                    
  // Убираем верхний предел
  // 2) флаг готовности по высоте
  // ДЛЯ ОТЛАДКИ (с верхним пределом):
  if ( (nAbsALT>ENG_ALT) | (fBackup_timer>UL) ) {  // Анализ высоты (24 км)с датчика (необходимо торир. отн. ур.моря)
    EXP_ALT_DT=true;                     // Если условие долго не срабат.
  }                                      // то запустить без него: fBackup_timer
     
  
  /* 
  // 2) флаг готовности по высоте !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // ДЛЯ РАБОТЫ (без верхнего предела):
  if (nAbsALT>ENG_ALT)                       // Анализ высоты (24 км)с датчика
      {                                      // (необходимо торир. отн. ур.моря)
        EXP_ALT_DT=true;                       
      }  
  */ 
                                                     
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  
  // 3) флаг фиксации разделения
  if (digitalRead(DETECTOR)==LOW) {                                               
    DEPLOY_DT=true;
   }
  //-----------------------------------------------------------------------------------------------
  
  //УСЛОВИЯ
  boolean ENG_OK=EXP_ALT_DT&&LTL&&!ENG_DONE&&EXPMT;
  boolean PAR_OK=DEPLOY_DT&&EXPMT&&ENG_READY;         
  // ENG_READY - чтобы не открылось раньше времени если сделать без него, то тогда сможет открываться по НШС
  // Приняты меры  предосторожности, чтобы не бахнуло раньше времени через флаг EXPMT==true
  
  // ОСНОВНОЙ АЛГОРИТМ ****************************************************************************
  // 4) Ждем высоты для запуска двигателя
  //    Запускаем двигатель
   if (ENG_OK) {
    m_ENGINE();
   }
  
  // 5) Запускаем парашют, отключаем двигатели
   if (PAR_OK) {                                                          
    m_PAR();                                                   
   }                                                           
      
  // 6) ТЕЛЕМЕТРИЯ ПО ФЛАГОВЫМ СИГНАЛАМ-----------------------------------------------------------------------------------   
   if (FNL==false) {   
     if (EXP_ALT_DT==false) {
      if (nDelay_Data_count % 5==0) {  
        get_data();
        send_data_b(); 
      }   
     } 
      else {
        get_data();
        send_data_b(); 
      }   
   }
   else  {
     if (nDelay_Data_count % 5==0)  {  
       get_data();
       send_data_b(); 
     } 
   }   
  // Сообщение в телеметрию о достижении нижнего предела времени
   if (LTL==true&&EXPMT==true)  {
       // 2.1) Вставить строчку в телеметрию
       if (nMSG==0)  {       
          printMultiOut("> TIMER COMPLETE > WAIT UNTILL ["+String(ENG_ALT)+" m]\n");
          nMSG++;
       }           
       /* Убираем верхний предел
       // 2.2) Резервный таймер для запуска двигателя в случае отказа баромера (верхний предел)
       fBackup_timer+=fFreq/1000;
       */
      } 
//----------------------------------------------------------------------------------------------------------------------
   
// 7) Контроль световой  индикации
//    Индикация миссии мощным светодиодом       
/*    Доступные флаги:
boolean     LTL=false        ;      // Фиксация готовности по таймеру
boolean     EXP_ALT_DT=false ;      // Фиксация высоты эксперимента
boolean     ENG_READY=false  ;      // Готовность к запуску движка
boolean     DEPLOY_DT=false  ;      // Фиксация отделения
boolean     EXPMT=false      ;      // Эксперимент
boolean     FNL=false        ;      // Финал миссии
boolean     ENG_DONE=false   ;      // Двигатель можно выключить
boolean     PAR_DONE=false   ;      // Парашют отработал
boolean     BLNK=false       ;      // Когда мигать
*/         
   if (FNL==false) {                 // МИГАТЬ ВСЕ ВРЕМЯ С НАЧАЛА МИССИИ     
       if (BLNK==false)  {           // ПОКА НЕ ВКЛЮЧИТСЯ ДВИГАТЕЛЬ
           power_light("blink");  
       } 
       else {                        // КОГДА ВКЛЮЧИТСЯ ДВИГАТЕЛЬ ГОРЕТЬ ВСЕ ВРЕМЯ
           power_light("on");  
       }   
   }
   else {                            // ПОСЛЕ РАСКРЫТИЯ ПАРАШЮТА МИГАТЬ БЫСТРО
       power_light("fast blink"); 
   }

// 8) Экстренное отключение всех систем командой с планшета
  if (mySerial.available()>0) {
    if (find_text("stop",mySerial.readString())!=-1) {
     digitalWrite(LIGHT, LOW);
     digitalWrite(ENGINE_1, LOW);
     digitalWrite(ENGINE_2, LOW);
     digitalWrite(LIGHT, LOW);
     printMultiOut("> ALL SYSTEMS OFF > PROGRAM IS STOPPED!\n");
     while(1) {// Ловушка программы в бесконечный цикл
      if(mySerial.available()) {
        String tmp;
        tmp=mySerial.readString();
        if (find_text("reset",tmp)!=-1) {
          resetFunc();
        }
      }
     }
    }
 }
   
// 9) Работа системы, таймеры и счетчики
// ------------------------------------------------------------------------------
//    Контроль времени, счетчики
//    - Строгий отсчет времени цикла fFreq=200 мс - время цикла
//    - Время для выполнения алгоритма (millis()-buf) порядка 50 мс (по факту 25 мс)
//    - fFreq=200 мс взято с запасом и учетом записи на microSD карту
// ------------------------------------------------------------------------------
   // ПРОЦЕДУРЫ ДЛЯ РАБОТЫ СИСТЕМЫ
   // 1) Ждать до 200 мс, строго соблюдать время периода
   float wait=fFreq - ((millis() - buf));
   delay(wait);
   // 2) Отсчет времени
   fTimer+=fFreq/1000;
   // 3) Счетчики
   nDelay_Data_count++;
   nCount++;
   nLight_count++; 
//**************************************************************************************************

}//  ПРОГРАММА ЗАВЕРШЕНА!

//**********************************************************************************
//                                      ФУНКЦИИ
//**********************************************************************************
// 1) Перключение режимов Master-Slave между датчиками датчика GY-86
void bypass_mode(boolean state)
{
  if (state == true) { 
      mpu.setI2CMasterModeEnabled(false);
      mpu.setI2CBypassEnabled(true);
      mpu.setSleepEnabled(false);
  }
  else {
      mpu.setI2CMasterModeEnabled(true);
      mpu.setI2CBypassEnabled(false);
      mpu.setSleepEnabled(false); 
   }
}

// 2) Сбор данных с датчиков
void get_data()
{
  // Считывание акселерометра и гироскопа
  bypass_mode(false); 
  vNormGyro = mpu.readNormalizeGyro();
  vNormAccel = mpu.readNormalizeAccel();  
  // Считывание компаса
  bypass_mode(true);
  vMag = compass.readNormalize();
  // Считывание давления
  lnAtmPRSS = ms5611.readPressure();
  // Расчет высоты
  nAbsALT = ms5611.getAltitude(lnAtmPRSS)-nStartALT;
  // Считывание температуры
  nTemp = mpu.readTemperature();
  // Считывание давления в баке
  nTankPRSS = (analogRead(A0));
}

// 3) Печать данных в порт ПК
void send_data_b()
{// ДАННЫЕ С ДАТЧИКОВ:                  ПРОБЕЛЫ-РАЗДЕЛИТЕЛИ:
  printMultiOut(nCount);                  printMultiOut(" ");    
  printMultiOut(fTimer);                  printMultiOut("\t|\t"); 
  printMultiOut(vNormGyro.XAxis);         printMultiOut(" ");
  printMultiOut(vNormGyro.YAxis);         printMultiOut(" ");
  printMultiOut(vNormGyro.ZAxis);         printMultiOut("\t|\t"); 
  printMultiOut(vNormAccel.XAxis);        printMultiOut(" ");
  printMultiOut(vNormAccel.YAxis);        printMultiOut(" ");
  printMultiOut(vNormAccel.ZAxis);        printMultiOut("\t|\t");
  printMultiOut(vMag.XAxis);              printMultiOut(" ");
  printMultiOut(vMag.YAxis);              printMultiOut(" ");
  printMultiOut(vMag.ZAxis);              printMultiOut("\t|\t");
  if (nDelay_Data_count % 20==0) {
      printMultiOut(lnAtmPRSS);           printMultiOut(" ");
      printMultiOut(nTankPRSS);           printMultiOut(" ");
      printMultiOut(nAbsALT);             printMultiOut(" ");
      printMultiOut(nTemp);             
   }
   printMultiOut("\n");
}

// ------------------------------------------------------------------------------
// 4) Режимы работы мощного светодиода
//    - Задержки организованы на счетчике nLight_count
//    - nLight_count % 7 ==0 - означает каждый 7ой счет (остаток от деления на 7 не 0)
// ------------------------------------------------------------------------------
void power_light(String mode) { // МИГАТЬ
  if (mode=="blink") {
   if (nLight_count % 10 ==0) {
     digitalWrite(LIGHT, HIGH);
     delay(10);
     digitalWrite(LIGHT, LOW);
   }
   else {
     digitalWrite(LIGHT, LOW);
   } 
  }          
   // БЫСТРО МИГАТЬ
   if (mode=="fast blink") {
     if (nLight_count % 4 ==0) {
         digitalWrite(LIGHT, HIGH);
         delay(10);
         digitalWrite(LIGHT, LOW);
     }
     else {
       digitalWrite(LIGHT, LOW);
     }
  }
  // ГОРЕТЬ ВСЕ ВРЕМЯ
   if (mode=="on") {
     digitalWrite(LIGHT, HIGH);
    }      
}

// ------------------------------------------------------------------------------
// 6) Обрабатываем двигатель по достижению высоты и ожидания задержки.
//    - Задержка: fENG_DELAY_timer>ENG_DELAY_TIME
//    - Выставляем флаги: ENG_READY=true
//                        BLNK=true 
//    - Двигатель работает время fENG_ON_timer>ENG_ON_TIME
//    - Далее двигатель выключается и взводятся флаги: ENG_DONE=true
//                                                     BLNK=false
// ------------------------------------------------------------------------------
void m_ENGINE() {           
  //вставить строчку в телеметрию
   if (nMSG==1) {
     if (nAbsALT>ENG_ALT) {
          printMultiOut("> Altitude "+String(ENG_ALT)+"m reached\n");
     }
     /* Убираем верхний предел
     if (fBackup_timer>UL)
         {                
          Serial.println("> Altitude ERROR! > Backup timer situation");
         } 
     */    
     nMSG++;
     printMultiOut("> Preparing engine ["+String(ENG_DELAY_TIME)+" sec]\n");
   }
    // Сделаем задержку минута, для того чтобы успеть отключить насос 
    // Эта задержка пригодится для анализа телеметрии до включения                    
    fENG_DELAY_timer+=fFreq/1000;
    if (fENG_DELAY_timer>ENG_DELAY_TIME) {
       // 3.2) Включить обе обмотки двигателя
       digitalWrite(ENGINE_1, HIGH);
       digitalWrite(ENGINE_2, HIGH);
       ENG_READY=true;
       BLNK=true; 
        if (nMSG==2) { 
           printMultiOut("> ENGINE IS ON > Powerlight ON > WAIT ["+String(ENG_ON_TIME)+" sec]\n");
           nMSG++;
        }
       
       // Временная задержка отключения двигателя, для испытания
       // в барокамере (10 сек)
       // После откл. выставляется флаг: ENG_DONE              
       // Таймер для задержки отключения двигателя (испытания в барокамере)
        fENG_ON_timer+=fFreq/1000;        
        if (fENG_ON_timer>ENG_ON_TIME) {
          digitalWrite(ENGINE_1, LOW);
          digitalWrite(ENGINE_2, LOW);
          ENG_DONE=true;
          BLNK=false;
          printMultiOut("> ENGINE IS OFF > WAITING DEPLOY\n");
        }
    }
}


// ------------------------------------------------------------------------------
// 7) При отделении после задержки расркыть парашют, отключить двигатель (резерв)
//    - Задержка fPAR_timer>PAR_DELAY
//    - Выставляются флаги: EXPMT=false;
//                          FNL=true; 
//    - Пережигатель парашюта работает время fTelem_timer>TT
// ------------------------------------------------------------------------------
void m_PAR()  {
  // Вставить строчку в телеметрию
  if (nMSG==3)  {
      printMultiOut("> Deploy detected > WAIT ["+String(PAR_DELAY)+" sec]\n");
      nMSG++;
  }     
  // 4.1) После получения сигнала включить таймер на 10 секунд
  fPAR_timer+=fFreq/1000;     
  if (fPAR_timer>PAR_DELAY) {  
    if (nMSG==4) {                 
        printMultiOut("> Parachute enabled > ENGINE IS OFF > WAIT ["+String(TT)+" sec]\n");
        nMSG++;
    }     
    // 4.2) Включить парашют, отключить двигатель и пережигатель
    digitalWrite(PARACHUTE, HIGH);
    digitalWrite(ENGINE_1, LOW);
    digitalWrite(ENGINE_2, LOW);
    fTelem_timer+=fFreq/1000; 
    // 4.4) Спустя время TT после раскрытия парашюта
    if (fTelem_timer>TT) { 
       // Выключаем парашют 
       digitalWrite(PARACHUTE, LOW);                
       printMultiOut("> PAR IS OFF > EXP DONE > Powerlight BLINK\n");
       // Переходим в режим записи телеметрии 1 раз в 2 секунды
       EXPMT=false;
       FNL=true;
     }
   }
 }


int find_text(String needle, String haystack) {
  int foundpos = -1;
  if (haystack.length()) {
    for (int i = 0; i <= haystack.length() - needle.length(); i++) {
      if (haystack.substring(i,needle.length()+i) == needle) {
        foundpos = i;
      }
    }
  }
  return foundpos;
}

void printMultiOut(String str){  
  Serial1.print(str);
  mySerial.print(str);
}

void printMultiOut(float str){  
  Serial1.print(str);
  mySerial.print(str);
}

