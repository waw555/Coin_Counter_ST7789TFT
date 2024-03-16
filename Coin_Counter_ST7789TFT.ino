/*
  Электронный распознаватель монет (по размеру) для копилки со счётчиком суммы и статистикой по каждому типу монет.
  Функционал:
  Распознавание размера монет с высокой точностью и его привязка к стоимости каждой монеты
  Вычисление общей суммы монет в копилке
  Статистика по числу монет каждого типа
  Все настройки сохраняются в энергонезависимую память и не сбрасываются при питании
  Накопленная сумма тоже хранится в энергонезависимой памяти и не боится сбоев питания
  Режим глубокого энергосбережения: в спящем режиме потребляется 0.07 мА, в схеме без преобразователя 0.02 мА
  Поддержка любого числа монет разного размера
  Автоматическая калибровка типов монет
  Сброс накопленного количества
  Подробности в видео: https://youtu.be/lH4qfGlK2Qk
  Created 2017
  by AlexGyver
  AlexGyver Home Labs Inc.

  Шрифт 2 - 12 пикселей на символ (Центр - 12*Кол символов - 240/2)
  Шрифт 3 - 18 пикселей на символ (Центр - 18*Кол символов - 240/2)
  Шрифт 4 - 24 пикселя на символ (Центр - 24* кол символов -240/2)

  РАСПИНОВКА
  TX1 - 
  RX0 - 
  RST - 
  GND - 
  D2 - Вход с сенсорной кнопки
  D3 - Выход на повышающий преобразователь
  D4 - Верхний ИК диод
  D5 - Нижний ИК диод
  D6 - Кнопка калибровки
  D7 - Сервопривод данные
  D8 - Mosfet сервопривода, подача питания
  D9 - Дисплей - Reset
  D10 - Дисплей - DC
  D11 - Дисплей - SDA
  D12 - 
  D13 - Дисплей - SCK
  3.3V - 
  REF - 
  A0 - Нижний фототранзистор
  A1 - Верхний фототранзистор
  A2 - Резистор верхнего фототранзистора
  A3 - Резистор нижнего фототранзистора
  A4 - 
  A5 - 
  A6 - 
  A7 - 
  5V - Питание микроконтроллера 2,5V - 5V
  RST - Кнопка перезагрузки
  GND - Питание микроконтроллера ЗЕМЛЯ GROUND
  VIN - Не подключается, удален стабилизатор

*/

/*************************БИБЛИОТЕКИ*******************************/
#include "LowPower.h"
#include "EEPROMex.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Arduino_ST7789_Fast.h>
#include <Servo.h>

/*************************ПИНЫ**************************************/
#define PIN_SENSOR_BUTTON 2                                         // Сенсорная кнопка "проснуться"
#define PIN_CALIBRATE_BUTTON 6                                      // Скрытая кнопка калибровкии сброса
#define PIN_SERVO_DATA 7                                            // Пин сервопривода
#define PIN_MOSFET_SERVO_ON 8                                       // Включаем питание сервопривода
#define PIN_IR_LED_TOP 4                                            // Питание верхнего светодиода 4
#define PIN_IR_LED_BOTTOM 5                                         // Питание нижнего светодиода 5
#define PIN_IR_EMMITING_POWER_TOP A2                                // Питание верхнего фототранзистора
#define PIN_IR_EMMITING_POWER_BOTTOM A3                             // Питание нижнего фототранзистора
#define PIN_IR_EMMITING_SENSOR_TOP A1                               // Сигнал верхнего фототранзистора
#define PIN_IR_EMMITING_SENSOR_BOTTOM A0                            // Сигнал нижнего фототранзистора
#define PIN_DC_DC_BOOST 3                                           // Включение повышающего преобразователя
#define PIN_BATTERY A7                                              // Аналоговый пин измерения напряжения батареи

/*************************ПОДКЛЮЧЕНИЕ ДИСПЛЕЯ***********************/
#define TFT_DC    10                                                //  Пин DC
#define TFT_RST   9                                                 //  Пин RST
#define SCR_WD   240                                                // Ширина дисплея в пикселях
#define SCR_HT   240                                                //  Высота дисплея в пикселях
#define CS_ALWAYS_LOW                                               //Указываем, что вывод дисплея CS подключен внутренне на самой плате дисплея

/*************************НАСТРОЙКИ********************************/
#define RADIUS_SIGNAL 16                                          //  Диапазон для сигналов монет
#define COIN_AMOUNT 5                                             //  Число монет, которые нужно распознать
#define SERVO_ANGLE_OPEN 110                                      //  Угол открытия сервопривода
#define SERVO_ANGLE_CLOSE 137                                     //  Угол закрытия сервопривода
#define SERVO_ANGLE_NEUTRAL 130                                   //  Угол нейтрального положения сервопривода
#define LED_TYPE 8                                                //  Тип включения светодиодов:
                                                                  //  1. Верхний диод и верхний транзистор. Пустой сигнал - 31. Ср. знач.1 -  (). 2. -  (). 
                                                                  //    3. -  (). 4. -  (). 5. -  ()
                                                                  //  2. Верхний диод и верхний и нижний транзистор. Пустой сигнал - 76. Ср. знач.1 - 826 (826,818,823,821,843). 2. - 955 (988,943,953,948,947). 
                                                                  //    3. - 1707 (1713,1700,1703,1702,1717). 4. - 914 (917,913,921,907,914). 5. - 1881 (1881,1880,1881,1881,1884)
                                                                  //  3. Верхний диод и нижний транзистор. Пустой сигнал - 45. Ср. знач.1 - 798 (814,776,797,792,811). 2. - 902 (908,898,902,902,900). 
                                                                  //    3. - 954 (957,952,952,953,956). 4. - 881 (884,883,886,875,881). 5. - 985 (985,984,986,984,987)
                                                                  //  4. Нижний диод и верхний транзистор. Пустой сигнал - 35. Ср. знач.1 - 862 (866,852,864,864,868)16/2=8. 2. - 944 (946,942,944,944,945)4/2=2. 
                                                                  //    3. - 976 (978,975,975,976,977)3/2=2. 4. - 924 (925,922,918,928,928)10/2=5. 5. - 998 (999,999,998,998,998)
                                                                  //  5. Нижний диод и верхний и нижний транзистор. Пустой сигнал - 65. Ср. знач.1 - 1843 (1849,1851,1829,1839,1849)22/2=11. 2. - 1947 (1960,1947,1946,1938,1946)22/2=11. 
                                                                  //    3. - 1988 (1989,1986,1987,1988,1991)5/2=3. 4. - 1926 (1924,1929,1930,1922,1929)8/2=4. 5. - 2016 (2017,2016,2017,2016,2016)
                                                                  //  6. Нижний диод и нижний транзистор. Пустой сигнал - 30. Ср. знач.1 - 988 (988,988,988,988,989). 2. - 1005 (1006,1005,1005,1004,1005). 
                                                                  //    3. - 1013 (1014,1013,1013,1013,1014). 4. - 1003 (1004,1003,1005,1003,1004). 5. - 1018 (1018,1018,1018,1018,1018)
                                                                  //  7. Нижний и верхний диод и верхний транзистор. Пустой сигнал - 28. Ср. знач.1 -  (). 2. -  (). 
                                                                  //    3. -  (). 4. - (). 5. -  ()
                                                                  //  8. Нижний и верхний диод и верхний и нижний транзистор. Пустой сигнал - 56. Ср. знач.1 - 791 (790,785,792,782,807)25/2=13. 2. - 932 (940,930,934,926,930)14/2=7. 
                                                                  //    3. - 1657 (1669,1653,1653,1647,1664)22/2=11. 4. - 898 (899,896,903,892,900)11/2=6. 5. - 1855 (1857,1852,1857,1853,1856)5/2=3
                                                                  //  9. Нижний и верхний диод и нижний транзистор. Пустой сигнал - 28. Ср. знач.1 - 759 (758,751,767,776,744)32/2=16. 2. - 881 (880,885,882,874,884)11/2=6. 
                                                                  //    3. - 942 (947,939,942,940,944)8/2=4. 4. - 862 (856,863,870,861,863)14/2=7. 5. - 979 (979,981,978,981,979)3/2=2
float coinDenomination[COIN_AMOUNT] = {1, 2, 5, 10, 10};          //  Номиналы монет
String currency = "руб";                                          //  Валюта монет
int stb_time = 10000;                                             //  Время бездействия, через которое система уйдёт в сон (миллисекунды)
long accumulated_amount = 150000;                                 //  Сумма которую необходимо накопить

/*************************ПЕРЕМЕННЫЕ******************************/
int coin_signal[COIN_AMOUNT];                                     // тут хранится значение сигнала для каждого размера монет
int coin_quantity[COIN_AMOUNT];                                   // количество монет
int arr_signal[5];                                                // для сред.арифм монет
int meanArithmetic, sens_signal, last_sens_signal;                // среднее арифметическое, сигнал с сенсора, предыдущий сигнал с сенсора                                         
byte empty_signal;                                                // храним уровень пустого сигнала
byte battery_level;                                              // Уровень заряда батареи
float battery_voltage;
unsigned long standby_timer, reset_timer;                         // таймеры
long summ_money = 0;                                              // сумма монет в копилке
boolean unknown_coin, recogn_flag, sleep_flag = true, coin_flag = false;  // флажки
bool memory_clear = false, long_calibrate = false, view_coin_signal = false;

/**************************ИНИЦИАЛИЗАЦИЯ ОБОРУДОВАНИЯ***************/
Arduino_ST7789 lcd = Arduino_ST7789(TFT_DC, TFT_RST);
Servo myservo;

void setup() {
  Serial.begin(9600);                   // открыть порт для связи с ПК для отладки

  //Инициализация дисплея
  lcd.init(SCR_WD, SCR_HT);
  lcd.setRotation(3);
  lcd.cls();
  lcd.setCursor(21, 100);
  lcd.setTextColor(WHITE, BLACK);
  lcd.setTextSize(3);
  lcd.print(utf8rus(F("ЗАГРУЗКА...")));
  Serial.println(F("Загрузка"));

  // подтягиваем кнопки
  pinMode(PIN_SENSOR_BUTTON, INPUT);
  pinMode(PIN_CALIBRATE_BUTTON, INPUT_PULLUP);
  pinMode(PIN_IR_EMMITING_SENSOR_TOP, INPUT);
  pinMode(PIN_IR_EMMITING_SENSOR_BOTTOM, INPUT);

  // пины питания как выходы
  pinMode(PIN_DC_DC_BOOST, OUTPUT);
  pinMode(PIN_MOSFET_SERVO_ON, OUTPUT);
  pinMode(PIN_IR_LED_TOP, OUTPUT);
  pinMode(PIN_IR_LED_BOTTOM, OUTPUT);
  pinMode(PIN_IR_EMMITING_POWER_TOP, OUTPUT);
  pinMode(PIN_IR_EMMITING_POWER_BOTTOM, OUTPUT);
  
  // подать питание на дисплей и датчик
  digitalWrite(PIN_DC_DC_BOOST, 1);
  digitalWrite(PIN_MOSFET_SERVO_ON, 1);
  if (LED_TYPE == 1 || LED_TYPE == 2 || LED_TYPE == 3 || LED_TYPE == 7 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_LED_TOP, 1);
  if (LED_TYPE == 4 || LED_TYPE == 5 || LED_TYPE == 6 || LED_TYPE == 7 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_LED_BOTTOM, 1);
  if (LED_TYPE == 1 || LED_TYPE == 2 || LED_TYPE == 4 || LED_TYPE == 5 || LED_TYPE == 7 || LED_TYPE == 8) digitalWrite(PIN_IR_EMMITING_POWER_TOP, 1);
  if (LED_TYPE == 2 || LED_TYPE == 3 || LED_TYPE == 5 || LED_TYPE == 6 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_EMMITING_POWER_BOTTOM, 1);
  myservo.attach(PIN_SERVO_DATA);
  // подключить прерывание
  attachInterrupt(0, wake_up, CHANGE);
  if (LED_TYPE == 1 || LED_TYPE == 4 || LED_TYPE == 7) empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP);  // считать пустой (опорный) сигнал
  if (LED_TYPE == 2 || LED_TYPE == 5 || LED_TYPE == 8) empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP)+analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
  if (LED_TYPE == 3 || LED_TYPE == 6 || LED_TYPE == 9) empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
  myservo.write(SERVO_ANGLE_CLOSE);

  if (!digitalRead(PIN_CALIBRATE_BUTTON)) {  // если при запуске нажата кнопка калибровки
    lcd.cls();
    lcd.setCursor(48, 100);
    lcd.setTextColor(RED, BLACK);
    lcd.setTextSize(4);
    lcd.println(utf8rus(F("СЕРВИС")));
    delay(500);
    reset_timer = millis();
    while (1) {   // бесконечный цикл
      if (millis() - reset_timer > 1000 && millis() - reset_timer < 1100) {
        lcd.cls();
        lcd.setCursor(36, 5);
        lcd.setTextColor(RED, BLACK);
        lcd.setTextSize(2); //1 символ = 12 пикселям
        lcd.println(utf8rus(F("АВТОМАТИЧЕСКАЯ")));
        lcd.setCursor(60, 25);
        lcd.println(utf8rus(F("КАЛИБРОВКА")));
        lcd.setTextColor(YELLOW, BLACK);
        lcd.setTextSize(2);
        lcd.setCursor(5, 125);
        lcd.println(utf8rus(F("Для запуска отпустите кнопку калибровки")));
        Serial.println(F("Автоматическая калибровка"));
        long_calibrate = true;
      }else if (millis() - reset_timer > 25000 && millis() - reset_timer < 25100) {        // если кнопка всё ещё удерживается и прошло 3 секунды
        lcd.cls();
        lcd.setTextColor(RED, BLACK);
        lcd.setTextSize(2);
        lcd.setCursor(78, 5);
        lcd.println(utf8rus(F("ОЧИСТКА")));
        lcd.setCursor(84, 25);
        lcd.println(utf8rus(F("ПАМЯТИ")));
        lcd.setTextColor(YELLOW, BLACK);
        lcd.setTextSize(2);
        lcd.setCursor(5, 125);
        lcd.println(utf8rus(F("Для запуска отпустите кнопку калибровки")));
        Serial.println(F("Очистка памяти"));
        long_calibrate = false;
        memory_clear = true;
      } else if (millis() - reset_timer > 30000) {        // если кнопка всё ещё удерживается и прошло 3 секунды
        lcd.cls();
        lcd.setTextColor(RED, BLACK);
        lcd.setTextSize(4);
        lcd.setCursor(48, 100);
        lcd.println(utf8rus(F("ОТМЕНА")));
        Serial.println(F("Отмена"));
        delay(1000);
        long_calibrate = false;
        memory_clear = false;
        break;
      }
      if (digitalRead(PIN_CALIBRATE_BUTTON)) {   // если отпустили кнопку, перейти к калибровке
        break;
      }
    }

    // 4. Точная калибровка
    if (long_calibrate){
        lcd.cls();
        lcd.setTextColor(YELLOW, BLACK);
        lcd.setTextSize(2);
        lcd.setCursor(84, 5);
        lcd.println(utf8rus(F("ТОЧНАЯ")));
        lcd.setCursor(60, 25);
        lcd.println(utf8rus(F("КАЛИБРОВКА")));
        lcd.setTextColor(GREEN, BLACK);
        lcd.setCursor(30, 55);
        lcd.println(utf8rus(F("Опустите монету")));
        // Упрощённая калибровка монеток
          while (1) {
            for (byte i = 0; i < COIN_AMOUNT; i++) {
              Serial.print("Калибруем: ");
              Serial.println(coinDenomination[i]);
              meanArithmetic = 0; // скипаем среднее
              for (byte j = 0; j < 5; j++) {
                delay(500);
                last_sens_signal = empty_signal;
                
                Serial.print("(Киньте монету 5 раз)");
                Serial.print(" 5 / ");
                Serial.println(j);
                lcd.setCursor(65, 75);
                lcd.print(coinDenomination[i]);
                lcd.print(" ");
                lcd.println(utf8rus(currency));
                lcd.setCursor(60, 95);
                lcd.print(utf8rus(F("Еще ")));
                lcd.print(5-j);
                lcd.println(utf8rus(F(" раз.")));
                while (1) {
                  if (LED_TYPE == 1 || LED_TYPE == 4 || LED_TYPE == 7) sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP);  // считать пустой (опорный) сигнал
                  if (LED_TYPE == 2 || LED_TYPE == 5 || LED_TYPE == 8) sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP)+analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
                  if (LED_TYPE == 3 || LED_TYPE == 6 || LED_TYPE == 9) sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
                  //sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM)+analogRead(PIN_IR_EMMITING_SENSOR_TOP); // считать датчик
                  //sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM); // считать датчик
                  if (sens_signal > last_sens_signal) last_sens_signal = sens_signal;  // если текущее значение больше предыдущего
                  if (sens_signal - empty_signal > 20) coin_flag = true; // если значение упало почти до "пустого", считать что монета улетела
                  while(sens_signal > last_sens_signal){
                      last_sens_signal = sens_signal;
                      break;
                  }
                  if (coin_flag && (abs(sens_signal - empty_signal)) < 10) {  // если монета точно улетела
                    Serial.print("Сигнал: ");
                    Serial.println(last_sens_signal);
                    arr_signal[j] = last_sens_signal;
                    coin_flag = false;
                    break;
                  }
                }
              }
              Serial.print("Все значения ");
              Serial.print(coinDenomination[i]);
              Serial.print(": ");
              for (byte f = 0; f < 5; f++) {
                meanArithmetic += arr_signal[f];
                Serial.print(arr_signal[f]);
                if (f != 4) Serial.print(",");
              }
              meanArithmetic = meanArithmetic / 5;
              Serial.print("\nСреднее арифметическое: ");
              Serial.println(meanArithmetic);
              coin_signal[i] = meanArithmetic;
              EEPROM.writeInt(i * 2, coin_signal[i]);
            }
            Serial.println("\nКалибровка пройдена");
            Serial.println("Таблица калибровки #2: ");
            Serial.println("------------------------------------");
            for (byte i = 0; i < COIN_AMOUNT; i++) {
              Serial.print(coinDenomination[i]);
              Serial.print(" ----> ");
              Serial.println(coin_signal[i]);
              Serial.println("------------------------------------");
            }
            if (!digitalRead(PIN_CALIBRATE_BUTTON)) {   // если отпустили кнопку, перейти к калибровке
              break;
            }
            delay(1000);
            break;
          }
    }

    // 5. Очистка памяти
    while (memory_clear) {
        // очистить количество монет
        for (byte i = 0; i < COIN_AMOUNT; i++) {
          coin_quantity[i] = 0;
          EEPROM.writeInt(20 + i * 2, 0);
        }
        lcd.cls();
        lcd.setCursor(50, 30);
        lcd.setTextColor(RED);
        lcd.setTextSize(4);
        lcd.println(utf8rus(F("ПАМЯТЬ")));
        lcd.setCursor(40, 65);
        lcd.println(utf8rus(F("ОЧИЩЕНА")));
        delay(2000);
        break; // выходим из режима замера фона по истечении времени 5 сек
    }
  }

  // при старте системы считать из памяти сигналы монет для дальнейшей работы, а также их количество в банке
  for (byte i = 0; i < COIN_AMOUNT; i++) {
    coin_signal[i] = EEPROM.readInt(i * 2);
    coin_quantity[i] = EEPROM.readInt(20 + i * 2);
    summ_money += coin_quantity[i] * coinDenomination[i];  // ну и сумму сразу посчитать, как произведение цены монеты на количество
  }

  
    // для отладки, вывести сигналы монет в порт
    for (byte i = 0; i < COIN_AMOUNT; i++) {
      Serial.println(coin_signal[i]);
    }
  
  standby_timer = millis();  // обнулить таймер ухода в сон
}

void show_home_page(){
    lcd.cls();
    lcd.setTextColor(YELLOW, BLACK);
    lcd.setTextSize(3);
    lcd.setCursor(75, 10);
    lcd.println(utf8rus(F("КОПЛЮ")));
    lcd.setCursor(35, 40);
    lcd.println(utf8rus(F("На iPhone")));
    lcd.setTextColor(GREEN, BLACK);
    lcd.setTextSize(3);
    lcd.setCursor(80, 80);
    lcd.println(utf8rus(F("ЦЕЛЬ")));
    lcd.setCursor(5, 110);
    lcd.print(accumulated_amount);
    lcd.print(utf8rus(F(" ")));
    lcd.println(utf8rus(currency));
    lcd.setTextColor(GREEN, BLACK);
    lcd.setTextSize(3);
    lcd.setCursor(39, 150);
    lcd.println(utf8rus(F("НАКОПЛЕНО")));
    lcd.setCursor(5, 180);
    lcd.print(summ_money);
    lcd.print(utf8rus(F(" ")));
    lcd.println(utf8rus(currency));
    battery_level = map(analogRead(PIN_BATTERY), 512, 860, 0, 100);
    lcd.setTextSize(2);
    lcd.setCursor(5, 220);
    lcd.print(utf8rus(F("Батарея - ")));
    lcd.print(battery_level);
    lcd.println(utf8rus(F("%")));

}

void loop() {
  if (sleep_flag) {  // если проснулись  после сна, инициализировать дисплей и вывести текст, сумму и валюту
    show_home_page();
    sleep_flag = false;
    delay(100);
    if (LED_TYPE == 1 || LED_TYPE == 4 || LED_TYPE == 7) empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP);  // считать пустой (опорный) сигнал
    if (LED_TYPE == 2 || LED_TYPE == 5 || LED_TYPE == 8) empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP)+analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
    if (LED_TYPE == 3 || LED_TYPE == 6 || LED_TYPE == 9) empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
    //empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM)+analogRead(PIN_IR_EMMITING_SENSOR_TOP);
    //empty_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);
  }

  

  // далее работаем в бесконечном цикле
  last_sens_signal = empty_signal;
  while (1) {
    if (LED_TYPE == 1 || LED_TYPE == 4 || LED_TYPE == 7) sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP);  // считать пустой (опорный) сигнал
    if (LED_TYPE == 2 || LED_TYPE == 5 || LED_TYPE == 8) sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_TOP)+analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
    if (LED_TYPE == 3 || LED_TYPE == 6 || LED_TYPE == 9) sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // считать пустой (опорный) сигнал
    //sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM)+analogRead(PIN_IR_EMMITING_SENSOR_TOP);  // далее такой же алгоритм, как при калибровке
    //sens_signal = analogRead(PIN_IR_EMMITING_SENSOR_BOTTOM);  // далее такой же алгоритм, как при калибровке
    Serial.println(sens_signal);
    if (sens_signal > last_sens_signal) last_sens_signal = sens_signal;
    if (sens_signal - empty_signal > 20){
      coin_flag = true;
      unknown_coin = true;
    }
    while(sens_signal > last_sens_signal){
      last_sens_signal = sens_signal;
      break;
    }
    if (coin_flag && (abs(sens_signal - empty_signal)) < 10) {
      for (byte i = 0; i < COIN_AMOUNT; i++) {
        int delta = abs(last_sens_signal - coin_signal[i]);   // вот самое главное! ищем АБСОЛЮТНОЕ (то бишь по модулю)
        if (delta < RADIUS_SIGNAL) {   // и вот тут если эта разность попадает в диапазон, то считаем монетку распознанной
          myservo.write(SERVO_ANGLE_OPEN);
          recogn_flag = false;
          unknown_coin=false;
          summ_money += coinDenomination[i];  // к сумме тупо прибавляем цену монетки (дада, сумма считается двумя разными способами. При старте системы суммой всех монет, а тут прибавление
          switch(i){
            case 0: //1 рубль
              lcd.cls();
              lcd.setTextColor(WHITE, BLACK);
              lcd.setTextSize(10);
              lcd.setCursor(95, 70);
              lcd.print(utf8rus(F("1")));
              lcd.setTextSize(5);
              lcd.setCursor(40, 170);
              lcd.print(utf8rus(F("РУБЛЬ")));
              if(view_coin_signal){
                lcd.setTextSize(2);
                lcd.setCursor(40, 220);
                lcd.print(last_sens_signal);
              }
              delay(750);
            break;
            case 1: // 2 рубля
              lcd.cls();
              lcd.setTextColor(WHITE, BLACK);
              lcd.setTextSize(10);
              lcd.setCursor(95, 70);
              lcd.print(utf8rus(F("2")));
              lcd.setTextSize(5);
              lcd.setCursor(40, 170);
              lcd.print(utf8rus(F("РУБЛЯ")));
              if(view_coin_signal){
                lcd.setTextSize(2);
                lcd.setCursor(40, 220);
                lcd.print(last_sens_signal);
              }
              delay(750);
            break;
            case 2: // 5 рублей
              lcd.cls();
              lcd.setTextColor(WHITE, BLACK);
              lcd.setTextSize(10);
              lcd.setCursor(95, 70);
              lcd.print(utf8rus(F("5")));
              lcd.setTextSize(5);
              lcd.setCursor(30, 170);
              lcd.print(utf8rus(F("РУБЛЕЙ")));
              if(view_coin_signal){
                lcd.setTextSize(2);
                lcd.setCursor(40, 220);
                lcd.print(last_sens_signal);
              }
              delay(750);
            break;
            case 3: // 10 рублей
              lcd.cls();
              lcd.setTextColor(WHITE, BLACK);
              lcd.setTextSize(10);
              lcd.setCursor(65, 70);
              lcd.print(utf8rus(F("10")));
              lcd.setTextSize(5);
              lcd.setCursor(30, 170);
              lcd.print(utf8rus(F("РУБЛЕЙ")));
              if(view_coin_signal){
                lcd.setTextSize(2);
                lcd.setCursor(40, 220);
                lcd.print(last_sens_signal);
              }
              delay(750);
            break;
            case 4: // 10 рублей
              lcd.cls();
              lcd.setTextColor(WHITE, BLACK);
              lcd.setTextSize(10);
              lcd.setCursor(65, 70);
              lcd.print(utf8rus(F("10")));
              lcd.setTextSize(5);
              lcd.setCursor(30, 170);
              lcd.print(utf8rus(F("РУБЛЕЙ")));
              if(view_coin_signal){
                lcd.setTextSize(2);
                lcd.setCursor(40, 220);
                lcd.print(last_sens_signal);
              }
              delay(750);
            break;        
          }
          show_home_page();
          coin_quantity[i]++;  // для распознанного номера монетки прибавляем количество
          break;
        }
      }

      if(unknown_coin){
            lcd.cls();
            lcd.setTextColor(RED, BLACK);
            lcd.setTextSize(5);
            lcd.setCursor(30, 70);
            lcd.print(utf8rus(F("ОШИБКА")));
            lcd.setTextSize(2);
            lcd.setCursor(10, 170);
            lcd.print(utf8rus(F("МОНЕТА НЕ ОПОЗНАНА")));
            lcd.setCursor(10, 190);
            lcd.print(utf8rus(F("ПОПРОБУЙТЕ ЕЩЕ РАЗ")));
            if(view_coin_signal){
                lcd.setCursor(40, 220);
                lcd.print(last_sens_signal);
            }
            delay(500);
            show_home_page();
            unknown_coin=false;
      }
      coin_flag = false;
      standby_timer = millis();  // сбросить таймер
      if (!recogn_flag){
        delay(200);
        myservo.write(SERVO_ANGLE_CLOSE);
        delay(200);
        myservo.write(SERVO_ANGLE_NEUTRAL);
        recogn_flag = true;
      }
      break;
    }

    // если ничего не делали, времят таймера вышло, спим
    if (millis() - standby_timer > stb_time) {
      good_night();
      break;
    }

    // если монетка вставлена (замыкает контакты) и удерживается 2 секунды
    while (!digitalRead(PIN_CALIBRATE_BUTTON)) {
      int y=0;
      if (millis() - standby_timer > 5000  && millis() - standby_timer < 5100) {
            lcd.cls();
            lcd.setTextColor(YELLOW, BLACK);
            lcd.setTextSize(3);
            lcd.setCursor(25, 10);
            lcd.println(utf8rus(F("СТАТИСТИКА")));
        // отобразить на дисплее: сверху цены монет (округлено до целых!!!!), снизу их количество
        for (byte i = 0; i < COIN_AMOUNT; i++) {
            lcd.setTextColor(GREEN, BLACK);
            lcd.setTextSize(2);
            lcd.setCursor(5, 40+y);
            lcd.print(coinDenomination[i]);
            lcd.print(utf8rus(F(" руб - ")));
            lcd.print(coin_quantity[i]);
            lcd.print(utf8rus(F(" шт")));
            y += 20;
        }
       // standby_timer = millis();  // сбросить таймер
      } else if (millis() - standby_timer > 10000 && millis() - standby_timer < 10100){
          lcd.cls();
          lcd.setTextColor(RED, BLACK);
          lcd.setTextSize(2);
          lcd.setCursor(20, 5);
          lcd.println(utf8rus(F("ОТОБРАЖЕНИЕ")));
          lcd.setCursor(20, 25);
          lcd.println(utf8rus(F("ЗНАЧЕНИЯ")));
          lcd.setCursor(20, 45);
          lcd.println(utf8rus(F("МОНЕТ")));
          lcd.setCursor(20, 65);
          if(view_coin_signal){
            lcd.println(utf8rus(F("ВЫКЛЮЧЕНО")));
            view_coin_signal = false;
          }else{
            lcd.println(utf8rus(F("ВКЛЮЧЕНО")));
            view_coin_signal = true;
          }
          standby_timer = millis();  // сбросить таймер
          delay(1000);
      }
    }
  }
}

// функция сна
void good_night() {
  // перед тем как пойти спать, записываем в EEPROM новые полученные количества монет по адресам начиная с 20го (пук кек)
  for (byte i = 0; i < COIN_AMOUNT; i++) {
    EEPROM.updateInt(20 + i * 2, coin_quantity[i]);
  }
  sleep_flag = true;
  // вырубить питание со всех дисплеев и датчиков
  lcd.cls();
  lcd.sleepDisplay(true);
  lcd.idleDisplay(false);
  lcd.enableDisplay(false);
  lcd.powerSave(4);
  if (LED_TYPE == 1 || LED_TYPE == 2 || LED_TYPE == 3 || LED_TYPE == 7 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_LED_TOP, 0);
  if (LED_TYPE == 4 || LED_TYPE == 5 || LED_TYPE == 6 || LED_TYPE == 7 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_LED_BOTTOM, 0);
  if (LED_TYPE == 1 || LED_TYPE == 2 || LED_TYPE == 4 || LED_TYPE == 5 || LED_TYPE == 7 || LED_TYPE == 8) digitalWrite(PIN_IR_EMMITING_POWER_TOP, 0);
  if (LED_TYPE == 2 || LED_TYPE == 3 || LED_TYPE == 5 || LED_TYPE == 6 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_EMMITING_POWER_BOTTOM, 0);
  myservo.detach();
  digitalWrite(PIN_MOSFET_SERVO_ON, 0);
  delay(100);
  
  digitalWrite(PIN_DC_DC_BOOST, 0);
  //digitalWrite(PIN_MOSFET, 0);
  // и вот теперь спать
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}

// просыпаемся по ПРЕРЫВАНИЮ (эта функция - обработчик прерывания)
void wake_up() {
  digitalWrite(PIN_DC_DC_BOOST, 1);
  digitalWrite(PIN_MOSFET_SERVO_ON, 1);
  myservo.attach(PIN_SERVO_DATA);
  lcd.sleepDisplay(false);
  lcd.idleDisplay(true);
  lcd.enableDisplay(true);
  lcd.powerSave(0);
  if (LED_TYPE == 1 || LED_TYPE == 2 || LED_TYPE == 3 || LED_TYPE == 7 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_LED_TOP, 1);
  if (LED_TYPE == 4 || LED_TYPE == 5 || LED_TYPE == 6 || LED_TYPE == 7 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_LED_BOTTOM, 1);
  if (LED_TYPE == 1 || LED_TYPE == 2 || LED_TYPE == 4 || LED_TYPE == 5 || LED_TYPE == 7 || LED_TYPE == 8) digitalWrite(PIN_IR_EMMITING_POWER_TOP, 1);
  if (LED_TYPE == 2 || LED_TYPE == 3 || LED_TYPE == 5 || LED_TYPE == 6 || LED_TYPE == 8 || LED_TYPE == 9) digitalWrite(PIN_IR_EMMITING_POWER_BOTTOM, 1);
  standby_timer = millis();  // и обнуляем таймер
}

//Перекодировка русских букв
/* Функция перекодировки русских букв из UTF-8 в Win-1251 */
String utf8rus(String source)
{
  int i, k;
  String target;
  unsigned char n;
  char m[2] = { '0', '\0' };
  k = source.length(); i = 0;
  while (i < k) {
    n = source[i]; i++;
    if (n >= 0xC0) {
      switch (n) {
        case 0xD0: {
            n = source[i]; i++;
            if (n == 0x81) {
              n = 0xA8;
              break;
            }
            if (n >= 0x90 && n <= 0xBF) n = n + 0x2F;
            break;
          }
        case 0xD1: {
            n = source[i]; i++;
            if (n == 0x91) {
              n = 0xB8;
              break;
            }
            if (n >= 0x80 && n <= 0x8F) n = n + 0x6F;
            break;
          }
      }
    }
    m[0] = n; target = target + String(m);
  }
  return target;
}
