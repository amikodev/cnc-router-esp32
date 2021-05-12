/*
amikodev/cnc-router-esp32 - CNC Router on esp-idf
Copyright © 2020-2021 Prihodko Dmitriy - asketcnc@yandex.ru
*/

/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#ifndef __PLASMA_HPP__
#define __PLASMA_HPP__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/gpio.h"

#include "Axe.hpp"
#include "StepDriver.hpp"
#include "nvs-storage.hpp"


/**
 * Плазморез CNC
 */
class Plasma{
public:

    typedef void (*ArcStartedFunc)(bool started, bool notifyIfStart);

    enum PLASMA_ARC{
        PLASMA_ARC_NONE = 0,
        PLASMA_ARC_START,
        PLASMA_ARC_STOP
    };

    struct PlasmaVolt{
        uint16_t count;
    };

    enum VOLTAGE_RANGE{
        RANGE_UNKNOWN = 0,
        RANGE_LOWER,
        RANGE_NORMAL,
        RANGE_HIGHER,
        RANGE_ABNORMAL
    };

    typedef void (*VoltageRangeFunc)(VOLTAGE_RANGE range, double v, uint16_t count);
    
private:

    static Plasma *instance;

    gpio_num_t _pinStart = GPIO_NUM_NC;
    gpio_num_t _pinArcStarted = GPIO_NUM_NC;
    gpio_num_t _pinArcVoltage = GPIO_NUM_NC;

    int currentArcValue = 1;

    static xQueueHandle arcStartedEvtQueue;
    static xQueueHandle arcVoltageEvtQueue;

    ArcStartedFunc arcStartedFunc = NULL;
    VoltageRangeFunc voltageRangeFunc = NULL;

    bool isInverseStart = false;
    bool notifyIfStart = false;

    static unsigned long startTime;
    static uint16_t voltCount;

    float workVoltage = 100.0;
    float deviationVoltage = 2.0;
    float abnormalVoltage = 150.0;

    float pK = 0.0;
    float pB = 0.0;

    bool thcStateOn = false;        // включение THC
    float thcSpeed = 1.0;           // скорость работы THC


public:

    /**
     * Конструктор
     */
    Plasma();

    /**
     * Инициализация выводов
     * @param pinStart вывод запуска плазмы
     * @param pinArcStarted вывод флага о рабочем режиме плазмы
     * @param pinArcVoltage вывод прерывания для рассчёта напряжения 
     */
    void initPins(gpio_num_t pinStart, gpio_num_t pinArcStarted, gpio_num_t pinArcVoltage);

    /**
     * Установка инвертированности значения вывода запуска плазмы
     * @param isInverse значение инвертировано
     */
    void setInverseStart(bool isInverse);

    /**
     * Получение вывода запуска плазмы
     */
    gpio_num_t getPinStart();

    /**
     * Получение вывода флага о рабочем режиме плазмы
     */
    gpio_num_t getPinArcStarted();

    /**
     * Функция прерывания по запуску дуги
     */
    static void IRAM_ATTR arcStartedIsrHandler(void *arg);

    /**
     * Задача по запуску дуги
     */
    static void arcStartedTask(void *arg);

    /**
     * Прерывание экземпляра
     * @param level значение вывода флага о рабочем режиме плазмы
     */
    void arcStartedInterrupt(int level);

    /**
     * Функция прерывания по напряжению дуги
     */
    static void IRAM_ATTR arcVoltageIsrHandler(void *arg);

    /**
     * Задача по напряжению дуги
     */
    static void arcVoltageTask(void *arg);

    /**
     * Прерывание экземпляра
     * @param count количество прерываний
     */
    void arcVoltageInterrupt(uint16_t count);

    /**
     * Запуск плазмы
     */
    void start();

    /**
     * Остановка плазмы
     */
    void stop();

    /**
     * Получение флага о рабочем режиме плазмы
     * true - дуга запущена, рабочий режим
     * false - рабочая дуга не запущена
     */
    bool getArcStarted();

    /**
     * Установка функции вызываемой при изменении значения флага о рабочем режиме плазмы
     * @param func функция обратного вызова
     */
    void setArcStartedCallback(ArcStartedFunc func);

    /**
     * Установка функции вызываемой с частотой 10 раз в секунду с информацией о том, в каком диапазоне лежит текущее напряжение дуги
     * @param func функция обратного вызова
     */
    void setVoltageRangeCallback(VoltageRangeFunc func);

    /**
     * Установить рабочее напряжение
     * @param wv напряжение, В
     */
    void setWorkVoltage(float wv);

    /**
     * Получить рабочее напряжение
     */
    float getWorkVoltage();

    /**
     * Установить допустимое расхождение со значением рабочего напряжения
     * @param dv значение расхождения, В
     */
    void setDeviationVoltage(float dv);

    /**
     * Получить расхождение напряжения
     */
    float getDeviationVoltage();

    /**
     * Установка параметров для рассчёта напряжения
     * по формуле v = k*intrCount + b;
     * @param k параметр формулы k 
     * @param b параметр формулы b
     */
    void setCalcParams(float k, float b);

    /**
     * Получить значение параметра k
     */
    float getCalcParamK();

    /**
     * Получить значение параметра b
     */
    float getCalcParamB();

    /**
     * Включить THC 
     */
    void thcOn();

    /**
     * Выключить THC 
     */
    void thcOff();

    /**
     * Получить статус включения THC
     */
    bool isThcOn();

    /**
     * Установить скорость работы THC
     * @param speed скорость, мм/с
     */
    void setThcSpeed(float speed);

    /**
     * Получить скорость работы THC
     */
    float getThcSpeed();

};

#endif      // __PLASMA_HPP__