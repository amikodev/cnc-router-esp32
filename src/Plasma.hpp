/*
amikodev/cnc-router-esp32 - CNC Router on esp-idf
Copyright © 2020 Prihodko Dmitriy - asketcnc@yandex.ru
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

    
private:

    static Plasma *instance;

    gpio_num_t _pinStart = GPIO_NUM_NC;
    gpio_num_t _pinArcStarted = GPIO_NUM_NC;

    int currentArcValue = 1;

    static xQueueHandle arcStartedEvtQueue;

    ArcStartedFunc arcStartedFunc = NULL;

    bool isInverseStart = false;
    bool notifyIfStart = false;


public:

    /**
     * Конструктор
     */
    Plasma();

    /**
     * Инициализация выводов
     * @param pinStart вывод запуска плазмы
     * @param pinArcStarted вывод флага о рабочем режиме плазмы
     */
    void initPins(gpio_num_t pinStart, gpio_num_t pinArcStarted);

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
     * 
     */
    static void IRAM_ATTR arcStartedIsrHandler(void *arg);

    /**
     * 
     */
    static void arcStartedTask(void *arg);

    /**
     * Прерывание экземпляра
     * @param level значение вывода флага о рабочем режиме плазмы
     */
    void arcInterrupt(int level);

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

};

#endif      // __PLASMA_HPP__