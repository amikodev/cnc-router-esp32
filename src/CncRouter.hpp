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

#ifndef __CNCROUTER_HPP__
#define __CNCROUTER_HPP__

#include "Axe.hpp"
#include "Geometry.hpp"
#include "StepDriver.hpp"
#include "ActionMove.hpp"
#include "gcode/GCode.hpp"
#include "gcode/GCodeCRP.hpp"
#include "Plasma.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <functional>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"

#include "driver/gpio.h"

#include "websocket_server.h"

/**
 * ЧПУ станок
 */
class CncRouter{

public:

    enum INPUT0{
        INPUT0_LIMITS = 1,
        INPUT0_HOMES,
        INPUT0_PROBE,
        INPUT0_ESTOP
    };

    struct Input0Interrupt{
        INPUT0 type;            // тип ввода
    };

    struct WsData{
        uint64_t size;
        void *ptr;
    };

    Geometry::Point _currentPoint;
    static TaskHandle_t _currentGCodeTask;


private:

    static CncRouter *instance;

    gpio_num_t _pinLimits = GPIO_NUM_NC;
    gpio_num_t _pinHomes = GPIO_NUM_NC;
    gpio_num_t _pinProbe = GPIO_NUM_NC;
    gpio_num_t _pinEStop = GPIO_NUM_NC;

    StepDriver *probeAxe = NULL;

    Plasma *_plasma = NULL;     // плазма

    bool _testRunChecked = false;

    static uint32_t currentNumLine;     // номер текущей строки gcode
    // static WsData wsData;

    static xQueueHandle input0EvtQueue;
    static xQueueHandle wsSendEvtQueue;
    static xQueueHandle wsSendCustomEvtQueue;
    static xQueueHandle gcodeFinishEvtQueue;


    static const uint8_t WS_OBJ_NAME_CNC_ROUTER;
    static const uint8_t WS_OBJ_NAME_CNC_GCODE;
    static const uint8_t WS_OBJ_NAME_CNC_GCODE_PREPARE;
    static const uint8_t WS_OBJ_NAME_AXE;
    static const uint8_t WS_OBJ_NAME_COORDS;
    static const uint8_t WS_OBJ_NAME_COORD_TARGET;
    static const uint8_t WS_OBJ_NAME_PLASMA_ARC;

    static const uint8_t WS_CMD_READ;
    static const uint8_t WS_CMD_WRITE;
    static const uint8_t WS_CMD_RUN;
    static const uint8_t WS_CMD_STOP;

    static const uint8_t WS_AXE_DIRECTION_NONE;
    static const uint8_t WS_AXE_DIRECTION_FORWARD;
    static const uint8_t WS_AXE_DIRECTION_BACKWARD;

    static const uint8_t WS_PREPARE_SIZE;
    static const uint8_t WS_PREPARE_RUN;
    static const uint8_t WS_PREPARE_STOP;

    static const uint8_t WS_PLASMA_ARC_START;
    static const uint8_t WS_PLASMA_ARC_STARTED;
    static const uint8_t WS_PLASMA_ARC_VOLTAGE;


public:

    /**
     * Конструктор
     */
    CncRouter();

    /**
     * Получение экземпляра
     */
    static CncRouter* getInstance();

    /**
     * Парсинг входящих данных по WebSocket
     * @param data указатель на данные
     * @param length длина данных
     */
    static void parseWsData(uint8_t *data, uint32_t length);

    /**
     * Установка прерывания на вывод
     * @param pin вывод
     * @param inp тип входящих данных
     */
    CncRouter* setInputPinInterrupt(gpio_num_t pin, INPUT0 inp);

    /**
     * Установка вывода предела
     * @param pin вывод предела
     */
    CncRouter* setPinLimits(gpio_num_t pin);

    /**
     * Получение вывода предела
     */
    gpio_num_t getPinLimits();

    /**
     * Установка вывода Home
     * @param pin вывод Home
     */
    CncRouter* setPinHomes(gpio_num_t pin);

    /**
     * Получение вывода Home
     */
    gpio_num_t getPinHomes();

    /**
     * Установка вывода Probe
     * @param pin вывод Probe
     */
    CncRouter* setPinProbe(gpio_num_t pin);

    /**
     * Получение вывода Probe
     */
    gpio_num_t getPinProbe();

    /**
     * Установка вывода EStop
     * @param pin вывод EStop
     */
    CncRouter* setPinEStop(gpio_num_t pin);

    /**
     * Получение вывода EStop
     */
    gpio_num_t getPinEStop();

    /**
     * 
     */
    static void IRAM_ATTR input0IsrHandler(void *arg);

    /**
     * 
     */
    static void input0Task(void *arg);

    /**
     * Установка оси для выполнения операции Probe
     * @param axe ось
     */
    void setProbeAxe(StepDriver *axe);

    /**
     * 
     */
    static void wsSendTask(void *arg);

    /**
     * 
     */
    static void wsSendCustomTask(void *arg);

    /**
     * 
     */
    static void currentPointTask(void *arg);

    /**
     * Запустить задачу уведомления об изменении текущих координат
     */
    void enableCurrentPointNotify();

    /**
     * Установка плазмы
     * @param plasma плазма
     */
    void setPlasma(Plasma *plasma);

    /**
     * Получение плазмы
     */
    Plasma* getPlasma();

    /**
     * Функция обратного вызова при изменении значения вывода флага рабочего режима дуги плазмы
     */
    static void plasmaArcStartedCallback(bool started, bool notifyIfStart);

    /**
     * Отправка номера строки gcode клиенту
     */
    static void notifyGcodeNumLine(uint32_t numLine);

    /**
     * 
     */
    static void currentNumLineTask(void *arg);

    /**
     * Уведомление о завершении выполнения программы gcode
     */
    static void notifyGcodeFinish();

};

#endif      // __CNCROUTER_HPP__
