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

#include "StepDriver.hpp"
#include "Plasma.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"

#include "driver/gpio.h"

#include "websocket_server.h"

#define PI 3.14159265

/**
 * ЧПУ станок
 */
class CncRouter{

public:

    enum AXES_COUNT{
        AXES_NC = 0,
        AXES_1,
        AXES_2,
        AXES_3,
        AXES_4,
        AXES_5,
        AXES_6
    };

    enum AXE{
        AXE_NC = 0,
        AXE_X,
        AXE_Y,
        AXE_Z,
        AXE_A,
        AXE_B,
        AXE_C
    };

    enum INPUT0{
        INPUT0_LIMITS = 1,
        INPUT0_HOMES,
        INPUT0_PROBE,
        INPUT0_ESTOP
    };

    struct Input0Interrupt{
        INPUT0 type;            // тип ввода
    };

    enum GCODE_LETTER{
        GCODE_LETTER_NONE = 0x00,

        GCODE_LETTER_G = 0x01,
        GCODE_LETTER_N = 0x02,
        GCODE_LETTER_M = 0x03,
        GCODE_LETTER_O = 0x04,

        GCODE_LETTER_X = 0x11,
        GCODE_LETTER_Y = 0x12,
        GCODE_LETTER_Z = 0x13,
        GCODE_LETTER_A = 0x14,
        GCODE_LETTER_B = 0x15,
        GCODE_LETTER_C = 0x16,

        GCODE_LETTER_P = 0x17,
        GCODE_LETTER_F = 0x18,
        GCODE_LETTER_S = 0x19,
        GCODE_LETTER_R = 0x1A,
        GCODE_LETTER_D = 0x1B,
        GCODE_LETTER_L = 0x1C,
        GCODE_LETTER_I = 0x1D,
        GCODE_LETTER_J = 0x1E,
        GCODE_LETTER_K = 0x1F,
    };

    struct GcodeCoord{
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
        float a = 0.0;
        float b = 0.0;
        float c = 0.0;
    };

    struct GcodeFrameSubData{
        GCODE_LETTER letter;
        float value;
    };



    enum COORD{
        COORD_ABSOLUTE = 0,
        COORD_RELATIVE
    };

    enum RUN_TYPE{
        RUN_FAST = 0,
        RUN_WORK_LINEAR,
        RUN_WORK_CW,
        RUN_WORK_CCW
    };

    enum UNIT_TYPE{
        UNIT_METRIC = 0,
        UNIT_INCH
    };

    enum CIRCLE_TYPE{
        CIRCLE_RADIUS = 0,
        CIRCLE_INC
    };

    enum PLASMA_ARC{
        PLASMA_ARC_NONE = 0,
        PLASMA_ARC_START,
        PLASMA_ARC_STOP
    };

    enum COMPENSATION_TYPE{
        COMPENSATION_NONE = 0,      // компенсация инструмента отключена
        COMPENSATION_LEFT,          // компенсация радиуса инструмента слева от траектории
        COMPENSATION_RIGHT,         // компенсация радиуса инструмента справа от траектории
        COMPENSATION_POS,           // компенсация длины инструмента положительно
        COMPENSATION_NEG            // компенсация длины инструмента отрицательно
    };

    struct GcodeCircleInc{
        float i = 0.0;
        float j = 0.0;
        float k = 0.0;
    };
    struct GcodeCircle{
        CIRCLE_TYPE type = CIRCLE_RADIUS;       // тип окружности (CIRCLE_RADIUS, CIRCLE_INC)
        float radius = 0.0;                     // радиус окружности
        GcodeCircleInc inc;                     // смещение центра окружности
    };
    struct GcodeCircleParams{                   // параметры дуги окружности программы
        float xc = 0.0;
        float yc = 0.0;
        float zc = 0.0;
        float radius = 0.0;
        float angle1 = 0.0;
        float angle2 = 360.0;
        bool ccw;
        GcodeCoord target;                      // абсолютная целевая точка
    };
    struct CompensationRadius{
        COMPENSATION_TYPE type = COMPENSATION_NONE;     // тип компенсации
        float value = 0.0;                              // значение
    };
    struct CompensationLength{
        COMPENSATION_TYPE type = COMPENSATION_NONE;     // тип компенсации
        float value = 0.0;                              // значение
    };

    struct ProgParams{
        COORD coordSystem = COORD_ABSOLUTE;
        RUN_TYPE runType = RUN_FAST;
        GcodeCoord currentCoord;        // текущие координаты (абсолютные)
        GcodeCoord targetCoord;         // целевые координаты (абсолютные или относительные, в зависимости от coordSystem)
        GcodeCoord systemCoord;         // координаты нулевой точки при переводе на другую систему координат (G90 или G91)
        GcodeCoord userZeroPoint;       // координаты пользовательского "нуля"
        UNIT_TYPE unit = UNIT_METRIC;   // единицы измерения
        GcodeCircle circle;             // окружность
        CompensationRadius compensationRadius;      // компенсация радиуса
        CompensationLength compensationLength;      // компенсация длины
        uint8_t finishCount = 0;        // счётчик осей, по которым перемещение прекратилось
        PLASMA_ARC plasmaArc = PLASMA_ARC_NONE;     // запуск плазмы
        float speed = 50;               // скорость перемещения, mm/sec
        float pause = 0.0;              // пауза задаваемая командой G04, в секундах
    };


private:

    static CncRouter *instance;

    AXES_COUNT _axesCount = AXES_NC;

    StepDriver *_x;
    StepDriver *_y;
    StepDriver *_z;
    StepDriver *_a;
    StepDriver *_b;
    StepDriver *_c;

    gpio_num_t _pinLimits = GPIO_NUM_NC;
    gpio_num_t _pinHomes = GPIO_NUM_NC;
    gpio_num_t _pinProbe = GPIO_NUM_NC;
    gpio_num_t _pinEStop = GPIO_NUM_NC;

    StepDriver *probeAxe = NULL;
    StepDriver *moveAxe = NULL;

    void *gcodePtr = NULL;
    uint32_t gcodeSize = 0;
    uint32_t gcodePtrOffset = 0;
    // void *gcodeFramePrt = NULL;
    GcodeCircleParams gcodeCircleParams;


    ProgParams progParams;
    GcodeCoord coordNull;       // нулевые координаты: x=0, y=0, z=0, a=0, b=0, c=0

    Plasma *_plasma = NULL;     // плазма

    bool _testRunChecked = false;

    static xQueueHandle input0EvtQueue;
    static xQueueHandle wsSendEvtQueue;
    static xQueueHandle gcodeFinishEvtQueue;


    static const char* LOG_NAME;


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

    static const uint8_t WS_PLASMA_ARC_START;
    static const uint8_t WS_PLASMA_ARC_STARTED;
    static const uint8_t WS_PLASMA_ARC_VOLTAGE;


public:

    /**
     * Конструктор
     * @param axesCount количество осей
     */
    CncRouter(AXES_COUNT axesCount);

    /**
     * Получение экземпляра
     */
    static CncRouter* getInstance();

    /**
     * Получение объекта оси
     * @param axe ось
     */
    StepDriver* getAxe(AXE axe);

    /**
     * Перемещение до целевой позиции
     * @param axe ось
     * @param targetMM целевая позиция, в мм
     * @param speedMmSec скорость, мм/сек
     * @param funcFinish функция вызываемая при окончании перемещения по оси
     */
    // void gotoTargetMM(AXE axe, float targetMM);
    // void gotoTargetMM(AXE axe, float targetMM, float speedMmSec, void (*funcFinish)(StepDriver *sd));
    void gotoTargetMM(AXE axe, float targetMM, float speedMmSec, std::function<void (StepDriver *sd)> funcFinish);

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
     * Установка оси для простого перемещения
     * @param axe ось
     */
    void setMoveAxe(StepDriver *axe);

    /**
     * 
     */
    static void wsSendTask(void *arg);

    /**
     * Установка программы GCode
     * @param ptr указатель на начало программы
     * @param size размер программы, в байтах
     */
    void setGcodePtr(void *ptr, uint32_t size);

    /**
     * Последовательное добавление команд GCode в конец списка при загрузке
     * @param data входящие данные
     * @param length размер данных
     */
    void gcodeAppend(void *data, uint32_t length);

    /**
     * Удаление программы GCode из памяти.
     * Освобождение памяти.
     */
    void removeGcode();

    /**
     * Получение указателя на начало программы
     */
    void* getGcodePtr();

    /**
     * Получение размера программы
     */
    uint32_t getGcodeSize();

    /**
     * Является ли загруженный GCode доступным для выполнения
     */
    bool isGcodeRunnable();

    /**
     * Запуск программы GCode
     */
    void runGcode();

    /**
     * Обработка фрейма программы GCode
     */
    void gcodeProcessFrame(GcodeFrameSubData *frame, uint8_t frameLength);

    /**
     * Обработка команды Gxx
     */
    bool gcodeProcessCommand_G(uint8_t value, GcodeFrameSubData *frame, uint8_t frameLength);

    /**
     * Обработка команды Mxx
     */
    bool gcodeProcessCommand_M(uint8_t value, GcodeFrameSubData *frame, uint8_t frameLength);

    /**
     * Пересчёт текущих координат
     * @param targetX целевая координата X
     * @param targetY целевая координата Y
     * @param targetZ целевая координата Z
     */
    void gcodeRecalcCoords(float targetX, float targetY, float targetZ);

    /**
     * Программа GCode.
     * Рисование прямой линии.
     * @param targetX целевая координата X
     * @param targetY целевая координата Y
     * @param targetZ целевая координата Z
     */
    void gcodeProcess_drawLine(float targetX, float targetY, float targetZ);

    /**
     * Программа GCode.
     * Рисование окружности.
     * 
     */
    void gcodeProcess_drawCircle(GcodeCircleParams *cp);

    /**
     * Задача рисования дуги окружности
     */
    static void gcodeProcess_drawCircleTask(void *arg);

    static TaskHandle_t gcodeDrawCircleTaskHandle;

    /**
     * Программа GCode.
     * Расчёт окружности с учётом параметра радиуса.
     * @param targetX целевая координата X
     * @param targetY целевая координата Y
     * @param targetZ целевая координата Z
     */
    void gcodeProcess_calcCircleByRadius(float targetX, float targetY, float targetZ, GcodeCircleParams *cp);

    /**
     * Программа GCode.
     * Расчёт окружности с учётом параметров смещения центра.
     * @param targetX целевая координата X
     * @param targetY целевая координата Y
     * @param targetZ целевая координата Z
     */
    void gcodeProcess_calcCircleByInc(float targetX, float targetY, float targetZ, GcodeCircleParams *cp);

    /**
     * Программа GCode.
     * Пауза выполнения программы.
     */
    void gcodeProcess_pause();

    /**
     * Программа GCode.
     * Завершение паузы выполнения программы.
     */
    static void gcodeProcess_pauseFinish(void *arg);

    /**
     * Функция вызываемая при окончании перемещения по оси
     * @param axe ось
     */
    static void gcodeGotoFinish(StepDriver *sd);

    /**
     * Программа GCode.
     * Включить плазму.
     */
    void gcodeProcess_plasmaStart();

    /**
     * Программа GCode.
     * Выключить плазму.
     */
    void gcodeProcess_plasmaStop();

    /**
     * Задача выполнения программы GCode
     */
    static void gcodeTask(void *arg);

    static TaskHandle_t gcodeTaskHandle;



    // /**
    //  * Получение текущих координат
    //  */
    // GcodeCoord* getCurrentCoord();

    /**
     * Получение параметров программы GCode
     */
    ProgParams* getProgParams();

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
    static void plasmaArcStartedCallback(bool started);

    /**
     * Установка тестового прогона программы gcode
     * @param checked тестовый прогон программы gcode
     */
    void setTestRunChecked(bool checked);

};

#endif      // __CNCROUTER_HPP__
