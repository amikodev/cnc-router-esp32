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

#ifndef __GCODE_HPP__
#define __GCODE_HPP__

// class GCode;

#include "Geometry.hpp"
#include "GCodeCR.hpp"
#include "ActionMove.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <list>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sleep.h"

// class GCodeCR;

/**
 * Управляющая программа GCode
 */
class GCode{

public:

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

    struct FrameSubData{
        GCODE_LETTER letter;
        float value;
    };

    enum COORD_TYPE{
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

    struct CircleInc{
        float i;
        float j;
        float k;
    };
    struct Circle{
        CIRCLE_TYPE type;                  // тип окружности (CIRCLE_RADIUS, CIRCLE_INC)
        float radius;                      // радиус окружности
        CircleInc inc;                     // смещение центра окружности
    };

    struct ProgParams{
        uint32_t numLine;
        COORD_TYPE coordType;
        RUN_TYPE runType;
        Geometry::Point currentCoord;       // текущие координаты (абсолютные)
        Geometry::Point targetCoord;        // целевые координаты (абсолютные или относительные, в зависимости от coordSystem)
        Geometry::Point systemCoord;        // координаты нулевой точки при переводе на другую систему координат (G90 или G91)
        Geometry::Point userZeroPoint;      // координаты пользовательского "нуля"
        UNIT_TYPE unit;                     // единицы измерения
        Circle circle;                      // окружность
        Geometry::CircleSegment circleSegment;  // сегмент окружности (рассчитанный на основе circle)
        GCodeCR::CompensationRadius compensationRadius;         // компенсация радиуса

        float speed;
        float pause;
    };

    static TaskHandle_t gcodeTaskHandle;


private:

    static void *_ptr;                     // указатель на начало памяти программы
    static uint32_t _size;                 // размер памяти
    static uint32_t _ptrOffset;            // смещение в памяти на конец принятых данных для последующего добавления данных

    static bool _testRunChecked;           // флаг тестового прогона программы gcode


    static const GCODE_LETTER codeVal4length[];     // параметры фреймов с типом данных Float (4 байта)

    static ProgParams progParams;           // текущие параметры программы

    static Geometry::Point pointNull;       // нулевые координаты: x=0, y=0, z=0, a=0, b=0, c=0


public:

    /**
     * Инициализация программы GCode
     * @param size размер памяти, в байтах
     * @return флаг успешности выделения памяти
     */
    static bool init(uint32_t size);

    /**
     * Удаление программы из памяти
     */
    static void remove();

    /**
     * Последовательное добавление команд GCode в конец списка при загрузке
     * @param data входящие данные
     * @param length размер данных
     */
    static void append(void *data, uint32_t length);

    /**
     * Является ли загруженный GCode доступным для выполнения
     */
    static bool isRunnable();

    /**
     * Запуск программы GCode
     */
    static void run();

    /**
     * Задача выполнения программы GCode
     */
    static void gcodeTask(void *arg);

    /**
     * Установка тестового прогона программы gcode
     * @param checked тестовый прогон программы gcode
     */
    static void setTestRunChecked(bool checked);

    /**
     * Установка текущих параметров программы по-умолчанию
     */
    static void setDefaultProgParams();

    /**
     * Обработка фрейма программы GCode
     */
    static bool processFrame(FrameSubData *frame, uint8_t frameLength);

    /**
     * Обработка команды Gxx
     */
    static bool processCommand_G(uint8_t value, FrameSubData *frame, uint8_t frameLength);

    /**
     * Обработка команды Mxx
     */
    static bool processCommand_M(uint8_t value, FrameSubData *frame, uint8_t frameLength);

    /**
     * Получение целевой точки
     * @param pParams параметры программы
     */
    static Geometry::Point calcTargetPoint(ProgParams *pParams);

    /**
     * Рассчёт полных параметров окружности
     * @param currentPoint текущая точка
     * @param targetPoint целевая точка
     * @param pParams параметры программы
     */
    static void calcCircleSegment(Geometry::Point *currentPoint, Geometry::Point *targetPoint, ProgParams *pParams);

    /**
     * Перемещение по требуемому пути (линейная или круговая интерполяция)
     * @param targetPoint целевая точка
     * @param speed скорость, мм/сек
     */
    static void processPath(Geometry::Point *targetPoint, ProgParams *targetParams, float speed);

    /**
     * Определить является ли путь линейной интерполяцией
     * @param pParams параметры программы
     */
    static bool isLinearInterpolation(ProgParams *pParams);

    /**
     * Определить является ли путь круговой интерполяцией
     * @param pParams параметры программы
     */
    static bool isCircleInterpolation(ProgParams *pParams);

};

#endif      // __GCODE_HPP__