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

#ifndef __ACTIONMOVE_HPP__
#define __ACTIONMOVE_HPP__

#include "Geometry.hpp"
#include "Axe.hpp"
#include "StepDriver.hpp"

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

/**
 * Перемещение каретки станка
 */
class ActionMove{

public:


private:

    static uint8_t axeMoveCounter;

public:

    /**
     * Перемещение до целевой позиции
     * @param axe ось
     * @param targetMM целевая позиция, в мм
     * @param speed скорость, мм/сек
     * @param funcStepDriverFinish функция вызываемая при окончании перемещения по оси
     */
    static void gotoAxePoint(Axe::AXE axe, float targetMM, float speed, std::function<void (StepDriver *sd)> funcStepDriverFinish);

    /**
     * Прямолинейное движение к точке
     * @param point целевая точка
     * @param speed скорость, мм/сек
     * @param doPause включить ожидание до окончания перемещения
     */
    static void gotoPoint(Geometry::Point *point, float speed, bool doPause=true);

    /**
     * Прямолинейное движение к точке
     * @param point целевая точка
     * @param speed скорость, мм/сек
     * @param doPause включить ожидание до окончания перемещения
     */
    static void gotoPoint(Geometry::PointXY *point, float speed, bool doPause=true);

    /**
     * Движение по кружности
     * @param circle окружность
     * @param speed скорость, мм/сек
     */
    static void circle(Geometry::CircleSegment *circle, float speed);

    /**
     * Остановка перемещения
     */
    static void stop();

};

#endif      // __ACTIONMOVE_HPP__
