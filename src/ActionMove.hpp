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

    struct CircleTask{
        Geometry::CircleSegment *circle;
        float speed;
        std::function<void ()> funcFinish;
    };


private:

    static uint8_t axeMoveCounter;

    static TaskHandle_t circleTaskHandle;

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
     * @param funcFinish функция вызываемая при окончании перемещения
     */
    static bool gotoPoint(Geometry::Point *point, float speed, std::function<void ()> funcFinish);

    /**
     * Прямолинейное движение к точке
     * @param point целевая точка
     * @param speed скорость, мм/сек
     * @param funcFinish функция вызываемая при окончании перемещения
     */
    static bool gotoPoint(Geometry::PointXY *point, float speed, std::function<void ()> funcFinish);

    /**
     * Движение по кружности
     * @param circle окружность
     * @param speed скорость, мм/сек
     * @param funcFinish функция вызываемая при окончании перемещения
     */
    static bool circle(Geometry::CircleSegment *circle, float speed, std::function<void ()> funcFinish);

    /**
     * Задача движения по окружности
     */
    static void circleTask(void *arg);

};

#endif      // __ACTIONMOVE_HPP__
