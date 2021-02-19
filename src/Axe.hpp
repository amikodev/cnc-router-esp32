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

#ifndef __AXE_HPP__
#define __AXE_HPP__

#include "StepDriver.hpp"

/**
 * Ось
 */
class Axe{

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


private:

    static StepDriver *_x;
    static StepDriver *_y;
    static StepDriver *_z;
    static StepDriver *_a;
    static StepDriver *_b;
    static StepDriver *_c;

    static AXES_COUNT _axesCount;

public:

    /**
     * Инициализация осей
     * @param axesCount количество осей
     */
    static void init(AXES_COUNT axesCount);

    /**
     * Получение объекта оси
     * @param axe ось
     */
    static StepDriver* getStepDriver(AXE axe);

    /**
     * Получение количества осей
     */
    static AXES_COUNT getAxesCount();

};

#endif      // __AXE_HPP__
