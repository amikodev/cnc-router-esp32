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

#ifndef __GCODE_COMPENSATION_RADIUS_HPP__
#define __GCODE_COMPENSATION_RADIUS_HPP__

#include "Geometry.hpp"
#include "ActionMove.hpp"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>

#include "esp_system.h"
#include "esp_log.h"

/**
 * Компенсация радиуса инструмента
 */
class GCodeCR{

public:

    enum COMPENSATION_SIDE{
        COMPENSATION_NONE = 0,          // компенсация инструмента отключена
        COMPENSATION_LEFT,              // компенсация радиуса инструмента слева от траектории
        COMPENSATION_RIGHT              // компенсация радиуса инструмента справа от траектории
    };

    struct CompensationRadius{
        COMPENSATION_SIDE side;         // сторона компенсации
        float value;                    // значение, в мм
    };

private:


public:


};

#endif      // __GCODE_COMPENSATION_RADIUS_HPP__