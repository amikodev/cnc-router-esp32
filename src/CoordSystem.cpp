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

#include "CoordSystem.hpp"

#define TAG "CoordSystem"

#define __POINT_NULL ((Geometry::Point){ .x=0, .y=0, .z=0, .a=0, .b=0, .c=0 })


/**
 * Конструктор
 */
CoordSystem::CoordSystem(){
    _systemNull = __POINT_NULL;
    _systemUserZero = __POINT_NULL;
}

/**
 * Установить точку пользовательского нуля
 */
void CoordSystem::setUserZero(Geometry::Point *point){
    memcpy(&_systemUserZero, point, sizeof(Geometry::Point));
}

/**
 * Получить точку пользовательского нуля
 */
Geometry::Point CoordSystem::getUserZero(){
    Geometry::Point point;
    memcpy(&point, &_systemUserZero, sizeof(Geometry::Point));
    return point;
}

