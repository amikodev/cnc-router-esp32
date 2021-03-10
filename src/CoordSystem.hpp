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

#ifndef __COORD_SYSTEM_HPP__
#define __COORD_SYSTEM_HPP__

#include "Geometry.hpp"

class CoordSystem{

public:

    enum SYSTEM_TYPE{
        COORD_SYSTEM_NULL = 0,
        COORD_SYSTEM_USER
    };

private:

    Geometry::Point _systemNull;
    Geometry::Point _systemUserZero;

public:

    /**
     * Конструктор
     */
    CoordSystem();    

    /**
     * Установить точку пользовательского нуля
     */
    void setUserZero(Geometry::Point *point);

    /**
     * Получить точку пользовательского нуля
     */
    Geometry::Point getUserZero();

};

#endif      // __COORD_SYSTEM_HPP__
