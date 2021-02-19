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

#include "Axe.hpp"

StepDriver* Axe::_x = NULL;
StepDriver* Axe::_y = NULL;
StepDriver* Axe::_z = NULL;
StepDriver* Axe::_a = NULL;
StepDriver* Axe::_b = NULL;
StepDriver* Axe::_c = NULL;

Axe::AXES_COUNT Axe::_axesCount = AXES_NC;

/**
 * Инициализация осей
 * @param axesCount количество осей
 */
void Axe::init(AXES_COUNT axesCount){
    if(axesCount == Axe::AXES_NC){
        return;
    }

    _axesCount = axesCount;
    if(axesCount >= Axe::AXES_1)
        _x = new StepDriver();
    if(axesCount >= Axe::AXES_2)
        _y = new StepDriver();
    if(axesCount >= Axe::AXES_3)
        _z = new StepDriver();
    if(axesCount >= Axe::AXES_4)
        _a = new StepDriver();
    if(axesCount >= Axe::AXES_5)
        _b = new StepDriver();
    if(axesCount >= Axe::AXES_6)
        _c = new StepDriver();

}

/**
 * Получение объекта оси
 * @param axe ось
 */
StepDriver* Axe::getStepDriver(AXE axe){
    StepDriver *sd = NULL;
    switch(axe){
        case AXE_X:
            sd = _x;
            break;
        case AXE_Y:
            sd = _y;
            break;
        case AXE_Z:
            sd = _z;
            break;
        case AXE_A:
            sd = _a;
            break;
        case AXE_B:
            sd = _b;
            break;
        case AXE_C:
            sd = _c;
            break;
        default:
            break;
    }

    if(sd == NULL){
        
    }

    return sd;
}

/**
 * Получение количества осей
 */
Axe::AXES_COUNT Axe::getAxesCount(){
    return _axesCount;
}


