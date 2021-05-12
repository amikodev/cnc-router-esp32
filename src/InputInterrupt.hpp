/*
amikodev/cnc-router-esp32 - CNC Router on esp-idf
Copyright © 2021 Prihodko Dmitriy - asketcnc@yandex.ru
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

#ifndef __INPUT_INTERRUPT_HPP__
#define __INPUT_INTERRUPT_HPP__

#include "driver/gpio.h"

/**
 * Обработка прерываний со входов
 */
class InputInterrupt{

public:

    enum INPUT0{
        INPUT0_NONE = 0,
        INPUT0_LIMITS,
        INPUT0_HOMES,
        INPUT0_PROBE,
        INPUT0_ESTOP,

        INPUT0_COUNT
    };

    struct Input0Interrupt{
        INPUT0 type;            // тип ввода
    };

    static gpio_num_t pins[INPUT0_COUNT];
    static int levels[INPUT0_COUNT];


};

#endif      // __INPUT_INTERRUPT_HPP__
