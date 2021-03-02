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

#ifndef __DEFINES_HPP__
#define __DEFINES_HPP__

#define EQUIPMENT_TYPE 0x03
#define EQUIPMENT_SUBTYPE 0x01

// (major<<24) + (minor<<16) + (build<8) + revision
#define VERSION ( (0x00 << 24) + (0x02 << 16) + (0x00 << 8) + 0x00 )



#endif      // __DEFINES_HPP__
