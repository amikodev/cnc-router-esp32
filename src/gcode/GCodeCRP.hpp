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

#ifndef __GCODE_COMPENSATION_RADIUS_PARAMS_HPP__
#define __GCODE_COMPENSATION_RADIUS_PARAMS_HPP__

#include "Geometry.hpp"
#include "GCodeCR.hpp"
#include "GCode.hpp"

class GCodeCRP : public GCodeCR{

public:

    struct CompensationPath{
        Geometry::Point *currentPoint;
        Geometry::Point *targetPoint;
        Geometry::Point *nextPoint;
        GCode::ProgParams *currentParams;
        GCode::ProgParams *targetParams;
        GCode::ProgParams *nextParams;
    };


private:

    static Geometry::Point *crossPoint;      // точка пересечения

public:

    /**
     * Перемещение по требуемому пути (линейная или круговая интерполяция) с учётом компенсации радиуса
     * @param compensationPath путь движения
     * @param speed скорость, мм/сек
     */
    static void processPath(CompensationPath *compensationPath, float speed);

    /**
     * Линейная интерполяция
     * @param compensationPath путь движения
     * @param speed скорость, мм/сек
     */
    static void processLine(CompensationPath *compensationPath, float speed);

    /**
     * Круговая интерполяция
     * @param compensationPath путь движения
     * @param speed скорость, мм/сек
     */
    static void processCircle(CompensationPath *compensationPath, float speed);

    /* *
     * Перемещение по соединяющему сегменту окружности
     * @param center центр окружности
     * @param p1 первая точка
     * @param p2 вторая точка
     * @param compensationRadius компенсация радиуса
     * @param speed скорость, мм/сек
     * @param funcFinish функция вызываемая при окончании перемещения
     */
    /*
    static bool moveCircleJoin(Geometry::PointXY *center, Geometry::PointXY *p1, Geometry::PointXY *p2, CompensationRadius *compensationRadius, float speed, std::function<void ()> funcFinish);
    */

    // static void funcFinish();

};

#endif      // __GCODE_COMPENSATION_RADIUS_PARAMS_HPP__