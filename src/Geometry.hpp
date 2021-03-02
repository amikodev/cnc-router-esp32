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

#ifndef __GEOMETRY_HPP__
#define __GEOMETRY_HPP__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <cmath>
#include <stdexcept>

#include "esp_system.h"
#include "esp_log.h"


/**
 * Геометрия
 */
class Geometry{

public:

    struct Point{
        float x;
        float y;
        float z;
        float a;
        float b;
        float c;

        /**
         * Логирование параметров в консоль
         */
        void log(const char *tag, const char *prefix);
    };

    struct PointXY{
        float x;
        float y;

        /**
         * Логирование параметров в консоль
         */
        void log(const char *tag, const char *prefix);
    };

    struct PointXYZ{
        float x;
        float y;
        float z;
    };

    struct LineSegment{
        PointXY p1;
        PointXY p2;
        float angle;
        float anglePerp;
    };

    struct IntersectPoint{
        PointXY point;
        bool inner;
    };

    struct CircleSegment{
        PointXY center;
        float r;
        float angle1;
        float angle2;
        bool ccw;
        PointXY p1;
        PointXY p2;
    };

    struct IntersectCircles{
        PointXY p0;
        PointXY p1;
        PointXY p2;
        float angle11;
        float angle12;
        float angle21;
        float angle22;
    };


private:


public:

    /**
     * Рассчёт параллельной линии
     * @param p1 первая точка отрезка
     * @param p2 вторая точка отрезка
     * @param length величина отступа
     * @param isLeftSide с левой стороны
     * @return параллельная линия
     */
    static LineSegment calcLineSegmentOffset(PointXY *p1, PointXY *p2, float length, bool isLeftSide);

    /**
     * Нахождение точки пересечения отрезков
     * @param segment1 первый отрезок
     * @param segment2 второй отрезок
     * @return точка пересечения
     */
    static IntersectPoint calcIntersectLineSegments(LineSegment *segment1, LineSegment *segment2);

    /**
     * Получение полных параметров окружности на основе радиуса
     * @param p1 первая точка
     * @param p2 вторая точка
     * @param r радиус
     * @param ccw против часовой стрелки
     * @return параметры окружности
     */
    static CircleSegment calcCircleByRadius(PointXY *p1, PointXY *p2, float r, float ccw);

    /**
     * Получение полных параметров окружности на основе инкрементальных координат
     * @param p1 первая точка
     * @param p2 вторая точка
     * @param incI смещение по оси X
     * @param incJ смещение по оси Y
     * @param ccw против часовой стрелки
     * @return параметры окружности
     */
    static CircleSegment calcCircleByInc(PointXY *p1, PointXY *p2, float incI, float incJ, bool ccw);

    /**
     * Рассчёт параллельного сегмента окружности
     * @param p1 первая точка сегмента
     * @param p2 вторая точка сегмента
     * @param r радиус окружности
     * @param ccw против часовой стрелки
     * @param length величина отступа
     * @param isLeftSide с левой стороны
     * @return параметры сегмента окружности
     */
    static CircleSegment calcCircleOffset(PointXY *p1, PointXY *p2, float r, float ccw, float length, bool isLeftSide);

    /**
     * Рассчёт параллельного сегмента окружности
     * @param circleOriginal оригинальный сегмент окружности
     * @param length величина отступа
     * @param isLeftSide с левой стороны
     * @return параметры сегмента окружности
     */
    static CircleSegment calcCircleOffset(CircleSegment *circleOriginal, float length, bool isLeftSide);

    /**
     * Нахождение точки пересечения отрезка и сегмента окружности
     * @param segment отрезок
     * @param circle сегмент окружности, полученный из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
     * @return точка пересечения
     */
    static IntersectPoint calcIntersectLineCircle(LineSegment *segment, CircleSegment *circle);

    /**
     * Нахождение точек пересечения двух окружностей
     * @param circle1 первая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
     * @param circle2 вторая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
     */
    static IntersectCircles getIntersectCircles(CircleSegment *circle1, CircleSegment *circle2);

    /**
     * Нахождение точки пересечения двух окружностей
     * @param circle1 первая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
     * @param circle2 вторая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
     */
    static IntersectPoint calcIntersectCircles(CircleSegment *circle1, CircleSegment *circle2);

    /**
     * Определение лежит ли точка с углом a3 внутри сегмента окружности
     * @param circle окружность
     * @param a3 произвольный угол
     */
    static bool withinCircleSegment(CircleSegment *circle, float a3);

    /**
     * Определение лежит ли точка внутри отрезка
     * @param segment отрезок
     * @param p3 произвольная точка
     */
    static bool withinLineSegment(LineSegment *segment, PointXY *p3);

    /**
     * Получение угла по двум точкам
     * @param p1 первая точка
     * @param p2 вторая точка
     */
    static float getAngle2(PointXY *p1, PointXY *p2);

    /**
     * Определение являются ли две точки с одними координатами
     * @param p1 первая точка
     * @param p2 вторая точка
     */
    static bool pointsIsEqual(Point *p1, Point *p2);

    /**
     * Определение являются ли две точки с одними координатами
     * @param p1 первая точка
     * @param p2 вторая точка
     */
    static bool pointsIsEqual(PointXY *p1, PointXY *p2);

    /**
     * Преобразование точки в тип Point
     * @param pointXY точка
     */
    static Point getPoint(PointXY *pointXY);


    static Point getPoint(PointXY *pointXY, Point defaultPoint);

    /**
     * Преобразование точки в тип PointXY
     * @param point точка
     */
    static PointXY getPointXY(Point *point);

    /**
     * Пересчёт параметров окружности на основе точек
     * @param circle окружность
     */
    static void recalcCircleSegment(CircleSegment *circle);

};

#endif      // __GEOMETRY_HPP__
