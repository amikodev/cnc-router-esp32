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

#include "Geometry.hpp"

#define PI 3.14159265
#define EPSG 1e-4

#define TAG "Geometry"

/**
 * Рассчёт параллельной линии
 * @param p1 первая точка отрезка
 * @param p2 вторая точка отрезка
 * @param length величина отступа
 * @param isLeftSide с левой стороны
 * @return параллельная линия
 */
Geometry::LineSegment Geometry::calcLineSegmentOffset(PointXY *p1, PointXY *p2, float length, bool isLeftSide){

    PointXY p1p = {
        .x = p1->x,
        .y = p1->y
    };
    PointXY p2p = {
        .x = p2->x,
        .y = p2->y
    };

    float dx = p2->x - p1->x;
    float dy = p2->y - p1->y;
    float angle = atan2(dy, dx);
    float anglePerp = angle + PI/2;

    if(!isLeftSide)     // с правой стороны
        anglePerp = angle - PI/2;

    float cdx = length * cos(anglePerp);
    float cdy = length * sin(anglePerp);

    // координаты отрезка параллельного первому
    p1p.x += cdx;
    p1p.y += cdy;
    p2p.x += cdx;
    p2p.y += cdy;

    LineSegment segment = {
        .p1 = p1p,
        .p2 = p2p,
        .angle = angle,
        .anglePerp = anglePerp
    };

    return segment;
}

/**
 * Нахождение точки пересечения отрезков
 * @param segment1 первый отрезок
 * @param segment2 второй отрезок
 * @return точка пересечения
 * @throw
 */
Geometry::IntersectPoint Geometry::calcIntersectLineSegments(LineSegment *segment1, LineSegment *segment2){

    IntersectPoint intersectPoint;

    if(segment1 == NULL || segment2 == NULL){
        throw std::runtime_error("Cross point not found");
    }

    bool inner = false;

    PointXY *p1 = &segment1->p1;
    PointXY *p2 = &segment1->p2;
    PointXY *p3 = &segment2->p1;
    PointXY *p4 = &segment2->p2;

    float a1 = p1->y - p2->y;
    float b1 = p2->x - p1->x;
    float c1 = p1->x*p2->y - p2->x*p1->y;

    float a2 = p3->y - p4->y;
    float b2 = p4->x - p3->x;
    float c2 = p3->x*p4->y - p4->x*p3->y;

    if(abs(a1) < EPSG) a1 = 0.0;
    if(abs(a2) < EPSG) a2 = 0.0;
    if(abs(b1) < EPSG) b1 = 0.0;
    if(abs(b2) < EPSG) b2 = 0.0;

    float d = a1*b2 - a2*b1;
    if(d != 0){
        float py = (a2*c1 - a1*c2) / d;
        float px = 0.0;

        if(a1 != 0)
            px = -(b1*py + c1) / a1;
        else
            px = -(b2*py + c2) / a2;

        if(
            ( (px >= p1->x-EPSG && px <= p2->x+EPSG) || (px <= p1->x+EPSG && px >= p2->x-EPSG) ) &&
            ( (py >= p1->y-EPSG && py <= p2->y+EPSG) || (py <= p1->y+EPSG && py >= p2->y-EPSG) )
        ){
            inner = true;
        }

        intersectPoint = {
            .point = {
                .x = px,
                .y = py
            },
            .inner = inner
        };

    } else{
        throw std::runtime_error("Cross point not found");
    }

    return intersectPoint;    
}

/**
 * Получение полных параметров окружности на основе радиуса
 * @param p1 первая точка
 * @param p2 вторая точка
 * @param r радиус
 * @param ccw против часовой стрелки
 * @return параметры окружности
 */
Geometry::CircleSegment Geometry::calcCircleByRadius(PointXY *p1, PointXY *p2, float r, float ccw){
    CircleSegment cp;

    float x1 = p1->x;
    float y1 = p1->y;
    float x2 = p2->x;
    float y2 = p2->y;
    float dx = x2-x1;
    float dy = y2-y1;

    bool radiusPositive = r >= 0;
    r = abs(r);

    float d = sqrt(dx*dx+dy*dy);
    float drd = r*r-(d/2)*(d/2);
    if(drd < EPSG) drd = 0;
    float h = sqrt(drd);

    float xc1 = x1 + dx/2 + h*dy / d;
    float yc1 = y1 + dy/2 - h*dx / d;

    float xc2 = x1 + dx/2 - h*dy / d;
    float yc2 = y1 + dy/2 + h*dx / d;


    float xc = xc2;
    float yc = yc2;

    if( (!ccw && radiusPositive) || (ccw && !radiusPositive) ){
        xc = xc1;
        yc = yc1;
    }

    float angle1 = atan2(y1-yc, x1-xc);
    float angle2 = atan2(y2-yc, x2-xc);

    // параметры окружности
    cp = {
        .center = { .x = xc, .y = yc },
        .r = r,  .angle1 = angle1, .angle2 = angle2, .ccw = ccw,
        .p1 = *p1, .p2 = *p2,
    };
    return cp;
}

/**
 * Получение полных параметров окружности на основе инкрементальных координат
 * @param p1 первая точка
 * @param p2 вторая точка
 * @param incI смещение по оси X
 * @param incJ смещение по оси Y
 * @param ccw против часовой стрелки
 * @return параметры окружности
 */
Geometry::CircleSegment Geometry::calcCircleByInc(PointXY *p1, PointXY *p2, float incI, float incJ, bool ccw){
    CircleSegment cp;

    float x1 = p1->x;
    float y1 = p1->y;
    float x2 = p2->x;
    float y2 = p2->y;

    float xc = x1 + incI;
    float yc = y1 + incJ;

    float r = sqrt(incI*incI + incJ*incJ);

    float angle1 = atan2(y1-yc, x1-xc);
    float angle2 = atan2(y2-yc, x2-xc);

    // // параметры окружности
    cp = {
        .center = { .x = xc, .y = yc },
        .r = r,  .angle1 = angle1, .angle2 = angle2, .ccw = ccw,
        .p1 = *p1, .p2 = *p2,
    };
    return cp;
}

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
Geometry::CircleSegment Geometry::calcCircleOffset(PointXY *p1, PointXY *p2, float r, float ccw, float length, bool isLeftSide){
    CircleSegment circle = calcCircleByRadius(p1, p2, r, ccw);
    return calcCircleOffset(&circle, length, isLeftSide);
}

/**
 * Рассчёт параллельного сегмента окружности
 * @param circleOriginal оригинальный сегмент окружности
 * @param length величина отступа
 * @param isLeftSide с левой стороны
 * @return параметры сегмента окружности
 */
Geometry::CircleSegment Geometry::calcCircleOffset(CircleSegment *circleOriginal, float length, bool isLeftSide){
    CircleSegment circle;
    memcpy(&circle, circleOriginal, sizeof(CircleSegment));

    if( (!circle.ccw && isLeftSide) || (circle.ccw && !isLeftSide) )
        circle.r += length;
    else 
        circle.r -= length;

    PointXY p1p = {
        .x = circle.center.x + (float)cos(circle.angle1) * circle.r,
        .y = circle.center.y + (float)sin(circle.angle1) * circle.r
    };
    PointXY p2p = {
        .x = circle.center.x + (float)cos(circle.angle2) * circle.r,
        .y = circle.center.y + (float)sin(circle.angle2) * circle.r
    };

    circle.p1 = p1p;
    circle.p2 = p2p;
    return circle;
}

/**
 * Нахождение точки пересечения отрезка и сегмента окружности
 * @param segment отрезок
 * @param circle сегмент окружности, полученный из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
 * @return точка пересечения
 * @throw
*/
Geometry::IntersectPoint Geometry::calcIntersectLineCircle(LineSegment *segment, CircleSegment *circle){
    IntersectPoint intersectPoint;

    bool inner = false;         // пересечение внутри отрезка

    PointXY pc = circle->center;
    float r = circle->r;

    PointXY *p1 = &segment->p1;
    PointXY *p2 = &segment->p2;

    // уравнение прямой a*x + b*y + c = 0
    // изменим параметр C с учётом того, что центр окружности считаем в начале координат
    float a1 = p1->y - p2->y;
    float b1 = p2->x - p1->x;
    float c1 = (p1->x-pc.x)*(p2->y-pc.y) - (p2->x-pc.x)*(p1->y-pc.y);

    float a2b2 = a1*a1 + b1*b1;

    float x0 = -a1*c1/a2b2;
    float y0 = -b1*c1/a2b2;

    if(c1*c1 > r*r*a2b2 +EPSG){
        // точек нет
        throw std::runtime_error("Cross point not found");

    } else if(abs(c1*c1 - r*r*a2b2) < EPSG){
        // одна точка
        PointXY p12 = {
            .x = x0 + pc.x,
            .y = y0 + pc.y
        };
        intersectPoint.point = p12;
    } else{
        // две точки
        float d = r*r - c1*c1/a2b2;
        float mult = sqrt(d / a2b2);
        PointXY p12 = {
            .x = x0 + b1 * mult + pc.x,
            .y = y0 - a1 * mult + pc.y
        };
        PointXY p13 = {
            .x = x0 - b1 * mult + pc.x,
            .y = y0 + a1 * mult + pc.y
        };

        bool cwa1 = withinCircleSegment(circle, getAngle2(&circle->center, &p12));
        bool cwa2 = withinCircleSegment(circle, getAngle2(&circle->center, &p13));

        bool cwp1 = withinLineSegment(segment, &p12);
        bool cwp2 = withinLineSegment(segment, &p13);

        if(cwa1 && cwp1){
            intersectPoint.point = p12;
            inner = true;
        } else if(cwa2 && cwp2){
            intersectPoint.point = p13;
            inner = true;
        } else if(cwa1){
            intersectPoint.point = p12;
        } else{
            intersectPoint.point = p13;
        }

    }

    intersectPoint.inner = inner;
    return intersectPoint;

}

/**
 * Нахождение точек пересечения двух окружностей
 * @param circle1 первая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
 * @param circle2 вторая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
 */
Geometry::IntersectCircles Geometry::getIntersectCircles(CircleSegment *circle1, CircleSegment *circle2){
    IntersectCircles intersectCircles;

    PointXY pc1 = circle1->center;
    PointXY pc2 = circle2->center;
    float r1 = circle1->r;
    float r2 = circle2->r;

    float dx = pc2.x-pc1.x;
    float dy = pc2.y-pc1.y;
    float d_2 = dx*dx + dy*dy;
    float d_2s = sqrt(d_2);
    float a = (r1*r1 - r2*r2 + d_2)/(2*d_2s);
    float h = sqrt(r1*r1 - a*a);
    
    PointXY p0 = {
        .x = pc1.x + a/d_2s*(pc2.x - pc1.x),
        .y = pc1.y + a/d_2s*(pc2.y - pc1.y)
    };

    float tpx = h/d_2s*dy;
    float tpy = h/d_2s*dx;
    
    PointXY p1 = {
        .x = p0.x + tpx,
        .y = p0.y - tpy
    };
    PointXY p2 = {
        .x = p0.x - tpx,
        .y = p0.y + tpy
    };

    float angle11 = atan2(p1.y-pc1.y, p1.x-pc1.x);
    float angle12 = atan2(p2.y-pc1.y, p2.x-pc1.x);

    float angle21 = atan2(p1.y-pc2.y, p1.x-pc2.x);
    float angle22 = atan2(p2.y-pc2.y, p2.x-pc2.x);

    intersectCircles = {
        .p0 = p0, .p1 = p1, .p2 = p2, 
        .angle11 = angle11, .angle12 = angle12, .angle21 = angle21, .angle22 = angle22
    };
    return intersectCircles;
}

/**
 * Нахождение точки пересечения двух окружностей
 * @param circle1 первая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
 * @param circle2 вторая окружность, полученная из методов calcCircleByRadius, calcCircleByInc, calcCircleOffset
 * @throw
 */
Geometry::IntersectPoint Geometry::calcIntersectCircles(CircleSegment *circle1, CircleSegment *circle2){
    IntersectPoint intersectPoint;

    IntersectCircles intersectCircles = getIntersectCircles(circle1, circle2);

    bool cwa11 = withinCircleSegment(circle1, intersectCircles.angle11);
    bool cwa12 = withinCircleSegment(circle1, intersectCircles.angle12);
    bool cwa21 = withinCircleSegment(circle2, intersectCircles.angle21);
    bool cwa22 = withinCircleSegment(circle2, intersectCircles.angle22);

    if(cwa11 && cwa21){
        intersectPoint.point = intersectCircles.p1;
    } else if(cwa12 && cwa22){
        intersectPoint.point = intersectCircles.p2;

    } else if(cwa11 || cwa21){
        throw std::runtime_error("Cross point not found");
    } else if(cwa12 || cwa22){
        throw std::runtime_error("Cross point not found");

    } else{
        throw std::runtime_error("Cross point not found");
    }

    return intersectPoint;
}

/**
 * Определение лежит ли точка с углом a3 внутри сегмента окружности
 * @param circle окружность
 * @param a3 произвольный угол
 */
bool Geometry::withinCircleSegment(CircleSegment *circle, float a3){
    float a1 = fmod(circle->angle1, PI*2);
    float a2 = fmod(circle->angle2, PI*2);
    a3 = fmod(a3, PI*2);

    bool inner = false;
    if(circle->ccw){
        inner = a2 > a1 ? (a3 >= a1 && a3 <= a2) : (a3 <= a2 || a3 >= a1);
    } else{
        inner = a2 < a1 ? (a3 >= a2 && a3 <= a1) : (a3 <= a1 || a3 >= a2);
    }

    return inner;
}

/**
 * Определение лежит ли точка внутри отрезка
 * @param segment отрезок
 * @param p3 произвольная точка
 */
bool Geometry::withinLineSegment(LineSegment *segment, PointXY *p3){
    PointXY *p1 = &segment->p1;
    PointXY *p2 = &segment->p2;
    bool within =
        ( (p3->x >= p1->x-EPSG && p3->x <= p2->x+EPSG) || (p3->x <= p1->x+EPSG && p3->x >= p2->x-EPSG) ) &&
        ( (p3->y >= p1->y-EPSG && p3->y <= p2->y+EPSG) || (p3->y <= p1->y+EPSG && p3->y >= p2->y-EPSG) )
    ;
    return within;
}

/**
 * Получение угла по двум точкам
 * @param p1 первая точка
 * @param p2 вторая точка
 */
float Geometry::getAngle2(PointXY *p1, PointXY *p2){
    float angle = atan2(p2->y-p1->y, p2->x-p1->x);
    return angle;
}

/**
 * Определение являются ли две точки с одними координатами
 * @param p1 первая точка
 * @param p2 вторая точка
 */
bool Geometry::pointsIsEqual(Point *p1, Point *p2){
    return 
        p1->x == p2->x &&
        p1->y == p2->y &&
        p1->z == p2->z &&
        p1->a == p2->a &&
        p1->b == p2->b &&
        p1->c == p2->c
    ;
}

/**
 * Определение являются ли две точки с одними координатами
 * @param p1 первая точка
 * @param p2 вторая точка
 */
bool Geometry::pointsIsEqual(PointXY *p1, PointXY *p2){
    return 
        p1->x == p2->x &&
        p1->y == p2->y
    ;
}

/**
 * Преобразование точки в тип Point
 * @param pointXY точка
 */
Geometry::Point Geometry::getPoint(PointXY *pointXY){
    Point point = {
        .x = pointXY->x,
        .y = pointXY->y,
        .z = 0.0, .a = 0.0, .b = 0.0, .c = 0.0
    };
    return point;
}


Geometry::Point Geometry::getPoint(PointXY *pointXY, Point defaultPoint){
    Point point = getPoint(pointXY);
    // ESP_LOGI(TAG, "get point {x: %.2f, y: %.2f} with default {z: %.2f, a: %.2f, b: %.2f, c: %.2f}",
    //     pointXY->x, pointXY->y,
    //     defaultPoint.z, defaultPoint.a, defaultPoint.b, defaultPoint.c
    // );

    point.z = defaultPoint.z;
    point.a = defaultPoint.a;
    point.b = defaultPoint.b;
    point.c = defaultPoint.c;

    return point;
}

/**
 * Преобразование точки в тип PointXY
 * @param point точка
 */
Geometry::PointXY Geometry::getPointXY(Point *point){
    PointXY pointXY = {
        .x = point->x,
        .y = point->y
    };
    return pointXY;
}

/**
 * Пересчёт параметров окружности на основе точек
 * @param circle окружность
 */
void Geometry::recalcCircleSegment(CircleSegment *circle){
    float x1 = circle->p1.x;
    float y1 = circle->p1.y;
    float x2 = circle->p2.x;
    float y2 = circle->p2.y;

    float xc = circle->center.x;
    float yc = circle->center.y;

    float angle1 = atan2(y1-yc, x1-xc);
    float angle2 = atan2(y2-yc, x2-xc);

    circle->angle1 = angle1;
    circle->angle2 = angle2;
}

/**
 * Логирование параметров в консоль
 */
void Geometry::Point::log(const char *tag, const char *prefix){
    ESP_LOGI(tag, "%s: {x: %.4f, y: %.4f, z: %.4f, a: %.4f, b: %.4f, c: %.4f}", prefix,
        x, y, z, a, b, c
    );
}

/**
 * Логирование параметров в консоль
 */
void Geometry::PointXY::log(const char *tag, const char *prefix){
    ESP_LOGI(tag, "%s: {x: %.4f, y: %.4f}", prefix,
        x, y
    );
}


