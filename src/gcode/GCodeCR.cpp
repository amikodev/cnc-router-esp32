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

#include "GCode.hpp"
#include "GCodeCRP.hpp"

#define PI 3.14159265
#define EPSG 1e-4

#define TAG "GCodeCR"
#define TAG_CIRCLE "GCodeCR::Circle"


Geometry::Point* GCodeCRP::crossPoint = NULL;

/**
 * Перемещение по требуемому пути (линейная или круговая интерполяция) с учётом компенсации радиуса
 * @param compensationPath путь движения
 * @param speed скорость, мм/сек
 */
void GCodeCRP::processPath(GCodeCRP::CompensationPath *compensationPath, float speed){
    // ESP_LOGI(TAG, "processPath");

    GCode::ProgParams *pParams = compensationPath->targetParams;
    if(GCode::isLinearInterpolation(pParams)){
        processLine(compensationPath, speed);
    } else if(GCode::isCircleInterpolation(pParams)){
        processCircle(compensationPath, speed);
    }
}

/**
 * Линейная интерполяция
 * @param compensationPath путь движения
 * @param speed скорость, мм/сек
 */
void GCodeCRP::processLine(GCodeCRP::CompensationPath *compensationPath, float speed){
    // ESP_LOGI(TAG, "processLine");

    std::function<void ()> funcTargetFinish = [&](){
        // ESP_LOGI(TAG, "funcTargetFinish");
        vTaskResume(GCode::gcodeTaskHandle);
    };

    GCode::ProgParams *cParams = compensationPath->currentParams;
    GCode::ProgParams *tParams = compensationPath->targetParams;
    GCode::ProgParams *nParams = compensationPath->nextParams;

    Geometry::LineSegment lineOffset = Geometry::calcLineSegmentOffset(
        (Geometry::PointXY *) compensationPath->currentPoint, 
        (Geometry::PointXY *) compensationPath->targetPoint, 
        tParams->compensationRadius.value, 
        tParams->compensationRadius.side == COMPENSATION_LEFT
    );

    // ESP_LOGI(TAG, "p1:[%.2f : %.2f], p2:[%.2f : %.2f], p1p:[%.2f : %.2f], p2p:[%.2f : %.2f]",
    //     compensationPath->currentPoint->x, compensationPath->currentPoint->y, compensationPath->targetPoint->x, compensationPath->targetPoint->y, 
    //     lineOffset.p1.x, lineOffset.p1.y, lineOffset.p2.x, lineOffset.p2.y
    // );

    Geometry::Point defaultPoint = { .x = 0.0, .y = 0.0, .z = Axe::getStepDriver(Axe::AXE_Z)->getPositionMM(), .a = 0.0, .b = 0.0, .c = 0.0};

    Geometry::Point p1p = Geometry::getPoint(&lineOffset.p1, defaultPoint);
    Geometry::Point p2p = Geometry::getPoint(&lineOffset.p2, defaultPoint);

    if(cParams->compensationRadius.side == COMPENSATION_NONE){
        if(ActionMove::gotoPoint(&p1p, speed, funcTargetFinish)){
            vTaskSuspend(GCode::gcodeTaskHandle);
        }
    } else{

        if(crossPoint != NULL){
            free(crossPoint);
            crossPoint = NULL;
        }

    }

    if(nParams->compensationRadius.side == COMPENSATION_NONE){
        if(ActionMove::gotoPoint(&p2p, speed, funcTargetFinish)){
            vTaskSuspend(GCode::gcodeTaskHandle);
        }
        if(ActionMove::gotoPoint(compensationPath->targetPoint, speed, funcTargetFinish)){
            vTaskSuspend(GCode::gcodeTaskHandle);
        }
    } else{

        // следующий участок пути - линейная интерполяция
        if(GCode::isLinearInterpolation(nParams)){
            Geometry::LineSegment lineOffsetNext = Geometry::calcLineSegmentOffset(
                (Geometry::PointXY *) compensationPath->targetPoint, 
                (Geometry::PointXY *) compensationPath->nextPoint, 
                nParams->compensationRadius.value, 
                nParams->compensationRadius.side == COMPENSATION_LEFT
            );

            // точка пересечения отрезка на текущем участке пути и
            // отрезком на следующем
            Geometry::IntersectPoint intersectPoint;
            bool intersectPointFounded = false;
            try{
                intersectPoint = Geometry::calcIntersectLineSegments(&lineOffset, &lineOffsetNext);
                intersectPointFounded = true;
            } catch(const std::runtime_error &error){}
            // ESP_LOGI(TAG, "intersect founded: %d, point: [%.2f, %.2f], inner: %d", intersectPointFounded, intersectPoint.point.x, intersectPoint.point.y, intersectPoint.inner);
            if(intersectPointFounded && intersectPoint.inner){      // точка пересечения внутри
                // отрезок на текущем участке пути
                // до точки пересечения отрезков
                Geometry::Point point = Geometry::getPoint(&intersectPoint.point, defaultPoint);
                crossPoint = new Geometry::Point();
                memcpy(crossPoint, &point, sizeof(Geometry::Point));
                if(ActionMove::gotoPoint(crossPoint, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }
            } else{     // окружность снаружи

                // отрезок на текущем участке пути
                // до конечной точки этого отрезка
                if(ActionMove::gotoPoint(&p2p, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }

                if(!Geometry::pointsIsEqual(compensationPath->currentPoint, compensationPath->targetPoint)){
                    // соединяющий сегмент окружности

                    // Geometry::PointXY tPoint = Geometry::getPointXY(compensationPath->targetPoint);
                    // if(moveCircleJoin(&tPoint, &lineOffset.p2, &lineOffsetNext.p1, &tParams->compensationRadius, speed, funcTargetFinish)){
                    //     vTaskSuspend(GCode::gcodeTaskHandle);
                    // }

                    Geometry::CircleSegment circleJoin = {
                        .center = {
                            .x = compensationPath->targetPoint->x,
                            .y = compensationPath->targetPoint->y,
                        },
                        .r = tParams->compensationRadius.value,
                        .angle1 = 0,    // пересчёт будет в recalcCircleSegment
                        .angle2 = 0,    // пересчёт будет в recalcCircleSegment
                        .ccw = tParams->compensationRadius.side != COMPENSATION_LEFT,
                        .p1 = lineOffset.p2,
                        .p2 = lineOffsetNext.p1
                    };
                    Geometry::recalcCircleSegment(&circleJoin);

                    if(abs(circleJoin.angle2-circleJoin.angle1) >= 0.0174 && abs(circleJoin.angle2-circleJoin.angle1) <= 6.2657){     // > 1 градуса
                        if(ActionMove::circle(&circleJoin, speed, funcTargetFinish)){
                            vTaskSuspend(GCode::gcodeTaskHandle);
                        }
                    }
                }
            }

        // следующий участок пути - круговая интерполяция
        } else if(GCode::isCircleInterpolation(nParams)){

            Geometry::CircleSegment *circle = &nParams->circleSegment;
            Geometry::CircleSegment circleOffset = Geometry::calcCircleOffset(
                circle, 
                nParams->compensationRadius.value, 
                nParams->compensationRadius.side == COMPENSATION_LEFT
            );
            // ESP_LOGI(TAG, "circle:       p1:[%.2f : %.2f], p2:[%.2f : %.2f], a1: %.2f, a2: %.2f", circle->p1.x, circle->p1.y, circle->p2.x, circle->p2.y, circle->angle1*180/PI, circle->angle2*180/PI);
            // ESP_LOGI(TAG, "circleOffset: p1:[%.2f : %.2f], p2:[%.2f : %.2f], a1: %.2f, a2: %.2f", circleOffset.p1.x, circleOffset.p1.y, circleOffset.p2.x, circleOffset.p2.y, circleOffset.angle1*180/PI, circleOffset.angle2*180/PI);

            // точка пересечения отрезка на текущем участке пути и
            // окружностью на следующем
            Geometry::IntersectPoint intersectPoint;
            bool intersectPointFounded = false;
            try{
                // intersectPoint = Geometry::calcIntersectLineSegments(&lineOffset, &lineOffsetNext);
                intersectPoint = Geometry::calcIntersectLineCircle(&lineOffset, &circleOffset);
                intersectPointFounded = true;
            } catch(const std::runtime_error &error){}
            // ESP_LOGI(TAG, "intersect founded: %d, point: [%.2f, %.2f], inner: %d", intersectPointFounded, intersectPoint.point.x, intersectPoint.point.y, intersectPoint.inner);
            if(intersectPointFounded && intersectPoint.inner){      // точка пересечения внутри
                // отрезок на текущем участке пути
                // до точки пересечения с окружностью
                Geometry::Point point = Geometry::getPoint(&intersectPoint.point, defaultPoint);
                crossPoint = new Geometry::Point();
                memcpy(crossPoint, &point, sizeof(Geometry::Point));
                if(ActionMove::gotoPoint(&point, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }
            } else{     // окружность снаружи

                // отрезок на текущем участке пути
                // до конечной точки этого отрезка
                if(ActionMove::gotoPoint(&p2p, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }

                if(!Geometry::pointsIsEqual(compensationPath->currentPoint, compensationPath->targetPoint)){
                    // соединяющий сегмент окружности

                    Geometry::CircleSegment circleJoin = {
                        .center = {
                            .x = compensationPath->targetPoint->x,
                            .y = compensationPath->targetPoint->y,
                        },
                        .r = tParams->compensationRadius.value,
                        .angle1 = 0,    // пересчёт будет в recalcCircleSegment
                        .angle2 = 0,    // пересчёт будет в recalcCircleSegment
                        .ccw = tParams->compensationRadius.side != COMPENSATION_LEFT,
                        .p1 = lineOffset.p2,
                        .p2 = circleOffset.p1
                    };
                    Geometry::recalcCircleSegment(&circleJoin);

                    if(abs(circleJoin.angle2-circleJoin.angle1) >= 0.0174 && abs(circleJoin.angle2-circleJoin.angle1) <= 6.2657){     // > 1 градуса
                        if(ActionMove::circle(&circleJoin, speed, funcTargetFinish)){
                            vTaskSuspend(GCode::gcodeTaskHandle);
                        }
                    }
                }
            }
        }
    }
}

/**
 * Круговая интерполяция
 * @param compensationPath путь движения
 * @param speed скорость, мм/сек
 */
void GCodeCRP::processCircle(GCodeCRP::CompensationPath *compensationPath, float speed){
    // ESP_LOGI(TAG_CIRCLE, "processCircle");

    std::function<void ()> funcTargetFinish = [&](){
        // ESP_LOGI(TAG_CIRCLE, "funcTargetFinish");
        vTaskResume(GCode::gcodeTaskHandle);
    };

    GCode::ProgParams *cParams = compensationPath->currentParams;
    GCode::ProgParams *tParams = compensationPath->targetParams;
    GCode::ProgParams *nParams = compensationPath->nextParams;

    Geometry::CircleSegment *circle = &tParams->circleSegment;
    Geometry::CircleSegment circleOffset = Geometry::calcCircleOffset(
        circle, 
        tParams->compensationRadius.value, 
        tParams->compensationRadius.side == COMPENSATION_LEFT
    );
    // ESP_LOGI(TAG_CIRCLE, "circle:       p1:[%.2f : %.2f], p2:[%.2f : %.2f], a1: %.2f, a2: %.2f", circle->p1.x, circle->p1.y, circle->p2.x, circle->p2.y, circle->angle1*180/PI, circle->angle2*180/PI);
    // ESP_LOGI(TAG_CIRCLE, "circleOffset: p1:[%.2f : %.2f], p2:[%.2f : %.2f], a1: %.2f, a2: %.2f", circleOffset.p1.x, circleOffset.p1.y, circleOffset.p2.x, circleOffset.p2.y, circleOffset.angle1*180/PI, circleOffset.angle2*180/PI);

    Geometry::Point defaultPoint = { .x = 0.0, .y = 0.0, .z = Axe::getStepDriver(Axe::AXE_Z)->getPositionMM(), .a = 0.0, .b = 0.0, .c = 0.0};

    Geometry::Point p1p = Geometry::getPoint(&circleOffset.p1, defaultPoint);

    if(cParams->compensationRadius.side == COMPENSATION_NONE){
        if(ActionMove::gotoPoint(&p1p, speed, funcTargetFinish)){
            vTaskSuspend(GCode::gcodeTaskHandle);
        }

    } else{

        if(crossPoint != NULL){
            if(!Geometry::pointsIsEqual(&p1p, crossPoint)){
                circleOffset.p1 = Geometry::getPointXY(crossPoint);
                Geometry::recalcCircleSegment(&circleOffset);
            }
            free(crossPoint);
            crossPoint = NULL;
        }

    }

    if(nParams->compensationRadius.side == COMPENSATION_NONE){
        if(ActionMove::circle(&circleOffset, speed, funcTargetFinish)){
            vTaskSuspend(GCode::gcodeTaskHandle);
        }
        if(ActionMove::gotoPoint(compensationPath->targetPoint, speed, funcTargetFinish)){
            vTaskSuspend(GCode::gcodeTaskHandle);
        }

    } else{

        // следующий участок пути - линейная интерполяция
        if(GCode::isLinearInterpolation(nParams)){
            Geometry::LineSegment lineOffsetNext = Geometry::calcLineSegmentOffset(
                (Geometry::PointXY *) compensationPath->targetPoint, 
                (Geometry::PointXY *) compensationPath->nextPoint, 
                nParams->compensationRadius.value, 
                nParams->compensationRadius.side == COMPENSATION_LEFT
            );

            // точка пересечения окружности на текущем участке пути и
            // отрезком на следующем
            Geometry::IntersectPoint intersectPoint;
            bool intersectPointFounded = false;
            try{
                intersectPoint = Geometry::calcIntersectLineCircle(&lineOffsetNext, &circleOffset);
                intersectPointFounded = true;
            } catch(const std::runtime_error &error){}
            // ESP_LOGI(TAG, "intersect founded: %d, point: [%.2f, %.2f], inner: %d", intersectPointFounded, intersectPoint.point.x, intersectPoint.point.y, intersectPoint.inner);
            if(intersectPointFounded && intersectPoint.inner){      // точка пересечения внутри
                // отрезок на текущем участке пути
                // до точки пересечения с окружностью
                Geometry::Point point = Geometry::getPoint(&intersectPoint.point, defaultPoint);
                crossPoint = new Geometry::Point();
                memcpy(crossPoint, &point, sizeof(Geometry::Point));
                circleOffset.p2 = Geometry::getPointXY(crossPoint);
                Geometry::recalcCircleSegment(&circleOffset);
                if(ActionMove::circle(&circleOffset, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }
            } else{     // окружность снаружи

                // окружность на текущем участке пути
                // до конечной точки этой окружность
                if(ActionMove::circle(&circleOffset, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }

                if(!Geometry::pointsIsEqual(compensationPath->currentPoint, compensationPath->targetPoint)){
                    // соединяющий сегмент окружности
                    Geometry::CircleSegment circleJoin = {
                        .center = {
                            .x = compensationPath->targetPoint->x,
                            .y = compensationPath->targetPoint->y,
                        },
                        .r = tParams->compensationRadius.value,
                        .angle1 = 0,    // пересчёт будет в recalcCircleSegment
                        .angle2 = 0,    // пересчёт будет в recalcCircleSegment
                        .ccw = tParams->compensationRadius.side != COMPENSATION_LEFT,
                        .p1 = circleOffset.p2,
                        .p2 = lineOffsetNext.p1
                    };
                    Geometry::recalcCircleSegment(&circleJoin);

                    if(abs(circleJoin.angle2-circleJoin.angle1) >= 0.0174 && abs(circleJoin.angle2-circleJoin.angle1) <= 6.2657){     // > 1 градуса
                        if(ActionMove::circle(&circleJoin, speed, funcTargetFinish)){
                            vTaskSuspend(GCode::gcodeTaskHandle);
                        }
                    }
                }
            }

        // следующий участок пути - круговая интерполяция
        } else if(GCode::isCircleInterpolation(nParams)){

            Geometry::CircleSegment *circleNext = &nParams->circleSegment;
            Geometry::CircleSegment circleOffsetNext = Geometry::calcCircleOffset(
                circleNext, 
                nParams->compensationRadius.value, 
                nParams->compensationRadius.side == COMPENSATION_LEFT
            );
            // ESP_LOGI(TAG, "circleNext:       p1:[%.2f : %.2f], p2:[%.2f : %.2f], a1: %.2f, a2: %.2f", circleNext->p1.x, circleNext->p1.y, circleNext->p2.x, circleNext->p2.y, circleNext->angle1*180/PI, circleNext->angle2*180/PI);
            // ESP_LOGI(TAG, "circleOffsetNext: p1:[%.2f : %.2f], p2:[%.2f : %.2f], a1: %.2f, a2: %.2f", circleOffsetNext.p1.x, circleOffsetNext.p1.y, circleOffsetNext.p2.x, circleOffsetNext.p2.y, circleOffsetNext.angle1*180/PI, circleOffsetNext.angle2*180/PI);

            // точка пересечения окружности на текущем участке пути и
            // окружностью на следующем
            Geometry::IntersectPoint intersectPoint;
            bool intersectPointFounded = false;
            try{
                intersectPoint = Geometry::calcIntersectCircles(&circleOffset, &circleOffsetNext);
                intersectPointFounded = true;
            } catch(const std::runtime_error &error){}
            // ESP_LOGI(TAG, "intersect founded: %d, point: [%.2f, %.2f], inner: %d", intersectPointFounded, intersectPoint.point.x, intersectPoint.point.y, intersectPoint.inner);
            if(intersectPointFounded && intersectPoint.inner){      // точка пересечения внутри
                // отрезок на текущем участке пути
                // до точки пересечения с окружностью
                Geometry::Point point = Geometry::getPoint(&intersectPoint.point, defaultPoint);
                crossPoint = new Geometry::Point();
                memcpy(crossPoint, &point, sizeof(Geometry::Point));
                circleOffset.p2 = Geometry::getPointXY(crossPoint);
                Geometry::recalcCircleSegment(&circleOffset);
                if(ActionMove::circle(&circleOffset, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }
            } else{     // окружность снаружи

                // окружность на текущем участке пути
                // до конечной точки этой окружность
                if(ActionMove::circle(&circleOffset, speed, funcTargetFinish)){
                    vTaskSuspend(GCode::gcodeTaskHandle);
                }

                if(!Geometry::pointsIsEqual(compensationPath->currentPoint, compensationPath->targetPoint)){
                    // соединяющий сегмент окружности
                    Geometry::CircleSegment circleJoin = {
                        .center = {
                            .x = compensationPath->targetPoint->x,
                            .y = compensationPath->targetPoint->y,
                        },
                        .r = tParams->compensationRadius.value,
                        .angle1 = 0,    // пересчёт будет в recalcCircleSegment
                        .angle2 = 0,    // пересчёт будет в recalcCircleSegment
                        .ccw = tParams->compensationRadius.side != COMPENSATION_LEFT,
                        .p1 = circleOffset.p2,
                        .p2 = circleOffsetNext.p1
                    };
                    Geometry::recalcCircleSegment(&circleJoin);

                    if(abs(circleJoin.angle2-circleJoin.angle1) >= 0.0174 && abs(circleJoin.angle2-circleJoin.angle1) <= 6.2657){     // > 1 градуса
                        if(ActionMove::circle(&circleJoin, speed, funcTargetFinish)){
                            vTaskSuspend(GCode::gcodeTaskHandle);
                        }
                    }
                }
            }
        }
    }
}

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
bool GCodeCRP::moveCircleJoin(Geometry::PointXY *center, Geometry::PointXY *p1, Geometry::PointXY *p2, CompensationRadius *compensationRadius, float speed, std::function<void ()> funcFinish){

    Geometry::CircleSegment circleJoin = {
        .center = {
            .x = center->x,
            .y = center->y
        },
        .r = compensationRadius->value,
        .angle1 = 0,    // пересчёт будет в recalcCircleSegment
        .angle2 = 0,    // пересчёт будет в recalcCircleSegment
        .ccw = compensationRadius->side != COMPENSATION_LEFT,
        .p1 = {
            .x = p1->x,
            .y = p1->y
        },
        .p2 = {
            .x = p2->x,
            .y = p2->y
        }
    };
    Geometry::recalcCircleSegment(&circleJoin);

    if(abs(circleJoin.angle2-circleJoin.angle1) >= 0.0174 && abs(circleJoin.angle2-circleJoin.angle1) <= 6.2657){     // > 1 градуса
        ActionMove::circle(&circleJoin, speed, funcFinish);
        // vTaskSuspend(GCode::gcodeTaskHandle);
        return true;
    }

    return false;
}
*/




// void GCodeCRP::funcFinish(){
//     ESP_LOGI(TAG, "static funcFinish");
//     vTaskResume(GCode::gcodeTaskHandle);
// }



