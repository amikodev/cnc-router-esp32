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

#include "ActionMove.hpp"

#define PI 3.14159265
#define EPSG 1e-4

#define TAG "ActionMove"
#define TAG_CIRCLE "ActionMove::Circle"

uint8_t ActionMove::axeMoveCounter = 0;

/**
 * Перемещение до целевой позиции
 * @param axe ось
 * @param targetMM целевая позиция, в мм
 * @param speed скорость, мм/сек
 * @param funcStepDriverFinish функция вызываемая при окончании перемещения по оси
 */
void ActionMove::gotoAxePoint(Axe::AXE axe, float targetMM, float speed, std::function<void (StepDriver *sd)> funcStepDriverFinish){

    StepDriver *sd = Axe::getStepDriver(axe);
    StepDriver::MotorTarget *mt = sd->calcTarget(targetMM, speed);

    if(mt != NULL){
        mt->cPulse = 0;
        mt->callbackFinish = funcStepDriverFinish;

        esp_timer_create_args_t timerRunArgs = {
            .callback = &StepDriver::timerRunCallback,
            .arg = (void *)mt,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "axe_"+sd->getLetter()
        };

        esp_timer_handle_t timerRun;
        // ESP_ERROR_CHECK(esp_timer_create(&timerRunArgs, &timerRun));
        esp_timer_create(&timerRunArgs, &timerRun);

        mt->stepDriver->setTimerRun(timerRun);

        uint64_t mksStep = mt->mksStep;
        if(mksStep < 50) mksStep = 50;
        // ESP_LOGI(TAG, "mksStep: %llu", mksStep);
        // ESP_LOGI(TAG, "\t\t axe: %c, dir: %d, [ %.4f -> %.4f ], _position: %llu, dxPulsed: %d; %s", sd->getLetter(), mt->dir, sd->getPositionMM(), targetMM, sd->getPosition(), mt->dxPulses, (mt->dir?" :: FAIL DIRECTION ::":""));

        sd->setDirection(mt->dir);
        // ESP_ERROR_CHECK(esp_timer_start_periodic(timerRun, mksStep));
        esp_timer_start_periodic(timerRun, mksStep);
    }

}

/**
 * Прямолинейное движение к точке
 * @param point целевая точка
 * @param speed скорость, мм/сек
 * @param doPause включить ожидание до окончания перемещения
 */
void ActionMove::gotoPoint(Geometry::Point *point, float speed, bool doPause){
    Axe::AXES_COUNT axesCount = Axe::getAxesCount();

    // point->log(TAG, "gotoPoint");

    float pvals[axesCount+1] = {0.0};       // целевые координаты
    float dvals[axesCount+1] = {0.0};       // смещение координат

    float length = 0.0;     // длина вектора

    // рассчёт общей длины вектора
    auto calcLength = [&](float pointVal, Axe::AXE axe){
        float dv = 0.0;
        if((uint8_t) axe <= (uint8_t) axesCount){
            StepDriver *sd = Axe::getStepDriver(axe);
            if(!sd->getIsSynced()){
                pvals[axe] = pointVal;
                float curPos = sd->getPositionMM();
                dv = pointVal - curPos;
                // if(abs(dv) < 0.01) dv = 0.0;
                if(abs(dv) < 0.03) dv = 0.0;        // необходимо уменьшать значение 0.3, потому что возможны проблемы с точностью, вероятно нужно будет перейти на double
                if(dv != 0.0){ 
                    length += dv*dv;
                    dvals[axe] = dv;
                    // ESP_LOGI(TAG, "calcLength: axe: %d, pointVal: %f, curPos: %f, dv: %f", axe, pointVal, curPos, dv);
                }
            }
        }
    };

    calcLength(point->x, Axe::AXE_X);
    calcLength(point->y, Axe::AXE_Y);
    calcLength(point->z, Axe::AXE_Z);
    calcLength(point->a, Axe::AXE_A);
    calcLength(point->b, Axe::AXE_B);
    calcLength(point->c, Axe::AXE_C);

    length = sqrt(length);


    uint8_t axeMoveCount = 0;
    ActionMove::axeMoveCounter = 0;
    for(uint8_t i=1; i<(uint8_t) axesCount; i++){
        if(dvals[i] != 0.0){
            axeMoveCount++;
        }
    }

    bool isWait = true;

    std::function<void (StepDriver *sd)> funcStepDriverFinish = [&, axeMoveCount](StepDriver *sd){
        // ESP_LOGI(TAG, "funcStepDriverFinish: axeMoveCount: %d, axeMoveCounter: %d", axeMoveCount, ActionMove::axeMoveCounter+1);
        if(++ActionMove::axeMoveCounter == axeMoveCount){
            ActionMove::axeMoveCounter = 0;
            // ESP_LOGI(TAG, "funcStepDriverFinish");
            isWait = false;
        }
    };

    // перемещение по требуемым осям с соответствующими скоростями
    for(uint8_t i=1; i<(uint8_t) axesCount; i++){
        if(dvals[i] != 0.0){
            float spV = abs(dvals[i])/length*speed;         // разложенная скорость по оси
            ActionMove::gotoAxePoint((Axe::AXE) i, pvals[i], spV, funcStepDriverFinish);
            // ESP_LOGI(TAG, "gotoAxePoint: axe: %c, value: %.4f, speed: %.4f", Axe::getStepDriver((Axe::AXE) i)->getLetter(), pvals[i], spV);
        }
    }

    if(doPause && axeMoveCount > 0){
        while(isWait){
            vTaskDelay(2);
        }
    }
        
}

/**
 * Прямолинейное движение к точке
 * @param point целевая точка
 * @param speed скорость, мм/сек
 * @param doPause включить ожидание до окончания перемещения
 */
void ActionMove::gotoPoint(Geometry::PointXY *point, float speed, bool doPause){
    Geometry::Point point2 = Geometry::getPoint(point);
    return gotoPoint(&point2, speed, doPause);
}

/**
 * Движение по кружности
 * @param circle окружность
 * @param speed скорость, мм/сек
 */
void ActionMove::circle(Geometry::CircleSegment *circle, float speed){
    Geometry::Point defaultPoint = { .x = 0.0, .y = 0.0, .z = Axe::getStepDriver(Axe::AXE_Z)->getPositionMM(), .a = 0.0, .b = 0.0, .c = 0.0};

    float dAngle = 1/circle->r;

    float a1 = circle->angle1;
    float a2 = circle->angle2;

    if(!circle->ccw){
        if(a1 < a2) a1 += 2*PI;
        dAngle = -dAngle;
    } else{
        if(a2 < a1) a2 += 2*PI;
    }

    float cAngle = a1;      // текущий угол
    uint16_t steps = abs((a2-a1)/dAngle);

    // if(circle->r < 1.0){
    //     ESP_LOGI(TAG_CIRCLE, "r: %.4f, a1: %f, a2: %f, dAngle: %f, steps: %d", circle->r, a1*180/PI, a2*180/PI, dAngle*180/PI, steps);
    //     circle->p1.log(TAG_CIRCLE, "p1");
    //     circle->p2.log(TAG_CIRCLE, "p2");
    // }

    if(circle->r < 0.5){         // окружности со слишком малым радиусом игнорируем
        ESP_LOGI(TAG_CIRCLE, "Ignore this circle, draw line");
    } else{
        cAngle += dAngle;
        for(uint16_t i=0; i<steps; i++){        // линейная интерполяция по каждой внутренней точке окружности
            Geometry::PointXY circlePointXY = {
                .x = circle->r * (float)cos(cAngle) + circle->center.x,
                .y = circle->r * (float)sin(cAngle) + circle->center.y
            };
            Geometry::Point circlePoint = Geometry::getPoint(&circlePointXY, defaultPoint);

            // ESP_LOGI(TAG_CIRCLE, "x: %.2f, y: %.2f, angle: %.2f", circlePoint.x, circlePoint.y, cAngle*180/PI);
            gotoPoint(&circlePoint, speed);
            cAngle += dAngle;
        }
    }

    // рисуем дополнительную линию
    if(steps > 0 && !Geometry::pointsIsEqual(&circle->p1, &circle->p2)){
        // ESP_LOGI(TAG_CIRCLE, "goto last point");
        Geometry::Point circlePoint = Geometry::getPoint(&(circle->p2), defaultPoint);
        gotoPoint(&circlePoint, speed);
    }
}

/**
 * Остановка перемещения
 */
void ActionMove::stop(){
    Axe::AXES_COUNT axesCount = Axe::getAxesCount();
    for(uint8_t i=1; i<(uint8_t) axesCount; i++){
        Axe::AXE axe = (Axe::AXE) i;
        StepDriver *sd = Axe::getStepDriver(axe);
        if(!sd->getIsSynced()){
            sd->stop();
            sd->actionRunStop();
        }
    }
}


