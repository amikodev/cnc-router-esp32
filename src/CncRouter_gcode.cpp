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

#include "CncRouter.hpp"


xQueueHandle CncRouter::gcodeFinishEvtQueue = NULL;

TaskHandle_t CncRouter::gcodeTaskHandle = NULL;
TaskHandle_t CncRouter::gcodeDrawCircleTaskHandle = NULL;



/**
 * Установка программы GCode
 * @param ptr указатель на начало программы
 * @param size размер программы, в байтах
 */
void CncRouter::setGcodePtr(void *ptr, uint32_t size){
    if(ptr != NULL){
        removeGcode();
        gcodePtr = ptr;
        gcodeSize = size;
        gcodePtrOffset = 0;
    }
}

/**
 * Последовательное добавление команд GCode в конец списка при загрузке
 * @param data входящие данные
 * @param length размер данных
 */
void CncRouter::gcodeAppend(void *data, uint32_t length){
    memcpy( ((void *) ( ((uint8_t *)gcodePtr)+gcodePtrOffset )), data, length);
    gcodePtrOffset += length;
}


/**
 * Удаление программы GCode из памяти.
 * Освобождение памяти.
 */
void CncRouter::removeGcode(){
    if(gcodePtr != NULL){
        free(gcodePtr);
        gcodePtr = NULL;
        gcodeSize = 0;
        gcodePtrOffset = 0;
    }
}

/**
 * Получение указателя на начало программы
 */
void* CncRouter::getGcodePtr(){
    return gcodePtr;
}

/**
 * Получение размера программы
 */
uint32_t CncRouter::getGcodeSize(){
    return gcodeSize;
}

/**
 * Является ли загруженный GCode доступным для выполнения
 */
bool CncRouter::isGcodeRunnable(){
    return gcodePtr != NULL && gcodeSize > 0;
}

/**
 * Запуск программы GCode
 */
void CncRouter::runGcode(){
    printf("CncRouter::runGcode \n");
    xTaskCreate(CncRouter::gcodeTask, "gcodeTask", 4096, NULL, 10, &CncRouter::gcodeTaskHandle);
}

/**
 * Обработка фрейма программы GCode
 */
bool CncRouter::gcodeProcessFrame(GcodeFrameSubData *frame, uint8_t frameLength){
    printf("CncRouter::gcodeProcessFrame frameLength: %d; %p \n", frameLength, frame);

    bool processThisFrame = true;
    for(uint8_t i=0; i<frameLength; i++){
        GcodeFrameSubData *el = frame+i;
        // printf("frame el: %d, %f \n", el->letter, el->value);
        if(!processThisFrame)       // текущую команду Gxx не обрабатываем
            break;

        switch(el->letter){
            case GCODE_LETTER_G:
                processThisFrame = gcodeProcessCommand_G((uint8_t) el->value, frame, frameLength);
                break;
            case GCODE_LETTER_M:
                processThisFrame = gcodeProcessCommand_M((uint8_t) el->value, frame, frameLength);
                break;

            case GCODE_LETTER_X:
                progParams.targetCoord.x = el->value;
                break;
            case GCODE_LETTER_Y:
                progParams.targetCoord.y = el->value;
                break;
            case GCODE_LETTER_Z:
                progParams.targetCoord.z = el->value;
                break;
            case GCODE_LETTER_A:
                progParams.targetCoord.a = el->value;
                break;
            case GCODE_LETTER_B:
                progParams.targetCoord.b = el->value;
                break;
            case GCODE_LETTER_C:
                progParams.targetCoord.c = el->value;
                break;

            case GCODE_LETTER_I:
                progParams.circle.type = CIRCLE_INC;
                progParams.circle.inc.i = el->value;
                break;
            case GCODE_LETTER_J:
                progParams.circle.type = CIRCLE_INC;
                progParams.circle.inc.j = el->value;
                break;
            case GCODE_LETTER_K:
                progParams.circle.type = CIRCLE_INC;
                progParams.circle.inc.k = el->value;
                break;

            // case GCODE_LETTER_F:
            //     // frame->f = floatValue;
            //     break;

            default:
                break;
        }
    }

    if(!processThisFrame)       // текущую команду Gxx, Mxx не обрабатываем
        return false;

    return true;


    // float targetX = 0.0;
    // float targetY = 0.0;
    // float targetZ = 0.0;

    // if( 
    //     progParams.runType == RUN_FAST || 
    //     progParams.runType == RUN_WORK_LINEAR || 
    //     progParams.runType == RUN_WORK_CW || 
    //     progParams.runType == RUN_WORK_CCW
    // ){
    //     targetX = progParams.targetCoord.x + progParams.systemCoord.x;
    //     targetY = progParams.targetCoord.y + progParams.systemCoord.y;
    //     targetZ = progParams.targetCoord.z + progParams.systemCoord.z;
    // }

    // // printf("progParams.runType: %d, targetX: %f, targetY: %f \n", progParams.runType, targetX, targetY);

    // if(progParams.pause > 0.0){
    //     // пауза выполнения программы
    //     gcodeProcess_pause();
    // }

    // if(progParams.runType == RUN_FAST){                         // быстрое линейное перемещение
    //     gcodeProcess_drawLine(targetX, targetY, targetZ);
    //     gcodeRecalcCoords(targetX, targetY, targetZ);
    // } else if(progParams.runType == RUN_WORK_LINEAR){           // рабочее линейное перемещение
    //     gcodeProcess_drawLine(targetX, targetY, targetZ);
    //     gcodeRecalcCoords(targetX, targetY, targetZ);
    // } else if(progParams.runType == RUN_WORK_CW){               // перемещение по окружности по часовой стрелке
    //     progParams.circle.type == CIRCLE_RADIUS ? gcodeProcess_calcCircleByRadius(targetX, targetY, targetZ, &gcodeCircleParams) : gcodeProcess_calcCircleByInc(targetX, targetY, targetZ, &gcodeCircleParams);
    //     gcodeProcess_drawCircle(&gcodeCircleParams);
    //     gcodeRecalcCoords(targetX, targetY, targetZ);
    // } else if(progParams.runType == RUN_WORK_CCW){              // перемещение по окружности против часовой стрелки
    //     progParams.circle.type == CIRCLE_RADIUS ? gcodeProcess_calcCircleByRadius(targetX, targetY, targetZ, &gcodeCircleParams) : gcodeProcess_calcCircleByInc(targetX, targetY, targetZ, &gcodeCircleParams);
    //     gcodeProcess_drawCircle(&gcodeCircleParams);
    //     gcodeRecalcCoords(targetX, targetY, targetZ);
    // }

    // if(progParams.plasmaArc == PLASMA_ARC_START){               // запуск плазмы
    //     gcodeProcess_plasmaStart();
    //     progParams.plasmaArc = PLASMA_ARC_NONE;
    // } else if(progParams.plasmaArc == PLASMA_ARC_STOP){         // остановка плазмы
    //     gcodeProcess_plasmaStop();
    //     progParams.plasmaArc = PLASMA_ARC_NONE;
    // }


}

/**
 * Обработка команды Gxx
 */
bool CncRouter::gcodeProcessCommand_G(uint8_t value, GcodeFrameSubData *frame, uint8_t frameLength){
    bool processThisCommand = true;

    switch(value){
        case 0:     // G00 - Ускоренное перемещение инструмента (холостой ход)
            progParams.runType = RUN_FAST;
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_F:    // задание скорости перемещения, mm/min
                        progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 1:     // G01 - Линейная интерполяция, скорость перемещения задаётся здесь же или ранее модальной командой F
            progParams.runType = RUN_WORK_LINEAR;
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_F:    // задание скорости перемещения, mm/min
                        progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 2:     // G02 - Круговая интерполяция по часовой стрелке
            progParams.runType = RUN_WORK_CW;
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_R:
                        progParams.circle.type = CIRCLE_RADIUS;
                        progParams.circle.radius = el->value;
                        break;
                    case GCODE_LETTER_I:
                        progParams.circle.type = CIRCLE_INC;
                        progParams.circle.inc.i = el->value;
                        break;
                    case GCODE_LETTER_J:
                        progParams.circle.type = CIRCLE_INC;
                        progParams.circle.inc.j = el->value;
                        break;
                    case GCODE_LETTER_K:
                        progParams.circle.type = CIRCLE_INC;
                        progParams.circle.inc.k = el->value;
                        break;
                    case GCODE_LETTER_F:    // задание скорости перемещения, mm/min
                        progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 3:     // G03 - Круговая интерполяция против часовой стрелки
            progParams.runType = RUN_WORK_CCW;
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_R:
                        progParams.circle.type = CIRCLE_RADIUS;
                        progParams.circle.radius = el->value;
                        break;
                    case GCODE_LETTER_I:
                        progParams.circle.type = CIRCLE_INC;
                        progParams.circle.inc.i = el->value;
                        break;
                    case GCODE_LETTER_J:
                        progParams.circle.type = CIRCLE_INC;
                        progParams.circle.inc.j = el->value;
                        break;
                    case GCODE_LETTER_K:
                        progParams.circle.type = CIRCLE_INC;
                        progParams.circle.inc.k = el->value;
                        break;
                    case GCODE_LETTER_F:    // задание скорости перемещения, mm/min
                        progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 4:     // G04 - Задержка выполнения программы, способ задания величины задержки зависит от реализации системы управления, P обычно задает паузу в миллисекундах, X — в секундах.
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_P:        // пауза в миллисекундах
                        progParams.pause = el->value/1000;
                        break;
                    case GCODE_LETTER_X:        // пауза в секундах
                        progParams.pause = el->value;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 10:    // G10 - Переключение абсолютной системы координат
        case 15:    // G15 - Отмена полярной системы координат
        case 16:    // G16 - Переход в полярную систему координат (X радиус Y угол)
        case 17:    // G17 - Выбор рабочей плоскости X-Y
        case 18:    // G18 - Выбор рабочей плоскости Z-X
        case 19:    // G19 - Выбор рабочей плоскости Y-Z
            break;
        case 20:    // G20 - Режим работы в дюймовой системе
            progParams.unit = UNIT_INCH;
            break;
        case 21:    // G21 - Режим работы в метрической системе
            progParams.unit = UNIT_METRIC;
            break;
        case 28:    // G28 - Вернуться на референтную точку
        case 30:    // G30 - Поднятие по оси Z на точку смены инструмента
        case 31:    // G31 - Подача до пропуска
        case 40:    // G40 - Отмена компенсации радиуса инструмента
            progParams.compensationRadius.type = GCodeCR::COMPENSATION_NONE;
            progParams.compensationRadius.value = 0.0;
            break;
        case 41:    // G41 - Компенсировать радиус инструмента слева от траектории
            progParams.compensationRadius.type = GCodeCR::COMPENSATION_LEFT;
            progParams.compensationRadius.value = 1.0;      // радиус по умолчанию
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_R:        // радиус
                        progParams.compensationRadius.value = el->value;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 42:    // G42 - Компенсировать радиус инструмента справа от траектории
            progParams.compensationRadius.type = GCodeCR::COMPENSATION_RIGHT;
            progParams.compensationRadius.value = 1.0;      // радиус по умолчанию
            for(uint8_t i=0; i<frameLength; i++){
                GcodeFrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_R:        // радиус
                        progParams.compensationRadius.value = el->value;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 43:    // G43 - Компенсировать длину инструмента положительно
            // progParams.compensationLength.type = COMPENSATION_POS;
            // progParams.compensationLength.value = 0.0;
            break;
        case 44:    // G44 - Компенсировать длину инструмента отрицательно
            // progParams.compensationLength.type = COMPENSATION_NEG;
            // progParams.compensationLength.value = 0.0;
            break;
        case 49:    // G49 - Отмена компенсации длины инструмента
            // progParams.compensationLength.type = COMPENSATION_NONE;
            // progParams.compensationLength.value = 0.0;
            break;
        case 50:    // G50 - Сброс всех масштабирующих коэффициентов в 1.0
        case 51:    // G51 - Назначение масштабов
        case 53:    // G53 - Переход в систему координат станка
        case 54:    // G54 - Переключиться на заданную оператором систему координат
        case 68:    // G68 - Поворот координат на нужный угол
        case 70:    // G70 - Цикл продольного чистового точения
        case 71:    // G71 - Цикл многопроходного продольного чернового точения
        case 80:    // G80 - Отмена циклов сверления, растачивания, нарезания резьбы метчиком и т. д.
        case 81:    // G81 - Цикл сверления
        case 82:    // G82 - Цикл сверления с задержкой
        case 83:    // G83 - Цикл прерывистого сверления (с полным выводом сверла)
        case 84:    // G84 - Цикл нарезания резьбы
            break;
        case 90:    // G90 - Задание абсолютных координат опорных точек траектории
            progParams.coordSystem = COORD_ABSOLUTE;
            memcpy(&progParams.targetCoord, &progParams.currentCoord, sizeof(GcodeCoord));
            memcpy(&progParams.systemCoord, &coordNull, sizeof(GcodeCoord));
            break;
        case 91:    // G91 - Задание координат инкрементально последней введённой опорной точки
            progParams.coordSystem = COORD_RELATIVE;
            memcpy(&progParams.targetCoord, &coordNull, sizeof(GcodeCoord));
            memcpy(&progParams.systemCoord, &progParams.currentCoord, sizeof(GcodeCoord));
            break;
        case 92:
            processThisCommand = false;
            break;
        case 94:    // G94 - F (подача) — в формате мм/мин
        case 95:    // G95 - F (подача) — в формате мм/об
        case 99:    // G99 - После каждого цикла не отходить на «проходную точку»

            break;
        default:
            break;
    }

    return processThisCommand;
}

/**
 * Обработка команды Mxx
 */
bool CncRouter::gcodeProcessCommand_M(uint8_t value, GcodeFrameSubData *frame, uint8_t frameLength){
    bool processThisCommand = true;

    switch(value){
        case 3:     // M03 - включить плазму
        case 7:     // M07 - включить плазму
            if(!_testRunChecked){       // тестовый прогон программы gcode выключен
                if(_plasma != NULL){
                    if(!_plasma->getArcStarted()){      // плазма не включена, включаем
                        progParams.plasmaArc = PLASMA_ARC_START;
                    }
                }
            }
            break;
        case 5:     // M05 - выключить плазму
        case 8:     // M08 - выключить плазму
            if(!_testRunChecked){       // тестовый прогон программы gcode выключен
                if(_plasma != NULL){
                    progParams.plasmaArc = PLASMA_ARC_STOP;
                }
            }
            break;
        default:
            break;
    }

    return processThisCommand;
}


/**
 * Пересчёт текущих координат
 * @param targetX целевая координата X
 * @param targetY целевая координата Y
 * @param targetZ целевая координата Z
 */
void CncRouter::gcodeRecalcCoords(float targetX, float targetY, float targetZ){
    progParams.currentCoord.x = targetX;
    progParams.currentCoord.y = targetY;
    progParams.currentCoord.z = targetZ;
}

/**
 * Пересчёт текущих координат
 * @param target целевая точка
 */
void CncRouter::gcodeRecalcCoords(GcodeCoord *target){
    memcpy(&progParams.currentCoord, target, sizeof(GcodeCoord));
}

/**
 * Программа GCode.
 * Рисование прямой линии.
 * @param targetX целевая координата X
 * @param targetY целевая координата Y
 * @param targetZ целевая координата Z
 */
void CncRouter::gcodeProcess_drawLine(float targetX, float targetY, float targetZ){
    printf("CncRouter::gcodeProcess_drawLine %f, %f \n", targetX, targetY);

    float dx = targetX - progParams.currentCoord.x;
    float dy = targetY - progParams.currentCoord.y;
    float dz = targetZ - progParams.currentCoord.z;

    if(abs(dx) < 0.01) dx = 0.0;
    if(abs(dy) < 0.01) dy = 0.0;
    if(abs(dz) < 0.01) dz = 0.0;

    float length = 0.0;
    if(dx != 0) length += dx*dx;
    if(dy != 0) length += dy*dy;
    if(dz != 0) length += dz*dz;
    length = sqrt(length);

    // float speed = progParams.runType == RUN_FAST ? 300 : 100;       // mm/sec
    // float speed = progParams.runType == RUN_FAST ? 70 : 40;       // mm/sec
    float speed = progParams.speed;     // mm/sec

    progParams.finishCount = 0;
    if(dx != 0)
        progParams.finishCount++;
    if(dy != 0)
        progParams.finishCount++;
    if(dz != 0)
        progParams.finishCount++;

    if(dx != 0){
        float spX = abs(dx)/length*speed;    // разложенная скорость по оси X
        gotoTargetMM(AXE_X, targetX, spX, CncRouter::gcodeGotoFinish);
    }
    if(dy!= 0){
        float spY = abs(dy)/length*speed;    // разложенная скорость по оси Y
        gotoTargetMM(AXE_Y, targetY, spY, CncRouter::gcodeGotoFinish);
    }
    if(dz != 0){
        float spZ = abs(dz)/length*speed;    // разложенная скорость по оси Z
        gotoTargetMM(AXE_Z, targetZ, spZ, CncRouter::gcodeGotoFinish);
    }

    if(progParams.finishCount > 0){
        vTaskSuspend(NULL);     // приостановить текущую задачу
    //     // переходим к следующему фрейму
    //     vTaskResume(CncRouter::gcodeTaskHandle);
    }

}

/**
 * Программа GCode.
 * Рисование окружности.
 * 
 */
void CncRouter::gcodeProcess_drawCircle(CncRouter::GcodeCircleParams *cp){
    xTaskCreate(CncRouter::gcodeProcess_drawCircleTask, "drawCircleTask", 4096, (void *)cp, 10, &CncRouter::gcodeDrawCircleTaskHandle);
    vTaskSuspend(CncRouter::gcodeTaskHandle);
}

/**
 * Задача рисования дуги окружности
 */
void CncRouter::gcodeProcess_drawCircleTask(void *arg){
    printf("CncRouter::gcodeProcess_drawCircleTask \n");
    ProgParams *progParams = instance->getProgParams();
    GcodeCircleParams *cp = (GcodeCircleParams *)arg;

    printf("circle radius: %f, a1: %f, a2: %f, ccw: %d \n", cp->radius, cp->angle1, cp->angle2, cp->ccw);

    uint16_t maxSteps = 360;
    float fullCircleSteps = (float) maxSteps;

    float circleLength = 2*PI*cp->radius;
    fullCircleSteps = circleLength;

    float dAngle = 2*PI/fullCircleSteps;

    float a1 = cp->angle1;
    float a2 = cp->angle2;

    if(!cp->ccw){
        if(a1 < a2)
            a1 += 2*PI;
        dAngle = -dAngle;
    } else{
        if(a2 < a1)
            a2 += 2*PI;
    }

    float aCurrent = a1;
    uint16_t steps = abs((a2-a1)/dAngle);
    uint8_t finishCount = 0;

    std::function<void (StepDriver *sd)> funcFinish = [&](StepDriver *sd){
        if(finishCount > 0){
            finishCount--;
            if(finishCount == 0){
                vTaskResume(gcodeDrawCircleTaskHandle);
            }
        }
    };

    printf("dAngle: %f, steps: %d \n", dAngle, steps);

    float x = progParams->currentCoord.x;
    float y = progParams->currentCoord.y;
    // float z = progParams->currentCoord.z;

    float targetX, targetY, dx, dy, length, spX, spY;
    // float speed = 100;      // mm/sec
    // float speed = 40;      // mm/sec
    float speed = progParams->speed;     // mm/sec


    auto funcLine = [&](float targetX, float targetY){
        dx = targetX - progParams->currentCoord.x;
        dy = targetY - progParams->currentCoord.y;
        // float dz = targetZ - progParams->currentCoord.z;

        dx = abs(dx) < 0.01 ? 0.0 : dx;
        dy = abs(dy) < 0.01 ? 0.0 : dy;
        // dz = abs(dz) < 0.01 ? 0.0 : dz;

        length = 0.0;
        if(dx != 0) length += dx*dx;
        if(dy != 0) length += dy*dy;
        // if(dz != 0) length += dz*dz;
        length = sqrt(length);

        finishCount = 0;
        if(dx != 0)
            finishCount++;
        if(dy != 0)
            finishCount++;

        if(dx != 0){
            spX = abs(dx)/length*speed;    // разложенная скорость по оси X
            instance->gotoTargetMM(AXE_X, targetX, spX, funcFinish);
        }
        if(dy != 0){
            spY = abs(dy)/length*speed;    // разложенная скорость по оси Y
            instance->gotoTargetMM(AXE_Y, targetY, spY, funcFinish);
        }

        if(finishCount > 0){
            vTaskSuspend(CncRouter::gcodeDrawCircleTaskHandle);
        }

        progParams->currentCoord.x = targetX;
        progParams->currentCoord.y = targetY;

    };

    if(cp->radius < 0.5){       // окружности со слишком малым радиусом игнорируем
        printf("Ignore this circle, draw line \n");
    } else{
        printf("current x: %f, y: %f \n", x, y);
        aCurrent += dAngle;
        for(uint16_t i=0; i<steps; i++){
            targetX = cp->radius * cos(aCurrent) + cp->xc;
            targetY = cp->radius * sin(aCurrent) + cp->yc;
            // float targetZ = z;
            funcLine(targetX, targetY);
            aCurrent += dAngle;
        }
    }

    printf("target x: %f, y: %f \n", cp->target.x, cp->target.y);

    if(x != cp->target.x || y != cp->target.y){
        // рисуем дополнительную линию
        targetX = cp->target.x;
        targetY = cp->target.y;
        funcLine(targetX, targetY);
    }

    printf("CncRouter::gcodeProcess_drawCircleTask FINISH \n");
    vTaskResume(gcodeTaskHandle);
    vTaskDelete(NULL);
}



/**
 * Программа GCode.
 * Расчёт окружности с учётом параметра радиуса.
 * @param targetX целевая координата X
 * @param targetY целевая координата Y
 * @param targetZ целевая координата Z
 */
void CncRouter::gcodeProcess_calcCircleByRadius(float targetX, float targetY, float targetZ, CncRouter::GcodeCircleParams *cp){

    printf("CncRouter::gcodeProcess_calcCircleByRadius \n");

    float x1 = progParams.currentCoord.x;
    float y1 = progParams.currentCoord.y;
    float x2 = targetX;
    float y2 = targetY;
    float dx = x2-x1;
    float dy = y2-y1;
    float r = progParams.circle.radius;

    bool radiusPositive = r >= 0;
    r = abs(r);

    float d = sqrt(dx*dx+dy*dy);
    float h = sqrt(r*r-(d/2)*(d/2));

    float xc1 = x1 + dx/2 + h*dy / d;
    float yc1 = y1 + dy/2 - h*dx / d;

    float xc2 = x1 + dx/2 - h*dy / d;
    float yc2 = y1 + dy/2 + h*dx / d;


    float xc = xc2;
    float yc = yc2;

    if( (progParams.runType == RUN_WORK_CW && radiusPositive) || (progParams.runType == RUN_WORK_CCW && !radiusPositive) ){
        xc = xc1;
        yc = yc1;
    }

    float dxc1 = x1-xc;
    float dyc1 = y1-yc;
    float dxc2 = x2-xc;
    float dyc2 = y2-yc;

    float angle1 = atan2(dyc1, dxc1);
    float angle2 = atan2(dyc2, dxc2);

    bool ccw = progParams.runType == RUN_WORK_CCW;

    cp->xc = xc;
    cp->yc = yc;
    cp->radius = r;
    cp->angle1 = angle1;
    cp->angle2 = angle2;
    cp->ccw = ccw;

    cp->target.x = targetX;
    cp->target.y = targetY;
    cp->target.z = targetZ;
}

/**
 * Программа GCode.
 * Расчёт окружности с учётом параметров смещения центра.
 * @param targetX целевая координата X
 * @param targetY целевая координата Y
 * @param targetZ целевая координата Z
 */
void CncRouter::gcodeProcess_calcCircleByInc(float targetX, float targetY, float targetZ, CncRouter::GcodeCircleParams *cp){

    printf("CncRouter::gcodeProcess_calcCircleByInc \n");

    float x1 = progParams.currentCoord.x;
    float y1 = progParams.currentCoord.y;
    float x2 = targetX;
    float y2 = targetY;

    float incI = progParams.circle.inc.i;
    float incJ = progParams.circle.inc.j;

    float xc = progParams.currentCoord.x + incI;
    float yc = progParams.currentCoord.y + incJ;

    float r = sqrt(incI*incI + incJ*incJ);

    float dxc1 = x1-xc;
    float dyc1 = y1-yc;
    float dxc2 = x2-xc;
    float dyc2 = y2-yc;

    float angle1 = atan2(dyc1, dxc1);
    float angle2 = atan2(dyc2, dxc2);

    float ccw = progParams.runType == RUN_WORK_CCW;

    cp->xc = xc;
    cp->yc = yc;
    cp->radius = r;
    cp->angle1 = angle1;
    cp->angle2 = angle2;
    cp->ccw = ccw;

    cp->target.x = targetX;
    cp->target.y = targetY;
    cp->target.z = targetZ;
}

/**
 * Программа GCode.
 * Пауза выполнения программы.
 */
void CncRouter::gcodeProcess_pause(){
    printf("GCode pause: %f sec \n", progParams.pause);
    esp_timer_create_args_t timerPauseArgs = {
        .callback = &CncRouter::gcodeProcess_pauseFinish,
        .arg = NULL,
        .name = "gcode_pause"
    };

    esp_timer_handle_t timerPause;
    ESP_ERROR_CHECK(esp_timer_create(&timerPauseArgs, &timerPause));
    ESP_ERROR_CHECK(esp_timer_start_once(timerPause, (uint64_t) (progParams.pause*1e6)));

    // задача gcode приостанавливается и
    // возобновляется в функции funcPauseCallback
    vTaskSuspend(CncRouter::gcodeTaskHandle);
    
    printf("GCode pause resume \n");
    progParams.pause = 0.0;
}

/**
 * Программа GCode.
 * Завершение паузы выполнения программы.
 */
void CncRouter::gcodeProcess_pauseFinish(void *arg){
    // возобновление основной задачи gcode
    vTaskResume(CncRouter::gcodeTaskHandle);
}

/**
 * Функция вызываемая при окончании перемещения по оси
 * @param axe ось
 */
void CncRouter::gcodeGotoFinish(StepDriver *sd){
    printf("CncRouter::gcodeGotoFinish axe: %c \n", sd->getLetter());

    ProgParams *progParams = instance->getProgParams();
    if(progParams->finishCount > 0){
        progParams->finishCount--;
        if(progParams->finishCount == 0){
            vTaskResume(gcodeTaskHandle);
        }
    }
}

/**
 * Программа GCode.
 * Включить плазму.
 */
void CncRouter::gcodeProcess_plasmaStart(){
    if(_plasma != NULL){
        _plasma->start();
        // задача gcode приостанавливается и
        // возобновляется в методе CncRouter::plasmaArcStartedCallback
        // ожидая подтверждения о начале резки от аппарата плазменной резки
        vTaskSuspend(CncRouter::gcodeTaskHandle);
    }
}

/**
 * Программа GCode.
 * Выключить плазму.
 */
void CncRouter::gcodeProcess_plasmaStop(){
    if(_plasma != NULL){
        _plasma->stop();
    }
}

/**
 * Задача выполнения программы GCode
 */
void CncRouter::gcodeTask(void *arg){
    printf("CncRouter::gcodeTask \n");

    void *ptr = instance->getGcodePtr();
    uint32_t size = instance->getGcodeSize();

    uint8_t data[16] = {0};
    uint8_t dataResp[16] = {0};

    uint32_t gcodeFramePtrOffset = 0;

    GcodeFrameSubData frame[10];
    uint8_t frameLength = 0;

    uint32_t numLine = 0;       // номер строки

    GcodeCoord targetPoint;
    MoveParams *pMoveParams[2] = {NULL, NULL};
    GcodeCoord p2;

    for(;;){
        uint8_t sDataInd = 0;
        frameLength = 0;

        // printf("gcodeFramePtrOffset: %d \n", gcodeFramePtrOffset);
        bool reqNextData = false;       // ожидается продолжение данных
        while(true){
            memcpy(&data, ((void *) ( ((uint8_t *)ptr)+gcodeFramePtrOffset )), 16);
            sDataInd = 0;
            if(!reqNextData){
                numLine = *(data+1) + (*(data+2)<<8) ;
                sDataInd = 3;       // 0 - OBJ_NAME_CNC_GCODE, 1,2 - номер строки
                printf("%d == ", numLine);

                // отправка по WebSocket обратно клиенту номера текущей строки выполняемого GCode
                // dataResp = {data[0], data[1], data[2]};
                memcpy(&dataResp, &data, 16);
                xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
            } else{

            }

            while(sDataInd < 16){
                GCODE_LETTER letter = (GCODE_LETTER) data[sDataInd];
                uint8_t lenValue = 1;
                uint8_t intValue = 0;
                float floatValue = 0.0;

                if(letter == GCODE_LETTER_NONE)
                    break;

                switch(letter){
                    case GCODE_LETTER_X:
                    case GCODE_LETTER_Y:
                    case GCODE_LETTER_Z:
                    case GCODE_LETTER_A:
                    case GCODE_LETTER_B:
                    case GCODE_LETTER_C:

                    case GCODE_LETTER_P:
                    case GCODE_LETTER_F:
                    case GCODE_LETTER_S:
                    case GCODE_LETTER_R:
                    case GCODE_LETTER_D:
                    case GCODE_LETTER_L:
                    case GCODE_LETTER_I:
                    case GCODE_LETTER_J:
                    case GCODE_LETTER_K:

                        lenValue = 4;
                        break;
                    default:
                        break;
                }
                if(lenValue == 1){
                    memcpy(&intValue, data+sDataInd+1, lenValue);
                    floatValue = (float) intValue;
                    // intValue = data[sDataInd+1];
                    printf("%d:%f ", letter, floatValue);
                } else if(lenValue == 4){
                    memcpy(&floatValue, data+sDataInd+1, lenValue);
                    printf("%d:%f ", letter, floatValue);
                }

                frame[frameLength].letter = letter;
                frame[frameLength].value = floatValue;
                frameLength++;

                sDataInd += lenValue+1;
            }

            if(data[15] == 0xFF){
                // ожидается продолжение данных
                reqNextData = true;
            } else{
                reqNextData = false;
                printf("\n");
                break;
            }

            gcodeFramePtrOffset += 16;
        }

        if(instance->gcodeProcessFrame(frame, frameLength)){
            ProgParams *pParams = new ProgParams();
            memcpy(pParams, instance->getProgParams(), sizeof(ProgParams));

            targetPoint.x = pParams->targetCoord.x + pParams->systemCoord.x;
            targetPoint.y = pParams->targetCoord.y + pParams->systemCoord.y;
            targetPoint.z = pParams->targetCoord.z + pParams->systemCoord.z;

            // printf("targetPoint: x=%f, y=%f \n", targetPoint.x, targetPoint.y);

            if(pMoveParams[0] != NULL){

                uint8_t count = 1 + (pMoveParams[1] != NULL ? 1 : 0);
                memcpy(&p2, &(pMoveParams[count-1]->target), sizeof(GcodeCoord));
                if(!gcodeProcess_pointsIsEqual(&targetPoint, &p2)){
                    if(count == 2){
                        memcpy(pMoveParams[0], pMoveParams[1], sizeof(MoveParams));
                    }
                    MoveParams *mp1 = new MoveParams();
                    mp1->target = targetPoint;
                    mp1->pParams = *pParams;
                    pMoveParams[1] = mp1;

                    instance->gcodeProcessMove(pMoveParams[0], pMoveParams[1]);
                    instance->gcodeRecalcCoords(&targetPoint);
                }

            } else{
                MoveParams *mp0 = new MoveParams();
                mp0->target = targetPoint;
                mp0->pParams = *pParams;
                pMoveParams[0] = mp0;
                instance->gcodeRecalcCoords(&targetPoint);
            }

        }

        gcodeFramePtrOffset += 16;

        if(gcodeFramePtrOffset >= size){
            // достигнут конец программы

            if(pMoveParams[0] != NULL && pMoveParams[1] != NULL){
                memcpy(pMoveParams[0], pMoveParams[1], sizeof(MoveParams));

                instance->gcodeProcessMove(pMoveParams[0], NULL);
                instance->gcodeRecalcCoords(&targetPoint);

                // free(&(pMoveParams[0]));
                // free(&(pMoveParams[1]));

                // pMoveParams[0] = NULL;
                // pMoveParams[1] = NULL;
            }

            // завершаем задачу
            printf("CncRouter::gcodeTask FINISH \n");
            vTaskDelete(NULL);
        }
    }
}

/**
 * Получение параметров программы GCode
 */
CncRouter::ProgParams* CncRouter::getProgParams(){
    return &progParams;
}

/**
 * Определение являются ли две точки с одними координатами
 * @param p1 первая точка
 * @param p2 вторая точка
 */
bool CncRouter::gcodeProcess_pointsIsEqual(GcodeCoord *p1, GcodeCoord *p2){
    return 
        p1->x == p2->x &&
        p1->y == p2->y &&
        p1->z == p2->z &&
        p1->a == p2->a &&
        p1->b == p2->b &&
        p1->c == p2->c
    ;
}

void CncRouter::gcodeProcessMove(MoveParams *pMoveParams, MoveParams *pMoveNextParams){
    ProgParams pParams = pMoveParams->pParams;
    if(pParams.runType == RUN_FAST || pParams.runType == RUN_WORK_LINEAR){
        gcodeProcess_drawLine(pMoveParams, pMoveNextParams);
    } else if(pParams.runType == RUN_WORK_CW || pParams.runType == RUN_WORK_CCW){
        gcodeProcess_drawCircle(pMoveParams, pMoveNextParams);
    }
}

/**
 * Программа GCode.
 * Рисование прямой линии.
 * 
 */
void CncRouter::gcodeProcess_drawLine(MoveParams *pMoveParams, MoveParams *pMoveNextParams){
    printf("CncRouter::gcodeProcess_drawLine %f, %f \n", pMoveParams->target.x, pMoveParams->target.y);

    ProgParams pParams = pMoveParams->pParams;
    float speed = pMoveParams->pParams.speed;

    GCode::PointXY p1 = {
        .x = pParams.currentCoord.x + pParams.userZeroPoint.x,
        .y = pParams.currentCoord.y + pParams.userZeroPoint.y
    };

    GCode::PointXY p2 = {
        .x = pMoveParams->target.x + pParams.userZeroPoint.x,
        .y = pMoveParams->target.y + pParams.userZeroPoint.y
    };

    // GcodeCoord *p1 = new GcodeCoord();
    // p1->x = pParams.currentCoord.x + pParams.userZeroPoint.x;
    // p1->y = pParams.currentCoord.y + pParams.userZeroPoint.y;

    // GcodeCoord *p2 = new GcodeCoord();
    // p2->x = pMoveParams->target.x + pParams.userZeroPoint.x;
    // p2->y = pMoveParams->target.y + pParams.userZeroPoint.y;

    // gotoTargetMM(&pMoveParams->target, &pMoveParams->pParams.currentCoord, speed, CncRouter::gcodeGotoFinish);
    if(pParams.compensationRadius.type == GCodeCR::COMPENSATION_NONE){
        gotoTargetMM(&pMoveParams->target, &pParams.currentCoord, speed, CncRouter::gcodeGotoFinish);

    } else if(pParams.runType != RUN_FAST && pParams.compensationRadius.type != GCodeCR::COMPENSATION_NONE){
        GCodeCR::COMPENSATION_TYPE side = pParams.compensationRadius.type;
        float length = pParams.compensationRadius.value;

        if(side == GCodeCR::COMPENSATION_LEFT){
            lastPath.left = gcodeProcess_drawLineOffset(&p1, &p2, length, side, speed);
        } else{
            lastPath.right = gcodeProcess_drawLineOffset(&p1, &p2, length, side, speed);
        }

        lastPath.path = {
            .exists = true,
            .type = TYPE_LINE,
            .side = side
        };

    } else{

        if(lastPath.path.exists){
            GCodeCR::COMPENSATION_TYPE side = lastPath.path.side;
            LastPathBySide lastPathBySide = side == GCodeCR::COMPENSATION_LEFT ? lastPath.left : lastPath.right;
            if(lastPathBySide.exists){
                if(lastPath.path.type == TYPE_LINE){

                    GCode::PointXY *p3 = lastPathBySide.lastEndPoint != NULL ? lastPathBySide.lastEndPoint : &lastPathBySide.p1;
                    GCode::PointXY *p4 = &lastPathBySide.p2;

                    gotoTargetMM(p4, p3, speed, CncRouter::gcodeGotoFinish);
                    gotoTargetMM(&p1, p4, speed, CncRouter::gcodeGotoFinish);

                } else if(lastPath.path.type == TYPE_CIRCLE){

                    // LastPathBySideCircle

                    
                }

            }
        }

        lastPath.path.exists = false;
        lastPath.left.exists = false;
        lastPath.right.exists = false;
    }



}


CncRouter::LastPathBySide CncRouter::gcodeProcess_drawLineOffset(GCode::PointXY *p1, GCode::PointXY *p2, float length, GCodeCR::COMPENSATION_TYPE side, float speed){
    // LastPathBySide ret;

// let { p1p, p2p, anglePerp } = GCodeCR.calcLineOffset(p1, p2, length, side);
    GCodeCR::LineOffset lineOffset = GCodeCR::calcLineOffset(p1, p2, length, side);
    GCode::PointXY p1p = lineOffset.p1p;
    GCode::PointXY p2p = lineOffset.p2p;
    float anglePerp = lineOffset.anglePerp;

    GCode::PointXY *lastEndPoint = NULL;

    if(lastPath.path.exists){

        LastPathBySide lastPathBySide = side == GCodeCR::COMPENSATION_LEFT ? lastPath.left : lastPath.right;
        lastEndPoint = lastPathBySide.lastEndPoint;

        if(lastEndPoint == NULL){
            lastEndPoint = &lastPathBySide.p1;
        }

        if(lastPath.path.type == TYPE_LINE){

                    // точка пересечения отрезка на предыдущем участке пути и
                    // отрезком на текущем
            GCodeCR::CrossPoint crossPoint;
            bool crossPointFounded = false;
            try{
                // crossPoint = GCodeCR::calcCrossLineCircle(&lastPathBySide.p1, &lastPathBySide.p2, )
                crossPoint = GCodeCR::calcCrossLines(&lastPathBySide.p1, &lastPathBySide.p2, &p1p, &p2p);
                crossPointFounded = true;
            } catch(const std::runtime_error& error){
            }

            if(crossPointFounded && crossPoint.inner){       // точка пересечения внутри

                // отрезок на предыдущем участке пути
                // до точки пересечения отрезков
                gotoTargetMM(&crossPoint.point, lastEndPoint, speed, CncRouter::gcodeGotoFinish);

            } else{                     // окружность снаружи

                // отрезок на предыдущем участке пути
                // до конечной точки этого отрезка
                gotoTargetMM(&lastPathBySide.p2, lastEndPoint, speed, CncRouter::gcodeGotoFinish);

                if(!GCode::pointsIsEqual(p1, p2)){
                    // соединяющий сегмент окружности

                    float angle3 = lastPathBySide.anglePerp;
                    float angle4 = anglePerp;

                    if(abs(angle4-angle3) >= 0.0174 && abs(angle4-angle3) <= 6.2657){       // > 1 градуса
// _ctx.arc(p1.x, p1.y, length, angle3, angle4, side === COMPENSATION_LEFT ? true : false);
                        // gcodeCircleParams.
                        // gcodeProcess_drawCircle(&gcodeCircleParams);
                    }

                }



            }

            lastEndPoint = crossPointFounded && crossPoint.inner ? &crossPoint.point : NULL;

        } else if(lastPath.path.type == TYPE_CIRCLE){



        }

    } else{
        // переход к точке коррекции радиуса инструмента
        gotoTargetMM(&p1p, p1, speed, CncRouter::gcodeGotoFinish);

    }

    LastPathBySide ret = {
        .exists = true,
        .p1 = p1p, 
        .p2 = p2p,
        .anglePerp = anglePerp,
        .lastEndPoint = lastEndPoint
    };
    return ret;
}


/**
 * Программа GCode.
 * Рисование окружности.
 * 
 */
void CncRouter::gcodeProcess_drawCircle(MoveParams *pMoveParams, MoveParams *pMoveNextParams){

}


