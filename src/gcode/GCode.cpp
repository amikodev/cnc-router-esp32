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

#define TAG "GCode"

#define __POINT_NULL ((Geometry::Point){ .x=0, .y=0, .z=0, .a=0, .b=0, .c=0 })
#define __POINTXY_NULL ((Geometry::PointXY){ .x=0, .y=0 })
#define __CIRCLE_NULL ((Geometry::CircleSegment){ \
    .center=__POINTXY_NULL, .r=0, .angle1=0, .angle2=0, .ccw=false, \
    .p1=__POINTXY_NULL, .p2=__POINTXY_NULL \
})

#define DEFAULT_PROG(...) ((ProgParams){ \
    .numLine = 0, \
    .coordType = COORD_ABSOLUTE, \
    .runType = RUN_FAST, \
    .currentCoord = __POINT_NULL, \
    .targetCoord = __POINT_NULL, \
    .offsetCoord = __POINT_NULL, \
    .unit = UNIT_METRIC, \
    .circle = { .type = CIRCLE_RADIUS, .radius = 0, .inc = { .i=0, .j=0, .k=0 } }, \
    .circleSegment = __CIRCLE_NULL, \
    .compensationRadius = { .side = GCodeCR::COMPENSATION_NONE, .value = 0 }, \
    .plasmaArc = Plasma::PLASMA_ARC_NONE, \
    .speed = 0, \
    .pause = 0, \
    .probe = { .point = __POINT_NULL, .speed = 0.0, .backOffset = 5.0 }, \
    ##__VA_ARGS__ \
})


void* GCode::_ptr = NULL;
uint32_t GCode::_size = 0;
uint32_t GCode::_ptrOffset = 0;
// bool GCode::_uploaded = false;
bool GCode::_testRunChecked = false;
bool GCode::_runned = false;
TaskHandle_t GCode::gcodeTaskHandle = NULL;

const GCode::GCODE_LETTER GCode::codeVal4length[] = {
    GCODE_LETTER_X, GCODE_LETTER_Y, GCODE_LETTER_Z, GCODE_LETTER_A, GCODE_LETTER_B, GCODE_LETTER_C, 
    GCODE_LETTER_P, GCODE_LETTER_F, GCODE_LETTER_S, GCODE_LETTER_R, GCODE_LETTER_D, GCODE_LETTER_L, 
    GCODE_LETTER_I, GCODE_LETTER_J, GCODE_LETTER_K
};

GCode::ProgParams GCode::progParams = DEFAULT_PROG();
Geometry::Point GCode::pointNull = __POINT_NULL;
Plasma* GCode::_plasma = NULL;              // плазма
CoordSystem* GCode::_coordSystem = NULL;    // системы координат

float GCode::_fastSpeed = 20.0;                // скорость быстрого перемещения, мм/сек
float GCode::_workSpeed = 20.0;                // рабочая скорость перемещения, мм/сек
float GCode::_fastSpeedModal = 20.0;
float GCode::_workSpeedModal = 20.0;

GCode::NotifyNumLineFunc GCode::notifyNumLineFunc = NULL;
GCode::NotifyFinishFunc GCode::notifyFinishFunc = NULL;

bool GCode::actionIsProbe = false;
float GCode::probeZOffset = 5.0;

/**
 * Инициализация программы GCode
 * @param size размер памяти, в байтах
 * @return флаг успешности выделения памяти
 */
bool GCode::init(uint32_t size){
    if(_ptr != NULL){
        remove();
    }
    _ptr = calloc(size>>4, 16);
    _size = size;
    _ptrOffset = 0;
    return _ptr != NULL;
}

/**
 * Удаление программы из памяти
 */
void GCode::remove(){
    if(_ptr != NULL){
        free(_ptr);
        _ptr = NULL;
        _size = 0;
        _ptrOffset = 0;
    }
}

/**
 * Последовательное добавление команд GCode в конец списка при загрузке
 * @param data входящие данные
 * @param length размер данных
 */
void GCode::append(void *data, uint32_t length){
    memcpy( ((void *) ( ((uint8_t *)_ptr)+_ptrOffset )), data, length);
    _ptrOffset += length;
}

/**
 * Является ли загруженный GCode доступным для выполнения
 */
bool GCode::isRunnable(){
    return _ptr != NULL && _size > 0;
}

/**
 * Выполняется ли GCode в настоящий момент
 */
bool GCode::isRunned(){
    return _runned;
}

/**
 * Обнуление немодальных параметров программы
 */
void GCode::cleanNotModalParams(){

    progParams.plasmaArc = Plasma::PLASMA_ARC_NONE;
    progParams.pause = 0.0;
    progParams.probe.speed = 0.0;

}

/**
 * Установка плазмы
 * @param plasma плазма
 */
void GCode::setPlasma(Plasma *plasma){
    _plasma = plasma;
}

/**
 * Запуск программы GCode
 */
void GCode::run(){
    ESP_LOGI(TAG, "RUN");
    _runned = true;
    setDefaultProgParams();
    xTaskCreate(gcodeTask, "gcodeTask", 4096, NULL, 15, &gcodeTaskHandle);
}

/**
 * Остановка программы GCode
 */
void GCode::stop(){
    ESP_LOGI(TAG, "STOP");
    _runned = false;
    vTaskDelete(gcodeTaskHandle);
}

/**
 * Задача выполнения программы GCode
 */
void GCode::gcodeTask(void *arg){
    ESP_LOGI(TAG, "gcodeTask");

    uint8_t data[16] = {0};
    // uint8_t dataResp[16] = {0};

    uint32_t gcodeFramePtrOffset = 0;

    FrameSubData frame[10];
    uint8_t frameLength = 0;

    uint32_t numLine = 0;       // номер строки

    ProgParams *pParams = new ProgParams();         // текущие параметры программы
    memcpy(pParams, &progParams, sizeof(ProgParams));
    Geometry::Point currentPoint = calcTargetPoint(pParams);    // текущая точка
    Geometry::Point targetPoint = calcTargetPoint(pParams);     // целевая точка
    std::list<ProgParams> pParamsList;              // список текущих параметров программы
    ProgParams currentParams;
    ProgParams targetParams;
    ProgParams nextParams;

    memcpy(&currentParams, &progParams, sizeof(ProgParams));
    memcpy(&targetParams, &progParams, sizeof(ProgParams));

    // отправка номера строки клиенту
    std::function<void (uint32_t)> sendNumLine = [&](uint32_t numLine){
        if(notifyNumLineFunc != NULL){
            notifyNumLineFunc(numLine);
        }
    };

    // скорость перемещения необходимая в данный момент, мм/сек
    std::function<float ()> calcSpeed = [&]() -> float{
        float speed = 0.0;
        if(targetParams.speed != 0.0) speed = targetParams.speed;
        else if(targetParams.runType == RUN_FAST) speed = _fastSpeed;
        else speed = _workSpeed;
        if(speed == 0.0) speed = 20.0;      // скорость по-умолчанию
        return speed;
    };

    std::function<void ()> calcFrame = [&](){
        uint8_t sDataInd = 0;
        frameLength = 0;

        bool reqNextData = false;       // ожидается продолжение данных
        while(true){
            memcpy(&data, ((void *) ( ((uint8_t *)_ptr)+gcodeFramePtrOffset )), 16);
            sDataInd = 0;
            if(!reqNextData){
                numLine = *(data+1) + (*(data+2)<<8) ;
                sDataInd = 3;       // 0 - OBJ_NAME_CNC_GCODE, 1,2 - номер строки
                progParams.numLine = numLine;
                // ESP_LOGI(TAG, "numLine: %d", numLine);
            }

            while(sDataInd < 16){
                GCODE_LETTER letter = (GCODE_LETTER) data[sDataInd];
                uint8_t lenValue = 1;
                uint8_t intValue = 0;
                float floatValue = 0.0;

                if(letter == GCODE_LETTER_NONE)
                    break;

                for(int j=0; j<sizeof(codeVal4length); j++){
                    if(letter == codeVal4length[j]){
                        lenValue = 4;
                        break;
                    }
                }

                if(lenValue == 1){
                    memcpy(&intValue, data+sDataInd+1, lenValue);
                    floatValue = (float) intValue;
                } else if(lenValue == 4){
                    memcpy(&floatValue, data+sDataInd+1, lenValue);
                }
                // ESP_LOGI(TAG, "\t%d:%f", letter, floatValue);

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
                break;
            }
            gcodeFramePtrOffset += 16;
        }
    };

    for(;;){
        calcFrame();

        if(processFrame(frame, frameLength)){
            Geometry::Point nextPoint = calcTargetPoint(&progParams);
            memcpy(&nextParams, &progParams, sizeof(ProgParams));
            if(Geometry::pointsIsEqual(&targetPoint, &nextPoint)){
                pParamsList.push_back(progParams);
            } else{
                if(isCircleInterpolation(&nextParams)){     // рассчёт полных параметров окружности
                    calcCircleSegment(&targetPoint, &nextPoint, &nextParams);
                }
                if(Geometry::pointsIsEqual(&currentPoint, &targetPoint)){
                    pParamsList.push_back(progParams);
                } else{
                    // выполняется перемещение на основе информации о текущей, целевой и следующей за целевой точками
                    ESP_LOGI(TAG, "numLine: %d", targetParams.numLine);
                    // ESP_LOGI(TAG, "current: [%.2f ; %.2f ; %.2f], target: [%.2f ; %.2f ; %.2f], next: [%.2f ; %.2f ; %.2f]",
                    //     currentPoint.x, currentPoint.y, currentPoint.z, 
                    //     targetPoint.x, targetPoint.y, targetPoint.z, 
                    //     nextPoint.x, nextPoint.y, nextPoint.z
                    // );
                    // ESP_LOGI(TAG, "CR current: side: %d; value: %f, CR target: side: %d; value: %f, CR next: side: %d; value: %f", 
                    //     currentParams.compensationRadius.side, currentParams.compensationRadius.value,
                    //     targetParams.compensationRadius.side, targetParams.compensationRadius.value,
                    //     nextParams.compensationRadius.side, nextParams.compensationRadius.value
                    // );

                    // скорость перемещения необходимая в данный момент
                    float speed = calcSpeed();

                    sendNumLine(targetParams.numLine);
                    if(targetParams.runType == RUN_FAST || targetParams.compensationRadius.side == GCodeCR::COMPENSATION_NONE){
                        // на целевой точке компенсация радиуса отсутствует 
                        processPath(&targetPoint, &targetParams, speed);
                    } else{
                        // перемещения с учётом компенсации радиуса инструмента
                        GCodeCRP::CompensationPath cpath = {
                            .currentPoint = &currentPoint, .targetPoint = &targetPoint, .nextPoint = &nextPoint,
                            .currentParams = &currentParams, .targetParams = &targetParams, .nextParams = &nextParams
                        };
                        GCodeCRP::processPath(&cpath, speed);
                    }

                    // проход по списку параметров программы
                    for(auto iter = pParamsList.begin(); iter != pParamsList.end(); iter++){
                        ProgParams pParams = (ProgParams) *iter;
                        if(pParams.numLine != currentParams.numLine){
                            ESP_LOGI(TAG, "numLine: %d [p]", pParams.numLine);
                            sendNumLine(pParams.numLine);
                            processProgParams(&pParams, &currentPoint, &targetPoint, &nextPoint);
                        }
                    }

                    // очистка списка
                    pParamsList.clear();

                    memcpy(&currentPoint, &targetPoint, sizeof(Geometry::Point));
                    memcpy(&currentParams, &targetParams, sizeof(ProgParams));
                }
                memcpy(&targetPoint, &nextPoint, sizeof(Geometry::Point));
                memcpy(&targetParams, &nextParams, sizeof(ProgParams));
            }
            
            cleanNotModalParams();      // обнуление немодальных параметров программы
            memcpy(&progParams.currentCoord, &targetPoint, sizeof(Geometry::Point));
        }

        gcodeFramePtrOffset += 16;

        if(gcodeFramePtrOffset >= _size){
            // достигнут конец программы

            if(!Geometry::pointsIsEqual(&currentPoint, &targetPoint)){
                // выполняется перемещение на основе информации о текущей, целевой и следующей за целевой точками
                ESP_LOGI(TAG, "last path");
                ESP_LOGI(TAG, "numLine: %d", targetParams.numLine);
                // ESP_LOGI(TAG, "current: [%.2f ; %.2f], target: [%.2f ; %.2f]",
                //     currentPoint.x, currentPoint.y, 
                //     targetPoint.x, targetPoint.y
                // );

                float speed = calcSpeed();
                sendNumLine(targetParams.numLine);
                // на последнем участке пути проходим без учёта компенсации радиуса инструмента
                processPath(&targetPoint, &targetParams, speed);

                // проход по списку параметров программы
                for(auto iter = pParamsList.begin(); iter != pParamsList.end(); iter++){
                    ProgParams pParams = (ProgParams) *iter;
                    if(pParams.numLine != currentParams.numLine){
                        ESP_LOGI(TAG, "numLine: %d [p]", pParams.numLine);
                        sendNumLine(pParams.numLine);
                        processProgParams(&pParams, &currentPoint, &targetPoint, NULL);
                    }
                }


            }

            // завершаем задачу
            _runned = false;
            ESP_LOGI(TAG, "FINISH");
            free(pParams);
            GCode::gcodeTaskHandle = NULL;
            if(notifyFinishFunc != NULL){
                notifyFinishFunc();
            }
            vTaskDelete(NULL);
        }
    }
    GCode::gcodeTaskHandle = NULL;
    vTaskDelete(NULL);
}

/**
 * Установка тестового прогона программы gcode
 * @param checked тестовый прогон программы gcode
 */
void GCode::setTestRunChecked(bool checked){
    _testRunChecked = checked;
}

/**
 * Установка текущих параметров программы по-умолчанию
 */
void GCode::setDefaultProgParams(){
    progParams = DEFAULT_PROG();

}

/**
 * Обработка фрейма программы GCode
 */
bool GCode::processFrame(FrameSubData *frame, uint8_t frameLength){

    bool processThisFrame = true;

    // функция рассчёта целевых координат на основе установленной единицы измерения, 
    // смещения нулевой точки, системы координат и др.
    std::function<void (char, float)> calcCoord = [&](char axeName, float value){
        float val = value;
        if(progParams.unit == UNIT_INCH){       // дюймы в миллиметры
            val *= 25.4;
        }

        switch(axeName){
            case 'x':
                progParams.targetCoord.x = (progParams.coordType == COORD_RELATIVE ? progParams.targetCoord.x : 0) + val;
                break;
            case 'y':
                progParams.targetCoord.y = (progParams.coordType == COORD_RELATIVE ? progParams.targetCoord.y : 0) + val;
                break;
            case 'z':
                progParams.targetCoord.z = (progParams.coordType == COORD_RELATIVE ? progParams.targetCoord.z : 0) + val;
                break;
            case 'a':
                progParams.targetCoord.a = (progParams.coordType == COORD_RELATIVE ? progParams.targetCoord.a : 0) + val;
                break;
            case 'b':
                progParams.targetCoord.b = (progParams.coordType == COORD_RELATIVE ? progParams.targetCoord.b : 0) + val;
                break;
            case 'c':
                progParams.targetCoord.c = (progParams.coordType == COORD_RELATIVE ? progParams.targetCoord.c : 0) + val;
                break;
            default:
                break;
        }

    };

    if(progParams.runType == RUN_WORK_CW || progParams.runType == RUN_WORK_CCW){
        for(uint8_t i=0; i<frameLength; i++){
            // FrameSubData *el = frame+i;
            if(!processThisFrame)       // текущую команду Gxx не обрабатываем
                break;

            progParams.circle.inc = { .i = 0.0, .j = 0.0, .k = 0.0 };
        }
    }

    for(uint8_t i=0; i<frameLength; i++){
        FrameSubData *el = frame+i;
        if(!processThisFrame)       // текущую команду Gxx не обрабатываем
            break;

        switch(el->letter){
            case GCODE_LETTER_G:
                processThisFrame = processCommand_G((uint8_t) el->value, frame, frameLength);
                break;
            case GCODE_LETTER_M:
                processThisFrame = processCommand_M((uint8_t) el->value, frame, frameLength);
                break;

            case GCODE_LETTER_X:
                calcCoord('x', el->value);
                break;
            case GCODE_LETTER_Y:
                calcCoord('y', el->value);
                break;
            case GCODE_LETTER_Z:
                calcCoord('z', el->value);
                break;
            case GCODE_LETTER_A:
                calcCoord('a', el->value);
                break;
            case GCODE_LETTER_B:
                calcCoord('b', el->value);
                break;
            case GCODE_LETTER_C:
                calcCoord('c', el->value);
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

            default:
                break;
        }
    }

    return true;
}

/**
 * Обработка команды Gxx
 */
bool GCode::processCommand_G(uint8_t value, FrameSubData *frame, uint8_t frameLength){
    bool processThisCommand = true;
    switch(value){
        case 0:     // G00 - Ускоренное перемещение инструмента (холостой ход)
            progParams.runType = RUN_FAST;
            progParams.speed = _fastSpeedModal;
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_F:    // задание скорости перемещения, mm/min
                        _fastSpeedModal = progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 1:     // G01 - Линейная интерполяция, скорость перемещения задаётся здесь же или ранее модальной командой F
            progParams.runType = RUN_WORK_LINEAR;
            progParams.speed = _workSpeedModal;
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_F:    // задание скорости перемещения, mm/min
                        _workSpeedModal = progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 2:     // G02 - Круговая интерполяция по часовой стрелке
            progParams.runType = RUN_WORK_CW;
            progParams.speed = _workSpeedModal;
            progParams.circle.inc = { .i = 0.0, .j = 0.0, .k = 0.0 };
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
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
                        _workSpeedModal = progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 3:     // G03 - Круговая интерполяция против часовой стрелки
            progParams.runType = RUN_WORK_CCW;
            progParams.speed = _workSpeedModal;
            progParams.circle.inc = { .i = 0.0, .j = 0.0, .k = 0.0 };
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
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
                        _workSpeedModal = progParams.speed = el->value/60;
                        break;
                    default:
                        break;
                }
            }
            break;
        case 4:     // G04 - Задержка выполнения программы, способ задания величины задержки зависит от реализации системы управления, P обычно задает паузу в миллисекундах, X — в секундах.
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
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
            processThisCommand = false;
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
            break;
        case 30:    // G30 - Поднятие по оси Z на точку смены инструмента
            break;
        case 31:    // G31 - Подача до пропуска
            if(!_testRunChecked){       // тестовый прогон программы gcode выключен
                progParams.probe.point = __POINT_NULL;
                progParams.probe.speed = getWorkSpeed();
                progParams.probe.backOffset = probeZOffset;
                for(uint8_t i=0; i<frameLength; i++){
                    FrameSubData *el = frame+i;
                    switch(el->letter){
                        case GCODE_LETTER_F:        // задание скорости перемещения, mm/min
                            progParams.probe.speed = el->value/60;
                            break;
                        case GCODE_LETTER_X:
                            progParams.probe.point.x = el->value;
                            break;
                        case GCODE_LETTER_Y:
                            progParams.probe.point.y = el->value;
                            break;
                        case GCODE_LETTER_Z:
                            progParams.probe.point.z = el->value;
                            break;
                        case GCODE_LETTER_A:
                            progParams.probe.point.a = el->value;
                            break;
                        case GCODE_LETTER_B:
                            progParams.probe.point.b = el->value;
                            break;
                        case GCODE_LETTER_C:
                            progParams.probe.point.c = el->value;
                            break;
                        case GCODE_LETTER_L:
                            progParams.probe.backOffset = el->value;
                            break;
                        default:
                            break;
                    }
                }
            }
            processThisCommand = false;
            break;
        case 40:    // G40 - Отмена компенсации радиуса инструмента
            progParams.compensationRadius.side = GCodeCR::COMPENSATION_NONE;
            progParams.compensationRadius.value = 0.0;
            break;
        case 41:    // G41 - Компенсировать радиус инструмента слева от траектории
            progParams.compensationRadius.side = GCodeCR::COMPENSATION_LEFT;
            progParams.compensationRadius.value = 1.0;      // радиус по умолчанию
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
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
            progParams.compensationRadius.side = GCodeCR::COMPENSATION_RIGHT;
            progParams.compensationRadius.value = 1.0;      // радиус по умолчанию
            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
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
            progParams.coordType = COORD_ABSOLUTE;
            break;
        case 91:    // G91 - Задание координат инкрементально последней введённой опорной точки
            progParams.coordType = COORD_RELATIVE;
            break;
        case 92:    // G92 - Смещение абсолютной системы координат
        {
            progParams.coordType = COORD_OFFSET;
            memcpy(&progParams.targetCoord, &pointNull, sizeof(Geometry::Point));
            memcpy(&progParams.offsetCoord, &progParams.currentCoord, sizeof(Geometry::Point));

            // выбранная система координат
            // в данный момент используется только пользовательский ноль
            Geometry::Point coordSystemPoint = _coordSystem != NULL ? _coordSystem->getUserZero() : __POINT_NULL;
            progParams.offsetCoord.x -= coordSystemPoint.x;
            progParams.offsetCoord.y -= coordSystemPoint.y;
            progParams.offsetCoord.z -= coordSystemPoint.z;
            progParams.offsetCoord.a -= coordSystemPoint.a;
            progParams.offsetCoord.b -= coordSystemPoint.b;
            progParams.offsetCoord.c -= coordSystemPoint.c;

            for(uint8_t i=0; i<frameLength; i++){
                FrameSubData *el = frame+i;
                switch(el->letter){
                    case GCODE_LETTER_X:
                        progParams.offsetCoord.x -= el->value;
                        break;
                    case GCODE_LETTER_Y:
                        progParams.offsetCoord.y -= el->value;
                        break;
                    case GCODE_LETTER_Z:
                        progParams.offsetCoord.z -= el->value;
                        break;
                    case GCODE_LETTER_A:
                        progParams.offsetCoord.a -= el->value;
                        break;
                    case GCODE_LETTER_B:
                        progParams.offsetCoord.b -= el->value;
                        break;
                    case GCODE_LETTER_C:
                        progParams.offsetCoord.c -= el->value;
                        break;
                    default:
                        break;
                }
            }
            processThisCommand = false;
            break;
        }
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
bool GCode::processCommand_M(uint8_t value, FrameSubData *frame, uint8_t frameLength){
    bool processThisCommand = true;
    switch(value){
        case 3:     // M03 - включить плазму
            if(!_testRunChecked){       // тестовый прогон программы gcode выключен
                if(_plasma != NULL){
                    progParams.plasmaArc = Plasma::PLASMA_ARC_START;
                }
            }
            break;
        case 5:     // M05 - выключить плазму
            if(!_testRunChecked){       // тестовый прогон программы gcode выключен
                if(_plasma != NULL){
                    progParams.plasmaArc = Plasma::PLASMA_ARC_STOP;
                }
            }
            break;
        default:
            break;
    }

    return processThisCommand;
}

/**
 * Получение целевой точки
 * @param pParams параметры программы
 */
Geometry::Point GCode::calcTargetPoint(ProgParams *pParams){
    
    // выбранная система координат
    // в данный момент используется только пользовательский ноль
    Geometry::Point coordSystemPoint = _coordSystem != NULL ? _coordSystem->getUserZero() : __POINT_NULL;

    Geometry::Point point = {
        .x = pParams->targetCoord.x + (pParams->coordType == COORD_OFFSET ? pParams->offsetCoord.x : 0) + coordSystemPoint.x,
        .y = pParams->targetCoord.y + (pParams->coordType == COORD_OFFSET ? pParams->offsetCoord.y : 0) + coordSystemPoint.y,
        .z = pParams->targetCoord.z + (pParams->coordType == COORD_OFFSET ? pParams->offsetCoord.z : 0) + coordSystemPoint.z,
        .a = pParams->targetCoord.a + (pParams->coordType == COORD_OFFSET ? pParams->offsetCoord.a : 0) + coordSystemPoint.a,
        .b = pParams->targetCoord.b + (pParams->coordType == COORD_OFFSET ? pParams->offsetCoord.b : 0) + coordSystemPoint.b,
        .c = pParams->targetCoord.c + (pParams->coordType == COORD_OFFSET ? pParams->offsetCoord.c : 0) + coordSystemPoint.c
    };
    return point;
}

/**
 * Рассчёт полных параметров окружности
 * @param currentPoint текущая точка
 * @param targetPoint целевая точка
 * @param pParams параметры программы
 */
void GCode::calcCircleSegment(Geometry::Point *currentPoint, Geometry::Point *targetPoint, GCode::ProgParams *pParams){
    Geometry::PointXY p1 = Geometry::getPointXY(currentPoint);
    Geometry::PointXY p2 = Geometry::getPointXY(targetPoint);
    Geometry::CircleSegment circle;
    if(pParams->circle.type == CIRCLE_RADIUS){
        circle = Geometry::calcCircleByRadius(&p1, &p2, pParams->circle.radius, pParams->runType == RUN_WORK_CCW);
    } else if(pParams->circle.type == CIRCLE_INC){
        circle = Geometry::calcCircleByInc(&p1, &p2, pParams->circle.inc.i, pParams->circle.inc.j, pParams->runType == RUN_WORK_CCW);
    } else{

    }
    memcpy(&pParams->circleSegment, &circle, sizeof(Geometry::CircleSegment));
}

/**
 * Перемещение по требуемому пути (линейная или круговая интерполяция)
 * @param targetPoint целевая точка
 * @param speed скорость, мм/сек
 */
void GCode::processPath(Geometry::Point *targetPoint, ProgParams *targetParams, float speed){
    if(isLinearInterpolation(targetParams)){
        ActionMove::gotoPoint(targetPoint, speed);
    } else if(isCircleInterpolation(targetParams)){
        ActionMove::circle(&targetParams->circleSegment, speed);
    }
}

/**
 * Определить является ли путь линейной интерполяцией
 * @param pParams параметры программы
 */
bool GCode::isLinearInterpolation(ProgParams *pParams){
    return pParams->runType == RUN_FAST || pParams->runType == RUN_WORK_LINEAR;
}

/**
 * Определить является ли путь круговой интерполяцией
 * @param pParams параметры программы
 */
bool GCode::isCircleInterpolation(ProgParams *pParams){
    return pParams->runType == RUN_WORK_CW || pParams->runType == RUN_WORK_CCW;
}

/**
 * Выполнение команд GCode не связанных с перемещением
 */
void GCode::processProgParams(ProgParams *pParams, Geometry::Point *currentCoord, Geometry::Point *targetCoord, Geometry::Point *nextCoord){

    if(pParams->pause > 0.0){
        // пауза выполнения программы
        actionPause(pParams->pause);
    }

    if(pParams->probe.speed != 0.0){
        // нахождение поверхности
        actionProbe(&pParams->probe, currentCoord, targetCoord, nextCoord);
    }

    if(pParams->plasmaArc == Plasma::PLASMA_ARC_START){             // запуск плазмы
        if(_plasma != NULL){
            if(!_plasma->getArcStarted()){      // плазма не включена, включаем
                _plasma->start();
                // задача gcode приостанавливается и
                // возобновляется в методе CncRouter::plasmaArcStartedCallback
                // ожидая подтверждения о начале резки от аппарата плазменной резки
                vTaskSuspend(gcodeTaskHandle);
            }
        }
    } else if(pParams->plasmaArc == Plasma::PLASMA_ARC_STOP){       // остановка плазмы
        if(_plasma != NULL){
            _plasma->stop();
        }
    }

}

/**
 * Пауза выполнения программы
 * @param pause пауза, сек
 */
void GCode::actionPause(float pause){
    ESP_LOGI(TAG, "Pause: %f sec", pause);
    esp_timer_create_args_t timerPauseArgs = {
        .callback = &GCode::actionPauseFinish,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "gcode_pause"
    };

    esp_timer_handle_t timerPause;
    ESP_ERROR_CHECK(esp_timer_create(&timerPauseArgs, &timerPause));
    ESP_ERROR_CHECK(esp_timer_start_once(timerPause, (uint64_t) (pause*1e6)));

    // задача gcode приостанавливается и
    // возобновляется в функции funcPauseCallback
    vTaskSuspend(gcodeTaskHandle);
    ESP_LOGI(TAG, "Pause resume");
}

/**
 * Завершение паузы выполнения программы.
 */
void GCode::actionPauseFinish(void *arg){
    // возобновление основной задачи gcode
    vTaskResume(gcodeTaskHandle);
}

/**
 * Подача до пропуска
 * @param probe параметры подачи
 * @param currentCoord текущие координаты
 */
void GCode::actionProbe(Probe *probe, Geometry::Point *currentCoord, Geometry::Point *targetCoord, Geometry::Point *nextCoord){
    ESP_LOGI(TAG, "Probe start");
    // опускание вниз по Z
    Geometry::Point targetPoint;
    memcpy(&targetPoint, targetCoord, sizeof(Geometry::Point));
    float targetZ = probe->point.z;
    bool moveIsDown = true;
    if(targetZ > 0){
        Geometry::Point userZero = _coordSystem->getUserZero();
        targetZ += userZero.z;
        moveIsDown = false;
    }
    targetPoint.z = targetZ;
    actionIsProbe = true;
    ActionMove::gotoPoint(&targetPoint, probe->speed, true);
    actionIsProbe = false;

    // получение текущей точки по оси z
    float z = Axe::getStepDriver(Axe::AXE_Z)->getPositionMM();

    ESP_LOGI(TAG, "Probe stop z: %f", z);

    if(moveIsDown){
        // поднятие вверх по Z на величину отскока
        // targetPoint.z = z + probeZOffset;
        targetPoint.z = z + probe->backOffset;
        ActionMove::gotoPoint(&targetPoint, probe->speed, true);
        z = Axe::getStepDriver(Axe::AXE_Z)->getPositionMM();
    }

    ESP_LOGI(TAG, "New probe  z: %f", z);
    currentCoord->z = z;
    targetCoord->z = z;
    if(nextCoord != NULL) nextCoord->z = z;

    // установка новой точки пользовательского нуля по оси z
    Geometry::Point userZero = _coordSystem->getUserZero();
    userZero.z = z;
    userZero.log(TAG, "New userZero");
    _coordSystem->setUserZero(&userZero);
}

/**
 * Установка значения "отскока" при поиске поверхности
 * @param offset смещение, мм
 */
void GCode::setProbeZOffset(float offset){
    probeZOffset = offset;
}

/**
 * установка скорости быстрого перемещения
 * @param speed скорость
 */
void GCode::setFastSpeed(float speed){
    _fastSpeed = speed;
    _fastSpeedModal = speed;
}

/**
 * Получение скорости быстрого перемещения
 */
float GCode::getFastSpeed(){
    return _fastSpeed;
}

/**
 * установка рабочей скорости перемещения
 * @param speed скорость
 */
void GCode::setWorkSpeed(float speed){
    _workSpeed = speed;
    _workSpeedModal = speed;
}

/**
 * Получение рабочей скорости перемещения
 */
float GCode::getWorkSpeed(){
    return _workSpeed;
}

/**
 * Устанвка функции для отправки номера текущей строки клиенту
 * @param func функция
 */
void GCode::setNotifyNumLineFunc(NotifyNumLineFunc func){
    notifyNumLineFunc = func;
}

/**
 * Устанвка функции вызываемой по завершению выполнения программы
 * @param func функция
 */
void GCode::setNotifyFinishFunc(NotifyFinishFunc func){
    notifyFinishFunc = func;
}

/**
 * Установка систем координат
 * @param system системы координат
 */
void GCode::setCoordSystem(CoordSystem *system){
    _coordSystem = system;
}

/**
 * Событие прерывания от входов группы 0
 * @param type тип ввода
 * @param value значение уровня
 */
void GCode::input0Event(InputInterrupt::INPUT0 type, int level){

    switch(type){
        case InputInterrupt::INPUT0_LIMITS:
            break;
        case InputInterrupt::INPUT0_HOMES:
            break;
        case InputInterrupt::INPUT0_PROBE:
            if(actionIsProbe && level == 0){
                ActionMove::stop();
            }
            break;
        case InputInterrupt::INPUT0_ESTOP:
            break;

        default:
            break;
    }

}



