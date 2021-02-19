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

#define TAG "CncRouter"

CncRouter* CncRouter::instance = NULL;

xQueueHandle CncRouter::input0EvtQueue = NULL;
xQueueHandle CncRouter::wsSendEvtQueue = NULL;




const uint8_t CncRouter::WS_OBJ_NAME_CNC_GCODE = 0x50;
const uint8_t CncRouter::WS_OBJ_NAME_CNC_GCODE_PREPARE = 0x51;
const uint8_t CncRouter::WS_OBJ_NAME_AXE = 0x52;
const uint8_t CncRouter::WS_OBJ_NAME_COORDS = 0x53;
const uint8_t CncRouter::WS_OBJ_NAME_COORD_TARGET = 0x54;
const uint8_t CncRouter::WS_OBJ_NAME_PLASMA_ARC = 0x55;

const uint8_t CncRouter::WS_CMD_READ = 0x01;
const uint8_t CncRouter::WS_CMD_WRITE = 0x02;
const uint8_t CncRouter::WS_CMD_RUN = 0x03;
const uint8_t CncRouter::WS_CMD_STOP = 0x04;

const uint8_t CncRouter::WS_AXE_DIRECTION_NONE = 0x00;
const uint8_t CncRouter::WS_AXE_DIRECTION_FORWARD = 0x01;
const uint8_t CncRouter::WS_AXE_DIRECTION_BACKWARD = 0x02;

const uint8_t CncRouter::WS_PREPARE_SIZE = 0x01;
const uint8_t CncRouter::WS_PREPARE_RUN = 0x02;

const uint8_t CncRouter::WS_PLASMA_ARC_START = 0x01;
const uint8_t CncRouter::WS_PLASMA_ARC_STARTED = 0x02;
const uint8_t CncRouter::WS_PLASMA_ARC_VOLTAGE = 0x03;



/**
 * Конструктор
 * @param axesCount количество осей
 */
CncRouter::CncRouter(){

    instance = this;

    // if(axesCount == Axe::AXES_NC){
    //     return;
    // }

    // _axesCount = axesCount;
    // if(axesCount >= Axe::AXES_1)
    //     _x = new StepDriver();
    // if(axesCount >= Axe::AXES_2)
    //     _y = new StepDriver();
    // if(axesCount >= Axe::AXES_3)
    //     _z = new StepDriver();
    // if(axesCount >= Axe::AXES_4)
    //     _a = new StepDriver();
    // if(axesCount >= Axe::AXES_5)
    //     _b = new StepDriver();
    // if(axesCount >= Axe::AXES_6)
    //     _c = new StepDriver();

    // создание очереди и запуск задачи для обработки прерываний со входов
    CncRouter::input0EvtQueue = xQueueCreate(10, sizeof(Input0Interrupt));
    xTaskCreate(CncRouter::input0Task, "input0Task", 2048, NULL, 10, NULL);

    // 
    void *ptr;
    CncRouter::wsSendEvtQueue = xQueueCreate(10, sizeof(ptr));
    xTaskCreate(CncRouter::wsSendTask, "wsSendTask", 2048, NULL, 10, NULL);

}

/**
 * Получение экземпляра
 */
CncRouter* CncRouter::getInstance(){
    return instance;
}

// /**
//  * Получение объекта оси
//  * @param axe ось
//  */
// StepDriver* CncRouter::getAxe(Axe::AXE axe){
//     StepDriver *sd = NULL;
//     switch(axe){
//         case Axe::AXE_X:
//             sd = _x;
//             break;
//         case Axe::AXE_Y:
//             sd = _y;
//             break;
//         case Axe::AXE_Z:
//             sd = _z;
//             break;
//         case Axe::AXE_A:
//             sd = _a;
//             break;
//         case Axe::AXE_B:
//             sd = _b;
//             break;
//         case Axe::AXE_C:
//             sd = _c;
//             break;
//         default:
//             break;
//     }

//     if(sd == NULL){
        
//     }

//     return sd;
// }

// /**
//  * Перемещение до целевой позиции
//  * @param axe ось
//  * @param targetMM целевая позиция, в мм
//  * @param speedMmSec скорость, мм/сек
//  * @param funcFinish функция вызываемая при окончании перемещения по оси
//  */
// void CncRouter::gotoTargetMM(Axe::AXE axe, float targetMM, float speedMmSec, std::function<void (StepDriver *sd)> funcFinish){

//     StepDriver *sd = Axe::getStepDriver(axe);
//     StepDriver::MotorTarget *mt = sd->calcTarget(targetMM, speedMmSec);

//     if(mt != NULL){
//         mt->cPulse = 0;
//         mt->callbackFinish = funcFinish;

//         esp_timer_create_args_t timerRunArgs = {
//             .callback = &StepDriver::timerRunCallback,
//             .arg = (void *)mt,
//             .name = "axe_"+sd->getLetter()
//         };

//         esp_timer_handle_t timerRun;
//         ESP_ERROR_CHECK(esp_timer_create(&timerRunArgs, &timerRun));

//         mt->axe->setTimerRun(timerRun);

//         uint64_t mksStep = mt->mksStep;
//         if(mksStep < 50) mksStep = 50;
//         // printf("mksStep: %llu \n", mksStep);

//         sd->setDirection(mt->dir);
//         ESP_ERROR_CHECK(esp_timer_start_periodic(timerRun, mksStep));
//     }
// }

// /**
//  * Перемещение до целевой позиции
//  * @param target целевая точка
//  * @param current текущая точка
//  * @param speedMmSec скорость, мм/сек
//  * @param funcFinish функция вызываемая при окончании перемещения по оси
//  */
// void CncRouter::gotoTargetMM(GcodeCoord *target, GcodeCoord *current, float speedMmSec, std::function<void (StepDriver *sd)> funcFinish){

//     float speed = speedMmSec;

//     float dx = target->x - current->x;
//     float dy = target->y - current->y;
//     float dz = target->z - current->z;

//     if(abs(dx) < 0.01) dx = 0.0;
//     if(abs(dy) < 0.01) dy = 0.0;
//     if(abs(dz) < 0.01) dz = 0.0;

//     float length = 0.0;
//     if(dx != 0) length += dx*dx;
//     if(dy != 0) length += dy*dy;
//     if(dz != 0) length += dz*dz;
//     length = sqrt(length);

//     if(dx != 0){
//         progParams.finishCount++;
//         float spX = abs(dx)/length*speed;    // разложенная скорость по оси X
//         gotoTargetMM(AXE_X, target->x, spX, funcFinish);
//     }
//     if(dy!= 0){
//         progParams.finishCount++;
//         float spY = abs(dy)/length*speed;    // разложенная скорость по оси Y
//         gotoTargetMM(AXE_Y, target->y, spY, funcFinish);
//     }
//     if(dz != 0){
//         progParams.finishCount++;
//         float spZ = abs(dz)/length*speed;    // разложенная скорость по оси Z
//         gotoTargetMM(AXE_Z, target->z, spZ, funcFinish);
//     }

//     if(progParams.finishCount > 0){
//         vTaskSuspend(NULL);     // приостановить текущую задачу
//     //     // переходим к следующему фрейму
//     //     vTaskResume(CncRouter::gcodeTaskHandle);
//     }



// }

// /**
//  * Перемещение до целевой позиции
//  * @param target целевая точка
//  * @param current текущая точка
//  * @param speedMmSec скорость, мм/сек
//  * @param funcFinish функция вызываемая при окончании перемещения по оси
//  */
// void CncRouter::gotoTargetMM(GCode::PointXY *target, GCode::PointXY *current, float speedMmSec, std::function<void (StepDriver *sd)> funcFinish){
//     float speed = speedMmSec;

//     float dx = target->x - current->x;
//     float dy = target->y - current->y;

//     if(abs(dx) < 0.01) dx = 0.0;
//     if(abs(dy) < 0.01) dy = 0.0;

//     float length = 0.0;
//     if(dx != 0) length += dx*dx;
//     if(dy != 0) length += dy*dy;
//     length = sqrt(length);

//     if(dx != 0){
//         progParams.finishCount++;
//         float spX = abs(dx)/length*speed;    // разложенная скорость по оси X
//         gotoTargetMM(AXE_X, target->x, spX, funcFinish);
//     }
//     if(dy!= 0){
//         progParams.finishCount++;
//         float spY = abs(dy)/length*speed;    // разложенная скорость по оси Y
//         gotoTargetMM(AXE_Y, target->y, spY, funcFinish);
//     }

//     if(progParams.finishCount > 0){
//         vTaskSuspend(NULL);     // приостановить текущую задачу
//     //     // переходим к следующему фрейму
//     //     vTaskResume(CncRouter::gcodeTaskHandle);
//     }

// }


/**
 * Парсинг входящих данных по WebSocket
 * @param data указатель на данные
 * @param length длина данных
 */
void CncRouter::parseWsData(uint8_t *data, uint32_t length){
    ESP_LOGI(TAG, "parseWsData: length: %d", length);
    ESP_LOG_BUFFER_HEXDUMP(TAG, data, length, ESP_LOG_INFO);

    // обработка входящих данных
    if(length == 16){

        if(*(data) == WS_OBJ_NAME_AXE){
            if(*(data+1) == WS_CMD_RUN){
                Axe::AXE axe = (Axe::AXE)(*(data+2));
                uint8_t dir = *(data+3);

                float speed = 1.0;
                memcpy(&speed, data+4, 4);

                bool runAfterLimit = *(data+8);

                StepDriver *sd = Axe::getStepDriver(axe);
                if(dir == WS_AXE_DIRECTION_FORWARD || dir == WS_AXE_DIRECTION_BACKWARD){
                    printf("WS_OBJ_NAME_AXE --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d \n", sd->getLetter(), dir, speed, runAfterLimit);
                    sd->actionRunStop();
                    sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
                    // instance->setMoveAxe(sd);
                }
            } else if(*(data+1) == WS_CMD_STOP){
                Axe::AXE axe = (Axe::AXE)(*(data+2));

                StepDriver *sd = Axe::getStepDriver(axe);
                printf("WS_OBJ_NAME_AXE --STOP-- axe: %c \n", sd->getLetter());

                sd->actionRunStop();
                // instance->setMoveAxe(NULL);
            }

        } else if(*(data) == WS_OBJ_NAME_CNC_GCODE_PREPARE){
            if(*(data+1) == WS_PREPARE_SIZE){
                uint32_t size = *(data+2) + (*(data+3)<<8) + (*(data+4)<<16) + (*(data+5)<<24);

                printf("Prepare size: %d \n", size);
                // void *dataProg = heap_caps_malloc((size_t)size, 0);
                // void *dataProg = calloc(size>>4, 16);
                // GCode::setPtr(dataProg, size);
                // instance->setGcodePtr(dataProg, size);

                uint8_t dataResp[16] = {0};
                dataResp[0] = WS_OBJ_NAME_CNC_GCODE_PREPARE;
                dataResp[1] = WS_PREPARE_SIZE;

                if(GCode::init(size)){
                // if(dataProg != NULL){
                    // памяти для программы хватает, выделяем и отправляем ответ клиенту
                    dataResp[2] = 0x01;
                } else{
                    // памяти для программы не хватает, отправляем ответ клиенту
                    dataResp[2] = 0x00;
                }
                xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
            } else if(*(data+1) == WS_PREPARE_RUN){
                uint8_t dataResp[16] = {0};
                dataResp[0] = WS_OBJ_NAME_CNC_GCODE_PREPARE;
                dataResp[1] = WS_PREPARE_RUN;

                bool testRunChecked = (bool)(*(data+2));        // тестовый прогон программы gcode

                if(GCode::isRunnable()){
                    // запускаем программу
                    GCode::setTestRunChecked(testRunChecked);
                    GCode::run();
                    dataResp[2] = 0x01;
                } else{
                    dataResp[2] = 0x00;
                }
                xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
            }
        } else if(*(data) == WS_OBJ_NAME_CNC_GCODE){
            uint8_t dataResp[16] = {0};
            dataResp[0] = WS_OBJ_NAME_CNC_GCODE;
            xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
        } else if(*(data) == WS_OBJ_NAME_COORD_TARGET){
            if(*(data+1) == WS_CMD_RUN){
                Geometry::Point point;
                // memcpy(&point.x, data+2, 4);
                // memcpy(&point.y, data+6, 4);
                // memcpy(&point.z, data+10, 4);
                memcpy(&point, data+2, 12);
                float speed = 50; //300;       // mm/sec
                ActionMove::gotoPoint(&point, speed, NULL);
            }
        } 
        
        // Plasma Arc
        else if(*(data) == WS_OBJ_NAME_PLASMA_ARC){
            uint8_t dataResp[16] = {0};
            dataResp[0] = WS_OBJ_NAME_PLASMA_ARC;
            if(*(data+1) == WS_PLASMA_ARC_START){
                dataResp[1] = WS_PLASMA_ARC_START;
                if(*(data+2) == WS_CMD_RUN){
                    instance->getPlasma()->start();
                    dataResp[2] = WS_CMD_RUN;
                    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
                } else if(*(data+2) == WS_CMD_STOP){
                    instance->getPlasma()->stop();
                    dataResp[2] = WS_CMD_STOP;
                    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
                }
            } else if(*(data+1) == WS_PLASMA_ARC_STARTED){
                dataResp[1] = WS_PLASMA_ARC_STARTED;
                if(*(data+2) == WS_CMD_READ){
                    bool started = instance->getPlasma()->getArcStarted();
                    dataResp[2] = (uint8_t) started;
                    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
                }                
            } else if(*(data+1) == WS_PLASMA_ARC_VOLTAGE){
                if(*(data+2) == WS_CMD_READ){

                }                
            }
        }

    } else if(length > 16){
        if(*(data) == WS_OBJ_NAME_CNC_GCODE){
            // первые 16 байт отбрасываются
            // instance->gcodeAppend(data+16, length-16);
            GCode::append(data+16, length-16);
            uint8_t dataResp[16] = {0};
            dataResp[0] = WS_OBJ_NAME_CNC_GCODE;
            xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
        }
    }
}


/**
 * Установка прерывания на вывод
 * @param pin вывод
 * @param inp тип входящих данных
 */
CncRouter* CncRouter::setInputPinInterrupt(gpio_num_t pin, INPUT0 inp){
    if(pin != GPIO_NUM_NC){

        uint64_t bitMask = 0;
        bitMask |= (1Ull << pin);

        // установка прерываний на предел
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pin_bit_mask = bitMask;
        gpio_config(&io_conf);
        
        int intrFlag = 0;
        gpio_install_isr_service(intrFlag);

        Input0Interrupt *ii = new Input0Interrupt();
        ii->type = inp;
        gpio_isr_handler_add(pin, CncRouter::input0IsrHandler, (void *)ii);
    }

    return this;
}

/**
 * Установка вывода предела
 * @param pin вывод предела
 */
CncRouter* CncRouter::setPinLimits(gpio_num_t pin){
    _pinLimits = pin;
    setInputPinInterrupt(pin, INPUT0_LIMITS);
    return this;
}

/**
 * Получение вывода предела
 */
gpio_num_t CncRouter::getPinLimits(){
    return _pinLimits;
}

/**
 * Установка вывода Home
 * @param pin вывод Home
 */
CncRouter* CncRouter::setPinHomes(gpio_num_t pin){
    _pinHomes = pin;
    setInputPinInterrupt(pin, INPUT0_HOMES);
    return this;
}

/**
 * Получение вывода Home
 */
gpio_num_t CncRouter::getPinHomes(){
    return _pinHomes;
}

/**
 * Установка вывода Probe
 * @param pin вывод Probe
 */
CncRouter* CncRouter::setPinProbe(gpio_num_t pin){
    _pinProbe = pin;
    setInputPinInterrupt(pin, INPUT0_PROBE);
    return this;
}

/**
 * Получение вывода Probe
 */
gpio_num_t CncRouter::getPinProbe(){
    return _pinProbe;
}

/**
 * Установка вывода EStop
 * @param pin вывод EStop
 */
CncRouter* CncRouter::setPinEStop(gpio_num_t pin){
    _pinEStop = pin;
    setInputPinInterrupt(pin, INPUT0_ESTOP);
    return this;
}

/**
 * Получение вывода EStop
 */
gpio_num_t CncRouter::getPinEStop(){
    return _pinEStop;
}

/**
 * 
 */
void IRAM_ATTR CncRouter::input0IsrHandler(void *arg){
    Input0Interrupt *ii = (Input0Interrupt *)arg;
    xQueueSendFromISR(input0EvtQueue, ii, NULL);
}

/**
 * 
 */
void CncRouter::input0Task(void *arg){
    Input0Interrupt *ii = new Input0Interrupt();
    for(;;){
        if(xQueueReceive(input0EvtQueue, ii, portMAX_DELAY)){
            switch(ii->type){
                case INPUT0_LIMITS:
                    {
                        int level = gpio_get_level(instance->getPinLimits());
                        printf("Limits interrupt: %d\n", level);
                    }
                    break;
                case INPUT0_HOMES:
                    {
                        int level = gpio_get_level(instance->getPinHomes());
                        printf("Homes interrupt: %d\n", level);
                    }
                    break;
                case INPUT0_PROBE:
                    {
                        int level = gpio_get_level(instance->getPinProbe());
                        printf("Probe interrupt: %d\n", level);
                    }
                    break;
                case INPUT0_ESTOP:
                    {
                        int level = gpio_get_level(instance->getPinEStop());
                        printf("EStop interrupt: %d\n", level);
                    }
                    break;
                default:
                    printf("Unknown interrupt \n");
                    break;
            }
            // int level = gpio_get_level(pinLimit);
            // StepDriver *sd = li->axe;
            // printf("Limit intr: %d\n", level);
            // if(sd->isModeProbe()){
            //     sd->actionProbeStop();
            // }
        }
    }
}

/**
 * Установка оси для выполнения операции Probe
 * @param axe ось
 */
void CncRouter::setProbeAxe(StepDriver *axe){
    probeAxe = axe;
}

// /**
//  * Установка оси для простого перемещения
//  * @param axe ось
//  */
// void CncRouter::setMoveAxe(StepDriver *axe){
//     moveAxe = axe;
// }

/**
 * 
 */
void CncRouter::wsSendTask(void *arg){
    uint8_t data[16] = {0};
    for(;;){
        if(xQueueReceive(wsSendEvtQueue, &data, portMAX_DELAY)){
            // printf("wsSendTask: %d %d %d %d %d %d %d %d  \n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
            ws_server_send_bin_all((char *)&data, 16);
        }
    }
}

/**
 * 
 */
void CncRouter::currentPointTask(void *arg){
    uint8_t data[16] = {0};

    data[0] = WS_OBJ_NAME_COORDS;
    data[1] = WS_CMD_READ;

    Geometry::Point _lastPoint;
    Geometry::Point _currentPoint;

    for(;;){
        _currentPoint.x = Axe::getStepDriver(Axe::AXE_X)->getPositionMM();
        _currentPoint.y = Axe::getStepDriver(Axe::AXE_Y)->getPositionMM();
        _currentPoint.z = Axe::getStepDriver(Axe::AXE_Z)->getPositionMM();
        // _currentPoint.a = Axe::getStepDriver(Axe::AXE_A)->getPositionMM();
        // _currentPoint.b = Axe::getStepDriver(Axe::AXE_B)->getPositionMM();
        // _currentPoint.c = Axe::getStepDriver(Axe::AXE_C)->getPositionMM();

        if(!Geometry::pointsIsEqual(&_lastPoint, &_currentPoint)){
            memcpy((data+2), &_currentPoint, 4*3);      // отправка координат x, y, z
            ws_server_send_bin_all((char *)data, 16);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else{
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        memcpy(&_lastPoint, &_currentPoint, sizeof(Geometry::Point));
    }
}

/**
 * Запустить задачу уведомления об изменении текущих координат
 */
void CncRouter::enableCurrentPointNotify(){
    // 
    xTaskCreate(CncRouter::currentPointTask, "currentPointTask", 2048, NULL, 10, NULL);
}



// /**
//  * Получение текущих координат
//  */
// GcodeCoord* CncRouter::getCurrentCoord(){
//     return progParams.currentCoord;
// }

/**
 * Установка плазмы
 * @param plasma плазма
 */
void CncRouter::setPlasma(Plasma *plasma){
    _plasma = plasma;
    _plasma->setArcStartedCallback(CncRouter::plasmaArcStartedCallback);
}

/**
 * Получение плазмы
 */
Plasma* CncRouter::getPlasma(){
    return _plasma;
}

/**
 * Функция обратного вызова при изменении значения вывода флага рабочего режима дуги плазмы
 */
void CncRouter::plasmaArcStartedCallback(bool started){
    printf("CncRouter::plasmaArcStartedCallback: %d \n", started);
    uint8_t dataResp[16] = {0};
    dataResp[0] = WS_OBJ_NAME_PLASMA_ARC;
    dataResp[1] = WS_PLASMA_ARC_STARTED;
    dataResp[2] = (uint8_t) started;
    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);

    // ProgParams *progParams = instance->getProgParams();
    // if(started && progParams->plasmaArc == PLASMA_ARC_START){
    //     // возобновление основной задачи gcode
    //     vTaskResume(gcodeTaskHandle);
    // }

}

// /**
//  * Установка тестового прогона программы gcode
//  * @param checked тестовый прогон программы gcode
//  */
// void CncRouter::setTestRunChecked(bool checked){
//     _testRunChecked = checked;
// }


