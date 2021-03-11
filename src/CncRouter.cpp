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
#define TAG_MAGICSEE "Magicsee"

CncRouter* CncRouter::instance = NULL;

uint32_t CncRouter::currentNumLine = 0;
TaskHandle_t CncRouter::_currentGCodeTask = NULL;
// CncRouter::WsData CncRouter::wsData;

xQueueHandle CncRouter::input0EvtQueue = NULL;
xQueueHandle CncRouter::wsSendEvtQueue = NULL;
xQueueHandle CncRouter::wsSendCustomEvtQueue = NULL;




const uint8_t CncRouter::WS_OBJ_NAME_CNC_ROUTER = 0x4F;
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
const uint8_t CncRouter::WS_PREPARE_STOP = 0x03;

const uint8_t CncRouter::WS_PLASMA_ARC_START = 0x01;
const uint8_t CncRouter::WS_PLASMA_ARC_STARTED = 0x02;
const uint8_t CncRouter::WS_PLASMA_ARC_VOLTAGE = 0x03;



/**
 * Конструктор
 */
CncRouter::CncRouter(){

    instance = this;

    // создание очереди и запуск задачи для обработки прерываний со входов
    CncRouter::input0EvtQueue = xQueueCreate(10, sizeof(Input0Interrupt));
    xTaskCreate(CncRouter::input0Task, "input0Task", 2048, NULL, 10, NULL);

    // очередь и задача по отправке данных клиенту
    void *ptr;
    CncRouter::wsSendEvtQueue = xQueueCreate(10, sizeof(ptr));
    xTaskCreate(CncRouter::wsSendTask, "wsSendTask", 2048, NULL, 10, NULL);

    // очередь и задача по отправке данных произвольного размера клиенту

    CncRouter::wsSendCustomEvtQueue = xQueueCreate(10, sizeof(WsData));
    xTaskCreate(CncRouter::wsSendCustomTask, "wsSendCustomTask", 2048, NULL, 10, NULL);

    // задача по отправке номера строки клиенту
    GCode::setNotifyNumLineFunc(notifyGcodeNumLine);
    xTaskCreate(CncRouter::currentNumLineTask, "currentNumLine", 2048, NULL, 10, NULL);

    // уведомление о завершении выполнения программы gcode
    GCode::setNotifyFinishFunc(notifyGcodeFinish);

}

/**
 * Получение экземпляра
 */
CncRouter* CncRouter::getInstance(){
    return instance;
}

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
                    ESP_LOGI(TAG, "WS_OBJ_NAME_AXE --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                    sd->actionRunStop();
                    sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
                }
            } else if(*(data+1) == WS_CMD_STOP){
                Axe::AXE axe = (Axe::AXE)(*(data+2));

                StepDriver *sd = Axe::getStepDriver(axe);
                ESP_LOGI(TAG, "WS_OBJ_NAME_AXE --STOP-- axe: %c", sd->getLetter());

                sd->actionRunStop();
            }

        } else if(*(data) == WS_OBJ_NAME_CNC_GCODE_PREPARE){
            if(*(data+1) == WS_PREPARE_SIZE){
                uint32_t size = *(data+2) + (*(data+3)<<8) + (*(data+4)<<16) + (*(data+5)<<24);

                ESP_LOGI(TAG, "Prepare size: %d", size);

                uint8_t dataResp[16] = {0};
                dataResp[0] = WS_OBJ_NAME_CNC_GCODE_PREPARE;
                dataResp[1] = WS_PREPARE_SIZE;

                if(GCode::init(size)){
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
            } else if(*(data+1) == WS_PREPARE_STOP){
                uint8_t dataResp[16] = {0};
                dataResp[0] = WS_OBJ_NAME_CNC_GCODE_PREPARE;
                dataResp[1] = WS_PREPARE_STOP;

                if(GCode::isRunned()){
                    // останавливаем программу
                    GCode::stop();
                    dataResp[2] = 0x01;
                    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);

                    // останавливаем плазму
                    instance->getPlasma()->stop();
                    memset(&dataResp, 0x00, 16);
                    dataResp[0] = WS_OBJ_NAME_PLASMA_ARC;
                    dataResp[1] = WS_PLASMA_ARC_START;
                    dataResp[2] = WS_CMD_STOP;
                    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);

                } else{
                    dataResp[2] = 0x00;
                    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
                }
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
                float speed = GCode::getFastSpeed();        // mm/sec
                if(speed == 0.0) speed = 50.0;
                ActionMove::gotoPoint(&point, speed, NULL);
            }
        } 
        
        // Plasma Arc
        else if(*(data) == WS_OBJ_NAME_PLASMA_ARC){
            uint8_t dataResp[16] = {0};
            // Plasma *plasma = instance->getPlasma();
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

        // общая информация о контроллере
        else if(*(data) == WS_OBJ_NAME_CNC_ROUTER){
            uint8_t dataResp[48] = {0};
            dataResp[0] = WS_OBJ_NAME_CNC_ROUTER;

            dataResp[1] = EQUIPMENT_TYPE;
            dataResp[2] = EQUIPMENT_SUBTYPE;

            uint32_t version = (VERSION_MAJOR<<24) + (VERSION_MINOR<<16) + (VERSION_BUILD<<8) + (VERSION_REVISION);
            memcpy((dataResp+3), &version, sizeof(uint32_t));

            Plasma *plasma = instance->getPlasma();
            if(plasma != NULL)
                dataResp[7] = plasma->getArcStarted();

            dataResp[8] = GCode::isRunnable();
            dataResp[9] = GCode::isRunned();

            // memset(&dataResp, 0x00, 16);
            const char *equipmentTypeName = "CNCROUTER";
            memcpy((dataResp+16), equipmentTypeName, std::min(16, (int)strlen(equipmentTypeName)));
            const char *equipmentSubTypeName = "PLASMACUT";
            memcpy((dataResp+32), equipmentSubTypeName, std::min(16, (int)strlen(equipmentSubTypeName)));

            static WsData wsData = {
                .size = 48,
                // .data = (uint8_t *) &dataResp
                .ptr = malloc(48)
            };
            memcpy(wsData.ptr, &dataResp, 48);

            // wsData.size = 48;
            // // wsData.ptr = dataResp;
            // wsData.ptr = malloc(48);
            // memcpy(wsData.ptr, &dataResp, 48);

            // ESP_LOGI(TAG, "xQueueGenericSend: %p", wsData.ptr);
            // ESP_LOG_BUFFER_HEXDUMP(TAG, (void *) wsData.ptr, wsData.size, ESP_LOG_INFO);

            // memcpy(&wsData.ptr, &dataResp, 48);
            // xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
            xQueueGenericSend(wsSendCustomEvtQueue, (void *) &wsData, (TickType_t) 0, queueSEND_TO_BACK);


            // xTaskCreate(CncRouter::wsSendCustomTask, "wsSendCustomTask", 2048, (void *) &wsData, 10, NULL);


        }

    } else if(length > 16){
        if(*(data) == WS_OBJ_NAME_CNC_GCODE){
            // первые 16 байт отбрасываются
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
    int level = -1;
    for(;;){
        if(xQueueReceive(input0EvtQueue, ii, portMAX_DELAY)){
            switch(ii->type){
                case INPUT0_LIMITS:
                    level = gpio_get_level(instance->getPinLimits());
                    ESP_LOGI(TAG, "Limits interrupt: %d", level);
                    break;
                case INPUT0_HOMES:
                    level = gpio_get_level(instance->getPinHomes());
                    ESP_LOGI(TAG, "Homes interrupt: %d", level);
                    break;
                case INPUT0_PROBE:
                    level = gpio_get_level(instance->getPinProbe());
                    ESP_LOGI(TAG, "Probe interrupt: %d", level);
                    break;
                case INPUT0_ESTOP:
                    level = gpio_get_level(instance->getPinEStop());
                    ESP_LOGI(TAG, "EStop interrupt: %d", level);
                    break;
                default:
                    ESP_LOGI(TAG, "Unknown interrupt");
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

/**
 * 
 */
void CncRouter::wsSendTask(void *arg){
    uint8_t data[16] = {0};
    for(;;){
        if(xQueueReceive(wsSendEvtQueue, &data, portMAX_DELAY)){
            ws_server_send_bin_all((char *)&data, 16);
        }
    }
}

/**
 * 
 */
void CncRouter::wsSendCustomTask(void *arg){
    WsData *wsData = new WsData();
    for(;;){
        if(xQueueReceive(wsSendCustomEvtQueue, wsData, portMAX_DELAY)){
            ws_server_send_bin_all((char *) wsData->ptr, wsData->size);
            // free(wsData->ptr);
        }
    }
}

/**
 * 
 */
void CncRouter::currentPointTask(void *arg){
    uint8_t data[32] = {0};

    data[0] = WS_OBJ_NAME_COORDS;
    data[1] = WS_CMD_READ;

    Geometry::Point _lastPoint;
    Geometry::Point _currentPoint;

    std::function<float (Axe::AXE)> getPosByAxe = [&](Axe::AXE axe)->float{
        StepDriver *sd = Axe::getStepDriver(axe);
        float pos = (sd != NULL && !sd->getIsSynced()) ? sd->getPositionMM() : 0.0;
        return pos;
    };

    for(;;){
        _currentPoint.x = getPosByAxe(Axe::AXE_X);
        _currentPoint.y = getPosByAxe(Axe::AXE_Y);
        _currentPoint.z = getPosByAxe(Axe::AXE_Z);
        _currentPoint.a = getPosByAxe(Axe::AXE_A);
        _currentPoint.b = getPosByAxe(Axe::AXE_B);
        _currentPoint.c = getPosByAxe(Axe::AXE_C);

        if(!Geometry::pointsIsEqual(&_lastPoint, &_currentPoint)){
            memcpy((data+2), &_currentPoint, 4*3);          // отправка координат x, y, z
            data[15] = 0x01;                                // продолжение данных в следующих 16 байтах
            memcpy((data+16), ((void *) ( ((uint8_t *)&_currentPoint)+12 )), 4*3);    // отправка координат a, b, c
            ws_server_send_bin_all((char *)data, 32);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else{
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        memcpy(&_lastPoint, &_currentPoint, sizeof(Geometry::Point));
    }
}

/**
 * Запустить задачу уведомления об изменении текущих координат
 */
void CncRouter::enableCurrentPointNotify(){
    xTaskCreate(CncRouter::currentPointTask, "currentPointTask", 2048, NULL, 10, NULL);
}

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
void CncRouter::plasmaArcStartedCallback(bool started, bool notifyIfStart){
    ESP_LOGI(TAG, "plasmaArcStartedCallback: %d", started);
    uint8_t dataResp[16] = {0};
    dataResp[0] = WS_OBJ_NAME_PLASMA_ARC;
    dataResp[1] = WS_PLASMA_ARC_STARTED;
    dataResp[2] = (uint8_t) started;
    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);

    if(started && notifyIfStart && GCode::gcodeTaskHandle != NULL){
        // возобновление основной задачи gcode
        vTaskResume(GCode::gcodeTaskHandle);
    }
}

/**
 * Отправка номера строки gcode клиенту
 */
void CncRouter::notifyGcodeNumLine(uint32_t numLine){
    currentNumLine = numLine;
}

/**
 * 
 */
void CncRouter::currentNumLineTask(void *arg){
    uint8_t data[16] = {0};
    data[0] = WS_OBJ_NAME_CNC_GCODE;

    uint32_t lastNumLine = 0;

    for(;;){
        if(lastNumLine != currentNumLine){
            data[1] = (currentNumLine & 0xFF);
            data[2] = (currentNumLine >> 8);
            ws_server_send_bin_all((char *)data, 16);
            vTaskDelay(pdMS_TO_TICKS(200));
        } else{
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    // // xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);
}

/**
 * Уведомление о завершении выполнения программы gcode
 */
void CncRouter::notifyGcodeFinish(){
    uint8_t dataResp[16] = {0};
    dataResp[0] = WS_OBJ_NAME_CNC_GCODE_PREPARE;
    dataResp[1] = WS_PREPARE_STOP;
    dataResp[2] = 0x01;
    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);

    memset(&dataResp, 0x00, 16);
    dataResp[0] = WS_OBJ_NAME_CNC_GCODE_PREPARE;
    dataResp[1] = WS_PREPARE_RUN;
    dataResp[2] = 0x00;
    xQueueGenericSend(wsSendEvtQueue, (void *) &dataResp, (TickType_t) 0, queueSEND_TO_BACK);

}

/**
 * Функция обработки нажатия кнопок джойстика Magicsee R1
 */
void CncRouter::magicseeKeyboardProcessEvent(esp_r1_keyboard_data_t *data){
    ESP_LOGI(TAG_MAGICSEE, "event: %d in state %d", data->id, data->state);

    float speed = GCode::getWorkSpeed();
    bool runAfterLimit = true;
    StepDriver *sd = NULL;
    uint8_t dir;

    // std::function<void (StepDriver *sd, uint8_t dir)> runAxe = [&](StepDriver *sd, uint8_t dir){

    // };

    switch(data->id){
        case R1_BUTTON2:        // кнопка A
        case R1_BUTTON4:        // кнопка B
        case R1_BUTTON1:        // кнопка C
        case R1_BUTTON5:        // кнопка D
            break;
        case R1_BUTTON7:        // нижняя кнопка 
            sd = Axe::getStepDriver(Axe::AXE_Z);
            if(data->state == R1_KEY_PRESSED){
                // Z-
                dir = WS_AXE_DIRECTION_BACKWARD;
                ESP_LOGI(TAG_MAGICSEE, " --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                sd->actionRunStop();
                sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
            } else{
                // остановить движение
                ESP_LOGI(TAG_MAGICSEE, " --STOP-- axe: %c", sd->getLetter());
                sd->actionRunStop();
            }
            break;
        case R1_BUTTON8:        // верхняя кнопка 
            sd = Axe::getStepDriver(Axe::AXE_Z);
            if(data->state == R1_KEY_PRESSED){
                // Z+
                dir = WS_AXE_DIRECTION_FORWARD;
                ESP_LOGI(TAG_MAGICSEE, " --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                sd->actionRunStop();
                sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
            } else{
                // остановить движение
                ESP_LOGI(TAG_MAGICSEE, " --STOP-- axe: %c", sd->getLetter());
                sd->actionRunStop();
            }
            break;
        case R1_AXIS_X:         // джойстик X
            sd = Axe::getStepDriver(Axe::AXE_X);
            if(data->state == R1_AXIS_PLUS){
                // X-
                dir = WS_AXE_DIRECTION_BACKWARD;
                ESP_LOGI(TAG_MAGICSEE, " --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                sd->actionRunStop();
                sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
            } else if(data->state == R1_AXIS_CENTER){
                // остановить движение
                ESP_LOGI(TAG_MAGICSEE, " --STOP-- axe: %c", sd->getLetter());
                sd->actionRunStop();
            } else if(data->state == R1_AXIS_MINUS){
                // X+
                dir = WS_AXE_DIRECTION_FORWARD;
                ESP_LOGI(TAG_MAGICSEE, " --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                sd->actionRunStop();
                sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
            }
            break;
        case R1_AXIS_Y:         // джойстик Y
            sd = Axe::getStepDriver(Axe::AXE_Y);
            if(data->state == R1_AXIS_PLUS){
                // Y+
                dir = WS_AXE_DIRECTION_FORWARD;
                ESP_LOGI(TAG_MAGICSEE, " --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                sd->actionRunStop();
                sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
            } else if(data->state == R1_AXIS_CENTER){
                // остановить движение
                ESP_LOGI(TAG_MAGICSEE, " --STOP-- axe: %c", sd->getLetter());
                sd->actionRunStop();
            } else if(data->state == R1_AXIS_MINUS){
                // Y-
                dir = WS_AXE_DIRECTION_BACKWARD;
                ESP_LOGI(TAG_MAGICSEE, " --RUN-- axe: %c; dir: %d; speed: %f; runAfterLimit: %d", sd->getLetter(), dir, speed, runAfterLimit);
                sd->actionRunStop();
                sd->actionRun(StepDriver::MODE_MOVE, dir == WS_AXE_DIRECTION_FORWARD ? false : true, speed, !runAfterLimit);
            }
            break;
        default:
            break;

    };
}

/**
 * Функция обработки событий от устройства джойстика Magicsee R1
 */
void CncRouter::magicseeDeviceEvent(esp_r1_device_event_e event){
    switch(event){
        case R1_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MAGICSEE, "event: disconnected");
            Axe::getStepDriver(Axe::AXE_X)->actionRunStop();
            Axe::getStepDriver(Axe::AXE_Y)->actionRunStop();
            Axe::getStepDriver(Axe::AXE_Z)->actionRunStop();
            break;
        case R1_EVENT_CONNECTED:
            ESP_LOGI(TAG_MAGICSEE, "event: connected");
            break;
        default:
            break;
    }
}


