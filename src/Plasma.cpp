/*
amikodev/cnc-router-esp32 - CNC Router on esp-idf
Copyright © 2020-2021 Prihodko Dmitriy - asketcnc@yandex.ru
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

#include "Plasma.hpp"

#define TAG "Plasma"

Plasma* Plasma::instance = NULL;

xQueueHandle Plasma::arcStartedEvtQueue = NULL;
xQueueHandle Plasma::arcVoltageEvtQueue = NULL;

unsigned long Plasma::startTime = 0;
uint16_t Plasma::voltCount = 0;

/**
 * Конструктор
 */
Plasma::Plasma(){
    instance = this;

    // создание очереди и запуск задачи для обработки прерываний со входов
    Plasma::arcStartedEvtQueue = xQueueCreate(10, 0);
    xTaskCreate(Plasma::arcStartedTask, "arcStarted", 2048, NULL, 10, NULL);

    Plasma::arcVoltageEvtQueue = xQueueCreate(10, sizeof(PlasmaVolt));
    xTaskCreate(Plasma::arcVoltageTask, "arcVoltage", 8000, NULL, 10, NULL);

    // значения параметров по умолчанию
    float wv = this->workVoltage;
    float dv = this->deviationVoltage;
    float pK = this->pK;
    float pB = this->pB;

    // получить параметры из хранилища NVS
    NvsStorage::open();
    try{
        wv = NvsStorage::getFloat((char *) "plasmaWV");
    } catch(const std::runtime_error &error){}
    try{
        dv = NvsStorage::getFloat((char *) "plasmaDV");
    } catch(const std::runtime_error &error){}
    try{
        pK = NvsStorage::getFloat((char *) "plasmaPK");
    } catch(const std::runtime_error &error){}
    try{
        pB = NvsStorage::getFloat((char *) "plasmaPB");
    } catch(const std::runtime_error &error){}
    NvsStorage::close();

    // применить параметры
    this->setWorkVoltage(wv);
    this->setDeviationVoltage(dv);
    this->setCalcParams(pK, pB);

}

/**
 * Инициализация выводов
 * @param pinStart вывод запуска плазмы
 * @param pinArcStarted вывод флага о рабочем режиме плазмы
* @param pinArcVoltage вывод прерывания для рассчёта напряжения 
 */
void Plasma::initPins(gpio_num_t pinStart, gpio_num_t pinArcStarted, gpio_num_t pinArcVoltage){
    _pinStart = pinStart;
    _pinArcStarted = pinArcStarted;
    _pinArcVoltage = pinArcVoltage;

    int intrFlag = 0;
    gpio_install_isr_service(intrFlag);

    // инициализация вывода запуска плазмы
    uint64_t bitMask = 0;
    if(pinStart != GPIO_NUM_NC){
        bitMask |= (1Ull << pinStart);

        gpio_config_t io_conf_output;
        io_conf_output.mode = GPIO_MODE_OUTPUT;
        io_conf_output.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf_output.pin_bit_mask = bitMask;
        gpio_config(&io_conf_output);
    }

    // инициализация вывода флага о рабочем режиме плазмы
    // установка прерывания
    if(pinArcStarted != GPIO_NUM_NC){
        bitMask = 0;
        bitMask |= (1Ull << pinArcStarted);

        gpio_config_t io_conf_input;
        io_conf_input.intr_type = GPIO_INTR_ANYEDGE;
        io_conf_input.mode = GPIO_MODE_INPUT;
        io_conf_input.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf_input.pin_bit_mask = bitMask;
        gpio_config(&io_conf_input);

        gpio_isr_handler_add(pinArcStarted, Plasma::arcStartedIsrHandler, NULL);
    }

    if(pinArcVoltage != GPIO_NUM_NC){
        bitMask = 0;
        bitMask |= (1Ull << pinArcVoltage);

        gpio_config_t io_conf_input;
        io_conf_input.intr_type = GPIO_INTR_LOW_LEVEL;
        io_conf_input.mode = GPIO_MODE_INPUT;
        // io_conf_input.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf_input.pin_bit_mask = bitMask;
        gpio_config(&io_conf_input);

        PlasmaVolt *pv = new PlasmaVolt();
        gpio_isr_handler_add(pinArcVoltage, Plasma::arcVoltageIsrHandler, (void *) pv);
    }
}

/**
 * Установка инвертированности значения вывода запуска плазмы
 * @param isInverse значение инвертировано
 */
void Plasma::setInverseStart(bool isInverse){
    isInverseStart = isInverse;
}

/**
 * Получение вывода запуска плазмы
 */
gpio_num_t Plasma::getPinStart(){
    return _pinStart;
}

/**
 * Получение вывода флага о рабочем режиме плазмы
 */
gpio_num_t Plasma::getPinArcStarted(){
    return _pinArcStarted;
}

/**
 * Функция прерывания по запуску дуги
 */
void IRAM_ATTR Plasma::arcStartedIsrHandler(void *arg){
    xQueueSendFromISR(arcStartedEvtQueue, NULL, NULL);
}

/**
 * Задача по запуску дуги
 */
void Plasma::arcStartedTask(void *arg){
    for(;;){
        if(xQueueReceive(arcStartedEvtQueue, NULL, portMAX_DELAY)){
            int level = gpio_get_level(instance->getPinArcStarted());
            instance->arcStartedInterrupt(level);
        }
    }
}

/**
 * Прерывание экземпляра
 * @param level значение вывода флага о рабочем режиме плазмы
 */
void Plasma::arcStartedInterrupt(int level){
    if(currentArcValue != level){
        currentArcValue = level;
        ESP_LOGI(TAG, "Arc started interrupt: level: %d, started: %d", level, getArcStarted());
        if(arcStartedFunc != NULL){
            arcStartedFunc(getArcStarted(), notifyIfStart);
        }
        if(getArcStarted()){
            notifyIfStart = false;
        }
    }
}

/**
 * Функция прерывания по напряжению дуги
 */
void IRAM_ATTR Plasma::arcVoltageIsrHandler(void *arg){

    PlasmaVolt *pv = (PlasmaVolt *) arg;

    unsigned long currentTime = esp_timer_get_time();
    if(startTime == 0){
        startTime = currentTime;
        return;
    }

    if(currentTime - startTime >= 100000){

        pv->count = voltCount;
        xQueueSendFromISR(arcVoltageEvtQueue, pv, NULL);

        voltCount = 0;
        startTime = currentTime;
    }

    voltCount++;

    // xQueueSendFromISR(arcVoltageEvtQueue, NULL, NULL);
}

/**
 * Задача по напряжению дуги
 */
void Plasma::arcVoltageTask(void *arg){
    PlasmaVolt *pv = new PlasmaVolt();
    for(;;){
        if(xQueueReceive(arcVoltageEvtQueue, pv, portMAX_DELAY)){
            instance->arcVoltageInterrupt(pv->count*10);
        }
    }
}

/**
 * Прерывание экземпляра
 * @param count количество прерываний
 */
void Plasma::arcVoltageInterrupt(uint16_t count){
    double v = pK * count + pB;
    // ESP_LOGI(TAG, "Plasma: %d; v: %f", count, v);

    VOLTAGE_RANGE range = RANGE_UNKNOWN;
    if(v >= abnormalVoltage){
        range = RANGE_ABNORMAL;
    } else if(v < workVoltage-deviationVoltage){
        range = RANGE_LOWER;
    } else if(v > workVoltage+deviationVoltage){
        range = RANGE_HIGHER;
    } else{
        range = RANGE_NORMAL;
    }

    if(voltageRangeFunc != NULL){
        voltageRangeFunc(range, v, count);
    }
}

/**
 * Запуск плазмы
 */
void Plasma::start(){
    gpio_set_level(_pinStart, !isInverseStart ? 1 : 0);
    notifyIfStart = true;
    ESP_LOGI(TAG, "Arc do start");
}

/**
 * Остановка плазмы
 */
void Plasma::stop(){
    gpio_set_level(_pinStart, !isInverseStart ? 0 : 1);
    ESP_LOGI(TAG, "Arc do stop");

    // остановка оси THC
    if(thcStateOn){
        StepDriver *sd = Axe::getStepDriver(Axe::AXE_Z);
        ESP_LOGI(TAG, "THC Z stop");
        sd->actionRunStop();
    }
}

/**
 * Получение флага о рабочем режиме плазмы
 * true - дуга запущена, рабочий режим
 * false - рабочая дуга не запущена
 */
bool Plasma::getArcStarted(){
    return currentArcValue == 0;
}

/**
 * Установка функции вызываемой при изменении значения флага о рабочем режиме плазмы
 * @param func функция обратного вызова
 */
void Plasma::setArcStartedCallback(ArcStartedFunc func){
    arcStartedFunc = func;
}

/**
 * Установка функции вызываемой с частотой 10 раз в секунду с информацией о том, в каком диапазоне лежит текущее напряжение дуги
 * @param func функция обратного вызова
 */
void Plasma::setVoltageRangeCallback(VoltageRangeFunc func){
    voltageRangeFunc = func;
}

/**
 * Установить рабочее напряжение
 * @param wv напряжение, В
 */
void Plasma::setWorkVoltage(float wv){
    workVoltage = wv;
    ESP_LOGI(TAG, "Set work voltage: %f", workVoltage);
}

/**
 * Получить рабочее напряжение
 */
float Plasma::getWorkVoltage(){
    return workVoltage;
}

/**
 * Установить допустимое расхождение со значением рабочего напряжения
 * @param dv значение расхождения, В
 */
void Plasma::setDeviationVoltage(float dv){
    deviationVoltage = dv;
    ESP_LOGI(TAG, "Set deviation voltage: %f", deviationVoltage);
}

/**
 * Получить расхождение напряжения
 */
float Plasma::getDeviationVoltage(){
    return deviationVoltage;
}

/**
 * Установка параметров для рассчёта напряжения
 * по формуле v = k*intrCount + b;
 * @param k параметр формулы k 
 * @param b параметр формулы b
 */
void Plasma::setCalcParams(float k, float b){
    pK = k;
    pB = b;
    ESP_LOGI(TAG, "Set calc params; k: %f, b: %f", pK, pB);
}

/**
 * Получить значение параметра k
 */
float Plasma::getCalcParamK(){
    return pK;
}

/**
 * Получить значение параметра b
 */
float Plasma::getCalcParamB(){
    return pB;
}

/**
 * Включить THC 
 */
void Plasma::thcOn(){
    thcStateOn = true;
}

/**
 * Выключить THC 
 */
void Plasma::thcOff(){
    thcStateOn = false;
}

/**
 * Получить статус включения THC
 */
bool Plasma::isThcOn(){
    return thcStateOn;
}

/**
 * Установить скорость работы THC
 * @param speed скорость, мм/с
 */
void Plasma::setThcSpeed(float speed){
    thcSpeed = speed;
}

/**
 * Получить скорость работы THC
 */
float Plasma::getThcSpeed(){
    return thcSpeed;
}


