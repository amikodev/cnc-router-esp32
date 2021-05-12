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

#include "StepDriver.hpp"

#define TAG "StepDriver"

/**
 * Конструктор
 */
StepDriver::StepDriver(){

    MotorActionRun *lmp = &lastMotorActionRun;
    lmp->stepDriver = NULL;
    lmp->cPulse = 0;
    lmp->dir = false;
    lmp->speed = 0.0;
    lmp->stopLimit = true;
    lmp->changeInternalPosition = false;

}


void StepDriver::calcK1(){
    k1 = _pulses*_reductor/_rmm;
}

/**
 * Инициализация выводов.
 * При использовании с методом initShiftLoad выбрасывается исключение.
 * @param pinPul вывод импульсов
 * @param pinDir вывод направления вращения
 */
StepDriver* StepDriver::initPins(gpio_num_t pinPul, gpio_num_t pinDir){
    _pinPul = pinPul;
    _pinDir = pinDir;

    uint64_t bitMask = 0;
    bitMask |= (1Ull << pinPul);
    bitMask |= (1Ull << pinDir);

    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pin_bit_mask = bitMask;
    gpio_config(&io_conf);

    return this;
}

/**
 * Инициализация сдвигового регистра.
 * При использовании с методом initPins выбрасывается исключение.
 * Не реализовано.
 * @param sl сдвиговый регистр
 * @param numPul номер вывода импульсов
 * @param numDir номер вывода направления вращения
 */
StepDriver* StepDriver::initShiftLoad(ShiftLoad *sl, uint8_t numPul, uint8_t numDir){


    return this;
}

/**
 * Установка количества импульсов на 1 оборот
 * @param pulses количество импульсов
 */
StepDriver* StepDriver::setPulses(uint16_t pulses){
    _pulses = pulses;
    calcK1();
    return this;
}

/**
 * Получение количества импульсов на 1 оборот
 */
uint16_t StepDriver::getPulses(){
    return _pulses;
}


/**
 * Установка буквы для оси
 * @param letter буква для оси
 */
StepDriver* StepDriver::setLetter(char letter){
    _letter = letter;
    return this;
}

/**
 * Получение буквы для оси
 */
char StepDriver::getLetter(){
    return _letter;
}

/**
 * Установка передаточного числа редуктора
 * @param reductor передаточное число
 */
StepDriver* StepDriver::setReductor(float reductor){
    _reductor = reductor;
    calcK1();
    return this;
}

/**
 * Получение передаточного числа редуктора
 */
float StepDriver::getReductor(){
    return _reductor;
}

/**
 * Установка расстояния проходимого за 1 оборот, в мм
 */
StepDriver* StepDriver::setRevMM(double mm){
    _rmm = mm;
    calcK1();
    return this;
}

/**
 * Рассчёт параметров до целевой позиции
 * @param targetMM целевая позиция, в мм
 * @param speed скорость, мм/сек
 */
StepDriver::MotorTarget* StepDriver::calcTarget(float targetMM, float speed){
    getPositionMM();

    if(speed > _maxSpeed)
        speed = _maxSpeed;

    // printf("axe: %c, targetMM: %f, _positionMM: %f, _positionMax: %llu, _position: %llu \n", _letter, targetMM, _positionMM, _positionMax, _position);
    if(_positionMM == targetMM)
        return NULL;

    double dxMM = targetMM - _positionMM;
    bool dir = false;       // направление: false - вперёд, true - назад
    if(dxMM < 0){
        dxMM = -dxMM;
        dir = true;
    }
    uint32_t dxPulses = ((uint32_t)(k1 * dxMM)) << 1;       // количество импульсов необходимое для достижения цели

    MotorTarget *mt = &motorTarget;
    mt->stepDriver = this;
    mt->dir = dir;
    mt->dxPulses = dxPulses;
    mt->speed = speed;

    mt->mksStep = 1000000.0/( k1*2 * speed );

    return mt;
}

/**
 * Передача импульса на контроллер
 * @param level уровень
 */
void StepDriver::setPulseLevel(bool level){
    gpio_set_level(_pinPul, level ? 1 : 0);
}

/**
 * Установка направления вращения
 * @param dir направление (false - вперёд, true - назад)
 */
void StepDriver::setDirection(bool dir){
    // направление для текущей оси
    gpio_set_level(_pinDir, dir ? 1 : 0);

    // направление для дочерних синхронизируемых
    for(uint8_t i=0; i<this->getSyncChildsCount(); i++){
        this->getSyncChilds()[i]->setDirection(dir ? 1 : 0);
    }
}

/**
 * Отправка импульсов по таймеру для рабочего режима
 * @param arg параметры таймера
 */
void StepDriver::timerRunCallback(void* arg){
    MotorTarget *mt = (MotorTarget *)arg;

    if(mt->cPulse < mt->dxPulses && !mt->stepDriver->getNeedStop()){

        bool imp = (mt->cPulse % 2) == 0;

        // импульс для текущей оси
        mt->stepDriver->setPulseLevel(imp);

        // импульсы для дочерних синхронизируемых
        for(uint8_t i=0; i<mt->stepDriver->getSyncChildsCount(); i++){
            mt->stepDriver->getSyncChilds()[i]->setPulseLevel(imp);
        }

        mt->cPulse++;
        if(imp){
            mt->stepDriver->positionInc(!mt->dir ? 1 : -1);
        }
    } else{
        // остановка и удаление таймера
        mt->stepDriver->stopTimerRun();

        // ESP_LOGI(TAG, "timerRun stop : axe: %c, position: %llu, positionMM: %f", mt->stepDriver->getLetter(), mt->stepDriver->getPosition(), mt->stepDriver->getPositionMM());
        if(mt->callbackFinish != NULL){
            mt->callbackFinish(mt->stepDriver);
        }
    }
}

/**
 * Установка таймера движения
 * @param timer таймер
 */
void StepDriver::setTimerRun(esp_timer_handle_t timer){
    _timerRun = timer;
    needStop = false;
}

/**
 * Получение таймера движения
 */
esp_timer_handle_t StepDriver::getTimerRun(){
    return _timerRun;
}

/**
 * Завершение таймера движения
 */
void StepDriver::stopTimerRun(){
    if(_timerRun != NULL){
        // ESP_ERROR_CHECK(esp_timer_stop(_timerRun));
        // ESP_ERROR_CHECK(esp_timer_delete(_timerRun));
        esp_timer_stop(_timerRun);
        esp_timer_delete(_timerRun);
        _timerRun = NULL;
    }
}

/**
 * Остановка перемещения
 */
void StepDriver::stop(){
    if(_timerRun != NULL){
        needStop = true;
    }
}

/**
 * Запуск простого перемещения без целевой точки
 * @param mode тип работы
 * @param direction направление (вперёд/назад)
 * @param speed скорость, мм/сек
 * @param stopLimit флаг прекращения движения при достижении предела
 */
void StepDriver::actionRun(RUN_MODE mode, bool direction, float speed, bool stopLimit, bool changeInternalPosition){
    // runMode = MODE_PROBE;

    MotorActionRun *mp = &motorActionRun;
    MotorActionRun *lmp = &lastMotorActionRun;

    if(speed > _maxSpeed)
        speed = _maxSpeed;

    if(_timerActionRun != NULL){
        // обработка в случае повторного вызова с теми же значениями скорости и направления
        if(direction == mp->dir && speed == mp->speed){
            return;
        } else{
            // this->actionRunStop();
            ESP_LOGI(TAG, "actionRun axe: %c; change direction from %d to %d", _letter, mp->dir, direction);
            mp->dir = direction;
            this->setDirection(mp->dir);
            return;
        }
    }

    mp->stepDriver = this;
    mp->cPulse = 0;
    mp->dir = direction;
    mp->speed = speed;
    mp->stopLimit = stopLimit;
    mp->changeInternalPosition = changeInternalPosition;

    esp_timer_create_args_t timerRunArgs = {
        .callback = &StepDriver::timerActionRunCallback,
        .arg = (void *)mp,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "actionRun_"+this->getLetter()
    };

    esp_timer_handle_t timerActionRun;
    ESP_ERROR_CHECK(esp_timer_create(&timerRunArgs, &timerActionRun));

    _timerActionRun = timerActionRun;

    uint64_t mksStep = 1000000.0/( _pulses*_reductor*speed/_rmm * 2 );
    if(mksStep < 60) mksStep = 60;
    this->setDirection(mp->dir);

    ESP_LOGI(TAG, "actionRun axe: %c, dir: %d, speed: %f, cip: %d", _letter, direction, speed, changeInternalPosition);
    ESP_ERROR_CHECK(esp_timer_start_periodic(timerActionRun, mksStep));

    memcpy(lmp, mp, sizeof(MotorActionRun));
}

/**
 * Окончание перемещения
 */
void StepDriver::actionRunStop(){
    runMode = MODE_NONE;

    if(_timerActionRun != NULL){
        ESP_LOGI(TAG, "actionRunStop axe: %c", _letter);
        ESP_ERROR_CHECK(esp_timer_stop(_timerActionRun));
        ESP_ERROR_CHECK(esp_timer_delete(_timerActionRun));
        _timerActionRun = NULL;

        MotorActionRun *mp = &motorActionRun;
        if((mp->cPulse % 2) != 0){
            // окончание периода сигнала перемещения
            // импульс для текущей оси
            mp->stepDriver->setPulseLevel((mp->cPulse % 2) == 0);
            // импульсы для дочерних синхронизируемых осей
            for(uint8_t i=0; i<mp->stepDriver->getSyncChildsCount(); i++){
                mp->stepDriver->getSyncChilds()[i]->setPulseLevel((mp->cPulse % 2) == 0);
            }
        }
    }
}

/**
 * Отправка импульсов по таймеру для рабочего режима
 * @param arg параметры таймера
 */
void StepDriver::timerActionRunCallback(void* arg){
    MotorActionRun *mp = (MotorActionRun *)arg;

    uint64_t position = mp->stepDriver->getPosition();
    int64_t pos = (int64_t) position;

    if(mp->dir){    // движемся в обратном направлении, к нулю
        if(mp->stopLimit && pos <= 0){
            // останавливаем движение и таймер
            mp->stepDriver->actionRunStop();
            return;
        }
    } else{         // движемся в прямом направлении, к максимальной позиции
        if(mp->stopLimit && pos >= mp->stepDriver->getPositionMax()){
            // останавливаем движение и таймер
            mp->stepDriver->actionRunStop();
            return;
        }
    }

    // импульс для текущей оси
    mp->stepDriver->setPulseLevel((mp->cPulse % 2) == 0);

    // импульсы для дочерних синхронизируемых осей
    for(uint8_t i=0; i<mp->stepDriver->getSyncChildsCount(); i++){
        mp->stepDriver->getSyncChilds()[i]->setPulseLevel((mp->cPulse % 2) == 0);
    }

    mp->cPulse++;
    if(mp->changeInternalPosition){
        if((mp->cPulse % 2) == 0){
            mp->stepDriver->positionInc(!mp->dir ? 1 : -1);
        }
    }
}

// /**
//  * Получить установку режима проверки начала и окончания координат
//  */
// bool StepDriver::isModeProbe(){
//     return runMode == MODE_PROBE;
// }

/**
 * Получение режима перемещения
 */
StepDriver::RUN_MODE StepDriver::getRunMode(){
    return runMode;
}

/**
 * Установка минимального предела
 * @param limMM минимальный предел, в мм
 */
StepDriver* StepDriver::setLimMin(float limMM){
    _limMinMM = limMM;
    if(_limMinMM != 0 || _limMaxMM != 0){
        // рассчёт максимальной позиции
        float dLimMM = _limMaxMM-_limMinMM;
        _positionMax = _pulses*_reductor/_rmm*dLimMM;
        k2 = dLimMM/_positionMax;
        // ESP_LOGI(TAG, "setLimMin : axe: %c; _positionMax: %llu", _letter, _positionMax);
    }
    return this;
}

/**
 * Получение минимального предела
 */
float StepDriver::getLimMin(){
    return _limMinMM;
}

/**
 * Установка максимального предела
 * @param limMM максимальный предел, в мм
 */
StepDriver* StepDriver::setLimMax(float limMM){
    _limMaxMM = limMM;
    if(_limMinMM != 0 || _limMaxMM != 0){
        // рассчёт максимальной позиции
        float dLimMM = _limMaxMM-_limMinMM;
        _positionMax = _pulses*_reductor/_rmm*dLimMM;
        k2 = dLimMM/_positionMax;
        // ESP_LOGI(TAG, "setLimMax : axe: %c; _positionMax: %llu", _letter, _positionMax);
    }
    return this;
}

/**
 * Получение максимального предела
 */
float StepDriver::getLimMax(){
    return _limMaxMM;
}

/**
 * Изменение текущей позиции
 * @param inc инкрементальное значение величины текущей позиции
 */
void StepDriver::positionInc(int32_t inc){
    _position += inc;
}

/**
 * Получение текущей позиции, в мм
 */
float StepDriver::getPositionMM(){
    int64_t pos = (int64_t)_position;
    // _positionMM = (_limMaxMM-_limMinMM)/_positionMax*pos+_limMinMM;
    _positionMM = k2*pos+_limMinMM;
    return _positionMM;
}

/**
 * Получение текущей позиции, в количестве импульсов
 */
uint64_t StepDriver::getPosition(){
    return _position;
}

/**
 * Получение максимальной позиции, в количестве импульсов
 */
uint64_t StepDriver::getPositionMax(){
    return _positionMax;
}

/**
 * Добавление синхронизируемой оси (максимум 4)
 * @param child синхронизируемая ось
 */
StepDriver* StepDriver::addSyncChild(StepDriver *child){
    if(syncChildsCount < 4){
        syncChilds[syncChildsCount] = child;
        child->setIsSynced(true);
        syncChildsCount++;
    } else{

    }

    return this;
}

/**
 * Получение списка синхронизируемых осей
 */
StepDriver** StepDriver::getSyncChilds(){
    return syncChilds;
}

/** 
 * Получение количества синхронизируемых осей
 */
uint8_t StepDriver::getSyncChildsCount(){
    return syncChildsCount;
}

/**
 * Установка флага того, что данная ось синхронизирована с другой и является дочерней
 * @param synced флаг синхронизации
 */
void StepDriver::setIsSynced(bool synced){
    isSynced = synced;
}

/**
 * Получение флага того, что данная ось синхронизирована с другой и является дочерней
 */
bool StepDriver::getIsSynced(){
    return isSynced;
}

/**
 * Получение флага необходимости остановки перемещения
 */
bool StepDriver::getNeedStop(){
    return needStop;
}

/**
 * Установить максимальную скорость перемещения
 * @param speed скорость перемещения, мм/с
 */
void StepDriver::setMaxSpeed(float speed){
    _maxSpeed = speed;
}

/**
 * Получить максимальную скорость перемещения
 */
float StepDriver::getMaxSpeed(){
    return _maxSpeed;
}


