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

#ifndef __STEPDRIVER_HPP__
#define __STEPDRIVER_HPP__

#include <functional>

#include "esp_system.h"
// #include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "shiftload.hpp"

/**
 * Шаговый двигатель
 */
class StepDriver{

public:

    struct MotorTarget{
        StepDriver *axe;    // ось
        uint32_t dxPulses;  // количество импульсов
        uint32_t cPulse;    // текущий номер импульса
        bool dir;           // направление
        float speed;        // скорость, мм/сек
        uint64_t mksStep;   // пауза между импульсами, мкс
        // void (*callbackFinish)(StepDriver *sd) = NULL;      // функция вызываемая при окончании перемещения по оси
        std::function<void (StepDriver *sd)> callbackFinish = NULL;
    };

    struct MotorActionRun{
        StepDriver *axe;                // ось
        uint32_t cPulse;                // текущий номер импульса
        bool dir;                       // направление
        float speed;                    // количество оборотов в секунду
        float stopLimit = true;         // флаг прекращения движения при достижении предела
    };

    enum RUN_MODE{
        MODE_NONE = 0,      // не работает
        MODE_MOVE,          // простое перемещение
        MODE_LIMIT,         // поиск предела
        MODE_HOME,          // поиск Home
        MODE_PROBE          // поиск датчика Probe
    };


private:

    gpio_num_t _pinPul = GPIO_NUM_NC;           // вывод импульсов на драйвер мотора
    gpio_num_t _pinDir = GPIO_NUM_NC;           // вывод направления на драйвер мотора
    uint16_t _pulses = 0;                       // количество импульсов на 1 оборот
    char _letter = '-';                         // буква оси
    float _reductor = 1.0;                      // передаточное число редуктора

    uint64_t _position = 0;       // текущая позиция, в количестве импульсов
    uint64_t _target = 0;         // целевая позиция, в количестве импульсов
    uint64_t _positionMax = 0;    // максимальная позиция, в количестве импульсов

    double _rmm = 0.0;            // расстояние проходимое за 1 оборот, в мм
    double _positionMM = 0.0;     // текущая позиция, в мм

    float _limMinMM = 0.0;        // минимальный предел, в мм
    float _limMaxMM = 0.0;        // максимальный предел, в мм

    esp_timer_handle_t _timerRun = NULL;

    RUN_MODE runMode = MODE_NONE;       // режим работы

    // bool _modeProbe = false;        // режим проверки координат
    esp_timer_handle_t _timerActionRun = NULL;

    MotorTarget motorTarget;
    MotorActionRun motorActionRun;

    StepDriver *syncChilds[4] = {NULL, NULL, NULL, NULL};
    uint8_t syncChildsCount = 0;


public:

    /**
     * Конструктор
     */
    StepDriver();

    /**
     * Инициализация выводов.
     * При использовании с методом initShiftLoad выбрасывается исключение.
     * @param pinPul вывод импульсов
     * @param pinDir вывод направления вращения
     */
    StepDriver* initPins(gpio_num_t pinPul, gpio_num_t pinDir);

    /**
     * Инициализация сдвигового регистра.
     * При использовании с методом initPins выбрасывается исключение.
     * @param sl сдвиговый регистр
     * @param numPul номер вывода импульсов
     * @param numDir номер вывода направления вращения
     */
    StepDriver* initShiftLoad(ShiftLoad *sl, uint8_t numPul, uint8_t numDir);

    /**
     * Установка количества импульсов на 1 оборот
     * @param pulses количество импульсов
     */
    StepDriver* setPulses(uint16_t pulses);

    /**
     * Получение количества импульсов на 1 оборот
     */
    uint16_t getPulses();

    /**
     * Установка буквы для оси
     * @param letter буква для оси
     */
    StepDriver* setLetter(char letter);

    /**
     * Получение буквы для оси
     */
    char getLetter();

    /**
     * Установка передаточного числа редуктора
     * @param reductor передаточное число
     */
    StepDriver* setReductor(float reductor);

    /**
     * Получение передаточного числа редуктора
     */
    float getReductor();

    /**
     * Установка расстояния проходимого за 1 оборот, в мм
     */
    StepDriver* setRevMM(double mm);

    /**
     * Рассчёт параметров до целевой позиции
     * @param targetMM целевая позиция, в мм
     * @param speedMmSec скорость, мм/сек
     */
    MotorTarget* calcTarget(float targetMM, float speedMmSec);

    /**
     * Передача импульса на контроллер
     * @param level уровень
     */
    void setPulseLevel(bool level);

    /**
     * Установка направления вращения
     * @param dir направление (false - вперёд, true - назад)
     */
    void setDirection(bool dir);

    /**
     * Отправка импульсов по таймеру для рабочего режима
     * @param arg параметры таймера
     */
    static void timerRunCallback(void* arg);

    /**
     * Установка таймера движения
     * @param timer таймер
     */
    void setTimerRun(esp_timer_handle_t timer);

    /**
     * Получение таймера движения
     */
    esp_timer_handle_t getTimerRun();

    /**
     * Завершение таймера движения
     */
    void stopTimerRun();

    /**
     * Запуск перемещения
     * @param mode тип работы
     * @param direction направление (вперёд/назад)
     * @param speedMmSec скорость, мм/сек
     * @param stopLimit флаг прекращения движения при достижении предела
     */
    void actionRun(RUN_MODE mode, bool direction, float speedMmSec, bool stopLimit=true);

    /**
     * Окончание перемещения
     */
    void actionRunStop();

    /**
     * Отправка импульсов по таймеру для перемещения
     * @param arg параметры таймера
     */
    static void timerActionRunCallback(void* arg);

    /**
     * Получить установку режима проверки начала и окончания координат
     */
    bool isModeProbe();

    /**
     * Получение режима перемещения
     */
    RUN_MODE getRunMode();

    /**
     * Установка минимального предела
     * @param limMM минимальный предел, в мм
     */
    StepDriver* setLimMin(float limMM);

    /**
     * Установка максимального предела
     * @param limMM максимальный предел, в мм
     */
    StepDriver* setLimMax(float limMM);

    /**
     * Изменение текущей позиции
     * @param inc инкрементальное значение величины текущей позиции
     */
    void positionInc(int32_t inc);

    /**
     * Получение текущей позиции, в мм
     */
    float getPositionMM();

    /**
     * Получение текущей позиции, в количестве импульсов
     */
    uint64_t getPosition();

    /**
     * Получение максимальной позиции, в количестве импульсов
     */
    uint64_t getPositionMax();

    /**
     * Добавление синхронизируемой оси (максимум 4)
     * @param child синхронизируемая ось
     */
    StepDriver* addSyncChild(StepDriver *child);

    /**
     * Получение списка синхронизируемых осей
     */
    StepDriver** getSyncChilds();

    /** 
     * Получение количества синхронизируемых осей
     */
    uint8_t getSyncChildsCount();

};

#endif      // __STEPDRIVER_HPP__