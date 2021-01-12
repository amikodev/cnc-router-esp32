/*
amikodev/cnc-router-esp32 - CNC Router on esp-idf
Copyright © 2020 Prihodko Dmitriy - prihdmitriy@yandex.ru
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

Plasma* Plasma::instance = NULL;

xQueueHandle Plasma::arcStartedEvtQueue = NULL;

/**
 * Конструктор
 */
Plasma::Plasma(){
    instance = this;

    // создание очереди и запуск задачи для обработки прерываний со входов
    Plasma::arcStartedEvtQueue = xQueueCreate(10, 0);
    xTaskCreate(Plasma::arcStartedTask, "arcStartedTask", 2048, NULL, 10, NULL);
}

/**
 * Инициализация выводов
 * @param pinStart вывод запуска плазмы
 * @param pinArcStarted вывод флага о рабочем режиме плазмы
 */
void Plasma::initPins(gpio_num_t pinStart, gpio_num_t pinArcStarted){
    _pinStart = pinStart;
    _pinArcStarted = pinArcStarted;

    // инициализация вывода запуска плазмы
    uint64_t bitMask = 0;
    bitMask |= (1Ull << pinStart);

    gpio_config_t io_conf_output;
    io_conf_output.mode = GPIO_MODE_OUTPUT;
    io_conf_output.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf_output.pin_bit_mask = bitMask;
    gpio_config(&io_conf_output);

    // инициализация вывода флага о рабочем режиме плазмы
    // установка прерывания
    bitMask = 0;
    bitMask |= (1Ull << pinArcStarted);

    gpio_config_t io_conf_input;
    io_conf_input.intr_type = GPIO_INTR_ANYEDGE;
    io_conf_input.mode = GPIO_MODE_INPUT;
    io_conf_input.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf_input.pin_bit_mask = bitMask;
    gpio_config(&io_conf_input);

    int intrFlag = 0;
    gpio_install_isr_service(intrFlag);

    gpio_isr_handler_add(pinArcStarted, Plasma::arcStartedIsrHandler, NULL);
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
 * 
 */
void IRAM_ATTR Plasma::arcStartedIsrHandler(void *arg){
    xQueueSendFromISR(arcStartedEvtQueue, NULL, NULL);
}

/**
 * 
 */
void Plasma::arcStartedTask(void *arg){
    for(;;){
        if(xQueueReceive(arcStartedEvtQueue, NULL, portMAX_DELAY)){
            int level = gpio_get_level(instance->getPinArcStarted());
            instance->arcInterrupt(level);
        }
    }
}

/**
 * Прерывание экземпляра
 * @param level значение вывода флага о рабочем режиме плазмы
 */
void Plasma::arcInterrupt(int level){
    if(currentArcValue != level){
        currentArcValue = level;
        printf("Plasma arc started interrupt: level: %d, started: %d \n", level, getArcStarted());
        if(arcStartedFunc != NULL){
            arcStartedFunc(getArcStarted());
        }
    }

}

/**
 * Запуск плазмы
 */
void Plasma::start(){
    gpio_set_level(_pinStart, !isInverseStart ? 1 : 0);
    printf("Plasma arc do start \n");
}

/**
 * Остановка плазмы
 */
void Plasma::stop(){
    gpio_set_level(_pinStart, !isInverseStart ? 0 : 1);
    printf("Plasma arc do stop \n");
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


