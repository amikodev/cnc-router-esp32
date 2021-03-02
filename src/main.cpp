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

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include <unistd.h>
#include "esp_timer.h"


#include "wifi.hpp"
#include "httprequest.hpp"
#include "sdcard-storage.hpp"
#include "spiffs-storage.hpp"
#include "shiftload.hpp"

#include "Axe.hpp"
#include "CncRouter.hpp"
#include "Plasma.hpp"

#define TAG "CNC Router main"

extern "C" {
    void app_main(void);
}

void app_main() {
    ESP_LOGI(TAG, "Project CNC Router");

    Axe::init(Axe::AXES_4);

    // CncRouter    
    CncRouter *router = new CncRouter();
    
    // ось X
    Axe::getStepDriver(Axe::AXE_X)
        ->initPins(GPIO_NUM_15, GPIO_NUM_2)
        ->setPulses(3200)
        ->setLetter('X')
        ->setRevMM(100.0*1.06273)
        ->setLimMin(0.0)
        ->setLimMax(1550.0)
    ;
    
    // ось Y
    Axe::getStepDriver(Axe::AXE_Y)
        ->initPins(GPIO_NUM_4, GPIO_NUM_5)
        ->setPulses(800)
        ->setReductor(4)
        ->setLetter('Y')
        ->setRevMM(100.0*1.04888)
        ->setLimMin(0.0)
        ->setLimMax(2600.0)
    ;
    
    // ось Z
    Axe::getStepDriver(Axe::AXE_Z)
        ->initPins(GPIO_NUM_18, GPIO_NUM_19)
        ->setPulses(800)
        ->setLetter('Z')
        ->setRevMM(1.75)
        ->setLimMin(-150.0)
        ->setLimMax(0.0)
    ;
    
    // ось A
    Axe::getStepDriver(Axe::AXE_A)
        ->initPins(GPIO_NUM_21, GPIO_NUM_22)
    ;

    // синхронизация оси A с осью Y
    Axe::getStepDriver(Axe::AXE_Y)
        ->addSyncChild(Axe::getStepDriver(Axe::AXE_A))
    ;

    router
        ->setPinLimits(GPIO_NUM_13)
        ->setPinHomes(GPIO_NUM_26)
        ->setPinProbe(GPIO_NUM_14)
        ->setPinEStop(GPIO_NUM_27)
    ;

    // PlasmaCut
    Plasma *plasma = new Plasma();
    plasma->initPins(GPIO_NUM_25, GPIO_NUM_33);
    plasma->setInverseStart(true);
    plasma->stop();
    router->setPlasma(plasma);
    GCode::setPlasma(plasma);

    GCode::setFastSpeed(70.0);
    GCode::setWorkSpeed(50.0);

    // Wifi
    Wifi wifi;
    wifi.setHostname((char *) "CNC-Router");
    wifi.setupAP();
    wifi.recieveBinary(CncRouter::parseWsData);

    // SD card
    // SdCardStorage *card = new SdCardStorage();
    // if(card->initSpi(GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_18, GPIO_NUM_5)){
    //     // sdmmc_card_t *cardInfo = card->getCardInfo();
    //     Wifi::setSdCard(card);
    // }

    // spiffs
    SpiffsStorage *spiffs = new SpiffsStorage();
    if(spiffs->init((char *)"/spiffs")){
        Wifi::setSpiffs(spiffs);
    }

    ws_server_start();
    xTaskCreate(&Wifi::serverTask, "server_task", 3000, NULL, 9, NULL);
    xTaskCreate(&Wifi::serverHandleTask, "server_handle_task", 4000, NULL, 6, NULL);

    // запустить задачу уведомления об изменении текущих координат
    router->enableCurrentPointNotify();

}

