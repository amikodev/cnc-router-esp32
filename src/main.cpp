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
#include "sdcard.hpp"
#include "shiftload.hpp"

#include "CncRouter.hpp"
#include "Plasma.hpp"


#define PI 3.14159265


extern "C" {
    void app_main(void);
}


// static void some_task2(void *arg){
//     bool state = false;

//     for(;;){
//         sl.writeByNum(0, 1, state);
//         state = !state;

//         vTaskDelay(pdMS_TO_TICKS(100));
//     }

// }

static void some_task(void *arg){

    uint8_t data[16] = {0};

    data[0] = 0x53;
    data[1] = 0x01;

    float lastX = -1.0;
    float lastY = -1.0;
    float lastZ = -1.0;

    float x, y, z;

    CncRouter *router = CncRouter::getInstance();
    for(;;){

        x = router->getAxe(CncRouter::AXE_X)->getPositionMM();
        y = router->getAxe(CncRouter::AXE_Y)->getPositionMM();
        z = router->getAxe(CncRouter::AXE_Z)->getPositionMM();

        if(x != lastX || y != lastY || z != lastZ){
            memcpy((data+2), &x, 4);
            memcpy((data+6), &y, 4);
            memcpy((data+10), &z, 4);

            // clients = ws_server_send_text_all(out, len);
            ws_server_send_bin_all((char *)data, 16);

            vTaskDelay(pdMS_TO_TICKS(100));

        } else{
            vTaskDelay(pdMS_TO_TICKS(500));

        }

        lastX = x;
        lastY = y;
        lastZ = z;

    }
}



void app_main() {

    printf("Project CNC Router \n");
    fflush(stdout);

    // // ShiftLoad
    // ShiftLoad sl;
    // sl.initTask(GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_15);
    // sl.registerCount(1);

    // CncRouter    
    CncRouter *router = new CncRouter(CncRouter::AXES_4);
    
    router->getAxe(CncRouter::AXE_X)
        ->initPins(GPIO_NUM_15, GPIO_NUM_2)
        // ->initShiftLoad(&sl, 0, 1)
        ->setPulses(3200)
        ->setLetter('X')
        ->setRevMM(100.0*1.06273)
        ->setLimMin(0.0)
        ->setLimMax(1550.0)
    ;
    router->getAxe(CncRouter::AXE_Y)
        ->initPins(GPIO_NUM_4, GPIO_NUM_5)
        // ->initPins(GPIO_NUM_4, GPIO_NUM_21)
        // ->setPulses(3200)
        ->setPulses(800)
        ->setReductor(4)
        ->setLetter('Y')
        ->setRevMM(100.0*1.04888)
        ->setLimMin(0.0)
        ->setLimMax(2600.0)
    ;
    router->getAxe(CncRouter::AXE_Z)
        ->initPins(GPIO_NUM_18, GPIO_NUM_19)
        // ->initPins(GPIO_NUM_22, GPIO_NUM_23)
        ->setPulses(800)
        ->setLetter('Z')
        ->setRevMM(1.75)
        ->setLimMin(-150.0)
        ->setLimMax(0.0)
        // ->setRev100mm(1750.0)
        // ->setLimMin(0.0)
        // ->setLimMax(150.0)
    ;
    router->getAxe(CncRouter::AXE_A)
        ->initPins(GPIO_NUM_21, GPIO_NUM_22)
        // ->initPins(GPIO_NUM_33, GPIO_NUM_32)
        // ->setPulses(3200)
        // ->setLetter('A')
        // ->setRev100mm(10000.0)
        // ->setLimMin(0.0)
        // ->setLimMax(2600.0)
    ;

    // синхронизация оси A с осью Y
    router->getAxe(CncRouter::AXE_Y)
        ->addSyncChild(router->getAxe(CncRouter::AXE_A))
    ;

    router
        ->setPinLimits(GPIO_NUM_13)
        ->setPinHomes(GPIO_NUM_26)
        ->setPinProbe(GPIO_NUM_14)
        ->setPinEStop(GPIO_NUM_27)
    ;

    Plasma *plasma = new Plasma();
    plasma->initPins(GPIO_NUM_25, GPIO_NUM_33);
    plasma->setInverseStart(true);
    plasma->stop();
    router->setPlasma(plasma);


    // router->gotoTargetMM(CncRouter::AXE_X, 100);


    // Wifi
    Wifi wifi;
    wifi.setHostname((char *) "CNC-Router");
    wifi.setupAP();
    wifi.recieveBinary(CncRouter::parseWsData);

    // SdCard *card = new SdCard();
    // if(card->initSpi(GPIO_NUM_19, GPIO_NUM_23, GPIO_NUM_18, GPIO_NUM_5)){
    //     // sdmmc_card_t *cardInfo = card->getCardInfo();
    //     Wifi::setSdCard(card);
    // }

    ws_server_start();
    xTaskCreate(&Wifi::serverTask, "server_task", 3000, NULL, 9, NULL);
    xTaskCreate(&Wifi::serverHandleTask, "server_handle_task", 4000, NULL, 6, NULL);

    // ShiftLoad
    // ShiftLoad sl;
    // sl.initSpi(GPIO_NUM_NC, GPIO_NUM_21, GPIO_NUM_18, GPIO_NUM_22);
    // sl.registerCount(1);
    // sl.write(0, 0xBB);

    xTaskCreate(some_task, "some_task", 2048, NULL, 10, NULL);

    // xTaskCreate(some_task2, "some_task2", 2048, NULL, 10, NULL);

}

