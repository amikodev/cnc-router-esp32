Автор: Приходько Дмитрий

# CNC Router for ESP32

## Настройка
* Скачать [VSCode](https://code.visualstudio.com/download)
* Установить [Platformio IDE for VSCode](https://platformio.org/install/ide?install=vscode)
* Изучить [основы работы Platformio](https://docs.platformio.org/en/latest/integration/ide/vscode.html#quick-start)
* Скачать [NodeJS](https://nodejs.org/en/)

## Структура проекта
```
Projects
└---PlatformIO
    └---CNC_Router
    └---components
    ...
└---reactjs
    └---factory-management
```

## Установка
* В папке `CNC_Router` выполнить `git clone https://github.com/amikodev/cnc-router-esp32.git .`
* В папке `components` выполнить `git clone https://github.com/amikodev/components-esp32.git .`
* В папке `factory-management` выполнить `git clone https://github.com/amikodev/factory-reactjs.git .`
* В папке `factory-management` запустить сервер `npm start`

## Настройка переменных для компиляции
* В папке `CNC_Router` выполнить `pio run -t menuconfig`
```
Component config --->
    Amikodev Wi-Fi --->
        далее задаёте настройки новой точки доступа и/или подключение к существующей сети
        пункт 'SD card' зарезервирован для последующей реализации
    FAT Filesystem support --->
        Long filename support --->
            Long filename buffer in heap
```

