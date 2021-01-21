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

## Поддерживаемые команды GCode

Gxx - Основные команды
| Код | Описание | Пример |
| --- | -------- | ------ |
| `G00` | Ускоренное перемещение инструмента (холостой ход) | `G00 X100 Y200 Z-5 F3000` |
| `G01` | Линейная интерполяция, скорость перемещения задаётся здесь же или ранее модальной командой F | `G01 X250 Y0 F600` |
| `G02` | Круговая интерполяция по часовой стрелке | `G02 X20 R50` |
| `G03` | Круговая интерполяция против часовой стрелки | `G02 I-20 J5` |
| `G04` | Задержка выполнения программы. P задает паузу в миллисекундах, X — в секундах. | `G04 X5.` |
| `G90` | Задание абсолютных координат опорных точек траектории | `G90 G1 X0.5. Y0.5. F10.` |
| `G91` | Задание координат инкрементально последней введённой опорной точки | `G91 G1 X4. Y5. F100.` |

Mxx - Технологические команды
| Код | Описание | Пример |
| --- | -------- | ------ |
| `M03` | Включить плазму | `M03` |
| `M07` | Включить плазму | `M07` |
| `M05` | Выключить плазму | `M05` |
| `M08` | Выключить плазму | `M08` |

