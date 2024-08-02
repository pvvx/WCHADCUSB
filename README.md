# WCHADCUSB
Test Web-oscilloscope on built-in 1Mbps 12-bit ADC in CH32V30x via USB2.0 HS bus (CDC)

Используется чип CH32V305 или CH32V307.

Значения от ADC передаются в USB-COM непрерывным потоком c с блочной синхронизацией.

Для просмотра используется Chrome Explorer с [wso_adcs.html](https://pvvx.github.io/tests/WCHADCUSB/wso_adcs.html).


## Отображение сигнала качающейся по частоте (1..10 кГц) "пилы" от генератора

![img](https://raw.githubusercontent.com/pvvx/WCHADCUSB/master/img/test1.gif)

## Отображение уровня шума при сигнале 20 мВ p-p (увеличение в 100 раз)

![img](https://raw.githubusercontent.com/pvvx/WCHADCUSB/master/img/test2.gif)

## Зависисмость пропускной спопобности от размера блока для CH32V30x в режиме echo (USB2.0 HS CDC)

![img](https://raw.githubusercontent.com/pvvx/WCHADCUSB/master/img/usb20hs_echo.gif)

Значения в Мегабайтах в секунду.

## Сборка проекта

Для сборки проекта используйте импорт в [MounRiver Studio](http://mounriver.com).
