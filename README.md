# Orientación relativa con Giroscopio y Acelerómetro
> Taller de Proyecto I - 2022
> Facultad de Ingeniería - UNLP

## Descripción
Este repositorio contiene todo el código necesario para ejecutar un Detector de Orientación sobre la EDU-CIAA.

En [processing](./processing/) se encuentra el código a ejecutar sobre el software Processing, junto con el modelo 3D del vehículo. El programa inicia un Servidor HTTP en el puerto 8082 y espera recibir de los clientes un quaternion.

En [firmware](./firmware/) se encuentra el código a ejecutar en la EDU-CIAA. Emplea las librerías firmware-v3 y FreeRTOS. Contiene los siguientes módulos de software:
- [orientacion](./firmware/src/orientacion.c) es el punto de entrada del programa. Inicia las tareas a ejecutar e inicializa los demás módulos.
- [imu](./firmware/src/imu.c) es el módulo encargado de obtener los datos del MPU6050 y pasarlos por el filtro a quaternion.
- [wifi](./firmware/src/wifi.c) encargado de establecer una conexión con la red WiFi mediante comandos AT. Posteriormente establece una conexión TCP con un Servidor ubicado en la misma red.
- [lcd](./firmware/src/lcd.c) es el módulo que controla la interfaz de usuario: LCD, pulsador y LEDs. Tiene una máquina de estados simple en la que el LCD muestra una pantalla distinta en cada uno: **Angles**: muestra los ángulos de inclinación en cada eje (Yaw, Pitch y Roll); **Quaternion**: muestra los cuatro valores del quaternion; **Acc**: muestra los datos del acelerómetro en cada eje; **Gyro**: muestra los datos del giroscopio en cada eje.

## Componentes
- EDU-CIAA
- MPU6050
- ESP8266 ESP01
- LCD 16x02

## Requerimientos Software
[Processing](https://processing.org/)
[Proyecto CIAA - Firmware v3](https://github.com/ciaa/firmware_v3)

## Cómo ejecutar
Configure el archivo `wifi.h` del Firmware, configurando el SSID y password de la red wifi. Luego especificar la IP estática del ESP8266.

Transferir los archivos de firmware al workspace de Eclipse del proyecto CIAA. Especificar el PATH de la carpeta en `program.mk`.

Iniciar el programa en Processing. Debuguear el firmware en Eclipse.