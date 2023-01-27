#ifndef WIFI_H_
#define WIFI_H_

#define UART_DEBUG                 UART_USB
#define UART_ESP01                 UART_232
#define UARTS_BAUD_RATE            9600

#define ESP01_RX_BUFF_SIZE         1024

#define WIFI_SSID                  "wifi"     // Setear Red Wi-Fi
#define WIFI_PASSWORD              "password" // Setear password

#define SERVER_IP      "10.0.22.142"
#define SERVER_PORT     8082


void taskWiFi( void* taskParam );

void WiFiLoadQuaternion( double qx, double qy, double qz, double qw);


#endif
