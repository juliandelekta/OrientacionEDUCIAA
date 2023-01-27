#include "sapi.h"
#include <string.h>
#include "wifi.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define pi 3.141592653589793
#define SEND_TIMEOUT 40

CONSOLE_PRINT_ENABLE
DEBUG_PRINT_ENABLE

// ESP01 Rx Buffer
char espResponseBuffer[ ESP01_RX_BUFF_SIZE ];
uint32_t espResponseBufferSize = ESP01_RX_BUFF_SIZE;

double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
float a = 0.0;
float speed = 0.1;

// UARTs
uartMap_t uartEsp01 = UART_232;
uartMap_t uartDebug = UART_USB;

char tcpIpDataToSend[100];


void WiFiLoadQuaternion( double _qx, double _qy, double _qz, double _qw) {
	qx = _qx;
	qy = _qy;
	qz = _qz;
	qw = _qw;
}


bool_t esp01Init( uartMap_t uartForEsp, uartMap_t uartForDebug, uint32_t baudRate );

void esp01CleanRxBuffer( void );

bool_t esp01ShowWiFiNetworks( void );

bool_t esp01ConnectToWifiAP( char* wiFiSSID, char* wiFiPassword );

bool_t esp01ConnectToServer( char* url, uint32_t port );

bool_t esp01SendTCPIPData( char* strData, uint32_t strDataLen );

bool_t esp01SendTPCIPDataToServer( char* url, uint32_t port, char* strData, uint32_t strDataLen );


void esp01CleanRxBuffer( void ){
   espResponseBufferSize = ESP01_RX_BUFF_SIZE;
   memset( espResponseBuffer, 0, espResponseBufferSize );
}

// AT+CIPSTART="TCP","api.thingspeak.com",80
bool_t esp01SendTPCIPDataToServer( char* url, uint32_t port, char* strData, uint32_t strDataLen ){

   // Enviar dato "data" al servidor "url", puerto "port".
   //debugPrintlnString( ">>>> ===========================================================" );
   //debugPrintString( ">>>> Enviar dato: \"" );
   //debugPrintString( strData );
   //debugPrintString( "\"\r\n>>>> al servidor \"" );
   //debugPrintString( url );
   //debugPrintString( "\", puerto \"" );
   //debugPrintInt( port );
   //debugPrintlnString( "\"..." );
   //debugPrintEnter();

   // AT+CIPSTART="TCP","url",port ---------------------------
   if( !esp01ConnectToServer( url, port ) )
      return FALSE;

   // Ejemplo:
   // AT+CIPSEND=47 ------------------------------------------
   // GET /update?api_key=7E7IOJ276BSDLOBA&field1=66 ---------
   if( !esp01SendTCPIPData( strData, strDataLen ) )
      return FALSE;

   return TRUE;
}



// AT+CIPSEND=39
// GET /update?key=7E7IOJ276BSDLOBA&1=69
bool_t esp01SendTCPIPData( char* strData, uint32_t strDataLen ){

   bool_t retVal = FALSE;

   // "GET /update?key=7E7IOJ276BS\"DL\"OBA&1=69"
   // AT+CIPSEND=strDataLen strData --------------------------

   // Limpiar Buffer (es necesario antes de usar
   // "receiveBytesUntilReceiveStringOrTimeoutBlocking")
   esp01CleanRxBuffer();

   // Envio datos TCP/IP al servidor.
   //debugPrintlnString( ">>>> Envio datos TCP/IP al servidor..." );

   //debugPrintString( ">>>> AT+CIPSEND=" );
   //debugPrintInt( strDataLen + 2 ); // El mas 2 es del \r\n
   //debugPrintString( "\r\n" );

   consolePrintString( "AT+CIPSEND=" );
   consolePrintInt( strDataLen + 2 ); // El mas 2 es del \r\n
   consolePrintString( "\r\n" );

   // No poner funciones entre el envio de comando y la espera de respuesta
   retVal = receiveBytesUntilReceiveStringOrTimeoutBlocking(
               uartEsp01,
               "\r\n\r\nOK\r\n>", strlen("\r\n\r\nOK\r\n>"),
               espResponseBuffer, &espResponseBufferSize,
			   SEND_TIMEOUT
            );
   if( retVal ){

      // Imprimo todo lo recibido
      //debugPrintString( espResponseBuffer );

      // strData\r\n --------------------------------------------

      // Limpiar Buffer (es necesario antes de usar
      // "receiveBytesUntilReceiveStringOrTimeoutBlocking")
      esp01CleanRxBuffer();

      // Envio los datos TCP/IP ------------------
      consolePrintString( strData );
      consolePrintString( "\r\n" );

      // No poner funciones entre el envio de comando y la espera de respuesta
      retVal = receiveBytesUntilReceiveStringOrTimeoutBlocking(
                  uartEsp01,
                  "SEND OK\r\n", 9,
                  espResponseBuffer, &espResponseBufferSize,
				  SEND_TIMEOUT
               );
      if( retVal ){

         // Imprimo todo lo recibido
         //debugPrintString( espResponseBuffer );

         // Limpiar Buffer (es necesario antes de usar
         // "receiveBytesUntilReceiveStringOrTimeoutBlocking")
         esp01CleanRxBuffer();

         // Envio los datos TCP/IP ------------------
         consolePrintlnString( strData );

         // No poner funciones entre el envio de comando y la espera de respuesta
         retVal = receiveBytesUntilReceiveStringOrTimeoutBlocking(
                     uartEsp01,
                     "CLOSED\r\n", 8,
                     espResponseBuffer, &espResponseBufferSize,
					 SEND_TIMEOUT
                  );

         if( retVal ){

            // DATO RECIBIDOOOOOOOOOOO -----------------

            // Imprimo todo lo recibido
            debugPrintString( espResponseBuffer );



         } else{
            debugPrintlnString( ">>>> Error al enviar los datos TCP/IP, en el envio del string" );
            debugPrintlnString( ">>>> \"strData\", cuando el ESP01 pone el prompt > " );
            debugPrintlnString( ">>>> y no se recibe la respuesta y \"CLOSED\"!!\r\n" );

            // Imprimo todo lo recibido
            debugPrintString( espResponseBuffer );
         }


      } else{
         debugPrintlnString( ">>>> Error al enviar los datos TCP/IP, en el envio del string" );
         debugPrintlnString( ">>>> \"strData\", cuando el ESP01 pone el prompt > " );
         debugPrintlnString( ">>>> y no se recibe \"SEND OK\"!!\r\n" );

         // Imprimo todo lo recibido
         debugPrintString( espResponseBuffer );
      }

   } else{
      debugPrintlnString( ">>>> Error al enviar los datos TCP/IP, en comando" );
      debugPrintlnString( ">>>> \"AT+CIPSEND\"!!\r\n" );
      // Imprimo todo lo recibido
      debugPrintString( espResponseBuffer );
   }
   return retVal;
}


// AT+CIPSTART="TCP","api.thingspeak.com",80
bool_t retVal = FALSE;
bool_t esp01ConnectToServer( char* url, uint32_t port ){

	if (retVal) return retVal;
   // AT+CIPSTART="TCP","url",port ---------------------------

   // Limpiar Buffer (es necesario antes de usar
   // "receiveBytesUntilReceiveStringOrTimeoutBlocking")
   esp01CleanRxBuffer();

   consolePrintString("AT+CIPCLOSE\r\n");

   debugPrintString( ">>>> Conectando al servidor \"" );
   debugPrintString( url );
   debugPrintString( "\", puerto \"" );
   debugPrintInt( port );
   debugPrintlnString( "\"..." );

   debugPrintString( ">>>> AT+CIPSTART=\"TCP\",\"" );
   debugPrintString( url );
   debugPrintString( "\"," );
   debugPrintInt( port );
   debugPrintString( "\r\n" );

   consolePrintString( "AT+CIPSTART=\"TCP\",\"" );
   consolePrintString( url );
   consolePrintString( "\"," );
   consolePrintInt( port );
   consolePrintString( "\r\n" );

   // No poner funciones entre el envio de comando y la espera de respuesta
   retVal = receiveBytesUntilReceiveStringOrTimeoutBlocking(
               uartEsp01,
               "CONNECT\r\n\r\nOK\r\n", 15,
               espResponseBuffer, &espResponseBufferSize,
               10000
            );
   if( !retVal ){
      debugPrintString( ">>>>    Error: No se puede conectar al servidor: \"" );
      debugPrintlnString( url );
      debugPrintString( "\"," );
      debugPrintInt( port );
      debugPrintlnString( "\"!!\r\n" );
   }
   // Imprimo todo lo recibido
   debugPrintString( espResponseBuffer );
   return retVal;
}


bool_t esp01ConnectToWifiAP( char* wiFiSSID, char* wiFiPassword ){

   bool_t retVal = FALSE;
   char* index = 0;

   // AT+CWJAP="wiFiSSID","wiFiPassword" ---------------------

   // Limpiar Buffer (es necesario antes de usar
   // "receiveBytesUntilReceiveStringOrTimeoutBlocking")
   consolePrintString("AT+RST\r\n");
   esp01CleanRxBuffer();
   // Conectar a la red Wi-Fi. se envia AT+CWJAP="wiFiSSID","wiFiPassword"
   debugPrintString( ">>>> Conectando a la red Wi-Fi: \"" );
   debugPrintString( wiFiSSID );
   debugPrintlnString( "\"..." );

   consolePrintString( "AT+CWJAP=\"" );
   consolePrintString( wiFiSSID );
   consolePrintString( "\",\"" );
   consolePrintString( wiFiPassword );
   consolePrintString( "\"\r\n" );

   // No poner funciones entre el envio de comando y la espera de respuesta
   retVal = receiveBytesUntilReceiveStringOrTimeoutBlocking(
               uartEsp01,
               //"WIFI CONNECTED\r\nWIFI GOT IP\r\n\r\nOK\r\n", 35,
               "WIFI CONNECTED\r\nWIFI GOT IP", 27,
               espResponseBuffer, &espResponseBufferSize,
               20000
            );
   if( retVal ){

      // Imprimo todo lo recibido filtrando la parte que muestra el password, llega:

      // AT+CWJAP="wiFiSSID","wiFiPassword"
      //
      // WIFI DISCONNECT ----> Opcional
      // WIFI CONNECTED
      // WIFI GOT IP
      //
      // OK

      // Pero imprimo:

      // WIFI CONNECTED
      // WIFI GOT IP
      //
      // OK

      index = strstr( (const char*)espResponseBuffer, (const char*)"WIFI CONNECTED" );
      if( index != 0 ){
         // Muestro desde " WIFI CONNECTED" en adelante
         debugPrintString( index );

      } else{
         // Muestro todo en caso de error
         debugPrintString( espResponseBuffer );
      }
   } else{
      debugPrintString( ">>>>    Error: No se puede conectar a la red: \"" );
      debugPrintString( wiFiSSID );
      debugPrintlnString( "\"!!\r\n" );

      // Muestro todo en caso de error
      debugPrintString( espResponseBuffer );
   }
   return retVal;
}


bool_t esp01ShowWiFiNetworks( void ){

   bool_t retVal = FALSE;

   // AT+CWLAP -----------------------------------------------

   // Limpiar Buffer (es necesario antes de usar
   // "receiveBytesUntilReceiveStringOrTimeoutBlocking")
   esp01CleanRxBuffer();

   // Mostrar lista de AP enviando "AT+CWLAP"
   debugPrintlnString( ">>>> Consultando las redes Wi-Fi disponibles.\r\n>>>>    Enviando \"AT+CWLAP\"..." );
   consolePrintString( "AT+CWLAP\r\n" );
   // No poner funciones entre el envio de comando y la espera de respuesta
   retVal = receiveBytesUntilReceiveStringOrTimeoutBlocking(
               uartEsp01,
               ")\r\n\r\nOK\r\n", 9,
               espResponseBuffer, &espResponseBufferSize,
               20000
            );
   if( !retVal ){
      debugPrintlnString( ">>>>    Error: No se encuentran redes disponibles!!\r\n" );
   }
   // Imprimo todo lo recibido
   debugPrintString( espResponseBuffer );
   return retVal;
}


bool_t esp01Init( uartMap_t uartForEsp, uartMap_t uartForDebug, uint32_t baudRate ){

    bool_t retVal = FALSE;

    uartEsp01 = uartForEsp;
    uartDebug = uartForDebug;

    // Initialize HW ------------------------------------------

    // Inicializar UART_USB como salida de debug
    debugPrintConfigUart( uartDebug, baudRate );
    debugPrintlnString( ">>>> UART_USB configurada como salida de debug." );

    // Inicializr otra UART donde se conecta el ESP01 como salida de consola
    consolePrintConfigUart( uartEsp01, baudRate );
    debugPrintlnString( ">>>> UART_ESP (donde se conecta el ESP01), \r\n>>>> configurada como salida de consola.\r\n" );

    // AT -----------------------------------------------------

    // Chequear si se encuentra el modulo Wi-Fi enviandole "AT"
    debugPrintlnString( ">>>> Chequear si se encuentra el modulo Wi-Fi.\r\n>>>>    Enviando \"AT\"..." );
    consolePrintString( "AT\r\n" );
    // No poner funciones entre el envio de comando y la espera de respuesta
    retVal = waitForReceiveStringOrTimeoutBlocking( uartEsp01, "AT\r\n", 4, 500 );
    if( retVal ){
        debugPrintlnString( ">>>>    Modulo ESP01 Wi-Fi detectado.\r\n" );
    } else{
        debugPrintlnString( ">>>>    Error: Modulo ESP01 Wi-Fi No detectado!!\r\n" );
        return retVal;
    }

    // AT+CWLAP -----------------------------------------------
    return esp01ShowWiFiNetworks();
}

static char result[100];
void taskWiFi ( void* taskParam ) {
    // Inicializar UART_USB como salida de consola
    debugPrintConfigUart( UART_DEBUG, UARTS_BAUD_RATE );

    if ( !esp01Init( UART_ESP01, UART_DEBUG, UARTS_BAUD_RATE ) ) {
        while(1); // Como dio falso (error) me quedo en un bucle infinito
    }

    if ( !esp01ConnectToWifiAP( WIFI_SSID, WIFI_PASSWORD ) ) {
        while(1); // Como dio falso (error) me quedo en un bucle infinito
    }

    // Configuro la IP estática
    consolePrintString("AT+CIPSTA=\"10.0.22.250\"\r\n");
    vTaskDelay( 4000 / portTICK_RATE_MS );

    esp01CleanRxBuffer();

    //consolePrintString( "AT+CIPPING=\"www.google.com\"\r\n"); <-- Comando de PING

    // Get Local Ip address
    consolePrintString( "AT+CIFSR\r\n");
    receiveBytesUntilReceiveStringOrTimeoutBlocking(
        uartEsp01,
        "", 0,
        espResponseBuffer, &espResponseBufferSize,
        3000
    );
    debugPrintString( espResponseBuffer );

    while(1) {
        // Armo el dato a enviar en una trama TCP

        tcpIpDataToSend[0] = 0; // Reseteo la cadena que guarda las otras agregando un caracter NULL al principio

        strcat( tcpIpDataToSend, floatToString(qx,result, 4) );
        strcat( tcpIpDataToSend, " " );
        strcat( tcpIpDataToSend, floatToString(qy,result, 4) );
        strcat( tcpIpDataToSend, " " );
        strcat( tcpIpDataToSend, floatToString(qz,result, 4) );
        strcat( tcpIpDataToSend, " " );
        strcat( tcpIpDataToSend, floatToString(qw,result, 4) );
        strcat( tcpIpDataToSend, "\r\n" );



        // Envio los datos TCP/IP al Servidor
        esp01SendTPCIPDataToServer( SERVER_IP, SERVER_PORT, tcpIpDataToSend, strlen( tcpIpDataToSend ) );
        //debugPrintlnString(tcpIpDataToSend);

        vTaskDelay( 300 / portTICK_RATE_MS ); // Periodo de 300ms
    }
}
