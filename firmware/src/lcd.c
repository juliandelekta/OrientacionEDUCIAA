/*
 * lcd.c
 *
 *  Created on: 12/12/2022
 *      Author: Barcala
 */
#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lcd.h"
#include <math.h>
//#include "convert.h"

#define LED_ROJO T_FIL2
#define LED_VERDE T_FIL3
#define PULSADOR T_COL0

typedef enum {
	ANGLES,
	QUATERNION,
	ACC,
	GYRO
} FSM_state_t;

int yaw, pitch, roll;
double qx, qy, qz, qw;
float ax, ay, az, gx, gy, gz;
uint8_t encender = 0, count = 10;

void LCDsetYPR(float y, float p, float r) {
    yaw   = round(y);
    pitch = round(p);
    roll  = round(r);
}


void LCDsetQ(double x, double y, double z, double w) {
	qx = x;
	qy = y;
	qz = z;
	qw = w;
}

void LCDsetRAW(float _ax, float _ay, float _az, float _gx, float _gy, float _gz) {
	ax = _ax;
	ay = _ay;
	az = _az;
	gx = _gx;
	gy = _gy;
	gz = _gz;
}

void LCDBlink() {
	encender = 1;
	count = 10;
}

// Funcion que inicializa la tarea
void taskLCD_Init( void ) {
	// Inicializar LCD de 16x2 (caracteres x lineas) con cada caracter de 5x2 pixeles
    lcdInit( 16, 2, 5, 8 );
    lcdCursorSet( LCD_CURSOR_OFF ); // Apaga el cursor
    lcdClear();                     // Borrar la pantalla
    lcdGoToXY( 0, 0 );
    lcdSendStringRaw( " Yaw Pitch Roll " );

    gpioConfig(LED_ROJO, GPIO_OUTPUT);
    gpioConfig(LED_VERDE, GPIO_OUTPUT);
    gpioConfig(PULSADOR, GPIO_INPUT);
}

// YAW PITCH ROLL
// 120° 120° 120°
static char result[100];
void taskLCD( void* taskParam ) {
    char line[16];
    uint8_t led = OFF;
    uint8_t valor = 0, last = 0;
    FSM_state_t estado = ANGLES;

    while (TRUE) {
    	switch(estado) {
    	case ANGLES:
			 	sprintf(line, " %3d° %3d° %3d° ", (yaw), (pitch), (roll));
				lcdGoToXY( 0, 0 );
				lcdSendStringRaw( " Yaw Pitch Roll " );
				lcdGoToXY( 0, 1 );
				lcdSendStringRaw(line);
    		break;
    	case QUATERNION:
    			line[0] = 0; // Reseteo la cadena que guarda las otras agregando un caracter NULL al principio

				strcat( line, " " );
				strcat( line, floatToString(qx,result, 5) );
				strcat( line, " " );
				strcat( line, floatToString(qy,result, 5) );
				strcat( line, " " );
    			lcdGoToXY( 0, 0 );
				lcdSendStringRaw(line);
				line[0] = 0;
				strcat( line, " " );
				strcat( line, floatToString(qz,result, 5) );
				strcat( line, " " );
				strcat( line, floatToString(qw,result, 5) );
				strcat( line, " " );
				lcdGoToXY( 0, 1 );
				lcdSendStringRaw(line);
    		break;
    	case ACC:
    			line[0] = 0; // Reseteo la cadena que guarda las otras agregando un caracter NULL al principio

				strcat( line, "Acc " );
				strcat( line, floatToString(ax,result, 5) );
				strcat( line, "        " );
				lcdGoToXY( 0, 0 );
				lcdSendStringRaw(line);
				line[0] = 0;
				strcat( line, " " );
				strcat( line, floatToString(ay,result, 5) );
				strcat( line, " " );
				strcat( line, floatToString(az,result, 5) );
				strcat( line, " " );
				lcdGoToXY( 0, 1 );
				lcdSendStringRaw(line);
    		break;
    	case GYRO:
    			line[0] = 0;
    			strcat( line, "Gyro " );
				strcat( line, floatToString(gx,result, 5) );
				lcdGoToXY( 0, 0 );
				lcdSendStringRaw(line);
				line[0] = 0;
				strcat( line, " " );
				strcat( line, floatToString(gy,result, 5) );
				strcat( line, " " );
				strcat( line, floatToString(gz,result, 5) );
				strcat( line, " " );
				lcdGoToXY( 0, 1 );
				lcdSendStringRaw(line);
			break;
    	}


        valor = gpioRead(PULSADOR);
        gpioWrite(LED_VERDE, valor);

        if (valor == 0 && last == 1) {
        	estado = estado == ANGLES ? QUATERNION : estado == QUATERNION ? ACC : estado == ACC ? GYRO : ANGLES;
        }

        if (encender) {
        	gpioWrite(LED_ROJO, 1);
        	if(!(count--)) {
        		encender = 0;
            	gpioWrite(LED_ROJO, 0);
        	}
        }

        last = valor;

        vTaskDelay(200 / portTICK_RATE_MS);
    }
}
