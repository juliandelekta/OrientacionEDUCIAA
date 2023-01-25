/*
 * imu.c
 *
 *  Created on: 12/12/2022
 *      Author: Barcala
 */
#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "imu.h"
#include "lcd.h"
#include "wifi.h"
#include <math.h>

float deltat = 0.02f; 										// periodo de muestreo en segundos (20ms)

#define PI 3.14159265358979

double q[4] = {1.0f, 0.0f, 0.0f, 0.0f};            // Vector que representa el quaternion
float GyroMeasError, beta, GyroMeasDrift, zeta;

// MPU9250 Address
MPU60X0_address_t addr = MPU60X0_ADDRESS_0; // If MPU60X0 AD0 pin is connected to GND

void quaternionFilterUpdate(float ax, float ay, float az, float gx, float gy, float gz) {
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
	float norm;                                               // vector norm
	float f1, f2, f3;                                         // objetive funcyion elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float qDot1, qDot2, qDot3, qDot4;
	float hatDot1, hatDot2, hatDot3, hatDot4;
	float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;        // gyro bias error

	// Auxiliary variables to avoid repeated arithmetic
	float _halfq1 = 0.5f * q1;
	float _halfq2 = 0.5f * q2;
	float _halfq3 = 0.5f * q3;
	float _halfq4 = 0.5f * q4;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Compute the objective function and Jacobian
	f1 = _2q2 * q4 - _2q1 * q3 - ax;
	f2 = _2q1 * q2 + _2q3 * q4 - ay;
	f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
	J_11or24 = _2q3;
	J_12or23 = _2q4;
	J_13or22 = _2q1;
	J_14or21 = _2q2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;

	// Compute the gradient (matrix multiplication)
	hatDot1 = J_14or21 * f2 - J_11or24 * f1;
	hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
	hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
	hatDot4 = J_14or21 * f1 + J_11or24 * f2;

	// Normalize the gradient
	norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
	hatDot1 /= norm;
	hatDot2 /= norm;
	hatDot3 /= norm;
	hatDot4 /= norm;

	// Compute estimated gyroscope biases
	gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
	gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
	gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

	// Compute and remove gyroscope biases
	gbiasx += gerrx * deltat * zeta;
	gbiasy += gerry * deltat * zeta;
	gbiasz += gerrz * deltat * zeta;
	gx -= gbiasx;
	gy -= gbiasy;
	gz -= gbiasz;

	// Compute the quaternion derivative
	qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
	qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
	qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
	qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

	// Compute then integrate estimated quaternion derivative
	q1 += (qDot1 -(beta * hatDot1)) * deltat;
	q2 += (qDot2 -(beta * hatDot2)) * deltat;
	q3 += (qDot3 -(beta * hatDot3)) * deltat;
	q4 += (qDot4 -(beta * hatDot4)) * deltat;

	// Normalize the quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

void quaternionFilterUpdateOld(float ax, float ay, float az, float gx, float gy, float gz) {

	// Variables locales
	double norm;																// Norma del vector
	double q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         				// Renombrado para la legibilidad
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; 	// Derivada de quaternino a partir de los elementos del giroscopio
	float f_1, f_2, f_3;													// Elementos de la función objetivo
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; 				// Elementos del Jacobiano de la función objetiva
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; 				// Dirección estimada del error del giroscopio

	// Variables auxiliares para evitar cálculos repetitivos
	float halfSEq_1 = 0.5f * q1;
	float halfSEq_2 = 0.5f * q2;
	float halfSEq_3 = 0.5f * q3;
	float halfSEq_4 = 0.5f * q4;
	float twoSEq_1 = 2.0f * q1;
	float twoSEq_2 = 2.0f * q2;
	float twoSEq_3 = 2.0f * q3;

	// Normalizar la medida del acelerómetro
	norm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Computar la función objetivo y el Jacobiano
	f_1 = twoSEq_2 * q4 - twoSEq_1 * q3 - ax;
	f_2 = twoSEq_1 * q2 + twoSEq_3 * q4 - ay;
	f_3 = 1.0f - twoSEq_2 * q2 - twoSEq_3 * q3 - az;
	J_11or24 = twoSEq_3;								// J_11 negado en la multiplicación matricial
	J_12or23 = 2.0f * q4;
	J_13or22 = twoSEq_1; 								// J_12 negado en la multiplicación matricial
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; 							// negado en la multiplicación matricial
	J_33 = 2.0f * J_11or24; 							// negado en la multiplicación matricial

	// Computar el gradiente (multiplicación matricial)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

	// Normalizar el gradiente
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;

	// Computar la derivada del quaternion medida por el giroscopio
	SEqDot_omega_1 = -halfSEq_2 * gx - halfSEq_3 * gy - halfSEq_4 * gz;
	SEqDot_omega_2 = halfSEq_1 * gx + halfSEq_3 * gz - halfSEq_4 * gy;
	SEqDot_omega_3 = halfSEq_1 * gy - halfSEq_2 * gz + halfSEq_4 * gx;
	SEqDot_omega_4 = halfSEq_1 * gz + halfSEq_2 * gy - halfSEq_3 * gx;

	// Integrar la derivada del quaternion estimado
	q1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	q2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	q3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	q4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

	// Normailizar el quaternion
	norm = 1.0f / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

// Funcion que inicializa la tarea
void taskIMU_Init( void ) {
	GyroMeasError = PI * (5.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
	GyroMeasDrift = PI * (0.2f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

	//printf("Inicializando IMU MPU6050...\r\n" );
	int8_t status;
	status = mpu60X0Init( addr );

	if( status < 0 ) {
		printf( "IMU MPU6050 no inicializado, chequee las conexiones:\r\n\r\n" );
		printf( "MPU6050 ---- EDU-CIAA-NXP\r\n\r\n" );
		printf( "    VCC ---- 3.3V\r\n" );
		printf( "    GND ---- GND\r\n" );
		printf( "    SCL ---- SCL\r\n" );
		printf( "    SDA ---- SDA\r\n" );
		printf( "    AD0 ---- GND\r\n\r\n" );
		printf( "Se detiene el programa.\r\n" );
		while(1);
	}
	//printf("IMU MPU6050 inicializado correctamente.\r\n\r\n" );
}

// Funcion que se ejecuta periodicamente
void taskIMU( void* taskParam ) {
    taskIMU_Init();

	// Variables medidas del acelerómetro
	float ax, ay, az;
	// Variables medidas del giroscopio
	float gx, gy, gz;
	// Ángulos
	float yaw, pitch, roll;

    tick_t startTime = tickRead();
    const TickType_t xFrequency = 20 / portTICK_RATE_MS; // Periodo de 20ms

    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

	for( ; ; ) {

		//Leer el sensor y guardar en estructura de control
		mpu60X0Read();

		// Cargar las lecturas del giroscopio
		gx = mpu60X0GetGyroX_rads();
		gy = mpu60X0GetGyroY_rads();
		gz = mpu60X0GetGyroZ_rads();

		// Cargar las lecturas del acelerómetro
		ax = mpu60X0GetAccelX_mss();
		ay = mpu60X0GetAccelY_mss();
		az = mpu60X0GetAccelZ_mss();


		//deltat = (tickRead() - startTime) * 0.001;
		// Cargar en q el valor del quaternion (salida del filtro)
		quaternionFilterUpdate(ax, ay, az, gx, gy, gz);


		yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
		pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
		roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);


		// Conversión a ángulos
		pitch *= 180.0f / PI;
		yaw   *= 180.0f / PI;
		roll  *= 180.0f / PI;


        LCDsetYPR(yaw, pitch, roll);
        LCDsetQ(q[1], q[2], q[3], q[0]);
        LCDsetRAW(ax, ay, az, gx, gy, gz);
        WiFiLoadQuaternion(q[1], q[2], q[3], q[0]);

        if (ax > 3 || ax < -3) {
        	LCDBlink();
        }

        startTime = tickRead();


		// Imprimir resultados
		/*printf( "Giroscopo:      (%f, %f, %f)   [rad/s]\r\n", gx, gy, gz);

		printf( "Acelerometro:   (%f, %f, %f)   [m/s2]\r\n", ax, ay, az);

		printf( "Temperatura:    %f   [C]\r\n",
			  mpu60X0GetTemperature_C()
			);

		printf( "Quaternion: (%f, %f, %f, %f)\r\n", q[0], q[1], q[2], q[3]);

		printf( "Angulos: (%f º, %f º, %f º)\r\n\r\n", pitch, yaw, roll);*/

		//vTaskDelay( 20 / portTICK_RATE_MS );
		// Wait for the next cycle.
		vTaskDelayUntil( &xLastWakeTime, xFrequency );
	}

	printf("ERRORRRRRRRRR");
}

