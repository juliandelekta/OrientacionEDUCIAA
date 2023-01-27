#ifndef LCD_H_
#define LCD_H_


// Funcion que inicializa la tarea
void taskLCD_Init( void );

void taskLCD( void* taskParam );

void LCDsetYPR( float y, float p, float r);
void LCDsetQ(double x, double y, double z, double w);
void LCDsetRAW(float _ax, float _ay, float _az, float _gx, float _gy, float _gz);

void LCDBlink();

#endif
