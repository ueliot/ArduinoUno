
/*

Uso del PWM cnfigurando el Timer2 y los registros asociados a este 
del ATMega328p (16Mhz)
usando arduino y registros de dichos perifericos
para generar un pulso de 500ns con periodo de 500ms.
cambiando el valor de T sepuede variar el ancho del pulso
usa el Pin3 de una placa Arduino uno. (PD3. --- pin5 del Microcontrolador).

*/

/*

Alguna Notas aqui:
https://www.laboratoriogluon.com/generar-un-pulso-de-duracion-concreta-con-arduino-atmega/
Cañon de gaus  https://www.youtube.com/watch?v=J0C9B5qFxDw
https://forum.arduino.cc/t/ch-9-timer-counter-module-of-atmega328p-mcu/662590
https://www.youtube.com/watch?v=89CmqWEsbkY        TIMER
https://www.youtube.com/watch?v=doK9qD1zDFs.       PWM
https://github.com/amirbawab/AVR-cheat-sheet
https://arduino.stackexchange.com/questions/38352/atmega-avr-frequency-measurement-with-timer-counter
https://www.arduino.cc/en/uploads/Main/arduino-uno-schematic.pdf
Tutorial Atmel https://www.youtube.com/watch?v=UoL0MX_Hxp0&list=PLfGfSiSrt7lS5WLVPJtW1vnfdnk3URYNv
Digitales
https://www.youtube.com/watch?v=vYDlazQJYSg&list=PLfGfSiSrt7lQM-ArgwZbRUvLGKNuTibb7

*/


//#include <Arduino.h>
void setup(){
    // Desactivamos el reloj del contador.
    TCCR2B = 0;
    // Hacemos que el contador se quede atascado.
    OCR2A = 0;
    // Ponemos el OCR2B para que haga 8 ciclos
    // Restamos uno porque entre el 0xFF y el 0x00 pasa otro ciclo


     //Configuramos en T el tamaño de pulso a generar
    //1 = 62,5 ns. (freq de 16Mhz de arduino)
    //2 = 125 ns
    //4 = 250 ns
    //8 = 500 ns
    //16 = 1000ns
    //32 = 2000ns
    //64 = 4 us
    //128 = 8 us.  is max. (256 = No Trabaja x q el OCR2B = 0)
    int T=8;

    OCR2B = 0xFF-(T-1);
    //
    //OCR2B = 0x30;
    // Configuramos el Compare Output y el Modo de PWM
    // Acorde a las tablas del datasheet
    TCCR2A = _BV(COM2B0) | _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
    TCCR2B = _BV(WGM22) | _BV(CS20);
    // Activamos el Pin D3 de Arduino
    pinMode(3, OUTPUT);
}
void loop(){
    // Ponemos el contador justo antes del OCR2B
    TCNT2 = OCR2B - 1;
    delay(500);   //cada tiempo de delay es la frecuencia del pulsito T 
}