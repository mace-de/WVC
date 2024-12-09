#include "myint.h"

void zcd_int() // Interrupt ausgelöst durch positiven Nulldurchgang der Netzspannung
{
    if (Synccounter < 256)
        TIMER_CAR(TIMER13)
    --; // Timer 13 so nachführen dass er netzsyncron läuft
    if (Synccounter > 256)
        TIMER_CAR(TIMER13)
    ++; // müsste bei 50Hz bei 5625 sein
    if (Synccounter == 256)
    {
        Sync = 1;
    }
    if ((TIMER_CAR(TIMER13) < 5400) || (TIMER_CAR(TIMER13) > 5737)) // Netzfrequenz überwachen Grenze 52,08Hz und 49,02Hz
    {
        Sync = 0;
        gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
        gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
    }
    if ((TIMER_CAR(TIMER13) >= 5400) && (TIMER_CAR(TIMER13) <= 5620))
    {
        abregelwert = 5625 - TIMER_CAR(TIMER13); // Zwischen 50 und 52Hz Leistung abregeln
    }
    Synccounter = 0;
}

void t13ov_int() // Interrupt alle 78us (78us * 256 = 20ms = 50Hz)
{
    TIMER_CH0CV(TIMER15) = (sinus2[(uint8_t)Synccounter & 0b01111111] * U_out) >> 8;
    if (cnt_Haupttakt < ((1282 * TaktHauptschleife) / 100)) // alle 1282 x 78us = 100ms (72MHz/5625=78us)
    {
        cnt_Haupttakt++;
    }
    else
    {
        cnt_Haupttakt = 0; // das Flag für die Hauptschleife setzen
        Flag_Haupttakt = true;
    }
    Synccounter++;
    if (Synccounter > 258) // wenn Synccounter überläuft = kein Zerocross = kein Netz
    {
        Synccounter--;
        Sync = 0;
        gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
        gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
    }
}
