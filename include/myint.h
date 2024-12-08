#pragma once
#include <Arduino.h>
#include "lut.h"
#include "config.h"

extern volatile uint32_t U_out, Synccounter, abregelwert, cnt_Haupttakt;
extern volatile bool Flag_Haupttakt, Sync;

void zcd_int();   // Interrupt ausgelöst durch positiven Nulldurchgang der Netzspannung
void t13ov_int(); // Interrupt alle 78us (78us * 256 = 20ms = 50Hz)