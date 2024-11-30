#include <Arduino.h>
#include "myadc.h"
#define TIMETOWAIT 1 // Minuten Wartezeit zwischen den Relaistests. Da der WR am früh evtl noch nicht genug Energie hat das Relais korrekt zu schalten
void relaycheck()
{
  boolean last_zcd = 0, last_opto = 0, zcd_temp, opto_temp;
  uint32_t zcd_millis, opto_millis, schritt = 0, repeat = 0;
  int32_t zcd_cnt = 0, opto_cnt = 0, waitcounter = 0;

  while (1)
  {
    zcd_temp = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
    opto_temp = gpio_input_bit_get(GPIOC, GPIO_PIN_14);
    if (adc_channel_sample(ADC_CHANNEL_7) < 1300) // wenn die PV-Spannung hier schon einbricht, dann ist zu wenig Licht
    {                                              // zum starten. Dann warten und später nochmal versuchen (970 (61) = 1V)
      schritt = 4;
    }
    if (zcd_temp != last_zcd) // ZCD Pin Flanken zählen
    {
      last_zcd = zcd_temp;
      zcd_cnt++;
    }
    if (opto_temp != last_opto) // Optokoppler Pin Flanken zählen
    {
      last_opto = opto_temp;
      opto_cnt++;
    }
    switch (schritt)
    {
    case 0:
    {
      gpio_bit_set(GPIOB, GPIO_PIN_11); // Relais 115V/230V Ein
      if (zcd_cnt > 100) // warten bis Netz vorhanden
      {
        schritt = 1;
        zcd_cnt = 0;
        opto_cnt = 0;
        zcd_millis = millis() + 2000;
        opto_millis = millis() + 1000;
      }
      break;
    }
    case 1: // 1 Sekunde nur ZCD Flanken zählen
    {
      if (millis() > opto_millis)
      {
        schritt = 2;
        gpio_bit_set(GPIOC, GPIO_PIN_13); // Netzrelais ein
      }
      break;
    }
    case 2: // 1 Sekunde ZCD und Optokoppler Flanken zählen
    {
      if (millis() > zcd_millis)
      {
        schritt = 3;
        gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
        gpio_bit_reset(GPIOB, GPIO_PIN_11); // Relais 115V/230V Aus
      }
      break;
    }
    case 3: // Ergebnis prüfen
    {
      if (abs(zcd_cnt - (2 * opto_cnt)) < 8) // es müssen etwa doppelt so viele Flanken an ZCD aufgetreten sein wie am Optokoppler
        return;                                // wenn ja, OK
      if (repeat < 1)                          // wenn nein, zweiter Versuch
      {
        delay(500);
        schritt = 0;
        repeat++;
        break;
      }
      repeat = 0;
      schritt = 4; // wenn zweiter Versuch auch fehlschlägt -> Warteschritt
      break;
    }
    case 4: // Warteschritt
    {
      gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
      gpio_bit_reset(GPIOB, GPIO_PIN_11); // Relais 115V/230V Aus
      waitcounter = 0;
      while (waitcounter < TIMETOWAIT * 30)
      {
        gpio_bit_reset(GPIOB, GPIO_PIN_13); // rot aus
        delay(1000);
        gpio_bit_set(GPIOB, GPIO_PIN_13); // rot ein
        delay(1000);
        waitcounter++;
      }
      repeat = 0;
      schritt = 0;
      break;
    }
    default:
    {
      while (1)
      {
      }
    }
    }
  }
}