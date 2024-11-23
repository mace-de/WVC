#include <Arduino.h>
void relaycheck()
{
  boolean last_zcd, last_opto, zcd_temp, opto_temp;
  uint32_t zcd_cnt2, opto_cnt2, zcd_millis, opto_millis, schritt = 0;
  int32_t zcd_cnt1 = 0, opto_cnt1 = 0;

  while (1)
  {
    zcd_temp = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
    opto_temp = gpio_input_bit_get(GPIOC, GPIO_PIN_14);
    if (zcd_temp == !last_zcd) // ZCD Pin Flanken zählen
    {
      last_zcd = zcd_temp;
      zcd_cnt1++;
    }
    if (opto_temp == !last_opto) // Optokoppler Pin Flanken zählen
    {
      last_opto = opto_temp;
      opto_cnt1++;
    }
    switch (schritt)
    {
    case 0:
    {
      if (zcd_cnt1 > 100) // warten bis Netz vorhanden
      {
        schritt = 1;
        zcd_cnt1 = 0;
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
      }
      break;
    }
    case 3: // Ergebnis prüfen
    {
      if (abs(zcd_cnt1 - (2 * opto_cnt1)) < 8) // es müssen etwa doppelt so viele Flanken an ZCD aufgetreten sein wie am Optokoppler
        return;                                // wenn ja, OK
      while (1)                                // wenn nein, WR sperren und rote LED langsam blinken da Relais möglicherweise fehlerhaft
      {
        gpio_bit_reset(GPIOB, GPIO_PIN_13); // rot aus
        delay(1000);
        gpio_bit_set(GPIOB, GPIO_PIN_13); // rot ein
        delay(1000);
      }
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