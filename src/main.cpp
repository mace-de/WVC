#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include "GD32FreeRTOSConfig.h"
#include <HardwareTimer.h>

// put function declarations here:

#define Mittel_aus 16 // gleitendens Mittel aus X Werten (Maximal 63 sonst Überlauf)

// LUT mit modifiziertem Sinus mit größeren Pausen in den Nulldurchgängen und etwas phasenverschoben
const uint8_t sinus2[] = {0, 0, 0, 0, 5, 12, 19, 26, 33, 40, 47, 55, 62, 69, 75, 82, 89, 95, 102, 108, 114, 121, 127, 133,
                          139, 145, 151, 155, 161, 166, 172, 177, 181, 186, 191, 196, 199, 204, 208, 212, 216, 219, 223,
                          227, 229, 232, 235, 237, 240, 242, 244, 246, 248, 249, 250, 251, 253, 254, 254, 255, 255, 255,
                          255, 255, 254, 254, 253, 251, 250, 249, 248, 246, 244, 242, 240, 237, 235, 232, 229, 227, 223,
                          219, 216, 212, 208, 204, 199, 196, 191, 186, 181, 177, 172, 166, 161, 155, 151, 145, 139, 133,
                          127, 121, 114, 108, 102, 95, 89, 82, 75, 69, 62, 55, 47, 40, 33, 26, 19, 12, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          0, 0, 0};
// LUT mit standard Sinus
const uint8_t sinus1[] = {0, 6, 13, 19, 25, 31, 37, 44, 50, 56, 62, 68, 74, 80, 86, 92, 98, 103, 109, 115, 120, 126, 131, 136,
                          142, 147, 152, 157, 162, 167, 171, 176, 180, 185, 189, 193, 197, 201, 205, 208, 212, 215, 219,
                          222, 225, 228, 231, 233, 236, 238, 240, 242, 244, 246, 247, 249, 250, 251, 252, 253, 254, 254,
                          255, 255, 255, 255, 255, 254, 254, 253, 252, 251, 250, 249, 247, 246, 244, 242, 240, 238, 236,
                          233, 231, 228, 225, 222, 219, 215, 212, 208, 205, 201, 197, 193, 189, 185, 180, 176, 171, 167,
                          162, 157, 152, 147, 142, 136, 131, 126, 120, 115, 109, 103, 98, 92, 86, 80, 74, 68, 62, 56, 50,
                          44, 37, 31, 25, 19, 13, 6,
                          0, 0, 0};

const uint32_t minimalspannung_abs = 1576 * Mittel_aus; //= etwa 26V darunter kann der Wandler wegen zu geringer Ausgangsspannung an den Scheitelpunkten nicht ins Netz einspeisen
const uint32_t maximalstrom_abs = 2200 * Mittel_aus;    //= etwa 15A darüber wird der WR wohl verglühen

volatile uint32_t Synccounter = 0, abregelwert = 0, cnt100ms = 0, gu, gi;
volatile uint8_t U_out = 0, counter = 0;
volatile bool flag100ms = 0, Sync = 0;

TaskHandle_t hdl1, hdl2;
HardwareSerial Serial(PB7, PB6, 0);
xQueueHandle queue1;
struct s1
{
  uint32_t adcmax;
  uint32_t adcmin;
};
void adc_config(void);
uint16_t adc_channel_sample(uint8_t channel);

void tsk_main(void *param)
{
  while (1)
  {
    volatile static uint8_t Schritt = 1, cnt_a = 0, startverz = 0;
    volatile static bool enable, teilbereichsuche = false;
    volatile static uint32_t spannung_MPP = 0, Langzeitzaehler = 0, minimalspannung = 0, maximalspannung = 0, maximalstrom = 0;
    volatile static uint32_t leistung_MPP = 0;
    volatile static uint32_t spannung = 0, strom = 0, spannung_a[Mittel_aus], strom_a[Mittel_aus], temperatur = 0;
    static uint32_t leistung;
    // sobald die Reglerfreigabe weg geht oder die Netzsyncronität verloren geht Wandler Stom abschalten und in den Warteschritt gehen
    enable = 0;
    if (enable || !Sync)
    {
      Schritt = 1;
      U_out = 0;
    }
    // wenn nix los dann messen und gleitendes Mittel aus "Mittel_aus" Werten bilden, aktuelle Leistung ausrechnen und im Suchmodus die Maximalwerte speichern
    if (flag100ms == false)
    {
      if (cnt_a > Mittel_aus - 1)
        cnt_a = 0;
      spannung -= spannung_a[cnt_a];
      spannung_a[cnt_a] = adc_channel_sample(ADC_CHANNEL_7); // 970 (61) = 1V
      spannung += spannung_a[cnt_a];
      strom -= strom_a[cnt_a];
      strom_a[cnt_a] = adc_channel_sample(ADC_CHANNEL_1) - 130; // 2375 (148) = 1A
      strom += strom_a[cnt_a];
      cnt_a++;
      gu = spannung;
      gi = strom;
      if (Schritt == 2)
      { // im Suchschritt suchen wir hier den Punkt mit der grösten Leistung und merken uns die Spannung dort. Auf die regeln wir später.
        leistung = spannung * strom;
        if (leistung > leistung_MPP)
        {
          spannung_MPP = spannung;
          leistung_MPP = leistung;
        }
      }
    }
    // Haupttakt alle 100ms
    else
    {
      flag100ms = false;
      temperatur = adc_channel_sample(ADC_CHANNEL_8);      // Temperatur im Gehäuse messen
      maximalstrom = maximalstrom_abs * (6 - abregelwert); // aktuellen Maximalstrom aus maximalem Wechselrichterstrom und dem Abregelwert berechnen
      switch (Schritt)
      {
      case 1: // Warteschritt
        // auf die Reglerfreigabe vom Hauptcontroller und Netzsyncronität warten
        {
          if (!enable && Sync)
          {
            if (startverz < 50)
            { // wenn startklar noch 5 Sekunden warten
              startverz++;
              gpio_bit_reset(GPIOB, GPIO_PIN_12);
              gpio_bit_write(GPIOB, GPIO_PIN_13, (FlagStatus)!gpio_output_bit_get(GPIOB, GPIO_PIN_13));
            }
            else
            {
              minimalspannung = minimalspannung_abs;
              maximalspannung = minimalspannung_abs;
              Langzeitzaehler = 0;
              leistung_MPP = 0;
              U_out = 0;
              Schritt = 2;
              startverz = 0;
            }
          }
          else
          {
            gpio_bit_reset(GPIOB, GPIO_PIN_12);
            gpio_bit_set(GPIOB, GPIO_PIN_13);
            startverz = 0;
          }
          break;
        }
      case 2: // MPP finden Schritt
        // Wandlerstom kontinuirlich hoch fahren bis die Zellspannung auf die Minimalspannung sinkt oder der Strom auf den Maximalstrom steigt
        // dabei den Punkt mit der grösten Leistung suchen und sich die Spannung dort merken = MPP Spannung
        {
          gpio_bit_reset(GPIOB, GPIO_PIN_13);
          gpio_bit_write(GPIOB, GPIO_PIN_12, (FlagStatus)!gpio_output_bit_get(GPIOB, GPIO_PIN_12));
          if (maximalspannung < spannung)
            maximalspannung = spannung;
          if (teilbereichsuche)
          {
            if ((maximalspannung - (maximalspannung >> 2)) > minimalspannung)
              minimalspannung = (maximalspannung - (maximalspannung >> 2));
          }
          else
            minimalspannung = minimalspannung_abs;

          if ((spannung > minimalspannung) && (strom < maximalstrom))
          {
            if (U_out < 255)
            {
              U_out++;
            }
            else
            {
              teilbereichsuche = false;
              Schritt = 3;
            }
          }
          else
          {
            teilbereichsuche = false;
            Schritt = 3;
          }
          break;
        }
      case 3: // Regelbetrieb Schritt
      {
        gpio_bit_set(GPIOB, GPIO_PIN_12);
        gpio_bit_reset(GPIOB, GPIO_PIN_13);
        // bei plötzlicher Verschattung leistung schnell reduzieren (Spannung fällt unter 24V)
        if (spannung < (minimalspannung_abs - 38 * Mittel_aus))
        {
          U_out = U_out >> 1;
        }
        // ansonsten Wandlerstrom kontinuirlich so einstellen dass die Zellen mit der in Schritt 2 gefundenen MPP Spannung laufen
        else
        {
          if ((spannung < spannung_MPP) || (strom > maximalstrom) || (temperatur < 500))
          { // 72°C
            if (U_out > 0)
              U_out--; // Ausgangsstrom reduzieren
          }
          if ((spannung > spannung_MPP) && (strom < maximalstrom) && (temperatur > 600))
          { // 68°C
            if (U_out < 255)
              U_out++; // Augangsstrom erhöhen
          }
        }
        Langzeitzaehler++;
        // alle 10 min gucken ob sich die MPP Spannung durch änderung der Zellentemperatur verschoben hat
        // Wandlerstom dazu um 1/4 senken und Suche starten
        if ((Langzeitzaehler == 6000) || (Langzeitzaehler == 12000))
        {
          leistung_MPP = 0;
          U_out -= U_out >> 2;
          teilbereichsuche = true;
          Schritt = 2;
        }
        // alle 30min gucken ob sich durch verschattung ein neues globales maximum gebildet hat
        // Wandlerstom dazu auf 1/8 absenken und Suche starten
        if (Langzeitzaehler == 18000)
        {
          leistung_MPP = 0;
          Langzeitzaehler = 0;
          U_out = U_out >> 3;
          Schritt = 2;
        }
        break;
      }
      }
    }
  }
}
volatile uint32_t val;
void tsk_com(void *param)
{
  while (1)
  {
    Serial.print("Hz: ");           /// 260.147);
    Serial.println(TIMER_CAR(TIMER13)/112.5); /// 260.147);
    Serial.print("V: ");                /// 260.147);
    Serial.println(gu/970.0);                 /// 260.147);
    Serial.print("A: ");                /// 260.147);
    Serial.println(gi/2375.0);                 /// 260.147);
    Serial.print("W: ");                /// 260.147);
    Serial.println((gu*gi)/2678779.07);                 /// 260.147);

    vTaskDelay(1000);
  }
}
PWM mypwm(PB8);
HardwareTimer mytim(TIMER13);
void zcd_int()
{
  if (Synccounter < 256)
    TIMER_CAR(TIMER13)
    --; // Timer 1 so nachführen dass er netzsyncron läuft
  if (Synccounter > 256)
    TIMER_CAR(TIMER13)
    ++; // müsste bei 50Hz bei OCR1C=156 sein
  if (Synccounter == 256)
  {
    Sync = 1;
    digitalWrite(PC13, 1);
    digitalWrite(PA12, 0);
  }
  if ((TIMER_CAR(TIMER13) < 5500) || (TIMER_CAR(TIMER13) > 5800))
    Sync = 0; // Netzfrequenz überwachen Grenze 52,06Hz und 48,82Hz
  if ((TIMER_CAR(TIMER13) >= 5500) && (TIMER_CAR(TIMER13) <= 5620))
    abregelwert = 5620 - TIMER_CAR(TIMER13); // Zwischen 50 und 52Hz Leistung abregeln
  Synccounter = 0;
}
void t13ov()
{
  TIMER_CH0CV(TIMER15) = (sinus2[(uint8_t)Synccounter & 0b01111111] * (uint32_t)U_out) >> 8; // 8
  if (cnt100ms < 1282)
  {
    cnt100ms++;
  }
  else
  {               // alle 1282 x 78us = 100ms (16MHz/8/156=78us)
    cnt100ms = 0; // das Flag für die Hauptschleife setzen
    flag100ms = true;
  }
  Synccounter++;
  if (Synccounter > 258)
  {
    Sync = 0;
    digitalWrite(PC13, 0);
    digitalWrite(PA12, 1);
  }
}
void setup()
{
  // put your setup code here, to run once:

  mypwm.setPeriodCycle(100, 0, FORMAT_US);
  TIMER_PSC(TIMER15) = 6;
  TIMER_CAR(TIMER15) = 1000;
  TIMER_CH0CV(TIMER15) = 1;
  mypwm.start();
  mytim.setPrescaler(1);
  mytim.setReloadValue(5625); //112,5 / 1Hz
  mytim.attachInterrupt(t13ov, 0);
  mytim.start();
  Serial.begin(9600);
  pinMode(PB13, OUTPUT); // LED-rot
  pinMode(PB12, OUTPUT); // LED-blau
  pinMode(PC13, OUTPUT); // Relais-Netz
  pinMode(PA12, OUTPUT); // Enable PWM-Regler
  pinMode(PB11, OUTPUT); // Relais 115V/230V
  digitalWrite(PB11, 1);
  digitalWrite(PA12, 1);
  digitalWrite(PB12, 0);
  digitalWrite(PB13, 1);
  digitalWrite(PC13, 0);
  pinMode(PA7, INPUT_ANALOG); // PV-Spannung
  pinMode(PA1, INPUT_ANALOG); // PV-Strom
  pinMode(PB0, INPUT_ANALOG); // Temperatursensor
  pinMode(PA0, INPUT_ANALOG); // Spannung Netz (glatt)
  pinMode(PA4, INPUT_ANALOG); // Spannung Netz (puls)
  pinMode(PA6, INPUT);        // Zerocross

  adc_config();
  xTaskCreate(tsk_main, "task1", 200, NULL, 0, &hdl1);
  xTaskCreate(tsk_com, "task2", 100, NULL, 1, &hdl2);
  // queue1 = xQueueCreate(1, 8);
  attachInterrupt(PA6, zcd_int, RISING);
  vTaskStartScheduler();
}

void loop()
{
}
void adc_config(void)
{
  rcu_periph_clock_enable(RCU_ADC);
  rcu_adc_clock_config(RCU_ADCCK_APB2_DIV4);
  /* ADC data alignment config */
  adc_data_alignment_config(ADC_DATAALIGN_RIGHT);
  /* ADC channel length config */
  adc_channel_length_config(ADC_REGULAR_CHANNEL, 1U);
  /* ADC trigger config */
  adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_NONE);
  /* ADC external trigger config */
  adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
  /* enable ADC interface */
  adc_enable();
  delay(1U);
  /* ADC calibration and reset calibration */
  adc_calibration_enable();
}

/*!
    \brief      ADC channel sample
    \param[in]  none
    \param[out] none
    \retval     none
*/
uint16_t adc_channel_sample(uint8_t channel)
{
  /* ADC regular channel config */
  adc_regular_channel_config(0U, channel, ADC_SAMPLETIME_7POINT5);
  /* ADC software trigger enable */
  adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
  /* wait the end of conversion flag */
  while (!adc_flag_get(ADC_FLAG_EOC))
    ;
  /* clear the end of conversion flag */
  adc_flag_clear(ADC_FLAG_EOC);
  /* return regular channel sample value */
  return (adc_regular_data_read());
}