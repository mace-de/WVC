#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "GD32FreeRTOSConfig.h"
#include <HardwareTimer.h>
#include "FlashStorage_mod.h"
#include "myadc.h"
#include "relaycheck.h"
#include "Temp_lut.h"
#include "mycom.h"
#include "flashdata.h"
#include "config.h"
#include "myint.h"

volatile uint32_t U_out = 0, Synccounter = 0, abregelwert = 0, cnt_Haupttakt = 0, gu = 0, gi = 0, gun = 0, gt = 0, gumpp = 0;
volatile bool Flag_Haupttakt = 0, Sync = 0, Flash_flag = 0;
volatile float energie_tag = 0;
volatile s_flashdata flashdata;

TaskHandle_t hdl1, hdl2, hdl3;
HardwareSerial Serial(PB7, PB6, 0);
FlashStorage<sizeof(flashdata)> myflash;
PWM mypwm(PB8);
HardwareTimer mytim(TIMER13);

void tsk_main(void *param) // Hauppttask
{
  static uint8_t Schritt = 1, cnt_a = 0, cnt_blink = 0, cnt_netz = 0;
  static uint32_t spannung_MPP = 0, Langzeitzaehler = 0, minimalspannung = 0, maximalstrom = 0, startverz = 0;
  static uint32_t leistung_MPP = 0, maximalstrom_la = 0;
  static uint32_t spannung = 0, strom = 0, netzspannung = 0, netzspannung_a[Mittel_aus], spannung_a[Mittel_aus], strom_a[Mittel_aus], temperatur = 0;
  uint32_t leistung;
  float f_U_out = 0.0;
  while (1)
  {
    // sobald die Netzsyncronität verloren geht oder der Ausschaltbefehl von der App kommt Wandler Stom abschalten und in den Warteschritt gehen
    if (!Sync || !flashdata.mainswitch)
    {
      Schritt = 1;
      f_U_out = 0;
      gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
      gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
    }
    // wenn nix los dann messen und gleitendes Mittel aus "Mittel_aus" Werten bilden, aktuelle Leistung ausrechnen und im Suchmodus die Maximalwerte speichern
    if (Flag_Haupttakt == false)
    {
      if (cnt_a > Mittel_aus - 1)
      {
        cnt_a = 0;
      }
      spannung -= spannung_a[cnt_a];
      int32_t t_spannung = (adc_channel_sample(ADC_CHANNEL_7) * 50) / flashdata.kalibrirung; // 970 (61) = 1V
      spannung_a[cnt_a] = t_spannung;
      spannung += spannung_a[cnt_a];
      strom -= strom_a[cnt_a];
      int32_t t_strom = ((adc_channel_sample(ADC_CHANNEL_1) - 130) * 50) / flashdata.kalibrirung; // 2375 (148) = 1A
      strom_a[cnt_a] = t_strom < 0 ? 0 : t_strom;
      strom += strom_a[cnt_a];
      netzspannung -= netzspannung_a[cnt_a];
      int32_t t_netzspannung = (adc_channel_sample(ADC_CHANNEL_0) * 50) / flashdata.kalibrirung; // 4,186 / 1V
      netzspannung_a[cnt_a] = t_netzspannung;
      netzspannung += netzspannung_a[cnt_a];
      cnt_a++;

      gun = netzspannung / Mittel_aus;
      gu = spannung;
      gi = strom;
      if (Schritt == 2)
      { // im Suchschritt suchen wir hier den Punkt mit der grösten Leistung und merken uns die Spannung dort. Auf die regeln wir später.
        leistung = spannung * strom;
        if (leistung > leistung_MPP)
        {
          spannung_MPP = spannung < minimalspannung_abs ? minimalspannung_abs : spannung;
          leistung_MPP = leistung;
        }
      }
      else
      {
        leistung_MPP = 0;
      }
    }
    // Haupttakt alle "TaktHauptschleife" ms
    else
    {
      if (((netzspannung > 1060 * Mittel_aus) || (netzspannung < 880 * Mittel_aus)) && Sync) // wenn Netz da aber Netzspannung unter 210V oder über 253V
      {                                                                                      // für 5 sekunden dann Wechselrichter aus und 10 min warten
        if (cnt_netz < 5000 / TaktHauptschleife)
        {
          cnt_netz++;
        }
        else
        {
          Schritt = 0;
          startverz = 0;
          cnt_netz = 0;
        }
      }
      else
      {
        cnt_netz = 0;
      }
      if ((spannung < Abschaltspannung) && (Schritt == 3)) // wenn PV-Spannung im Regelbetrieb unter Abschaltspannung dann neu hochfahren
      {
        Schritt = 1;
        startverz = 0;
      }
      if (spannung < 700 * Mittel_aus) // wenn PV-Spannung unter 13V dann Wechselrichter aus und 10min warten zu wenig Licht
      {
        Schritt = 0;
        startverz = 0;
      }
      Flag_Haupttakt = false;
      temperatur = adc_channel_sample(ADC_CHANNEL_8); // Temperatur im Gehäuse messen
      temperatur = temperatur < TEMP_LUT_OFFSET ? 0 : temperatur - TEMP_LUT_OFFSET;
      temperatur = temperatur > TEMP_LUT_END ? TEMP_LUT_END : temperatur;
      temperatur = temparray[temperatur];
      gt = temperatur;
      maximalstrom = (maximalstrom_abs * (225 - abregelwert)) / 225;               // aktuellen Maximalstrom aus maximalem Wechselrichterstrom und dem Abregelwert berechnen
      maximalstrom_la = (maximalstrom_abs * flashdata.leistungsanforderung) / 100; // Maximalstrom über die App begrenzen
      if (maximalstrom > maximalstrom_la)
        maximalstrom = maximalstrom_la;
      switch (Schritt)
      {
      case 0: // 10min Warteschritt
      {
        gpio_bit_reset(GPIOB, GPIO_PIN_11); // Relais 115V/230V Aus
        gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
        gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
        f_U_out = 0.0;
        if (startverz < 600000 / TaktHauptschleife)
        {
          startverz++;
          if (cnt_blink < 10)
          {
            cnt_blink++;
          }
          else
          {
            cnt_blink = 0;
            gpio_bit_reset(GPIOB, GPIO_PIN_12);                                                       // blau aus
            gpio_bit_write(GPIOB, GPIO_PIN_13, (FlagStatus)!gpio_output_bit_get(GPIOB, GPIO_PIN_13)); // rot blinkt langsam
          }
        }
        else
        {
          startverz = 0;
          Schritt = 1;
        }
        break;
      }

      case 1: // Warteschritt
        // auf Netzsyncronität warten
        {
          gpio_bit_set(GPIOB, GPIO_PIN_11); // Relais 115V/230V Ein
          gpio_bit_set(GPIOA, GPIO_PIN_12); // Regler aus
          f_U_out = 0.0;
          if (Sync && flashdata.mainswitch) // alles ok?
          {
            if (startverz < (flashdata.startverzoegerung * 1000) / TaktHauptschleife)
            { // wenn startklar noch 30 Sekunden warten
              startverz++;
              gpio_bit_reset(GPIOB, GPIO_PIN_12);                                                       // blau aus
              gpio_bit_write(GPIOB, GPIO_PIN_13, (FlagStatus)!gpio_output_bit_get(GPIOB, GPIO_PIN_13)); // rot blinkt
            }
            else
            {                                     // wenn Wartezeit abgelaufen starten
              gpio_bit_set(GPIOC, GPIO_PIN_13);   // Netzrelais ein
              gpio_bit_reset(GPIOA, GPIO_PIN_12); // Regler ein
              minimalspannung = minimalspannung_abs;
              Langzeitzaehler = 0;
              Schritt = 2;
              startverz = 0;
            }
          }
          else // wenn kein Netz oder andere Störung
          {
            gpio_bit_reset(GPIOB, GPIO_PIN_12); // blau aus
            gpio_bit_set(GPIOB, GPIO_PIN_13);   // rot leuchtet
            startverz = 0;
          }
          break;
        }
      case 2: // MPP finden Schritt
        // Wandlerstom kontinuirlich hoch fahren bis die Zellspannung auf die Minimalspannung sinkt oder der Strom auf den Maximalstrom steigt
        // dabei den Punkt mit der grösten Leistung suchen und sich die Spannung dort merken = MPP Spannung (passiert weiter oben)
        {
          gpio_bit_reset(GPIOB, GPIO_PIN_13);                                                       // rot aus
          gpio_bit_write(GPIOB, GPIO_PIN_12, (FlagStatus)!gpio_output_bit_get(GPIOB, GPIO_PIN_12)); // blau blinkt
          minimalspannung = minimalspannung < minimalspannung_abs + (2 * 970 * flashdata.spannungsgrenze) ? minimalspannung_abs + (2 * 970 * flashdata.spannungsgrenze) : minimalspannung;
          if ((spannung > minimalspannung) && (strom < maximalstrom))
          {
            if (f_U_out < 1023)
            {
              f_U_out += (flashdata.reglergeschwindigkeit / 6.0);
            }
            else
            {
              f_U_out *= 0.8;
              Schritt = 3;
            }
          }
          else
          {
            f_U_out *= 0.8;
            Schritt = 3;
          }
          break;
        }
      case 3: // Regelbetrieb Schritt
      {
        gpio_bit_set(GPIOB, GPIO_PIN_12);   // blau leuchtet
        gpio_bit_reset(GPIOB, GPIO_PIN_13); // rot aus
        gumpp = spannung_MPP;
        // bei plötzlicher Verschattung leistung schnell reduzieren (Spannung fällt unter 24V)
        if (spannung < (minimalspannung_abs - 40 * Mittel_aus))
        {
          f_U_out *= 0.8;
        }
        // ansonsten Wandlerstrom kontinuirlich so einstellen dass die Zellen mit der in Schritt 2 gefundenen MPP Spannung laufen
        // und die Temperatur auf der platine nicht über 70°C steigt
        else
        {
          if ((strom > maximalstrom) || (temperatur > 7000)) // 70°C
          {
            f_U_out *= 0.9; // Ausgangsstrom reduzieren
          }
          else
          {
            f_U_out = f_U_out + (((float)spannung - (float)spannung_MPP) * flashdata.reglergeschwindigkeit) / (Mittel_aus * 61 * 24); // Augangsstrom regeln
          }
          f_U_out = f_U_out < 0.0 ? 0.0 : f_U_out;   // Reglerausgang nach unten
          f_U_out = f_U_out > 1023 ? 1023 : f_U_out; // und oben begrenzen
        }
        Langzeitzaehler++;
        // alle 10 min gucken ob sich die MPP Spannung durch änderung der Zellentemperatur verschoben hat
        // Wandlerstom dazu um 1/4 senken und Suche starten
        if ((Langzeitzaehler == 600000 / TaktHauptschleife) || (Langzeitzaehler == 1200000 / TaktHauptschleife))
        {
          f_U_out *= 0.75;
          minimalspannung = spannung_MPP - 300 * Mittel_aus; // suchen bis 5V unter aktueller MPP-Spannung
          Schritt = 2;
          Flash_flag = true;
        }
        // alle 30min gucken ob sich durch verschattung ein neues globales maximum gebildet hat
        // Wandlerstom dazu auf 1/8 absenken und Suche starten
        if (Langzeitzaehler == 1800000 / TaktHauptschleife)
        {
          Langzeitzaehler = 0;
          minimalspannung = minimalspannung_abs; // suchen bis zur Minimalspannung
          f_U_out *= 0.125;
          Schritt = 2;
          Flash_flag = true;
        }
        break;
      }
      }
    }
    U_out = f_U_out;
    vTaskDelay(1);
  }
}

void setup()
{
  if (myflash.begin())
  {
    myflash.read(0, (uint8_t *)&flashdata, sizeof(flashdata)); // Gespeicherte Daten aus dem Flash lesen
  }
  else
  { // Wenn Flash leer, auf standard Werte setzen.
    flashdata.energie_gesamt = 0.0;
    flashdata.leistungsanforderung = 100;
    flashdata.mainswitch = 1;
    flashdata.kalibrirung = 50;
    flashdata.startverzoegerung = 30;
    flashdata.reglergeschwindigkeit = 6;
    flashdata.spannungsgrenze = 0;
  }
  Serial.begin(115200);
  // I/O-Pins einstellen
  pinMode(PB13, OUTPUT);       // LED-rot
  pinMode(PB12, OUTPUT);       // LED-blau
  pinMode(PC13, OUTPUT);       // Relais-Netz
  pinMode(PA12, OUTPUT);       // Enable PWM-Regler (active low!)
  pinMode(PB11, OUTPUT);       // Relais 115V/230V
  pinMode(PA7, INPUT_ANALOG);  // PV-Spannung
  pinMode(PA1, INPUT_ANALOG);  // PV-Strom
  pinMode(PB0, INPUT_ANALOG);  // Temperatursensor
  pinMode(PA0, INPUT_ANALOG);  // Spannung Netz (glatt)
  pinMode(PA4, INPUT_ANALOG);  // Spannung Netz (puls)
  pinMode(PA6, INPUT);         // Zerocross
  pinMode(PC14, INPUT_PULLUP); // Optokoppler zwischen Wandler und Netzrelais
  digitalWrite(PA12, 1);       // PWM-Regler aus
  digitalWrite(PB12, 0);       // LED-blau aus
  digitalWrite(PC13, 0);       // Relais-Netz aus
  digitalWrite(PB13, 1);       // LED-rot ein
  //--------------------------------------------------------------------------
  adc_config();
#ifdef RELAYCHECK
  relaycheck(); // Netzrelais testen
#endif
  digitalWrite(PB11, 1);      // Relais 115V/230V Ein
  Serial.println("AT+E=off"); // WLAN-Modul Echo aus
  delay(1000);
  // Timer für PWM und Haupttakt einstellen 72MHz/7/1024=10kHz PWM und 72MHz/256/5625=50Hz
  // einstellen über Arduino Funktionen dann Zugriff nur noch über SPL

  mypwm.setPeriodCycle(100, 0);
  TIMER_PSC(TIMER15) = 6;
  TIMER_CAR(TIMER15) = 1024;
  TIMER_CH0CV(TIMER15) = 1;
  TIMER_SWEVG(TIMER15) |= (uint32_t)TIMER_SWEVG_UPG;
  mypwm.start();
  mytim.setPrescaler(1);
  mytim.setReloadValue(5625); // 112,5 / 1Hz
  mytim.attachInterrupt(t13ov_int, 0);
  mytim.start();
  //---------------------------------------------------------------------------

  // Tasks und Interrupt erzeugen
  xTaskCreate(tsk_main, "task1", 80, NULL, 1, &hdl1);      // Haupttask
  xTaskCreate(tsk_com_send, "task2", 110, NULL, 0, &hdl2); // Kommunikationstask zum senden
  xTaskCreate(tsk_com_rcv, "task3", 80, NULL, 0, &hdl3);   // Kommunikationstask zum empfangen
  attachInterrupt(PA6, zcd_int, RISING);                   // Zerocross Interrupt
  vTaskStartScheduler();                                   // Tasks starten
}

void loop()
{
}