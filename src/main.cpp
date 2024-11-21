#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include "GD32FreeRTOSConfig.h"
#include <HardwareTimer.h>
#include "FlashStorage_mod.h"
#include "myadc.h"
#include "lut.h"

#define Mittel_aus 16                       // gleitendens Mittel aus X Werten (Maximal 63 sonst Überlauf)
#define Einschaltspannung 1700 * Mittel_aus // Einschaltspannung 28V
#define Abschaltspannung 1160               // Abschaltspannung 19V

const uint32_t minimalspannung_abs = 1576 * Mittel_aus; //= etwa 26V darunter kann der Wandler wegen zu geringer Ausgangsspannung
                                                        // an den Scheitelpunkten nicht ins Netz einspeisen (61) = 1V
const uint32_t maximalstrom_abs = 2200 * Mittel_aus;    //= etwa 15A darüber wird der WR wohl verglühen (148) = 1A

volatile uint32_t U_out = 0, Synccounter = 0, abregelwert = 0, cnt100ms = 0, gu = 0, gi = 0, gun = 0, gt = 0, gumpp = 0;
volatile uint8_t counter = 0;
volatile bool flag100ms = 0, Sync = 0;
volatile float energie_tag = 0;
volatile struct s_flashdata
{
  float energie_gesamt;
  uint8_t mainswitch, leistungsanforderung;
} flashdata;
TaskHandle_t hdl1, hdl2, hdl3;
HardwareSerial Serial(PB7, PB6, 0);
FlashStorage<sizeof(flashdata)> myflash;
void relaischeck();
void tsk_main(void *param)
{
  while (1)
  {
    static uint8_t Schritt = 1, cnt_a = 0;
    static bool teilbereichsuche = false, vlock = true;
    static uint32_t spannung_MPP = 0, Langzeitzaehler = 0, minimalspannung = 0, maximalspannung = 0, maximalstrom = 0, startverz = 0;
    static uint32_t leistung_MPP = 0, maximalstrom_la = 0, store_enable_counter = 0;
    static uint32_t spannung = 0, strom = 0, spannung_a[Mittel_aus], strom_a[Mittel_aus], temperatur = 0;
    uint32_t leistung, netzspannung;
    // sobald die Netzsyncronität verloren geht Wandler Stom abschalten und in den Warteschritt gehen
    if (!Sync || !flashdata.mainswitch)
    {
      Schritt = 1;
      U_out = 0;
      gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
      gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
    }
    // wenn nix los dann messen und gleitendes Mittel aus "Mittel_aus" Werten bilden, aktuelle Leistung ausrechnen und im Suchmodus die Maximalwerte speichern
    if (flag100ms == false)
    {
      if (cnt_a > Mittel_aus - 1)
        cnt_a = 0;
      spannung -= spannung_a[cnt_a];
      spannung_a[cnt_a] = adc_channel_sample(ADC_CHANNEL_7); // 970 (61) = 1V
      spannung += spannung_a[cnt_a];
      if (spannung > Einschaltspannung) // wenn PV-Spannung größer Einschaltspannung, Wechselrichter Freigeben
      {
        vlock = false;
      }
      if ((vlock == false) && (spannung_a[cnt_a] < Abschaltspannung)) // wenn PV-Spannung unter Abschaltspannung fällt, permanente Daten im Flash sichern
      {                                                               // und Wechselrichter sperren
        vlock = true;
        Schritt = 0;
        if (store_enable_counter > 36000) // Flash speichern nur wenn PV-Spannung eine Stunde größer als Abschaltspannung
        {                                 // verhindert wiederholtes flashen bei zu wenig Licht
          myflash.write(0, (uint8_t *)&flashdata, sizeof(flashdata));
          myflash.commit();
        }
        store_enable_counter = 0;
      }
      strom -= strom_a[cnt_a];
      int32_t t_strom = adc_channel_sample(ADC_CHANNEL_1) - 130; // 2375 (148) = 1A
      strom_a[cnt_a] = t_strom < 0 ? 0 : t_strom;
      strom += strom_a[cnt_a];
      cnt_a++;
      netzspannung = adc_channel_sample(ADC_CHANNEL_0);  // 4,186 / 1V
      if ((netzspannung > 1060) || (netzspannung < 880)) // wenn Netzspannung unter 210V oder über 253V dann Wechselrichter aus
      {
        Sync = 0;
        gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
        gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
      }
      gun = netzspannung;
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
      store_enable_counter++;
      flag100ms = false;
      temperatur = adc_channel_sample(ADC_CHANNEL_8); // Temperatur im Gehäuse messen
      gt = temperatur;
      maximalstrom = (maximalstrom_abs * (225 - abregelwert)) / 225; // aktuellen Maximalstrom aus maximalem Wechselrichterstrom und dem Abregelwert berechnen
      maximalstrom_la = (maximalstrom_abs * flashdata.leistungsanforderung) / 100;
      if (maximalstrom > maximalstrom_la)
        maximalstrom = maximalstrom_la;
      switch (Schritt)
      {
      case 1: // Warteschritt
        // auf Netzsyncronität warten
        {
          gumpp = 0;
          if (Sync && flashdata.mainswitch && !vlock)
          {
            if (startverz < 300)
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
              maximalspannung = minimalspannung_abs;
              Langzeitzaehler = 0;
              leistung_MPP = 0;
              U_out = 0;
              Schritt = 2;
              startverz = 0;
            }
          }
          else
          {                                     // wenn kein Netz
            gpio_bit_reset(GPIOB, GPIO_PIN_12); // blau aus
            gpio_bit_set(GPIOB, GPIO_PIN_13);   // rot leuchtet
            startverz = 0;
          }
          break;
        }
      case 2: // MPP finden Schritt
        // Wandlerstom kontinuirlich hoch fahren bis die Zellspannung auf die Minimalspannung sinkt oder der Strom auf den Maximalstrom steigt
        // dabei den Punkt mit der grösten Leistung suchen und sich die Spannung dort merken = MPP Spannung
        {
          gpio_bit_reset(GPIOB, GPIO_PIN_13);                                                       // rot aus
          gpio_bit_write(GPIOB, GPIO_PIN_12, (FlagStatus)!gpio_output_bit_get(GPIOB, GPIO_PIN_12)); // blau blinkt
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
            if (U_out < 1023)
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
        gpio_bit_set(GPIOB, GPIO_PIN_12);   // blau leuchtet
        gpio_bit_reset(GPIOB, GPIO_PIN_13); // rot aus
        gumpp = spannung_MPP;
        // bei plötzlicher Verschattung leistung schnell reduzieren (Spannung fällt unter 24V)
        if (spannung < (minimalspannung_abs - 40 * Mittel_aus))
        {
          U_out = U_out >> 1;
        }
        // ansonsten Wandlerstrom kontinuirlich so einstellen dass die Zellen mit der in Schritt 2 gefundenen MPP Spannung laufen
        // und die Temperatur auf der platine nicht über 70°C steigt
        else
        {
          if ((spannung < spannung_MPP) || (strom > maximalstrom) || (temperatur < 550))
          { // 70°C
            if (U_out > 0)
              U_out--; // Ausgangsstrom reduzieren
          }
          if ((spannung > spannung_MPP) && (strom < maximalstrom) && (temperatur > 600))
          { // 68°C
            if (U_out < 1023)
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
        // Wandlerstom dazu auf 1/8 absenken, Suche starten und die permanenten Daten in den Flash schreiben
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
    vTaskDelay(1);
  }
}

void tsk_com_send(void *param)
{
  static uint32_t lastwaketime;
  while (1)
  {
    float temperatur = ((double)(gt / -658.6337) * (gt / -658.6337) * (gt / -658.6337)) + ((gt * gt) / 41895.521) - (gt * 0.0728544965) + 103.9;
    float leistung = (gu * gi) / 2678779.07;
    energie_tag += leistung;
    flashdata.energie_gesamt += leistung;
    vTaskDelay(500);
    Serial.print("AT+SENDICA=property,PV_Volt,");
    Serial.print(gu / 970.0, 1); // ausgabe Solarspannung in V
    Serial.print(",PV_Current,");
    Serial.print(gi / 2375.0, 1); // Ausgabe Solarstrom in A
    Serial.print(",PV_Power,");
    Serial.print((gi * gu) / (2375.0 * 970.0), 1); // Ausgabe Solarleistung in W
    Serial.print(",AC_Volt,");
    Serial.print(gun / 4.186, 1); // Ausgabe Netzspannung
    Serial.print(",AC_Current,");
    Serial.print(((gu * gi) / 2678779.07) / (gun / 4.186)); // Ausgabe berechneter Ausgangsstrom in A Wirkungsgrad ~86%
    Serial.print(",Out_Power,");
    Serial.print(leistung, 1); // Ausgabe berechnete Ausgangsleistung
    Serial.print(",Temperature,");
    Serial.print(temperatur, 1); // Ausgabe gemessene Temperatur auf Platine
    Serial.print(",Power_adjustment,");
    Serial.print(flashdata.leistungsanforderung); // Ausgabe aktuelle Leistungsanforderung
    Serial.print(",Energy,");
    Serial.println(flashdata.energie_gesamt / 654545.4, 2); // Ausgabe gemessene Netzfrequenz
    Serial.println();
    vTaskDelay(500);
    Serial.print("AT+SENDICA=property,PowerSwitch,");
    Serial.print(flashdata.mainswitch); // Ausgabe aktueller Ein/Aus Status
    Serial.print(",Day_Energy,");
    Serial.print(energie_tag / 654545.4, 2); // Ausgabe berechnete Energie seit einschalten
    Serial.print(",Plant,");
    Serial.print(TIMER_CAR(TIMER13) / 112.5); // Ausgabe gemessene Netzfrequenz
    Serial.print(",Emission,");
    Serial.print(gumpp / 970.0, 2); // Ausgabe MPP-Spannung im CO2 Datenfeld
    Serial.print(",Time,30,P_adj,");
    Serial.print(flashdata.leistungsanforderung); // Ausgabe aktuelle Leistungsanforderung
    Serial.println(",TEMP_SET,64");
    Serial.println();
    xTaskDelayUntil(&lastwaketime, 5500);
  }
}

void tsk_com_rcv(void *param)
{
  static uint8_t step = 0;
  char ch[6];
  while (1)
  {
    if (Serial.available()) // sind Daten im Puffer?
    {
      switch (step)
      {
      case 0:
      {
        if (Serial.read() == '+') // Startzeichen suchen
        {
          step = 1;
          vTaskDelay(1); // kurz warten damit der String auch komplett im Puffer ist
        }
        break;
      }
      case 1:
      {
        if (Serial.read() == 'I') // ILOPDATA?
          step = 2;               // dann zur Auswertung
        else
          step = 0; // sonst wieder zur Startzeichen Suche
        break;
      }
      case 2:
      {
        ch[0] = Serial.read();
        if (ch[0] == ',') // erstes Komma finden
        {
          ch[0] = Serial.read();
          ch[1] = Serial.read();
          if (ch[0] == 'E' && ch[1] == 'n') //+ILOPDATA=ICA,Energy_Cleared,0<\r><\n>
            step = 3;
          if (ch[0] == 'P')
          {
            for (int i = 2; i < 6; i++)
            {
              ch[i] = Serial.read();
            }
            if (ch[5] == 'S') //+ILOPDATA=ICA,PowerSwitch,0<\r><\n>
              step = 4;
            if (ch[5] == '_') //+ILOPDATA=ICA,Power_adjust,100<\r><\n>
              step = 8;
          }
          if (ch[0] == 'T' && ch[1] == 'i') //+ILOPDATA=ICA,Time,40<\r><\n>
            step = 5;
          if (ch[0] == 'C' && ch[1] == 'h') //+ILOPDATA=ICA,Channel,1<\r><\n>
            step = 6;
          if (ch[0] == 'M' && ch[1] == '0') //+ILOPDATA=ICA,Model,4<\r><\n>
            step = 7;
        }
        break;
      }
      case 3: // Gesamtenergie Rücksetzen
      {
        if (Serial.read() == ',')
        {
          ch[0] = Serial.read();
          if (ch[0] == '1')
            flashdata.energie_gesamt = 0.0;
          myflash.write(0, (uint8_t *)&flashdata, sizeof(flashdata));
          myflash.commit();
          step = 0;
        }
        break;
      }
      case 4: // Powerswitch
      {
        if (Serial.read() == ',')
        {
          ch[0] = Serial.read();
          if (ch[0] == '0')
            flashdata.mainswitch = 0;
          if (ch[0] == '1')
            flashdata.mainswitch = 1;
          myflash.write(0, (uint8_t *)&flashdata, sizeof(flashdata));
          myflash.commit();
          step = 0;
        }
        break;
      }
      case 8: // Leistungsbegrenzung
      {
        if (Serial.read() == ',')
        {
          flashdata.leistungsanforderung = Serial.parseInt();
          myflash.write(0, (uint8_t *)&flashdata, sizeof(flashdata));
          myflash.commit();
          step = 0;
        }
        break;
      }
      default:
      {
        step = 0;
        break;
      }
      }
    }
    else
    {
      vTaskDelay(100); // wenn keine Daten im Puffer 100ms warten
    }
  }
}

void zcd_int()
{
  if (Synccounter < 256)
    TIMER_CAR(TIMER13)
  --; // Timer 1 so nachführen dass er netzsyncron läuft
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

void t13ov_int()
{
  TIMER_CH0CV(TIMER15) = (sinus2[(uint8_t)Synccounter & 0b01111111] * U_out) >> 8;
  if (cnt100ms < 1282) // alle 1282 x 78us = 100ms (72MHz/5625=78us)
  {
    cnt100ms++;
  }
  else
  {
    cnt100ms = 0; // das Flag für die Hauptschleife setzen
    flag100ms = true;
  }
  Synccounter++;
  if (Synccounter > 258) // wenn Synccounter überläuft = kein Zerocross = kein Netz
  {
    Sync = 0;
    gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
    gpio_bit_set(GPIOA, GPIO_PIN_12);   // Regler aus
  }
}

PWM mypwm(PB8);
HardwareTimer mytim(TIMER13);
void setup()
{
  myflash.begin();
  myflash.read(0, (uint8_t *)&flashdata, sizeof(flashdata));
  if ((flashdata.leistungsanforderung > 100) || (flashdata.mainswitch > 1))
  {
    flashdata.energie_gesamt = 0.0;
    flashdata.leistungsanforderung = 100;
    flashdata.mainswitch = 1;
  }
  Serial.begin(115200);
  delay(4000);

  // I/O-Pins einstellen
  pinMode(PB13, OUTPUT);      // LED-rot
  pinMode(PB12, OUTPUT);      // LED-blau
  pinMode(PC13, OUTPUT);      // Relais-Netz
  pinMode(PA12, OUTPUT);      // Enable PWM-Regler (active low!)
  pinMode(PB11, OUTPUT);      // Relais 115V/230V
  pinMode(PA7, INPUT_ANALOG); // PV-Spannung
  pinMode(PA1, INPUT_ANALOG); // PV-Strom
  pinMode(PB0, INPUT_ANALOG); // Temperatursensor
  pinMode(PA0, INPUT_ANALOG); // Spannung Netz (glatt)
  pinMode(PA4, INPUT_ANALOG); // Spannung Netz (puls)
  pinMode(PA6, INPUT);        // Zerocross
  pinMode(PC14, INPUT_ANALOG);       // Optokoppler zwischen Wandler und Netzrelais
  digitalWrite(PB11, 1);      // Relais 115V/230V immer auf 230V
  digitalWrite(PA12, 1);      // PWM-Regler aus
  digitalWrite(PB12, 0);      // LED-blau aus
  digitalWrite(PC13, 0);      // Relais-Netz aus
  digitalWrite(PB13, 1);      // LED-rot ein
  //--------------------------------------------------------------------------
  adc_config(); 
  relaischeck();
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
  xTaskCreate(tsk_main, "task1", 200, NULL, 1, &hdl1);    // Haupttask
  xTaskCreate(tsk_com_send, "task2", 80, NULL, 0, &hdl2); // Kommunikationstask zum senden
  xTaskCreate(tsk_com_rcv, "task3", 80, NULL, 0, &hdl3);  // Kommunikationstask zum empfangen
  attachInterrupt(PA6, zcd_int, RISING);                  // Zerocross Interrupt

  vTaskStartScheduler(); // Tasks starten
}

void loop()
{
}
void relaischeck()
{
  boolean last_zcd, last_opto, zcd_temp, opto_temp;
  uint32_t zcd_cnt2, opto_cnt2, zcd_millis, opto_millis, schritt = 0;
  int32_t zcd_cnt1 = 0, opto_cnt1 = 0;

  while (1)
  {
    zcd_temp = gpio_input_bit_get(GPIOA, GPIO_PIN_6);
    opto_temp = gpio_input_bit_get(GPIOC, GPIO_PIN_14);
    if (zcd_temp == !last_zcd)
    {
      last_zcd = zcd_temp;
      zcd_cnt1++;
    }
    if (opto_temp == !last_opto)
    {
      last_opto = opto_temp;
      opto_cnt1++;
    }
    switch (schritt)
    {
    case 0:
    {
      if (zcd_cnt1 > 100)
      {
        schritt = 1;
        zcd_cnt1 = 0;
        zcd_millis = millis() + 2000;
        opto_millis = millis() + 1000;
      }
      break;
    }
    case 1:
    {
      if (millis() > opto_millis)
      {
        schritt = 2;
        gpio_bit_set(GPIOC, GPIO_PIN_13); // Netzrelais ein
      }
      break;
    }
    case 2:
    {
      if (millis() > zcd_millis)
      {
        schritt = 3;
        gpio_bit_reset(GPIOC, GPIO_PIN_13); // Netzrelais aus
      }
      break;
    }
    case 3:
    {
      if (abs(zcd_cnt1 - (2 * opto_cnt1)) < 5)
        return;
      while (1)
      {
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