#include"mycom.h"

void tsk_com_send(void *param) // Kommunikationstask zum senden
{
  static uint32_t lastwaketime;
  while (1)
  {
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
    Serial.print(gun / 3.95, 1); // Ausgabe Netzspannung
    Serial.print(",AC_Current,");
    Serial.print(((gu * gi) / 2678779.07) / (gun / 3.95)); // Ausgabe berechneter Ausgangsstrom in A Wirkungsgrad ~86%
    Serial.print(",Out_Power,");
    Serial.print(leistung, 1); // Ausgabe berechnete Ausgangsleistung
    Serial.print(",Temperature,");
    Serial.print(gt / 100.0, 1); // Ausgabe gemessene Temperatur auf Platine
    Serial.print(",Power_adjustment,");
    Serial.print(flashdata.leistungsanforderung); // Ausgabe aktuelle Leistungsanforderung
    Serial.print(",Energy,");
    Serial.println(flashdata.energie_gesamt / 654545.4, 2); // Ausgabe Gesamtenergiemenge
    Serial.println();
    vTaskDelay(500);
    Serial.print("AT+SENDICA=property,PowerSwitch,");
    Serial.print(flashdata.mainswitch); // Ausgabe aktueller Ein/Aus Status
    Serial.print(",Day_Energy,");
    Serial.print(energie_tag / 654545.4, 2); // Ausgabe berechnete Energie seit einschalten
    Serial.print(",Plant,");
    Serial.print(U_out / 10.0, 1); // Ausgabe MPP-Regler Ausgang in % oder TIMER_CAR(TIMER13) / 112.5, 2); // Ausgabe gemessene Netzfrequenz
    Serial.print(",Emission,");
    Serial.print(gumpp / 970.0, 2); // Ausgabe MPP-Spannung im CO2 Datenfeld
    Serial.print(",Time,");
    Serial.print(flashdata.startverzoegerung); // Ausgabe Zeit Startverzögerung
    Serial.print(",P_adj,");
    Serial.print(flashdata.kalibrirung); // Ausgabe aktueller Kalibrierungswert
    Serial.print(",Model,");
    Serial.print(flashdata.reglergeschwindigkeit-1); // Ausgabe Zeit Startverzögerung
    Serial.print(",Chanel,");
    Serial.print(flashdata.spannungsgrenze); // Ausgabe aktueller Kalibrierungswert
    Serial.print(",TEMP_SET,");
    Serial.println(70);
    Serial.println();
    xTaskDelayUntil(&lastwaketime, 5500);
  }
}

void tsk_com_rcv(void *param) // Kommunikationstask zum empfangen
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
          vTaskDelay(10); // kurz warten damit der String auch komplett im Puffer ist
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
            step = 3;                       //              ^^
          if (ch[0] == 'P')                 //+ILOPDATA=ICA,PowerSwitch,0<\r><\n>
          {                                 //              ^
            for (int i = 2; i < 6; i++)
            {
              ch[i] = Serial.read();
            }
            if (ch[5] == 'S') //+ILOPDATA=ICA,PowerSwitch,0<\r><\n>
              step = 4;       //                   ^
            if (ch[5] == '_') //+ILOPDATA=ICA,Power_adjust,100<\r><\n>
              step = 8;       //                   ^
          }
          if (ch[0] == 'T' && ch[1] == 'i') //+ILOPDATA=ICA,Time,40<\r><\n>
            step = 5;                       //              ^^
          if (ch[0] == 'C' && ch[1] == 'h') //+ILOPDATA=ICA,Channel,1<\r><\n>
            step = 6;                       //              ^^
          if (ch[0] == 'M' && ch[1] == 'o') //+ILOPDATA=ICA,Model,4<\r><\n>
            step = 7;                       //              ^^
          if (ch[0] == 'P' && ch[1] == '_') //+ILOPDATA=ICA,P_adj,70<\r><\n>
            step = 9;                       //              ^^
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
          Flash_flag = true;
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
          Flash_flag = true;
          step = 0;
        }
        break;
      }
      case 5: // Startverzögerung
      {
        if (Serial.read() == ',')
        {
          flashdata.startverzoegerung = Serial.parseInt();
          Flash_flag = true;
          step = 0;
        }
        break;
      }
      case 6: // Startverzögerung
      {
        if (Serial.read() == ',')
        {
          flashdata.spannungsgrenze = Serial.parseInt();
          Flash_flag = true;
          step = 0;
        }
        break;
      }
      case 7: // Reglergeschwindigkeit
      {
        if (Serial.read() == ',')
        {
          flashdata.reglergeschwindigkeit = Serial.parseInt() + 1;
          Flash_flag = true;
          step = 0;
        }
        break;
      }
      case 8: // Leistungsbegrenzung
      {
        if (Serial.read() == ',')
        {
          flashdata.leistungsanforderung = Serial.parseInt();
          Flash_flag = true;
          step = 0;
        }
        break;
      }
      case 9: // ADC Kalibrierung
      {
        if (Serial.read() == ',')
        {
          flashdata.kalibrirung = Serial.parseInt();
          Flash_flag = true;
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
      if (Flash_flag)
      {
        myflash.write(0, (uint8_t *)&flashdata, sizeof(flashdata));
        myflash.commit();
        Flash_flag = false;
      }
      vTaskDelay(100); // wenn keine Daten im Puffer 100ms warten
    }
  }
}

