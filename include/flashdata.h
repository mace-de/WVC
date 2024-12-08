#pragma once
#include <Arduino.h>
struct s_flashdata
{
    float energie_gesamt;
    uint8_t mainswitch, leistungsanforderung, kalibrirung, startverzoegerung, reglergeschwindigkeit, spannungsgrenze;
};