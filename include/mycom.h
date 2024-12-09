#pragma once
#include <Arduino.h>
#include "FlashStorage_mod.h"
#include <FreeRTOS.h>
#include <task.h>
#include "GD32FreeRTOSConfig.h"
#include "flashdata.h"
extern volatile uint32_t U_out, gu, gi, gun, gt, gumpp;
extern volatile float energie_tag;
extern volatile bool Flash_flag;
extern volatile s_flashdata flashdata;
extern FlashStorage<sizeof(flashdata)> myflash;

void tsk_com_send(void *param); // Kommunikationstask zum senden
void tsk_com_rcv(void *param);  // Kommunikationstask zum empfangen
