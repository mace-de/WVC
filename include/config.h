#pragma once
#include <Arduino.h>
#define RELAYCHECK                          // Testfunktion Netzrelais für Nutzung muss R88 mit 10k Ohm überbrückt werden
#define Mittel_aus 16                       // gleitendens Mittel aus X Werten (Maximal 63 sonst Überlauf)
#define Einschaltspannung 1700 * Mittel_aus // Einschaltspannung 28V
#define Abschaltspannung 1160 * Mittel_aus  // Abschaltspannung 19V
#define TaktHauptschleife 100               // Zykluszeit Haupttakt in ms

const uint32_t minimalspannung_abs = 1576 * Mittel_aus; //= etwa 26V darunter kann der Wandler wegen zu geringer Ausgangsspannung
                                                        // an den Scheitelpunkten nicht ins Netz einspeisen (61) = 1V
const uint32_t maximalstrom_abs = 2200 * Mittel_aus;    //= etwa 15A darüber wird der WR wohl verglühen (148) = 1A
