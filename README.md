# RF1000 Firmware
Based on Repetier-Firmware - the fast and user friendly firmware.

## Installation

The firmware is compiled and downloaded with Arduino V 1.0.5 or later.

## Version 0.91.48-wkd (2015-05-07 ff.)

* Current stable release.
* with Modifications from Wolf Karsten Dietz

## Documentation

For documentation please visit [http://www.repetier.com/documentation/repetier-firmware/](http://www.repetier.com/documentation/repetier-firmware/)

## Introduction

This variant of the Repetier-firmware has been optimized for the use with the
Renkforce RF1000 3D Printer of Conrad Electronic SE.
The firmware adds functionality which is not available within the standard
Repetier-firmware and uses the settings which match the available hardware.

The main differences to the standard Repetier-firmware are:

* Support for the RF1000 motherboard.
* Support for the RF1000 feature controller (= 6 additional hardware buttons).
* Support for motor current control via the TI DRV8711.
* Support for an external watchdog via the TI TPS382x.
* Printing can be paused/continued via a hardware button.
* Heat bed scan and Z-compensation via the built-in strain gauge of the extruder.
* Automatic emergency pause in case the strain gauge delivers too high measurements.
* Additional M-codes have been defined in order to control the RF1000-specific functionality.
* There is no support for the ArduinoDUE.

## Status of WKD-Modifications:
DONE :
- Schnelle Fahrt in die Heizbettmitte für Wartung und HBS -> Park-Funktion ===> Erledigt
- Anzeige Datum/Uhrzeit der letzten Compilierung beim Start ===> Erledigt
- Erkennung Filamentbruch/verstopfte Düse über Drucksensor -> Auto-Pause ===> Erledigt
- Bei Filamentbruch/verstopfter Düse Gerät andauernd Piepen lassen und passende Anzeige im Display ===> Erledigt
- Nach doppelter Pause kein Vorschub bei Widerstart, wenn in der Pause der manuelle Vorschub am Gerät betätigt wurde. ===> Erledigt
- Bei doppelter Pause Positionierung auf absolut Fixer Position von X/Y vor dem HB um Filamentwechsel und Düsenwechsel zu erleichtern. ===> Erledigt

IN WORK :
- Manuelle Korrekturwerte am Display für X-Y-Z (inkl. Abspeichern und je nach Extruder) 

IN CONCEPT :
- Änderungen Dual-Betrieb
- Lifteransteuerung

## Changelog

See changelog.txt
