# OpenCTDwithConduinoV03

This has drift issues, see https://github.com/kpdangelo/OpenCTD-Conduino-K1.0

This is version 3 of my conduino implementation
The circuit more closely follows the Analog Devices app note, CN-0217, in an attempt to mitigate the drift I was seeing in the Conduino reading with versions 1 and 2.
Namely, the AC coupling capacitors in series with the USB electrodes were taken studied, and a SPDT analog mux was added to swap the electrodes for each measurement in an attempt to see if there was biasing going on of the electrodes.
All fairly inconclusive at this time.

