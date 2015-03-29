# Wordclock

## Features
- runs on Arduino UNO / Arduino Pro Mini
- adressing of 12x9 LEDs (here just blue LEDs are used)
- MAX7221/MAX7219 support
- shows the current time on 5min steps (provided by the realtime-clock DS1302)
- after any 30secs the current temperature is shown for 5 secs (provided by BMP180 sensoric)

## Setting the time manually

1) connect over a RS232 connection
2) when connected you will get the time contonually
3) for testing the current time just time "sTIMESTAMP", the new time will be applied

A strange behaviour has been seen, when I try to set the time with the timestamp from http://www.currenttimestamp.com/ I have to change the value by the use of the following formular: `Timestamp - 2700 + (10 * 3600) - 300`
