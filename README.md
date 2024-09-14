# Power Meter for a bicycle cleat

## Goal
Visualize real time reading of power (voltage) from a power meter on a simple, affordable bicycle shoe cleat. This involves three energy-efficient programmes:  
a) Programme that runs on the board in the cleat (with a microchip Nordic Semiconductor nRF5340)  
b) App that runs on a PC, communacting with the board in the cleat via USB  
c) App that runs on an Android phone, communicating with the board via Bluetooth (later: following ANT+ profile)  

## Detailed description
Function: Read capacitor charge (voltage) at magnetic switch triggered interrupts. Convert to power reading with a data-defined calibration function, and Kalman filter. Broadcast the power reading via Bluetooth (later: following ANT+ Bicycle Power profile (either 1Hz, or event-synchronous))

## Platform
Nordic nRF5340 eventually on application specific circuitry, but can be tested on a Thingy:53 or nRF5340 DK, running Zephyr RTOS. Compliance with the ANT+ protocol (Bicycle Power profile). Language: C. Development in VSCode with the nrfConnect Add-On.

## Resources
- Nordic semiconductors website
- nrfConnect
- Forum: DevZone

## Contact
Institution: Microelectronics Research Unit, ITEE, University of Oulu  
*We are developers of energy-efficient IoT devices*  
Name: Yang Bai  
Email: yang.bai@oulu.fi  

For questions regarding the board: contact Eetu Virta (or Kuisma Hannuksela)  

To be completed: 31.12
