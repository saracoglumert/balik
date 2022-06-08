# Sonar Array Analysis
Created Wednesday 08 June 2022

TO-DO
-----

* ☐ identify all components
* ☐ understand circuit
* ☐ where to probe to see if board is working?


Parts list:
-----------
SensComp sonar sensors (8x)
-aka Polaroid electrostatic transducer
-markings: SensComp 8/2005
-potential datasheet: <https://senscomp.com/wp-content/uploads/2019/12/V2_Series-600-Instr-Grade-Ultrasonic-Sensor-Spec-05Dec14.pdf>
TL851CN (1x)
-sonar ranging control IC
-datasheet: <https://www.ti.com/lit/ds/symlink/tl851.pdf>
TL852CN (1x)
-sonar ranging receiver IC
-datasheet: <https://www.ti.com/lit/ds/symlink/tl852.pdf>
AQW214 (4x)
-solid state relay
-datasheet: <https://datasheet.octopart.com/AQW214-Panasonic-datasheet-10534418.pdf>
74HC138N (1x)
-3 to 8 inverting decoder/demux
-datasheet: <https://eu.mouser.com/datasheet/2/916/74hc_hct138_3-1319710.pdf>
Unidentified blue thing (1x)
-at the bottom side of the board
-3 terminals
-markings on board: NC-1, 94V-0, 4305
Transformer (?) (1x)
-marked T1
Inductor (?) (1x)
-marked L1

Driver Board:
-------------
Seems similar to the board in the SensComp ranging module datasheet (<https://senscomp.com/wp-content/uploads/2019/12/6500-Ranging-Modules-Spec08Dec14-Rev.2.51_v2.pdf>), which has one potentiometer, one TL851, one TL852. 
Another similar board: <https://www.logosfoundation.org/logoscollectie/catalogus/pdf/Sonar_Ranger/Datasheet%20SN28827.pdf>
Our board seems to be multiplexing between sensors using the AQW214 and 74HC138N ICs.

### Connections:

* AQW214's are in two groups of two
	* each group has its pin 1 and 4 connected to a resistor
		* R5 for front, R6 for back.
		* both resistors are connected to the same terminal of the transformer.


### Connection Short Tests:
**Note: I think my multimeter was running out of battery which meant it didn't always beep when there was a short, pins that seem unconnected may be connected in reality**

#### TL851-TL852 Part:

* ☑ TL851
	* ☑ pin 1 - vcc
	* ☑ pin 2 - Q1 base
	* ☑ pin 3 - gnd 
	* ☑ pin 4 - TL852 pin 15
	* ☒ pin 5 - TL852 pin 14 **(unsure, multimeter seems to show 0, so might be connected)**
	* ☑ pin 6 - TL852 pin 13
	* ☑ pin 7 - TL852 pin 12
	* ☑ pin 8 - TL852 pin 9
	* ☑ pin 9 - echo output
	* ☒ pin 10 - osc output **(unsure)**
	* ☑ pin 11 - Y1
	* ☑ pin 12 - Y1 (other end)
	* ☐ pin 13 - C3 which goes to gnd **(probably connected, there was some resistance though)**
	* ☑ pin 14 - init input
	* ☒ pin 15 - ginh input **(unsure)**
	* ☑ pin 16 - blnk input
* ☑ TL852
	* ☑ pin 1 - R1 and R2 (R7 and R3 on the PCB)
	* ☑ pin 2 - R2 and transformer pin 4 (R3 on the PCB)
	* ☑ pin 3 - R1 (R7 on the PCB)
	* ☑ pin 4 - L1 and C1 which are then connected to vcc (C3 on the PCB) **(didn't check if L1 was connected to vcc but it probably is)**
	* ☑ pin 5 - vcc
	* ☐ pin 6 - C2 **(unsure)**
	* ☐ pin 7 - C2 **(unsure)**
	* ☑ pin 8 - R4 which is connected to gnd (R2 on schematic)
	* ☑ pin 9 - C4, TL851
	* ☐ pin 10, nothing **(don't know how to verify)**
	* ☐ pin 11, nothing **(don't know how to verify)**
	* ☑ pin 12 - TL851
	* ☑ pin 13 - TL851
	* ☒ pin 14 - TL851
	* ☑ pin 15 - TL851
	* ☑ pin 16 - gnd
* ☐ Transformer
	* ☑ pin 1 - Q1 collector
	* ☑ pin 2 - vcc
	* ☑ pin 3 - C5, which connects to two diodes to gnd and the sonar output **(tentative)**
	* ☑ pin 4 - TL852 pin 2


#### Muxing Part:

* ☐ figure out the intended connections first


#### Interface Between Parts:

* ☐ figure out the intended connections first


