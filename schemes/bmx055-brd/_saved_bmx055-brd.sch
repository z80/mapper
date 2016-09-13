EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:bmx055
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L BMX055 U1
U 1 1 57D714D8
P 4500 4250
F 0 "U1" H 4500 4250 60  0000 C CNN
F 1 "BMX055" H 4500 4700 60  0000 C CNN
F 2 "libs:bmx055" H 4500 3550 60  0001 C CNN
F 3 "" H 4500 3550 60  0001 C CNN
	1    4500 4250
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 57D7157B
P 4000 1900
F 0 "C1" H 4025 2000 50  0000 L CNN
F 1 "100n" H 4025 1800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4038 1750 50  0001 C CNN
F 3 "" H 4000 1900 50  0000 C CNN
	1    4000 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 57D716B0
P 4000 2250
F 0 "#PWR01" H 4000 2000 50  0001 C CNN
F 1 "GND" H 4000 2100 50  0000 C CNN
F 2 "" H 4000 2250 50  0000 C CNN
F 3 "" H 4000 2250 50  0000 C CNN
	1    4000 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 2250 4000 2050
Wire Wire Line
	4400 2550 4400 1650
Wire Wire Line
	4400 1650 4000 1650
Wire Wire Line
	4000 1550 4000 1750
Connection ~ 4000 1650
$Comp
L VDD #PWR02
U 1 1 57D716DA
P 4000 1550
F 0 "#PWR02" H 4000 1400 50  0001 C CNN
F 1 "VDD" H 4000 1700 50  0000 C CNN
F 2 "" H 4000 1550 50  0000 C CNN
F 3 "" H 4000 1550 50  0000 C CNN
	1    4000 1550
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 57D7170F
P 4900 1900
F 0 "C2" H 4925 2000 50  0000 L CNN
F 1 "100n" H 4925 1800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4938 1750 50  0001 C CNN
F 3 "" H 4900 1900 50  0000 C CNN
	1    4900 1900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 57D71715
P 4900 2250
F 0 "#PWR03" H 4900 2000 50  0001 C CNN
F 1 "GND" H 4900 2100 50  0000 C CNN
F 2 "" H 4900 2250 50  0000 C CNN
F 3 "" H 4900 2250 50  0000 C CNN
	1    4900 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4900 2250 4900 2050
Wire Wire Line
	4900 1550 4900 1750
Connection ~ 4900 1650
$Comp
L VDD #PWR04
U 1 1 57D7171F
P 4900 1550
F 0 "#PWR04" H 4900 1400 50  0001 C CNN
F 1 "VDD" H 4900 1700 50  0000 C CNN
F 2 "" H 4900 1550 50  0000 C CNN
F 3 "" H 4900 1550 50  0000 C CNN
	1    4900 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1650 5300 1650
Wire Wire Line
	4600 1650 4600 2550
$Comp
L GND #PWR05
U 1 1 57D717C2
P 4400 6150
F 0 "#PWR05" H 4400 5900 50  0001 C CNN
F 1 "GND" H 4400 6000 50  0000 C CNN
F 2 "" H 4400 6150 50  0000 C CNN
F 3 "" H 4400 6150 50  0000 C CNN
	1    4400 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 6150 4400 5950
$Comp
L GND #PWR06
U 1 1 57D71871
P 4600 6150
F 0 "#PWR06" H 4600 5900 50  0001 C CNN
F 1 "GND" H 4600 6000 50  0000 C CNN
F 2 "" H 4600 6150 50  0000 C CNN
F 3 "" H 4600 6150 50  0000 C CNN
	1    4600 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 6150 4600 5950
NoConn ~ 5300 5450
NoConn ~ 5300 5250
Wire Wire Line
	3700 4350 2950 4350
Text Label 3000 4350 0    60   ~ 0
mosi
Wire Wire Line
	3700 4150 2950 4150
Text Label 3000 4150 0    60   ~ 0
sck
Wire Wire Line
	5300 3250 5500 3250
Wire Wire Line
	5500 3250 5500 3450
Wire Wire Line
	5500 3450 5300 3450
Wire Wire Line
	5500 3350 6200 3350
Connection ~ 5500 3350
Text Label 5950 3350 0    60   ~ 0
miso
Wire Wire Line
	3700 3050 2950 3050
Text Label 3000 3050 0    60   ~ 0
~accel-cs~
Wire Wire Line
	3700 3250 2950 3250
Text Label 3000 3250 0    60   ~ 0
~gyro-cs~
Wire Wire Line
	3700 3450 2950 3450
Text Label 3000 3450 0    60   ~ 0
~magnet-cs~
NoConn ~ 5300 3950
NoConn ~ 5300 4150
NoConn ~ 5300 4350
NoConn ~ 5300 4550
NoConn ~ 5300 4750
Wire Wire Line
	3700 5450 3500 5450
Wire Wire Line
	3500 5450 3500 5650
$Comp
L GND #PWR07
U 1 1 57D71D3F
P 3500 5650
F 0 "#PWR07" H 3500 5400 50  0001 C CNN
F 1 "GND" H 3500 5500 50  0000 C CNN
F 2 "" H 3500 5650 50  0000 C CNN
F 3 "" H 3500 5650 50  0000 C CNN
	1    3500 5650
	1    0    0    -1  
$EndComp
NoConn ~ 5300 3050
Wire Wire Line
	7000 3400 7800 3400
Text Label 7050 3400 0    60   ~ 0
mosi
Wire Wire Line
	7000 3200 7800 3200
Text Label 7050 3200 0    60   ~ 0
sck
Wire Wire Line
	7000 2400 7800 2400
Text Label 7050 2400 0    60   ~ 0
~accel-cs~
Wire Wire Line
	7000 2600 7800 2600
Text Label 7050 2600 0    60   ~ 0
~gyro-cs~
Wire Wire Line
	7000 2800 7800 2800
Text Label 7050 2800 0    60   ~ 0
~magnet-cs~
Wire Wire Line
	7000 3600 7800 3600
Text Label 7050 3600 0    60   ~ 0
miso
$Comp
L VDD #PWR08
U 1 1 57D7204F
P 7000 1400
F 0 "#PWR08" H 7000 1250 50  0001 C CNN
F 1 "VDD" H 7000 1550 50  0000 C CNN
F 2 "" H 7000 1400 50  0000 C CNN
F 3 "" H 7000 1400 50  0000 C CNN
	1    7000 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 1400 7000 1500
Wire Wire Line
	7000 1500 7800 1500
$Comp
L GND #PWR09
U 1 1 57D72087
P 7000 1800
F 0 "#PWR09" H 7000 1550 50  0001 C CNN
F 1 "GND" H 7000 1650 50  0000 C CNN
F 2 "" H 7000 1800 50  0000 C CNN
F 3 "" H 7000 1800 50  0000 C CNN
	1    7000 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 1800 7000 1700
Wire Wire Line
	7000 1700 7800 1700
$Comp
L R R1
U 1 1 57D722A1
P 7950 2400
F 0 "R1" V 8030 2400 50  0000 C CNN
F 1 "100" V 7950 2400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7880 2400 50  0001 C CNN
F 3 "" H 7950 2400 50  0000 C CNN
	1    7950 2400
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 2400 8500 2400
$Comp
L TST P3
U 1 1 57D723A1
P 8500 2400
F 0 "P3" H 8500 2700 50  0000 C BNN
F 1 "accel-cs" H 8500 2650 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 2400 50  0001 C CNN
F 3 "" H 8500 2400 50  0000 C CNN
	1    8500 2400
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 57D72417
P 7950 2600
F 0 "R2" V 8030 2600 50  0000 C CNN
F 1 "100" V 7950 2600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7880 2600 50  0001 C CNN
F 3 "" H 7950 2600 50  0000 C CNN
	1    7950 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 2600 8500 2600
$Comp
L TST P4
U 1 1 57D7241E
P 8500 2600
F 0 "P4" H 8500 2900 50  0000 C BNN
F 1 "gyro-cs" H 8500 2850 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 2600 50  0001 C CNN
F 3 "" H 8500 2600 50  0000 C CNN
	1    8500 2600
	0    1    1    0   
$EndComp
$Comp
L R R3
U 1 1 57D7244C
P 7950 2800
F 0 "R3" V 8030 2800 50  0000 C CNN
F 1 "100" V 7950 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7880 2800 50  0001 C CNN
F 3 "" H 7950 2800 50  0000 C CNN
	1    7950 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 2800 8500 2800
$Comp
L TST P5
U 1 1 57D72453
P 8500 2800
F 0 "P5" H 8500 3100 50  0000 C BNN
F 1 "magnet-cs" H 8500 3050 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 2800 50  0001 C CNN
F 3 "" H 8500 2800 50  0000 C CNN
	1    8500 2800
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 57D72495
P 7950 3200
F 0 "R4" V 8030 3200 50  0000 C CNN
F 1 "100" V 7950 3200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7880 3200 50  0001 C CNN
F 3 "" H 7950 3200 50  0000 C CNN
	1    7950 3200
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3200 8500 3200
$Comp
L TST P6
U 1 1 57D7249C
P 8500 3200
F 0 "P6" H 8500 3500 50  0000 C BNN
F 1 "sck" H 8500 3450 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 3200 50  0001 C CNN
F 3 "" H 8500 3200 50  0000 C CNN
	1    8500 3200
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 57D724D2
P 7950 3400
F 0 "R5" V 8030 3400 50  0000 C CNN
F 1 "100" V 7950 3400 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7880 3400 50  0001 C CNN
F 3 "" H 7950 3400 50  0000 C CNN
	1    7950 3400
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3400 8500 3400
$Comp
L TST P7
U 1 1 57D724D9
P 8500 3400
F 0 "P7" H 8500 3700 50  0000 C BNN
F 1 "mosi" H 8500 3650 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 3400 50  0001 C CNN
F 3 "" H 8500 3400 50  0000 C CNN
	1    8500 3400
	0    1    1    0   
$EndComp
$Comp
L R R6
U 1 1 57D72513
P 7950 3600
F 0 "R6" V 8030 3600 50  0000 C CNN
F 1 "100" V 7950 3600 50  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 7880 3600 50  0001 C CNN
F 3 "" H 7950 3600 50  0000 C CNN
	1    7950 3600
	0    1    1    0   
$EndComp
Wire Wire Line
	8100 3600 8500 3600
$Comp
L TST P8
U 1 1 57D7251A
P 8500 3600
F 0 "P8" H 8500 3900 50  0000 C BNN
F 1 "miso" H 8500 3850 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 3600 50  0001 C CNN
F 3 "" H 8500 3600 50  0000 C CNN
	1    8500 3600
	0    1    1    0   
$EndComp
$Comp
L TST P1
U 1 1 57D7262E
P 7800 1500
F 0 "P1" H 7800 1800 50  0000 C BNN
F 1 "vcc" H 7800 1750 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 7800 1500 50  0001 C CNN
F 3 "" H 7800 1500 50  0000 C CNN
	1    7800 1500
	0    1    1    0   
$EndComp
$Comp
L TST P2
U 1 1 57D72696
P 7800 1700
F 0 "P2" H 7800 2000 50  0000 C BNN
F 1 "gnd" H 7800 1950 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 7800 1700 50  0001 C CNN
F 3 "" H 7800 1700 50  0000 C CNN
	1    7800 1700
	0    1    1    0   
$EndComp
$Comp
L PWR_FLAG #FLG010
U 1 1 57D729F8
P 5300 1550
F 0 "#FLG010" H 5300 1645 50  0001 C CNN
F 1 "PWR_FLAG" H 5300 1730 50  0000 C CNN
F 2 "" H 5300 1550 50  0000 C CNN
F 3 "" H 5300 1550 50  0000 C CNN
	1    5300 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1650 5300 1550
$Comp
L PWR_FLAG #FLG011
U 1 1 57D730EA
P 5700 1550
F 0 "#FLG011" H 5700 1645 50  0001 C CNN
F 1 "PWR_FLAG" H 5700 1730 50  0000 C CNN
F 2 "" H 5700 1550 50  0000 C CNN
F 3 "" H 5700 1550 50  0000 C CNN
	1    5700 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1550 5700 1650
$Comp
L GND #PWR012
U 1 1 57D733B7
P 5700 1650
F 0 "#PWR012" H 5700 1400 50  0001 C CNN
F 1 "GND" H 5700 1500 50  0000 C CNN
F 2 "" H 5700 1650 50  0000 C CNN
F 3 "" H 5700 1650 50  0000 C CNN
	1    5700 1650
	1    0    0    -1  
$EndComp
$EndSCHEMATC
