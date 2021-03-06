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
LIBS:bmx055-brd-cache
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
F 2 "libs:bmx055-w" H 4500 3550 60  0001 C CNN
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
Text Label 3000 4150 0    60   ~ 0
scl
Wire Wire Line
	3700 4150 2950 4150
Text Label 3000 4350 0    60   ~ 0
sda
Wire Wire Line
	5300 3250 5500 3250
Wire Wire Line
	5500 3250 5500 3450
Wire Wire Line
	5500 3450 5300 3450
Wire Wire Line
	5500 3350 6200 3350
Connection ~ 5500 3350
NoConn ~ 5300 3950
NoConn ~ 5300 4150
NoConn ~ 5300 4350
NoConn ~ 5300 4550
NoConn ~ 5300 4750
Wire Wire Line
	3700 5450 3500 5450
NoConn ~ 5300 3050
Wire Wire Line
	7000 2400 8500 2400
Text Label 7050 2400 0    60   ~ 0
scl
Wire Wire Line
	7000 2600 8500 2600
Text Label 7050 2600 0    60   ~ 0
sda
$Comp
L VDD #PWR07
U 1 1 57D7204F
P 7000 1400
F 0 "#PWR07" H 7000 1250 50  0001 C CNN
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
L GND #PWR08
U 1 1 57D72087
P 7000 1800
F 0 "#PWR08" H 7000 1550 50  0001 C CNN
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
L TST P3
U 1 1 57D723A1
P 8500 2400
F 0 "P3" H 8500 2700 50  0000 C BNN
F 1 "scl" H 8500 2650 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 2400 50  0001 C CNN
F 3 "" H 8500 2400 50  0000 C CNN
	1    8500 2400
	0    1    1    0   
$EndComp
$Comp
L TST P4
U 1 1 57D7241E
P 8500 2600
F 0 "P4" H 8500 2900 50  0000 C BNN
F 1 "sda" H 8500 2850 50  0000 C CNN
F 2 "Measurement_Points:Measurement_Point_Round-SMD-Pad_Small" H 8500 2600 50  0001 C CNN
F 3 "" H 8500 2600 50  0000 C CNN
	1    8500 2600
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
L PWR_FLAG #FLG09
U 1 1 57D729F8
P 5300 1550
F 0 "#FLG09" H 5300 1645 50  0001 C CNN
F 1 "PWR_FLAG" H 5300 1730 50  0000 C CNN
F 2 "" H 5300 1550 50  0000 C CNN
F 3 "" H 5300 1550 50  0000 C CNN
	1    5300 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 1650 5300 1550
$Comp
L PWR_FLAG #FLG010
U 1 1 57D730EA
P 5700 1550
F 0 "#FLG010" H 5700 1645 50  0001 C CNN
F 1 "PWR_FLAG" H 5700 1730 50  0000 C CNN
F 2 "" H 5700 1550 50  0000 C CNN
F 3 "" H 5700 1550 50  0000 C CNN
	1    5700 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1550 5700 1650
$Comp
L GND #PWR011
U 1 1 57D733B7
P 5700 1650
F 0 "#PWR011" H 5700 1400 50  0001 C CNN
F 1 "GND" H 5700 1500 50  0000 C CNN
F 2 "" H 5700 1650 50  0000 C CNN
F 3 "" H 5700 1650 50  0000 C CNN
	1    5700 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 3450 3500 3550
$Comp
L GND #PWR012
U 1 1 57D86788
P 3500 3550
F 0 "#PWR012" H 3500 3300 50  0001 C CNN
F 1 "GND" H 3500 3400 50  0000 C CNN
F 2 "" H 3500 3550 50  0000 C CNN
F 3 "" H 3500 3550 50  0000 C CNN
	1    3500 3550
	1    0    0    -1  
$EndComp
NoConn ~ 3700 3050
NoConn ~ 3700 3250
Wire Wire Line
	3700 3450 3500 3450
Wire Wire Line
	6200 3350 6200 3450
$Comp
L GND #PWR013
U 1 1 57D869DF
P 6200 3450
F 0 "#PWR013" H 6200 3200 50  0001 C CNN
F 1 "GND" H 6200 3300 50  0000 C CNN
F 2 "" H 6200 3450 50  0000 C CNN
F 3 "" H 6200 3450 50  0000 C CNN
	1    6200 3450
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR014
U 1 1 57D86BFA
P 3500 5350
F 0 "#PWR014" H 3500 5200 50  0001 C CNN
F 1 "VDD" H 3500 5500 50  0000 C CNN
F 2 "" H 3500 5350 50  0000 C CNN
F 3 "" H 3500 5350 50  0000 C CNN
	1    3500 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 5450 3500 5350
$EndSCHEMATC
