EESchema Schematic File Version 4
EELAYER 30 0
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
L AD8226:AD8226ARMZ U1
U 1 1 5FE6F31F
P 4850 3750
F 0 "U1" H 5250 3600 50  0000 L CNN
F 1 "AD8226ARMZ" H 5250 3500 50  0000 L CNN
F 2 "Package_SO:MSOP-8_3x3mm_P0.65mm" H 4650 3750 50  0001 C CNN
F 3 "" H 5200 3350 50  0001 C CNN
	1    4850 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5FE7100F
P 4100 3750
F 0 "R1" H 4170 3796 50  0000 L CNN
F 1 "R" H 4170 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" V 4030 3750 50  0001 C CNN
F 3 "~" H 4100 3750 50  0001 C CNN
	1    4100 3750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5FE72372
P 5950 3750
F 0 "C1" H 5835 3704 50  0000 R CNN
F 1 "100n" H 5835 3795 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5988 3600 50  0001 C CNN
F 3 "~" H 5950 3750 50  0001 C CNN
	1    5950 3750
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5FE728AE
P 6300 3750
F 0 "C2" H 6185 3704 50  0000 R CNN
F 1 "10u" H 6185 3795 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6338 3600 50  0001 C CNN
F 3 "~" H 6300 3750 50  0001 C CNN
	1    6300 3750
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 5FE70826
P 4500 3100
F 0 "J2" H 4500 3300 50  0000 L CNN
F 1 "Conn_01x04" H 4418 2766 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4500 3100 50  0001 C CNN
F 3 "~" H 4500 3100 50  0001 C CNN
	1    4500 3100
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 5FE6FA2B
P 3600 3100
F 0 "J1" H 3600 3300 50  0000 C CNN
F 1 "Conn_01x04" H 3518 3326 50  0001 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 3600 3100 50  0001 C CNN
F 3 "~" H 3600 3100 50  0001 C CNN
	1    3600 3100
	-1   0    0    -1  
$EndComp
Text Label 5600 3750 2    50   ~ 0
Vout
Wire Wire Line
	5600 3750 5350 3750
Text Label 5250 4200 2    50   ~ 0
Vref
Wire Wire Line
	5250 4200 5050 4200
Wire Wire Line
	5050 4200 5050 4050
Text Label 4750 4200 0    50   ~ 0
V-
Wire Wire Line
	4750 4200 4850 4200
Wire Wire Line
	4850 4200 4850 4050
Text Label 4750 3300 0    50   ~ 0
V+
Wire Wire Line
	4750 3300 4850 3300
Wire Wire Line
	4850 3300 4850 3450
Text Label 4300 3550 0    50   ~ 0
In-
Wire Wire Line
	4300 3550 4550 3550
Text Label 4300 3950 0    50   ~ 0
In+
Wire Wire Line
	4300 3950 4550 3950
Text Label 4300 3650 0    50   ~ 0
Rg1
Text Label 4300 3850 0    50   ~ 0
Rg2
Wire Wire Line
	4250 3650 4250 3550
Wire Wire Line
	4250 3550 4100 3550
Wire Wire Line
	4100 3550 4100 3600
Wire Wire Line
	4250 3650 4550 3650
Wire Wire Line
	4100 3900 4100 3950
Wire Wire Line
	4100 3950 4250 3950
Wire Wire Line
	4250 3950 4250 3850
Wire Wire Line
	4250 3850 4550 3850
Text Label 6100 3450 0    50   ~ 0
V+
Wire Wire Line
	5950 3600 5950 3450
Wire Wire Line
	5950 3450 6300 3450
Wire Wire Line
	6300 3450 6300 3600
Text Label 6200 4000 2    50   ~ 0
Vref
Wire Wire Line
	5950 3900 5950 4000
Wire Wire Line
	5950 4000 6300 4000
Wire Wire Line
	6300 4000 6300 3900
$Comp
L Device:C C3
U 1 1 5FE7ECB4
P 6750 3750
F 0 "C3" H 6635 3704 50  0000 R CNN
F 1 "100n" H 6635 3795 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6788 3600 50  0001 C CNN
F 3 "~" H 6750 3750 50  0001 C CNN
	1    6750 3750
	-1   0    0    1   
$EndComp
$Comp
L Device:C C4
U 1 1 5FE7ECBE
P 7100 3750
F 0 "C4" H 6985 3704 50  0000 R CNN
F 1 "10u" H 6985 3795 50  0000 R CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7138 3600 50  0001 C CNN
F 3 "~" H 7100 3750 50  0001 C CNN
	1    7100 3750
	-1   0    0    1   
$EndComp
Text Label 6900 3450 0    50   ~ 0
V-
Wire Wire Line
	6750 3600 6750 3450
Wire Wire Line
	6750 3450 7100 3450
Wire Wire Line
	7100 3450 7100 3600
Text Label 7000 4000 2    50   ~ 0
Vref
Wire Wire Line
	6750 3900 6750 4000
Wire Wire Line
	6750 4000 7100 4000
Wire Wire Line
	7100 4000 7100 3900
Text Label 4000 3000 2    50   ~ 0
In-
Text Label 4000 3100 2    50   ~ 0
Rg1
Wire Wire Line
	4000 3100 3800 3100
Wire Wire Line
	3800 3000 4000 3000
Text Label 4000 3200 2    50   ~ 0
Rg2
Wire Wire Line
	4000 3200 3800 3200
Text Label 4000 3300 2    50   ~ 0
In+
Wire Wire Line
	4000 3300 3800 3300
Text Label 4100 3300 0    50   ~ 0
V-
Text Label 4100 3200 0    50   ~ 0
Vref
Text Label 4100 3100 0    50   ~ 0
Vout
Wire Wire Line
	4100 3100 4300 3100
Wire Wire Line
	4100 3200 4300 3200
Wire Wire Line
	4100 3300 4300 3300
Text Label 4100 3000 0    50   ~ 0
V+
Wire Wire Line
	4100 3000 4300 3000
$EndSCHEMATC
