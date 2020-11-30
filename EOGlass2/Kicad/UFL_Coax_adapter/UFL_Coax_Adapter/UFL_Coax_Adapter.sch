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
L Connector:Conn_Coaxial J1
U 1 1 5FC306A0
P 5650 3250
F 0 "J1" H 5750 3179 50  0000 L CNN
F 1 "Conn_Coaxial" H 5750 3134 50  0001 L CNN
F 2 "Connector_Coaxial:U.FL_Molex_MCRF_73412-0110_Vertical" H 5650 3250 50  0001 C CNN
F 3 " ~" H 5650 3250 50  0001 C CNN
	1    5650 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector:TestPoint TP1
U 1 1 5FC314C3
P 5350 3250
F 0 "TP1" V 5453 3322 50  0001 C CNN
F 1 "TestPoint" V 5454 3322 50  0001 C CNN
F 2 "TestPoint:TestPoint_Pad_2.0x2.0mm" H 5550 3250 50  0001 C CNN
F 3 "~" H 5550 3250 50  0001 C CNN
	1    5350 3250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 3250 5450 3250
$Comp
L power:GND #PWR?
U 1 1 5FC322DA
P 5650 3500
F 0 "#PWR?" H 5650 3250 50  0001 C CNN
F 1 "GND" H 5655 3327 50  0001 C CNN
F 2 "" H 5650 3500 50  0001 C CNN
F 3 "" H 5650 3500 50  0001 C CNN
	1    5650 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5650 3500 5650 3450
$EndSCHEMATC
