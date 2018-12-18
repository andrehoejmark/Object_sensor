EESchema Schematic File Version 4
EELAYER 26 0
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
L Device:Q_NMOS_GDS Q1
U 1 1 5C18153B
P 3100 2550
F 0 "Q1" H 3305 2596 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 3305 2505 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3300 2650 50  0001 C CNN
F 3 "~" H 3100 2550 50  0001 C CNN
	1    3100 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GDS Q2
U 1 1 5C1815D3
P 3100 3050
F 0 "Q2" H 3305 3096 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 3305 3005 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3300 3150 50  0001 C CNN
F 3 "~" H 3100 3050 50  0001 C CNN
	1    3100 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GDS Q5
U 1 1 5C18164D
P 4400 2550
F 0 "Q5" H 4605 2596 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 4605 2505 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4600 2650 50  0001 C CNN
F 3 "~" H 4400 2550 50  0001 C CNN
	1    4400 2550
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GDS Q6
U 1 1 5C181653
P 4400 3050
F 0 "Q6" H 4605 3096 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 4605 3005 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4600 3150 50  0001 C CNN
F 3 "~" H 4400 3050 50  0001 C CNN
	1    4400 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2750 3200 2850
Wire Wire Line
	4500 2850 4500 2750
Wire Wire Line
	4500 2350 4500 2150
Wire Wire Line
	4500 2150 3200 2150
Wire Wire Line
	3200 2150 3200 2350
Wire Wire Line
	3200 3250 3200 3400
Wire Wire Line
	3200 3400 4500 3400
Wire Wire Line
	4500 3400 4500 3250
Wire Wire Line
	4200 3050 4200 3300
Wire Wire Line
	4200 3300 2700 3300
Wire Wire Line
	2700 3300 2700 2550
Wire Wire Line
	2700 2550 2900 2550
Wire Wire Line
	4200 2550 4200 2300
Wire Wire Line
	4200 2300 2850 2300
Wire Wire Line
	2850 2300 2850 3050
Wire Wire Line
	2850 3050 2900 3050
$Comp
L Connector:Conn_01x04_Male J3
U 1 1 5C18190B
P 6150 2700
F 0 "J3" H 6123 2580 50  0000 R CNN
F 1 "Conn_01x04_Male" H 6123 2671 50  0000 R CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-4-5.08_1x04_P5.08mm_Horizontal" H 6150 2700 50  0001 C CNN
F 3 "~" H 6150 2700 50  0001 C CNN
	1    6150 2700
	-1   0    0    1   
$EndComp
Text Label 5950 2800 2    50   ~ 0
M1_A
Text Label 5950 2700 2    50   ~ 0
M1_B
Text Label 5950 2600 2    50   ~ 0
M2_A
Text Label 5950 2500 2    50   ~ 0
M2_B
$Comp
L Device:Q_NMOS_GDS Q3
U 1 1 5C181EA5
P 3100 4250
F 0 "Q3" H 3305 4296 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 3305 4205 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3300 4350 50  0001 C CNN
F 3 "~" H 3100 4250 50  0001 C CNN
	1    3100 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GDS Q4
U 1 1 5C181EAB
P 3100 4750
F 0 "Q4" H 3305 4796 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 3305 4705 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 3300 4850 50  0001 C CNN
F 3 "~" H 3100 4750 50  0001 C CNN
	1    3100 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GDS Q7
U 1 1 5C181EB1
P 4400 4250
F 0 "Q7" H 4605 4296 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 4605 4205 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4600 4350 50  0001 C CNN
F 3 "~" H 4400 4250 50  0001 C CNN
	1    4400 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NMOS_GDS Q8
U 1 1 5C181EB7
P 4400 4750
F 0 "Q8" H 4605 4796 50  0000 L CNN
F 1 "Q_NMOS_GDS" H 4605 4705 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Vertical" H 4600 4850 50  0001 C CNN
F 3 "~" H 4400 4750 50  0001 C CNN
	1    4400 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 4450 3200 4550
Wire Wire Line
	4500 4550 4500 4450
Wire Wire Line
	4500 4050 4500 3850
Wire Wire Line
	4500 3850 3200 3850
Wire Wire Line
	3200 3850 3200 4050
Wire Wire Line
	3200 4950 3200 5100
Wire Wire Line
	3200 5100 4500 5100
Wire Wire Line
	4500 5100 4500 4950
Wire Wire Line
	4200 4750 4200 5000
Wire Wire Line
	4200 5000 2700 5000
Wire Wire Line
	2700 5000 2700 4250
Wire Wire Line
	2700 4250 2900 4250
Wire Wire Line
	4200 4250 4200 4000
Wire Wire Line
	4200 4000 2850 4000
Wire Wire Line
	2850 4000 2850 4750
Wire Wire Line
	2850 4750 2900 4750
Text Label 3200 2800 0    50   ~ 0
M1_A
Text Label 4500 2800 2    50   ~ 0
M1_B
Text Label 3200 4550 0    50   ~ 0
M2_A
Text Label 4500 4550 2    50   ~ 0
M2_B
$Comp
L Connector:Conn_01x05_Male J1
U 1 1 5C1832A3
P 1000 2750
F 0 "J1" H 1106 3128 50  0000 C CNN
F 1 "Conn_01x05_Male" H 1106 3037 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-5-5.08_1x05_P5.08mm_Horizontal" H 1000 2750 50  0001 C CNN
F 3 "~" H 1000 2750 50  0001 C CNN
	1    1000 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J2
U 1 1 5C1833A9
P 1000 3400
F 0 "J2" H 1106 3578 50  0000 C CNN
F 1 "Conn_01x02_Male" H 1106 3487 50  0000 C CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2_1x02_P5.00mm_Horizontal" H 1000 3400 50  0001 C CNN
F 3 "~" H 1000 3400 50  0001 C CNN
	1    1000 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5C183490
P 1200 3500
F 0 "#PWR03" H 1200 3250 50  0001 C CNN
F 1 "GND" H 1205 3327 50  0000 C CNN
F 2 "" H 1200 3500 50  0001 C CNN
F 3 "" H 1200 3500 50  0001 C CNN
	1    1200 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5C1834C8
P 3200 3400
F 0 "#PWR06" H 3200 3150 50  0001 C CNN
F 1 "GND" H 3205 3227 50  0000 C CNN
F 2 "" H 3200 3400 50  0001 C CNN
F 3 "" H 3200 3400 50  0001 C CNN
	1    3200 3400
	1    0    0    -1  
$EndComp
Connection ~ 3200 3400
$Comp
L power:GND #PWR08
U 1 1 5C183500
P 3200 5100
F 0 "#PWR08" H 3200 4850 50  0001 C CNN
F 1 "GND" H 3205 4927 50  0000 C CNN
F 2 "" H 3200 5100 50  0001 C CNN
F 3 "" H 3200 5100 50  0001 C CNN
	1    3200 5100
	1    0    0    -1  
$EndComp
Connection ~ 3200 5100
$Comp
L power:VCC #PWR05
U 1 1 5C183563
P 3200 2150
F 0 "#PWR05" H 3200 2000 50  0001 C CNN
F 1 "VCC" H 3217 2323 50  0000 C CNN
F 2 "" H 3200 2150 50  0001 C CNN
F 3 "" H 3200 2150 50  0001 C CNN
	1    3200 2150
	1    0    0    -1  
$EndComp
Connection ~ 3200 2150
$Comp
L power:VCC #PWR07
U 1 1 5C18359B
P 3200 3850
F 0 "#PWR07" H 3200 3700 50  0001 C CNN
F 1 "VCC" H 3217 4023 50  0000 C CNN
F 2 "" H 3200 3850 50  0001 C CNN
F 3 "" H 3200 3850 50  0001 C CNN
	1    3200 3850
	1    0    0    -1  
$EndComp
Connection ~ 3200 3850
$Comp
L power:VCC #PWR02
U 1 1 5C184481
P 1200 3400
F 0 "#PWR02" H 1200 3250 50  0001 C CNN
F 1 "VCC" H 1217 3573 50  0000 C CNN
F 2 "" H 1200 3400 50  0001 C CNN
F 3 "" H 1200 3400 50  0001 C CNN
	1    1200 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5C184510
P 1200 2950
F 0 "#PWR01" H 1200 2700 50  0001 C CNN
F 1 "GND" H 1205 2777 50  0000 C CNN
F 2 "" H 1200 2950 50  0001 C CNN
F 3 "" H 1200 2950 50  0001 C CNN
	1    1200 2950
	1    0    0    -1  
$EndComp
Text Label 1200 2550 0    50   ~ 0
IN_1_A
Text Label 1200 2650 0    50   ~ 0
IN_1_B
Text Label 1200 2750 0    50   ~ 0
IN_2_A
Text Label 1200 2850 0    50   ~ 0
IN_2_B
Text Label 2850 2300 2    50   ~ 0
IN_1_A
Text Label 2700 2550 2    50   ~ 0
IN_1_B
Text Label 2850 4000 2    50   ~ 0
IN_2_A
Text Label 2700 4250 2    50   ~ 0
IN_2_B
Text Label 1100 4250 2    50   ~ 0
IN_1_A
Text Label 1100 4350 2    50   ~ 0
IN_1_B
Text Label 1100 4450 2    50   ~ 0
IN_2_A
Text Label 1100 4550 2    50   ~ 0
IN_2_B
$Comp
L Device:R R1
U 1 1 5C1874FD
P 1250 4250
F 0 "R1" V 1457 4250 50  0000 C CNN
F 1 "R" V 1366 4250 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1180 4250 50  0001 C CNN
F 3 "~" H 1250 4250 50  0001 C CNN
	1    1250 4250
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 5C187F72
P 1250 4350
F 0 "R2" V 1457 4350 50  0000 C CNN
F 1 "R" V 1366 4350 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1180 4350 50  0001 C CNN
F 3 "~" H 1250 4350 50  0001 C CNN
	1    1250 4350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5C187FFB
P 1250 4450
F 0 "R3" V 1457 4450 50  0000 C CNN
F 1 "R" V 1366 4450 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1180 4450 50  0001 C CNN
F 3 "~" H 1250 4450 50  0001 C CNN
	1    1250 4450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 5C188001
P 1250 4550
F 0 "R4" V 1457 4550 50  0000 C CNN
F 1 "R" V 1366 4550 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1180 4550 50  0001 C CNN
F 3 "~" H 1250 4550 50  0001 C CNN
	1    1250 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1400 4550 1400 4450
Wire Wire Line
	1400 4450 1400 4350
Connection ~ 1400 4450
Wire Wire Line
	1400 4350 1400 4250
Connection ~ 1400 4350
$Comp
L power:GND #PWR04
U 1 1 5C189544
P 1400 4550
F 0 "#PWR04" H 1400 4300 50  0001 C CNN
F 1 "GND" H 1405 4377 50  0000 C CNN
F 2 "" H 1400 4550 50  0001 C CNN
F 3 "" H 1400 4550 50  0001 C CNN
	1    1400 4550
	1    0    0    -1  
$EndComp
Connection ~ 1400 4550
$EndSCHEMATC
