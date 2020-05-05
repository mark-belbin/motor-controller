EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
Title "FOC Thruster Controller "
Date "2020-05-04"
Rev "V2"
Comp "Mark Belbin"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text Notes 4255 2375 0    118  Italic 24
TI MCU
Text Notes 8025 2350 0    118  Italic 24
Powertrain
Text Notes 650  4080 0    118  Italic 24
Power Input and Conversion
Text Notes 810  1705 0    118  Italic 24
Programming
Text Notes 5955 3965 0    118  Italic 24
CAN
Text Notes 10300 1725 0    118  Italic 24
Phases
Text Notes 1195 5065 0    79   Italic 16
3.3V Linear Regulator
Wire Wire Line
	1525 4435 1525 4485
Wire Wire Line
	1525 4485 1325 4485
$Comp
L power:+BATT #PWR0110
U 1 1 5E584027
P 1525 4435
F 0 "#PWR0110" H 1525 4285 50  0001 C CNN
F 1 "+BATT" H 1540 4608 50  0000 C CNN
F 2 "" H 1525 4435 50  0001 C CNN
F 3 "" H 1525 4435 50  0001 C CNN
	1    1525 4435
	1    0    0    -1  
$EndComp
Wire Wire Line
	1325 4610 1525 4610
Wire Wire Line
	1525 4610 1525 4660
$Comp
L power:GND #PWR?
U 1 1 5E585248
P 1525 4660
AR Path="/5E20A2BB/5E585248" Ref="#PWR?"  Part="1" 
AR Path="/5E585248" Ref="#PWR0111"  Part="1" 
F 0 "#PWR0111" H 1525 4410 50  0001 C CNN
F 1 "GND" H 1675 4585 50  0000 C CNN
F 2 "" H 1525 4660 50  0001 C CNN
F 3 "" H 1525 4660 50  0001 C CNN
	1    1525 4660
	1    0    0    -1  
$EndComp
Text Notes 1625 4435 0    50   Italic 10
Up to 15V 
$Comp
L Connector_Generic:Conn_01x01 J?
U 1 1 5E5B11C6
P 10600 2000
AR Path="/5E20A2BB/5E5B11C6" Ref="J?"  Part="1" 
AR Path="/5E5B11C6" Ref="J1"  Part="1" 
F 0 "J1" H 10680 2042 50  0000 L CNN
F 1 "PHASE_A" H 10680 1951 50  0000 L CNN
F 2 "Thruster_Controller:Phase_Pad" H 10600 2000 50  0001 C CNN
F 3 "~" H 10600 2000 50  0001 C CNN
	1    10600 2000
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J?
U 1 1 5E5B11CC
P 10600 2225
AR Path="/5E20A2BB/5E5B11CC" Ref="J?"  Part="1" 
AR Path="/5E5B11CC" Ref="J2"  Part="1" 
F 0 "J2" H 10680 2267 50  0000 L CNN
F 1 "PHASE_B" H 10680 2176 50  0000 L CNN
F 2 "Thruster_Controller:Phase_Pad" H 10600 2225 50  0001 C CNN
F 3 "~" H 10600 2225 50  0001 C CNN
	1    10600 2225
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J?
U 1 1 5E5B11D2
P 10600 2450
AR Path="/5E20A2BB/5E5B11D2" Ref="J?"  Part="1" 
AR Path="/5E5B11D2" Ref="J3"  Part="1" 
F 0 "J3" H 10680 2492 50  0000 L CNN
F 1 "PHASE_C" H 10680 2401 50  0000 L CNN
F 2 "Thruster_Controller:Phase_Pad" H 10600 2450 50  0001 C CNN
F 3 "~" H 10600 2450 50  0001 C CNN
	1    10600 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6120 1525 7175 1525
Wire Wire Line
	7175 1600 6120 1600
Wire Wire Line
	6120 1675 7175 1675
Wire Wire Line
	7175 1750 6120 1750
Wire Wire Line
	6120 1875 7175 1875
Wire Wire Line
	7175 1950 6120 1950
Wire Wire Line
	6120 2025 7175 2025
Wire Wire Line
	7175 2100 6120 2100
Wire Wire Line
	6120 2175 7175 2175
Wire Wire Line
	7175 2250 6120 2250
Wire Wire Line
	6120 2325 7175 2325
Wire Wire Line
	7175 2450 6120 2450
Wire Wire Line
	6120 2525 7175 2525
Wire Wire Line
	7175 2600 6120 2600
Wire Wire Line
	6120 2675 7175 2675
Wire Wire Line
	7175 2750 6120 2750
Wire Wire Line
	6120 2825 7175 2825
Wire Wire Line
	7175 2925 6120 2925
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J5
U 1 1 5E2798B5
P 1350 2475
F 0 "J5" H 1400 2800 50  0000 C CNN
F 1 "Pogo Conn" H 1400 2150 50  0000 C CNN
F 2 "Connectors:Tag-Connect_TC2050-IDC-NL" H 1350 2475 50  0001 C CNN
F 3 "~" H 1350 2475 50  0001 C CNN
F 4 "" H 1350 2475 50  0001 C CNN "Digikey"
	1    1350 2475
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 2275 950  2275
Wire Wire Line
	950  2275 950  2200
Wire Wire Line
	1150 2575 1075 2575
Wire Wire Line
	1075 2575 1075 2675
Wire Wire Line
	1150 2675 1075 2675
Wire Wire Line
	1075 2675 950  2675
Wire Wire Line
	950  2675 950  2725
Connection ~ 1075 2675
$Comp
L power:GND #PWR?
U 1 1 5E2798C3
P 950 2725
AR Path="/5E20A2BB/5E2798C3" Ref="#PWR?"  Part="1" 
AR Path="/5E2798C3" Ref="#PWR0115"  Part="1" 
F 0 "#PWR0115" H 950 2475 50  0001 C CNN
F 1 "GND" H 955 2552 50  0000 C CNN
F 2 "" H 950 2725 50  0001 C CNN
F 3 "" H 950 2725 50  0001 C CNN
	1    950  2725
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E2798C9
P 950 2200
AR Path="/5E20A2BB/5E2798C9" Ref="#PWR?"  Part="1" 
AR Path="/5E2798C9" Ref="#PWR0116"  Part="1" 
F 0 "#PWR0116" H 950 2050 50  0001 C CNN
F 1 "+3V3" H 1100 2275 50  0000 C CNN
F 2 "" H 950 2200 50  0001 C CNN
F 3 "" H 950 2200 50  0001 C CNN
	1    950  2200
	1    0    0    -1  
$EndComp
Text Label 875  2375 0    50   ~ 0
GPIO28
Text Label 875  2475 0    50   ~ 0
GPIO29
Text Label 1900 2275 2    50   ~ 0
TMS
Text Label 1900 2375 2    50   ~ 0
TCK
Text Label 1900 2475 2    50   ~ 0
TDO
Text Label 1900 2575 2    50   ~ 0
TDI
Text Label 1900 2675 2    50   ~ 0
TRST
$Sheet
S 3170 1450 2950 1925
U 5E20A372
F0 "TI_MCU" 50
F1 "TI_MCU.sch" 50
F2 "GPIO28" I R 6120 1600 50 
F3 "SPI_CLK" O R 6120 3150 50 
F4 "ADCINA7" I R 6120 1875 50 
F5 "ADCINA3" I R 6120 1950 50 
F6 "ADCINA1" I R 6120 2025 50 
F7 "ADCINA0" I R 6120 2100 50 
F8 "ADCINB1" I R 6120 2175 50 
F9 "ADCINB3" I R 6120 2250 50 
F10 "ADCINB7" I R 6120 2325 50 
F11 "EPWM1A" O R 6120 2450 50 
F12 "EPWM1B" O R 6120 2525 50 
F13 "EPWM2A" O R 6120 2600 50 
F14 "EPWM2B" O R 6120 2675 50 
F15 "EPWM3A" O R 6120 2750 50 
F16 "EPWM3B" O R 6120 2825 50 
F17 "SPI_CS_0" O R 6120 2925 50 
F18 "~RESET~" I R 6120 1525 50 
F19 "SPI_MOSI" B R 6120 3000 50 
F20 "SPI_MISO" B R 6120 3075 50 
F21 "GPIO6" O R 6120 1750 50 
F22 "GPIO7" O R 6120 1675 50 
F23 "GPIO29" I L 3170 2050 50 
F24 "TRST" I L 3170 2675 50 
F25 "TCK" I L 3170 2375 50 
F26 "TMS" I L 3170 2275 50 
F27 "TDI" I L 3170 2575 50 
F28 "TDO" I L 3170 2475 50 
F29 "GPIO28" I L 3170 1950 50 
$EndSheet
$Comp
L power:GND #PWR?
U 1 1 5E4A3E7B
P 5650 4900
AR Path="/5E20A2BB/5E4A3E7B" Ref="#PWR?"  Part="1" 
AR Path="/5E4A3E7B" Ref="#PWR0121"  Part="1" 
F 0 "#PWR0121" H 5650 4650 50  0001 C CNN
F 1 "GND" H 5800 4850 50  0000 C CNN
F 2 "" H 5650 4900 50  0001 C CNN
F 3 "" H 5650 4900 50  0001 C CNN
	1    5650 4900
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5E5059A5
P 4750 4750
AR Path="/5E20A2BB/5E5059A5" Ref="C?"  Part="1" 
AR Path="/5E5059A5" Ref="C23"  Part="1" 
F 0 "C23" H 4575 4825 50  0000 L CNN
F 1 "0.1uF" H 4525 4675 50  0000 L CNN
F 2 "Capacitors_SMD:C_0402" H 4750 4750 50  0001 C CNN
F 3 "" H 4750 4750 50  0001 C CNN
F 4 "490-10698-1-ND" H 4750 4750 50  0001 C CNN "Digikey"
	1    4750 4750
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E507468
P 4850 4525
AR Path="/5E20A2BB/5E507468" Ref="#PWR?"  Part="1" 
AR Path="/5E507468" Ref="#PWR0124"  Part="1" 
F 0 "#PWR0124" H 4850 4375 50  0001 C CNN
F 1 "+3V3" H 4850 4700 50  0000 C CNN
F 2 "" H 4850 4525 50  0001 C CNN
F 3 "" H 4850 4525 50  0001 C CNN
	1    4850 4525
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E50E8F3
P 4750 4925
AR Path="/5E20A2BB/5E50E8F3" Ref="#PWR?"  Part="1" 
AR Path="/5E50E8F3" Ref="#PWR0125"  Part="1" 
F 0 "#PWR0125" H 4750 4675 50  0001 C CNN
F 1 "GND" H 4750 4775 50  0000 C CNN
F 2 "" H 4750 4925 50  0001 C CNN
F 3 "" H 4750 4925 50  0001 C CNN
	1    4750 4925
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4850 4750 4925
Wire Wire Line
	4750 4650 4750 4575
$Comp
L Device:C_Small C?
U 1 1 5E595A4F
P 4950 4750
AR Path="/5E20A2BB/5E595A4F" Ref="C?"  Part="1" 
AR Path="/5E595A4F" Ref="C28"  Part="1" 
F 0 "C28" H 4800 4825 50  0000 L CNN
F 1 "1uF" H 4800 4675 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4950 4750 50  0001 C CNN
F 3 "" H 4950 4750 50  0001 C CNN
F 4 "1276-1102-1-ND" H 4950 4750 50  0001 C CNN "Digikey"
	1    4950 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 4575 4850 4575
Wire Wire Line
	4850 4575 4850 4525
Wire Wire Line
	4850 4575 4950 4575
Wire Wire Line
	4950 4575 4950 4650
Connection ~ 4850 4575
Wire Wire Line
	4950 4850 4950 4925
$Comp
L power:GND #PWR?
U 1 1 5E5AB2A9
P 4950 4925
AR Path="/5E20A2BB/5E5AB2A9" Ref="#PWR?"  Part="1" 
AR Path="/5E5AB2A9" Ref="#PWR0129"  Part="1" 
F 0 "#PWR0129" H 4950 4675 50  0001 C CNN
F 1 "GND" H 4950 4775 50  0000 C CNN
F 2 "" H 4950 4925 50  0001 C CNN
F 3 "" H 4950 4925 50  0001 C CNN
	1    4950 4925
	1    0    0    -1  
$EndComp
Text Label 6825 4850 2    50   ~ 0
CANH
Text Label 4800 4150 0    50   ~ 0
TXCAN
Text Label 4800 4250 0    50   ~ 0
RXCAN
$Comp
L power:+3V3 #PWR?
U 1 1 5E49D988
P 5175 3900
AR Path="/5E20A2BB/5E49D988" Ref="#PWR?"  Part="1" 
AR Path="/5E49D988" Ref="#PWR0130"  Part="1" 
F 0 "#PWR0130" H 5175 3750 50  0001 C CNN
F 1 "+3V3" H 5175 4050 50  0000 C CNN
F 2 "" H 5175 3900 50  0001 C CNN
F 3 "" H 5175 3900 50  0001 C CNN
	1    5175 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small_US R35
U 1 1 5E50F8DF
P 6550 5025
F 0 "R35" H 6482 4979 50  0000 R CNN
F 1 "120" H 6482 5070 50  0000 R CNN
F 2 "Resistors_SMD:R_0402" H 6550 5025 50  0001 C CNN
F 3 "~" H 6550 5025 50  0001 C CNN
F 4 "311-120JRCT-ND" H 6550 5025 50  0001 C CNN "Digikey"
	1    6550 5025
	-1   0    0    1   
$EndComp
Wire Wire Line
	6550 4850 6550 4925
Wire Wire Line
	6550 4850 6825 4850
Wire Wire Line
	6550 5125 6550 5225
Wire Wire Line
	6550 5225 6825 5225
Text Label 6300 4250 0    50   ~ 0
CANH
Text Label 6825 5225 2    50   ~ 0
CANL
$Comp
L Interface_CAN_LIN:MCP2562-E-MF U8
U 1 1 5E558E9D
P 5700 4350
F 0 "U8" H 5350 4700 50  0000 C CNN
F 1 "MCP2562" H 5350 4000 50  0000 C CNN
F 2 "Housings_DFN_QFN:DFN-8-1EP_3x3mm_Pitch0.65mm" H 5700 3850 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/25167A.pdf" H 5700 4350 50  0001 C CNN
F 4 "MCP2562-E/MF-ND" H 5700 4350 50  0001 C CNN "Digikey"
	1    5700 4350
	1    0    0    -1  
$EndComp
Text Notes 6450 5075 2    50   ~ 0
Termination\nResistor
Text Label 6305 4450 0    50   ~ 0
CANL
Wire Wire Line
	5200 4450 5175 4450
Wire Wire Line
	5175 3900 5175 4450
Wire Wire Line
	5600 4750 5600 4800
Wire Wire Line
	5600 4800 5650 4800
Wire Wire Line
	5700 4800 5700 4750
Wire Wire Line
	5650 4800 5650 4900
Connection ~ 5650 4800
Wire Wire Line
	5650 4800 5700 4800
$Comp
L power:GND #PWR?
U 1 1 5E66F004
P 5125 4825
AR Path="/5E20A2BB/5E66F004" Ref="#PWR?"  Part="1" 
AR Path="/5E66F004" Ref="#PWR0170"  Part="1" 
F 0 "#PWR0170" H 5125 4575 50  0001 C CNN
F 1 "GND" H 5275 4775 50  0000 C CNN
F 2 "" H 5125 4825 50  0001 C CNN
F 3 "" H 5125 4825 50  0001 C CNN
	1    5125 4825
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4550 5125 4550
Wire Wire Line
	5125 4550 5125 4825
$Sheet
S 7175 1450 2875 1875
U 5E20A2BB
F0 "Powertrain" 50
F1 "Powertrain.sch" 50
F2 "Phase_A" O R 10050 2000 50 
F3 "Phase_B" O R 10050 2225 50 
F4 "Phase_C" O R 10050 2450 50 
F5 "VSEN_A" O L 7175 1875 50 
F6 "VSEN_B" O L 7175 1950 50 
F7 "VSEN_C" O L 7175 2025 50 
F8 "VSEN_PVDD" O L 7175 2100 50 
F9 "ISEN_A" O L 7175 2175 50 
F10 "ISEN_B" O L 7175 2250 50 
F11 "ISEN_C" O L 7175 2325 50 
F12 "INH_A" I L 7175 2450 50 
F13 "INL_A" I L 7175 2525 50 
F14 "INH_B" I L 7175 2600 50 
F15 "INL_B" I L 7175 2675 50 
F16 "INH_C" I L 7175 2750 50 
F17 "INL_C" I L 7175 2825 50 
F18 "SPI_CS_0" I L 7175 2925 50 
F19 "SPI_MOSI" B L 7175 3000 50 
F20 "SPI_MISO" B L 7175 3075 50 
F21 "SPI_CLK" I L 7175 3150 50 
F22 "PWRGD" O L 7175 1525 50 
F23 "EN_GATE" I L 7175 1750 50 
F24 "WAKE" I L 7175 1675 50 
F25 "nFAULT" O L 7175 1600 50 
$EndSheet
Wire Wire Line
	4625 4150 5200 4150
Wire Wire Line
	4625 4250 5200 4250
Wire Wire Line
	6200 4250 6525 4250
Wire Wire Line
	10050 2450 10400 2450
Wire Wire Line
	10050 2225 10400 2225
Wire Wire Line
	10050 2000 10400 2000
Wire Wire Line
	1650 2275 3170 2275
Wire Wire Line
	1650 2375 3170 2375
Wire Wire Line
	1650 2475 3170 2475
Wire Wire Line
	1650 2575 3170 2575
Wire Wire Line
	1650 2675 3170 2675
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 5EE5F897
P 6725 4250
F 0 "J6" H 6805 4246 50  0000 L CNN
F 1 "Conn_01x01" H 6805 4201 50  0001 L CNN
F 2 "Thruster_Controller:CAN_Pad" H 6725 4250 50  0001 C CNN
F 3 "~" H 6725 4250 50  0001 C CNN
	1    6725 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 4450 6520 4450
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 5EE66AC9
P 6720 4450
F 0 "J4" H 6800 4446 50  0000 L CNN
F 1 "Conn_01x01" H 6800 4401 50  0001 L CNN
F 2 "Thruster_Controller:CAN_Pad" H 6720 4450 50  0001 C CNN
F 3 "~" H 6720 4450 50  0001 C CNN
	1    6720 4450
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J7
U 1 1 5EED2A54
P 1125 4485
F 0 "J7" H 1290 4490 50  0000 C CNN
F 1 "Conn_01x01" H 1043 4351 50  0001 C CNN
F 2 "Thruster_Controller:Bus_Pad" H 1125 4485 50  0001 C CNN
F 3 "~" H 1125 4485 50  0001 C CNN
	1    1125 4485
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J8
U 1 1 5EED568B
P 1125 4610
F 0 "J8" H 1295 4615 50  0000 C CNN
F 1 "Conn_01x01" H 1043 4476 50  0001 C CNN
F 2 "Thruster_Controller:Bus_Pad" H 1125 4610 50  0001 C CNN
F 3 "~" H 1125 4610 50  0001 C CNN
	1    1125 4610
	-1   0    0    1   
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 5EE8919F
P 3430 4965
AR Path="/5E20A2BB/5EE8919F" Ref="#PWR?"  Part="1" 
AR Path="/5EE8919F" Ref="#PWR0104"  Part="1" 
F 0 "#PWR0104" H 3430 4815 50  0001 C CNN
F 1 "+BATT" H 3445 5138 50  0000 C CNN
F 2 "" H 3430 4965 50  0001 C CNN
F 3 "" H 3430 4965 50  0001 C CNN
	1    3430 4965
	1    0    0    -1  
$EndComp
Wire Wire Line
	3430 4965 3430 5015
$Comp
L power:GND #PWR?
U 1 1 5EE89198
P 3430 5790
AR Path="/5E20A2BB/5EE89198" Ref="#PWR?"  Part="1" 
AR Path="/5EE89198" Ref="#PWR0101"  Part="1" 
F 0 "#PWR0101" H 3430 5540 50  0001 C CNN
F 1 "GND" H 3435 5617 50  0000 C CNN
F 2 "" H 3430 5790 50  0001 C CNN
F 3 "" H 3430 5790 50  0001 C CNN
	1    3430 5790
	1    0    0    -1  
$EndComp
Wire Wire Line
	3430 5715 3430 5790
$Comp
L Device:R_US R?
U 1 1 5EE89191
P 3430 5565
AR Path="/5E20A2BB/5EE89191" Ref="R?"  Part="1" 
AR Path="/5EE89191" Ref="R7"  Part="1" 
F 0 "R7" H 3480 5565 50  0000 L CNN
F 1 "2.7k" H 3480 5490 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 3470 5555 50  0001 C CNN
F 3 "~" H 3430 5565 50  0001 C CNN
F 4 "RMCF0603JT2K70CT-ND" H 3430 5565 50  0001 C CNN "Digikey"
	1    3430 5565
	1    0    0    -1  
$EndComp
Wire Wire Line
	3430 5315 3430 5415
$Comp
L Device:LED_ALT D?
U 1 1 5EE89189
P 3430 5165
AR Path="/5E20A2BB/5EE89189" Ref="D?"  Part="1" 
AR Path="/5EE89189" Ref="D1"  Part="1" 
F 0 "D1" V 3469 5048 50  0000 R CNN
F 1 "BLUE" V 3378 5048 50  0000 R CNN
F 2 "LEDs:LED_0603" H 3430 5165 50  0001 C CNN
F 3 "~" H 3430 5165 50  0001 C CNN
F 4 "732-4966-1-ND" V 3430 5165 50  0001 C CNN "Digikey"
	1    3430 5165
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E5672C9
P 3835 5785
AR Path="/5E20A2BB/5E5672C9" Ref="#PWR?"  Part="1" 
AR Path="/5E5672C9" Ref="#PWR0109"  Part="1" 
F 0 "#PWR0109" H 3835 5535 50  0001 C CNN
F 1 "GND" H 3840 5612 50  0000 C CNN
F 2 "" H 3835 5785 50  0001 C CNN
F 3 "" H 3835 5785 50  0001 C CNN
	1    3835 5785
	1    0    0    -1  
$EndComp
Wire Wire Line
	3835 5710 3835 5785
Wire Wire Line
	3835 5310 3835 5410
Wire Wire Line
	3835 5010 3835 4960
$Comp
L Device:LED_ALT D?
U 1 1 5E5672B4
P 3835 5160
AR Path="/5E20A2BB/5E5672B4" Ref="D?"  Part="1" 
AR Path="/5E5672B4" Ref="D3"  Part="1" 
F 0 "D3" V 3874 5042 50  0000 R CNN
F 1 "GREEN" V 3783 5042 50  0000 R CNN
F 2 "LEDs:LED_0603" H 3835 5160 50  0001 C CNN
F 3 "~" H 3835 5160 50  0001 C CNN
F 4 "732-4980-1-ND" V 3835 5160 50  0001 C CNN "Digikey"
	1    3835 5160
	0    -1   -1   0   
$EndComp
Wire Wire Line
	815  1950 815  2375
Wire Wire Line
	815  2375 1150 2375
Wire Wire Line
	815  1950 3170 1950
Wire Wire Line
	725  2475 725  2050
Wire Wire Line
	725  2475 1150 2475
Wire Wire Line
	725  2050 3170 2050
$Comp
L Connector:TestPoint TP1
U 1 1 5E97E448
P 3845 4495
F 0 "TP1" H 3787 4567 50  0000 R CNN
F 1 "TestPoint" H 3787 4612 50  0001 R CNN
F 2 "Thruster_Controller:TestPoint_Pad_D1.0mm" H 4045 4495 50  0001 C CNN
F 3 "~" H 4045 4495 50  0001 C CNN
	1    3845 4495
	-1   0    0    1   
$EndComp
$Comp
L Connector:TestPoint TP3
U 1 1 5E97F5C4
P 3450 4495
F 0 "TP3" H 3392 4567 50  0000 R CNN
F 1 "TestPoint" H 3392 4612 50  0001 R CNN
F 2 "Thruster_Controller:TestPoint_Pad_D1.0mm" H 3650 4495 50  0001 C CNN
F 3 "~" H 3650 4495 50  0001 C CNN
	1    3450 4495
	-1   0    0    1   
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E98BDFE
P 3845 4435
AR Path="/5E20A2BB/5E98BDFE" Ref="#PWR?"  Part="1" 
AR Path="/5E98BDFE" Ref="#PWR0132"  Part="1" 
F 0 "#PWR0132" H 3845 4285 50  0001 C CNN
F 1 "+3V3" H 3840 4585 50  0000 C CNN
F 2 "" H 3845 4435 50  0001 C CNN
F 3 "" H 3845 4435 50  0001 C CNN
	1    3845 4435
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR?
U 1 1 5E98D34F
P 3450 4435
AR Path="/5E20A2BB/5E98D34F" Ref="#PWR?"  Part="1" 
AR Path="/5E98D34F" Ref="#PWR0140"  Part="1" 
F 0 "#PWR0140" H 3450 4285 50  0001 C CNN
F 1 "+BATT" H 3440 4585 50  0000 C CNN
F 2 "" H 3450 4435 50  0001 C CNN
F 3 "" H 3450 4435 50  0001 C CNN
	1    3450 4435
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 4435 3450 4495
Wire Wire Line
	3845 4435 3845 4495
$Comp
L power:+3V3 #PWR?
U 1 1 5E5672BA
P 3835 4960
AR Path="/5E20A2BB/5E5672BA" Ref="#PWR?"  Part="1" 
AR Path="/5E5672BA" Ref="#PWR0108"  Part="1" 
F 0 "#PWR0108" H 3835 4810 50  0001 C CNN
F 1 "+3V3" H 3850 5133 50  0000 C CNN
F 2 "" H 3835 4960 50  0001 C CNN
F 3 "" H 3835 4960 50  0001 C CNN
	1    3835 4960
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5EB91BC0
P 5700 3900
AR Path="/5E20A2BB/5EB91BC0" Ref="#PWR?"  Part="1" 
AR Path="/5EB91BC0" Ref="#PWR0102"  Part="1" 
F 0 "#PWR0102" H 5700 3750 50  0001 C CNN
F 1 "+3V3" H 5700 4050 50  0000 C CNN
F 2 "" H 5700 3900 50  0001 C CNN
F 3 "" H 5700 3900 50  0001 C CNN
	1    5700 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3900 5700 3950
Wire Wire Line
	6120 3000 7175 3000
Wire Wire Line
	6120 3075 7175 3075
Wire Wire Line
	6120 3150 7175 3150
$Comp
L Thruster_Controller:NCP718AMT330 U1
U 1 1 5EBF94AC
P 1675 5750
F 0 "U1" H 1700 6050 50  0000 C CNN
F 1 "NCP718AMT330" H 1350 5200 50  0000 C CNN
F 2 "Housings_DFN_QFN:DFN-6-1EP_2x2mm_Pitch0.65mm" H 1675 5600 50  0001 C CNN
F 3 "https://www.mouser.ca/datasheet/2/308/NCP718-D-1224359.pdf" H 1625 5400 50  0001 C CNN
F 4 "" H 1675 5750 50  0001 C CNN "Digikey"
F 5 "863-NCP718AMT330TBG" H 1675 5750 50  0001 C CNN "Mouser"
	1    1675 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5EBFB8C1
P 2250 5775
AR Path="/5E20A2BB/5EBFB8C1" Ref="C?"  Part="1" 
AR Path="/5EBFB8C1" Ref="C4"  Part="1" 
F 0 "C4" H 2100 5850 50  0000 L CNN
F 1 "1uF" H 2100 5700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 2250 5775 50  0001 C CNN
F 3 "" H 2250 5775 50  0001 C CNN
F 4 "1276-1102-1-ND" H 2250 5775 50  0001 C CNN "Digikey"
	1    2250 5775
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5675 2250 5600
Wire Wire Line
	2250 5600 2075 5600
Wire Wire Line
	2250 5875 2250 5975
$Comp
L power:GND #PWR?
U 1 1 5EC003D4
P 2250 5975
AR Path="/5E20A2BB/5EC003D4" Ref="#PWR?"  Part="1" 
AR Path="/5EC003D4" Ref="#PWR0103"  Part="1" 
F 0 "#PWR0103" H 2250 5725 50  0001 C CNN
F 1 "GND" H 2255 5802 50  0000 C CNN
F 2 "" H 2250 5975 50  0001 C CNN
F 3 "" H 2250 5975 50  0001 C CNN
	1    2250 5975
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5EC008C6
P 2250 5450
AR Path="/5E20A2BB/5EC008C6" Ref="#PWR?"  Part="1" 
AR Path="/5EC008C6" Ref="#PWR0106"  Part="1" 
F 0 "#PWR0106" H 2250 5300 50  0001 C CNN
F 1 "+3V3" H 2265 5623 50  0000 C CNN
F 2 "" H 2250 5450 50  0001 C CNN
F 3 "" H 2250 5450 50  0001 C CNN
	1    2250 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5450 2250 5600
Connection ~ 2250 5600
Wire Wire Line
	1325 5600 1275 5600
Wire Wire Line
	1275 5600 1275 5800
Wire Wire Line
	1275 5800 1325 5800
$Comp
L power:+BATT #PWR0107
U 1 1 5EC056DF
P 1050 5450
F 0 "#PWR0107" H 1050 5300 50  0001 C CNN
F 1 "+BATT" H 1065 5623 50  0000 C CNN
F 2 "" H 1050 5450 50  0001 C CNN
F 3 "" H 1050 5450 50  0001 C CNN
	1    1050 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 5450 1050 5600
Wire Wire Line
	1050 5600 1275 5600
Connection ~ 1275 5600
Wire Wire Line
	1675 6200 1725 6200
Wire Wire Line
	1725 6200 1725 6350
Connection ~ 1725 6200
Wire Wire Line
	1725 6200 1775 6200
$Comp
L power:GND #PWR?
U 1 1 5EC10263
P 1725 6350
AR Path="/5E20A2BB/5EC10263" Ref="#PWR?"  Part="1" 
AR Path="/5EC10263" Ref="#PWR0112"  Part="1" 
F 0 "#PWR0112" H 1725 6100 50  0001 C CNN
F 1 "GND" H 1730 6177 50  0000 C CNN
F 2 "" H 1725 6350 50  0001 C CNN
F 3 "" H 1725 6350 50  0001 C CNN
	1    1725 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5EC10B5C
P 1050 5775
AR Path="/5E20A2BB/5EC10B5C" Ref="C?"  Part="1" 
AR Path="/5EC10B5C" Ref="C3"  Part="1" 
F 0 "C3" H 875 5850 50  0000 L CNN
F 1 "1uF" H 825 5700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 1050 5775 50  0001 C CNN
F 3 "" H 1050 5775 50  0001 C CNN
F 4 "1276-6470-1-ND" H 1050 5775 50  0001 C CNN "Digikey"
F 5 "" H 1050 5775 50  0001 C CNN "Mouser"
F 6 "50V" H 900 5775 50  0000 C CNN "Voltage"
	1    1050 5775
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 5600 1050 5675
Connection ~ 1050 5600
Wire Wire Line
	1050 5875 1050 5975
$Comp
L power:GND #PWR?
U 1 1 5EC1731A
P 1050 5975
AR Path="/5E20A2BB/5EC1731A" Ref="#PWR?"  Part="1" 
AR Path="/5EC1731A" Ref="#PWR0113"  Part="1" 
F 0 "#PWR0113" H 1050 5725 50  0001 C CNN
F 1 "GND" H 1055 5802 50  0000 C CNN
F 2 "" H 1050 5975 50  0001 C CNN
F 3 "" H 1050 5975 50  0001 C CNN
	1    1050 5975
	1    0    0    -1  
$EndComp
Wire Wire Line
	1575 6150 1575 6200
Wire Wire Line
	1575 6200 1675 6200
Wire Wire Line
	1675 6150 1675 6200
Connection ~ 1675 6200
Wire Wire Line
	1775 6150 1775 6200
Wire Wire Line
	1775 6200 1875 6200
Wire Wire Line
	1875 6200 1875 6150
Connection ~ 1775 6200
$Comp
L Device:R_US R?
U 1 1 5E5672C2
P 3835 5560
AR Path="/5E20A2BB/5E5672C2" Ref="R?"  Part="1" 
AR Path="/5E5672C2" Ref="R19"  Part="1" 
F 0 "R19" H 3885 5560 50  0000 L CNN
F 1 "330" H 3885 5485 50  0000 L CNN
F 2 "Resistors_SMD:R_0603" V 3875 5550 50  0001 C CNN
F 3 "~" H 3835 5560 50  0001 C CNN
F 4 "RMCF0603JT330RCT-ND" H 3835 5560 50  0001 C CNN "Digikey"
	1    3835 5560
	1    0    0    -1  
$EndComp
$EndSCHEMATC
