EESchema Schematic File Version 2
LIBS:capacitors
LIBS:disc
LIBS:ics
LIBS:mech
LIBS:resistors_us
LIBS:various
LIBS:ac-dc
LIBS:adc-dac
LIBS:Altera
LIBS:analog_devices
LIBS:analog_switches
LIBS:atmel
LIBS:audio
LIBS:Battery_Management
LIBS:bbd
LIBS:Bosch
LIBS:brooktre
LIBS:Connector
LIBS:contrib
LIBS:cypress
LIBS:dc-dc
LIBS:Decawave
LIBS:device
LIBS:digital-audio
LIBS:Diode
LIBS:Display
LIBS:driver_gate
LIBS:dsp
LIBS:DSP_Microchip_DSPIC33
LIBS:elec-unifil
LIBS:ESD_Protection
LIBS:Espressif
LIBS:FPGA_Actel
LIBS:ftdi
LIBS:gennum
LIBS:Graphic
LIBS:hc11
LIBS:infineon
LIBS:intel
LIBS:interface
LIBS:intersil
LIBS:ir
LIBS:Lattice
LIBS:LED
LIBS:LEM
LIBS:linear
LIBS:Logic_74xgxx
LIBS:Logic_74xx
LIBS:Logic_CMOS_4000
LIBS:Logic_CMOS_IEEE
LIBS:logic_programmable
LIBS:Logic_TTL_IEEE
LIBS:maxim
LIBS:MCU_Microchip_PIC10
LIBS:MCU_Microchip_PIC12
LIBS:MCU_Microchip_PIC16
LIBS:MCU_Microchip_PIC18
LIBS:MCU_Microchip_PIC24
LIBS:MCU_Microchip_PIC32
LIBS:MCU_NXP_Kinetis
LIBS:MCU_NXP_LPC
LIBS:MCU_NXP_S08
LIBS:MCU_Parallax
LIBS:MCU_ST_STM8
LIBS:MCU_ST_STM32
LIBS:MCU_Texas_MSP430
LIBS:Mechanical
LIBS:memory
LIBS:microchip
LIBS:microcontrollers
LIBS:modules
LIBS:Motor
LIBS:motor_drivers
LIBS:motorola
LIBS:nordicsemi
LIBS:nxp
LIBS:onsemi
LIBS:opto
LIBS:Oscillators
LIBS:philips
LIBS:power
LIBS:Power_Management
LIBS:powerint
LIBS:pspice
LIBS:references
LIBS:regul
LIBS:Relay
LIBS:RF_Bluetooth
LIBS:rfcom
LIBS:RFSolutions
LIBS:Sensor_Current
LIBS:Sensor_Humidity
LIBS:sensors
LIBS:silabs
LIBS:siliconi
LIBS:supertex
LIBS:Switch
LIBS:texas
LIBS:Transformer
LIBS:Transistor
LIBS:triac_thyristor
LIBS:Valve
LIBS:video
LIBS:wiznet
LIBS:Worldsemi
LIBS:Xicor
LIBS:xilinx
LIBS:xilinx-artix7
LIBS:xilinx-kintex7
LIBS:xilinx-spartan6
LIBS:xilinx-virtex5
LIBS:xilinx-virtex6
LIBS:xilinx-virtex7
LIBS:zetex
LIBS:Zilog
LIBS:controller-cache
EELAYER 25 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 2
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
L Q_TRIAC_GAA D2
U 1 1 5B0094B6
P 11700 2350
F 0 "D2" H 11825 2375 50  0000 L CNN
F 1 "BTA16-800" H 11825 2300 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-220-3_Horizontal" V 11775 2375 50  0001 C CNN
F 3 "" V 11700 2350 50  0001 C CNN
	1    11700 2350
	1    0    0    -1  
$EndComp
$Comp
L C C16
U 1 1 5B00963D
P 12050 2100
F 0 "C16" V 12200 2200 50  0000 L CNN
F 1 "47nF/630V" V 12200 1750 50  0000 L CNN
F 2 "Capacitors_THT:C_Axial_L5.1mm_D3.1mm_P10.00mm_Horizontal" H 12088 1950 50  0001 C CNN
F 3 "" H 12050 2100 50  0001 C CNN
	1    12050 2100
	0    -1   -1   0   
$EndComp
$Comp
L R R23
U 1 1 5B0099AC
P 12050 2650
F 0 "R23" V 12150 2650 50  0000 C CNN
F 1 "100/2W" V 11950 2650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0411_L9.9mm_D3.6mm_P15.24mm_Horizontal" V 11980 2650 50  0001 C CNN
F 3 "" H 12050 2650 50  0001 C CNN
	1    12050 2650
	0    -1   -1   0   
$EndComp
$Comp
L MOC3083M U4
U 1 1 5B009B37
P 10750 2350
F 0 "U4" H 10500 2550 50  0000 L CNN
F 1 "MOC3083M" H 10650 2550 50  0000 L CNN
F 2 "Housings_DIP:DIP-6_W7.62mm_SMDSocket_SmallPads" H 10550 2150 50  0001 L CIN
F 3 "" H 10715 2350 50  0001 L CNN
	1    10750 2350
	1    0    0    -1  
$EndComp
$Comp
L R R22
U 1 1 5B009D4D
P 11450 2100
F 0 "R22" V 11550 2100 50  0000 C CNN
F 1 "1k" V 11450 2100 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 11380 2100 50  0001 C CNN
F 3 "" H 11450 2100 50  0001 C CNN
	1    11450 2100
	0    -1   -1   0   
$EndComp
Text HLabel 13450 1800 2    60   Output ~ 0
P1
$Comp
L R R21
U 1 1 5B00A3A7
P 10200 2250
F 0 "R21" V 10300 2250 50  0000 C CNN
F 1 "2k" V 10200 2250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 10130 2250 50  0001 C CNN
F 3 "" H 10200 2250 50  0001 C CNN
	1    10200 2250
	0    -1   -1   0   
$EndComp
Text HLabel 9950 2250 0    60   Input ~ 0
12VDC
$Comp
L Thermistor_NTC TH1
U 1 1 5B00C3A7
P 1800 4500
F 0 "TH1" V 1625 4500 50  0001 C CNN
F 1 "NTC10K" V 1925 4500 50  0000 C CNN
F 2 "" H 1800 4550 50  0001 C CNN
F 3 "" H 1800 4550 50  0001 C CNN
	1    1800 4500
	0    -1   -1   0   
$EndComp
$Comp
L Screw_Terminal_01x02 T2
U 1 1 5B00C67C
P 2150 4450
F 0 "T2" H 2150 4550 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 2150 4250 50  0001 C CNN
F 2 "" H 2150 4450 50  0001 C CNN
F 3 "" H 2150 4450 50  0001 C CNN
	1    2150 4450
	-1   0    0    -1  
$EndComp
Text HLabel 2450 4450 2    60   Input ~ 0
3VDC
$Comp
L Thermistor_NTC TH2
U 1 1 5B00D655
P 1800 4850
F 0 "TH2" V 1625 4850 50  0001 C CNN
F 1 "NTC10K" V 1925 4850 50  0000 C CNN
F 2 "" H 1800 4900 50  0001 C CNN
F 3 "" H 1800 4900 50  0001 C CNN
	1    1800 4850
	0    -1   -1   0   
$EndComp
$Comp
L Screw_Terminal_01x02 T3
U 1 1 5B00D65B
P 2150 4800
F 0 "T3" H 2150 4900 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 2150 4600 50  0001 C CNN
F 2 "" H 2150 4800 50  0001 C CNN
F 3 "" H 2150 4800 50  0001 C CNN
	1    2150 4800
	-1   0    0    -1  
$EndComp
Text HLabel 2450 4800 2    60   Input ~ 0
3VDC
$Comp
L Thermistor_NTC TH3
U 1 1 5B00DB10
P 1800 5200
F 0 "TH3" V 1625 5200 50  0001 C CNN
F 1 "NTC10K" V 1925 5200 50  0000 C CNN
F 2 "" H 1800 5250 50  0001 C CNN
F 3 "" H 1800 5250 50  0001 C CNN
	1    1800 5200
	0    -1   -1   0   
$EndComp
$Comp
L Screw_Terminal_01x02 T4
U 1 1 5B00DB16
P 2150 5150
F 0 "T4" H 2150 5250 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 2150 4950 50  0001 C CNN
F 2 "" H 2150 5150 50  0001 C CNN
F 3 "" H 2150 5150 50  0001 C CNN
	1    2150 5150
	-1   0    0    -1  
$EndComp
Text HLabel 2450 5150 2    60   Input ~ 0
3VDC
$Comp
L Thermistor_NTC TH5
U 1 1 5B00ED87
P 1800 4150
F 0 "TH5" V 1625 4150 50  0001 C CNN
F 1 "PTC1000" V 1925 4150 50  0000 C CNN
F 2 "" H 1800 4200 50  0001 C CNN
F 3 "" H 1800 4200 50  0001 C CNN
	1    1800 4150
	0    -1   -1   0   
$EndComp
$Comp
L Screw_Terminal_01x02 T1
U 1 1 5B00ED8D
P 2150 4100
F 0 "T1" H 2150 4200 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 2150 3900 50  0001 C CNN
F 2 "" H 2150 4100 50  0001 C CNN
F 3 "" H 2150 4100 50  0001 C CNN
	1    2150 4100
	-1   0    0    -1  
$EndComp
$Comp
L Thermistor_NTC TH4
U 1 1 5B01161F
P 1800 3800
F 0 "TH4" V 1625 3800 50  0001 C CNN
F 1 "PTC1000" V 1925 3800 50  0000 C CNN
F 2 "" H 1800 3850 50  0001 C CNN
F 3 "" H 1800 3850 50  0001 C CNN
	1    1800 3800
	0    -1   -1   0   
$EndComp
$Comp
L Screw_Terminal_01x02 T0
U 1 1 5B011625
P 2150 3750
F 0 "T0" H 2150 3850 50  0000 C CNN
F 1 "Screw_Terminal_01x02" H 2150 3550 50  0001 C CNN
F 2 "" H 2150 3750 50  0001 C CNN
F 3 "" H 2150 3750 50  0001 C CNN
	1    2150 3750
	-1   0    0    -1  
$EndComp
$Comp
L FINDER-32.21-x300 K1
U 1 1 5B029A8D
P 12000 3600
F 0 "K1" H 12450 3750 50  0000 L CNN
F 1 "P2" H 12450 3650 50  0000 L CNN
F 2 "Relays_THT:Relay_SPST_Finder_32.21-x300" H 13270 3570 50  0001 C CNN
F 3 "" H 12000 3600 50  0001 C CNN
	1    12000 3600
	1    0    0    -1  
$EndComp
$Comp
L FINDER-32.21-x300 K2
U 1 1 5B029C3E
P 12000 4600
F 0 "K2" H 12450 4750 50  0000 L CNN
F 1 "R1" H 12450 4650 50  0000 L CNN
F 2 "Relays_THT:Relay_SPST_Finder_32.21-x300" H 13270 4570 50  0001 C CNN
F 3 "" H 12000 4600 50  0001 C CNN
	1    12000 4600
	1    0    0    -1  
$EndComp
Text HLabel 11700 3200 0    60   Input ~ 0
12VDC
Text HLabel 11700 4200 0    60   Input ~ 0
12VDC
Text Label 11350 4000 0    60   ~ 0
REL_P2
Text Label 11350 5000 0    60   ~ 0
REL_R1
Text HLabel 13450 3200 2    60   Output ~ 0
P2
Text HLabel 13450 4200 2    60   Output ~ 0
R1
$Comp
L FINDER-30.22 K3
U 1 1 5B02C658
P 12000 6000
F 0 "K3" H 12850 6150 50  0000 L CNN
F 1 "HEA" H 12850 6050 50  0000 L CNN
F 2 "Relays_THT:Relay_DPDT_Finder_40.52" H 13550 5970 50  0001 C CNN
F 3 "" H 12000 6000 50  0001 C CNN
	1    12000 6000
	1    0    0    -1  
$EndComp
$Comp
L FINDER-30.22 K4
U 1 1 5B02C847
P 12000 7200
F 0 "K4" H 12850 7350 50  0000 L CNN
F 1 "HEB" H 12850 7250 50  0000 L CNN
F 2 "Relays_THT:Relay_DPDT_Finder_40.52" H 13550 7170 50  0001 C CNN
F 3 "" H 12000 7200 50  0001 C CNN
	1    12000 7200
	1    0    0    -1  
$EndComp
Text HLabel 13450 5600 2    60   Output ~ 0
HE1A
Text HLabel 13450 6800 2    60   Output ~ 0
HE1B
Text HLabel 13450 6400 2    60   Input ~ 0
VACN
Text HLabel 13450 7600 2    60   Input ~ 0
VACL
Text HLabel 11700 5600 0    60   Input ~ 0
12VDC
Text HLabel 11700 6800 0    60   Input ~ 0
12VDC
Text Label 11350 6400 0    60   ~ 0
REL_HEA
Text Label 11350 7600 0    60   ~ 0
REL_HEB
$Comp
L SW_Push SW1
U 1 1 5BB5CC5D
P 2650 2800
F 0 "SW1" H 2450 2900 50  0000 L CNN
F 1 "RESET" H 2800 2900 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_FSMSM" H 2650 3000 50  0001 C CNN
F 3 "" H 2650 3000 50  0001 C CNN
	1    2650 2800
	1    0    0    -1  
$EndComp
$Comp
L SW_Push SW2
U 1 1 5BB5E27D
P 2650 3100
F 0 "SW2" H 2450 3200 50  0000 L CNN
F 1 "BOOT" H 2800 3200 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_FSMSM" H 2650 3300 50  0001 C CNN
F 3 "" H 2650 3300 50  0001 C CNN
	1    2650 3100
	1    0    0    -1  
$EndComp
$Comp
L FuseHolder_FSH F1
U 1 1 5BB63EDF
P 12650 2950
F 0 "F1" V 12550 2850 50  0000 C CNN
F 1 "6.3A" V 12550 3050 50  0000 C CNN
F 2 "mech:Fuseholder5x20_horiz_SemiClosed_Casing10x25mm" H 14100 2800 50  0001 C CNN
F 3 "" H 12730 2950 50  0001 C CNN
F 4 "ZEL005" H 12650 2650 60  0001 C CNN "ventcode"
F 5 "-" V 12750 2950 60  0001 C CNN "Nominalas"
F 6 "0" H 13950 2900 60  0001 C CNN "Kaina"
F 7 "-" H 13050 2900 60  0001 C CNN "Korpusas"
F 8 "13" H 13750 2900 60  0001 C CNN "Aukstis"
F 9 "-" H 13150 2900 60  0001 C CNN "Marke"
F 10 "Saugiklio laikiklis" H 13450 3000 60  0001 C CNN "Pavadinimas"
F 11 "-" H 13250 2900 60  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 13350 2900 60  0001 C CNN "Nuoroda"
F 13 "DIP" H 14500 3000 60  0001 C CNN "Korpuso tipas"
F 14 "-" H 13450 2900 60  0001 C CNN "Parametras"
F 15 "-" H 13550 2900 60  0001 C CNN "Komentaras"
	1    12650 2950
	0    1    1    0   
$EndComp
Text HLabel 13450 2950 2    60   Input ~ 0
VACL
$Comp
L MSTBA_3C_5.08_Angled J2
U 1 1 5BB6F407
P 13150 9300
F 0 "J2" H 13250 9400 50  0000 C CNN
F 1 "MSTBA_3C_5.08_Angled" H 14020 9290 50  0001 C CNN
F 2 "mech:PhoenixContact_MSTBA-G_03x5.08mm_Angled" H 14520 9090 50  0001 C CNN
F 3 "" H 13350 9400 50  0001 C CNN
F 4 "ZAJ372" H 13250 9000 60  0001 C CNN "ventcode"
F 5 "-" H 13250 9000 60  0001 C CNN "Nominalas"
F 6 "0" H 14520 9190 60  0001 C CNN "Kaina"
F 7 "-" H 13570 9190 60  0001 C CNN "Korpusas"
F 8 "16" H 14720 9190 60  0001 C CNN "Aukstis"
F 9 "-" H 13670 9190 60  0001 C CNN "Marke"
F 10 "Kištukas" H 14770 9290 60  0001 C CNN "Pavadinimas"
F 11 "-" H 13770 9190 60  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 13870 9190 60  0001 C CNN "Nuoroda"
F 13 "DIP" H 14320 9190 60  0001 C CNN "Korpuso tipas"
F 14 "-" H 13970 9190 60  0001 C CNN "Parametras"
F 15 "-" H 14070 9190 60  0001 C CNN "Komentaras"
	1    13150 9300
	1    0    0    -1  
$EndComp
Text Label 12100 9400 0    60   ~ 0
VACN
Text Label 12100 9500 0    60   ~ 0
VACL
Text Label 12700 9300 0    60   ~ 0
PE
Wire Wire Line
	2450 4100 2350 4100
Wire Wire Line
	2350 4200 3100 4200
Wire Wire Line
	2450 5150 2350 5150
Wire Wire Line
	2450 4800 2350 4800
Wire Wire Line
	2450 4450 2350 4450
Wire Wire Line
	9950 2250 10050 2250
Wire Wire Line
	10450 2250 10350 2250
Wire Wire Line
	10950 2950 11700 2950
Wire Wire Line
	11700 2950 12450 2950
Wire Wire Line
	11700 1800 13450 1800
Connection ~ 11700 2650
Wire Wire Line
	11200 2250 11050 2250
Wire Wire Line
	11200 2100 11200 2250
Wire Wire Line
	11300 2100 11200 2100
Connection ~ 11700 2100
Wire Wire Line
	11050 2450 11550 2450
Wire Wire Line
	12350 2650 12200 2650
Wire Wire Line
	12350 2100 12350 2650
Wire Wire Line
	12200 2100 12350 2100
Wire Wire Line
	11700 2500 11700 2650
Wire Wire Line
	11700 2650 11700 2950
Wire Wire Line
	11700 1800 11700 2100
Wire Wire Line
	11700 2100 11700 2200
Wire Wire Line
	11600 2100 11700 2100
Wire Wire Line
	11700 2100 11900 2100
Wire Wire Line
	2450 3750 2350 3750
Wire Wire Line
	2350 3850 3100 3850
Wire Wire Line
	11700 3200 11800 3200
Wire Wire Line
	11800 3200 11800 3300
Wire Wire Line
	11700 4200 11800 4200
Wire Wire Line
	11800 4200 11800 4300
Wire Wire Line
	11800 3900 11800 4000
Wire Wire Line
	11800 4000 11350 4000
Wire Wire Line
	11800 4900 11800 5000
Wire Wire Line
	11800 5000 11350 5000
Wire Wire Line
	12200 4000 13100 4000
Wire Wire Line
	12200 4000 12200 3900
Wire Wire Line
	13100 5000 12200 5000
Wire Wire Line
	12200 5000 12200 4900
Wire Wire Line
	12300 4300 12300 4200
Wire Wire Line
	12300 4200 13450 4200
Wire Wire Line
	12300 3300 12300 3200
Wire Wire Line
	12300 3200 13450 3200
Wire Wire Line
	12300 5700 12300 5600
Wire Wire Line
	12300 5600 12700 5600
Wire Wire Line
	12700 5600 13450 5600
Wire Wire Line
	12700 5600 12700 5700
Wire Wire Line
	12300 6900 12300 6800
Wire Wire Line
	12300 6800 12700 6800
Wire Wire Line
	12700 6800 13450 6800
Wire Wire Line
	12700 6800 12700 6900
Wire Wire Line
	12200 6300 12200 6400
Wire Wire Line
	12200 6400 12600 6400
Wire Wire Line
	12600 6400 13450 6400
Wire Wire Line
	12600 6400 12600 6300
Wire Wire Line
	12200 7500 12200 7600
Wire Wire Line
	12600 7600 12600 7500
Connection ~ 12700 5600
Connection ~ 12700 6800
Connection ~ 12600 7600
Connection ~ 12600 6400
Wire Wire Line
	11700 5600 11800 5600
Wire Wire Line
	11800 5600 11800 5700
Wire Wire Line
	11700 6800 11800 6800
Wire Wire Line
	11800 6800 11800 6900
Wire Wire Line
	11800 6300 11800 6400
Wire Wire Line
	11800 6400 11350 6400
Wire Wire Line
	11800 7500 11800 7600
Wire Wire Line
	11800 7600 11350 7600
Connection ~ 13100 5000
Wire Wire Line
	11900 2650 11700 2650
Wire Wire Line
	13100 4000 13100 5000
Wire Wire Line
	13100 5000 13100 7600
Wire Wire Line
	12200 7600 12600 7600
Wire Wire Line
	12600 7600 13100 7600
Wire Wire Line
	13100 7600 13450 7600
Connection ~ 13100 7600
Wire Wire Line
	12850 2950 13450 2950
Wire Wire Line
	12100 9500 13100 9500
Wire Wire Line
	13100 9500 13150 9500
Wire Wire Line
	12100 9400 12400 9400
Wire Wire Line
	12400 9400 13150 9400
Wire Wire Line
	13150 9300 12700 9300
Connection ~ 11700 2950
Text Label 10950 2950 0    60   ~ 0
AC1
$Comp
L MOLEX_KK_X04 J1
U 1 1 5BB78493
P 2150 5850
F 0 "J1" H 2150 5600 50  0000 C CNN
F 1 "IIC" V 2250 5850 50  0000 C CNN
F 2 "mech:Molex_KK-6410-04_04x2.54mm_Straight" H 3400 5700 50  0001 C CNN
F 3 "" H 2150 6100 50  0001 C CNN
F 4 "ventcode" H 2100 5500 60  0001 C CNN "ventcode"
F 5 "-" H 2150 5600 60  0001 C CNN "Nominalas"
F 6 "-" H 2550 5800 60  0001 C CNN "Kaina"
F 7 "-" H 2650 5800 60  0001 C CNN "Korpusas"
F 8 "12" H 3350 5800 60  0001 C CNN "Aukstis"
F 9 "-" H 2750 5800 60  0001 C CNN "Marke"
F 10 "Jungtis" H 2700 5900 60  0001 C CNN "Pavadinimas"
F 11 "-" H 2850 5800 60  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 2950 5800 60  0001 C CNN "Nuoroda"
F 13 "DIP" H 3650 5900 60  0001 C CNN "Korpuso tipas"
F 14 "-" H 3050 5800 60  0001 C CNN "Parametras"
F 15 "-" H 3150 5800 60  0001 C CNN "Komentaras"
	1    2150 5850
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 6000 2700 6000
Wire Wire Line
	2350 5700 2700 5700
Wire Wire Line
	2350 5900 3100 5900
Wire Wire Line
	2350 5800 3100 5800
Text Label 2700 6000 2    60   ~ 0
COM
Text Label 2700 5700 2    60   ~ 0
5VDC
$Comp
L VAR_VE13M00275K RV1
U 1 1 5BB7C2D5
P 12750 9050
F 0 "RV1" V 12600 8800 50  0000 C CNN
F 1 "VE13M00275K" V 12600 9250 50  0000 C CNN
F 2 "disc:VR_VE13M00275K" H 13550 8900 50  0001 C CNN
F 3 "" V 12875 9050 50  0001 C CNN
F 4 "ZELR034" H 12750 8750 60  0001 C CNN "ventcode"
F 5 "0" H 13200 9050 60  0001 C CNN "Kaina"
F 6 "-" H 13200 8950 60  0001 C CNN "Korpusas"
F 7 "15" H 13400 9050 60  0001 C CNN "Aukstis"
F 8 "Varistorius" H 13400 9150 60  0001 C CNN "Pavadinimas"
F 9 "-" H 13300 8950 60  0001 C CNN "Gamintojo Kodas"
F 10 "https://elektronik-lavpris.dk/p85171/ve13m00275k-varistor-275v-430v-1250a/" H 15050 8800 60  0001 C CNN "Nuoroda"
F 11 "DIP" H 13800 9150 60  0001 C CNN "Korpuso tipas"
F 12 "-" H 13500 8950 60  0001 C CNN "Parametras"
F 13 "-" H 13600 8950 60  0001 C CNN "Komentaras"
	1    12750 9050
	0    1    1    0   
$EndComp
Wire Wire Line
	12600 9050 12400 9050
Wire Wire Line
	12400 9050 12400 9400
Connection ~ 12400 9400
Wire Wire Line
	12900 9050 13100 9050
Wire Wire Line
	13100 9050 13100 9500
Connection ~ 13100 9500
$Comp
L MOLEX_KK_X03 J3
U 1 1 5BB7DCD3
P 2150 6450
F 0 "J3" H 2150 6250 50  0000 C CNN
F 1 "UART" V 2250 6450 50  0000 C CNN
F 2 "mech:Molex_KK-6410-03_03x2.54mm_Straight" H 3500 6400 50  0001 C CNN
F 3 "" H 2150 6650 50  0001 C CNN
F 4 "ZAJ214" H 2150 6250 60  0001 C CNN "ventcode"
F 5 "-" H 3100 6600 60  0001 C CNN "Nominalas"
F 6 "-" H 2700 6500 60  0001 C CNN "Kaina"
F 7 "-" H 2800 6500 60  0001 C CNN "Korpusas"
F 8 "12" H 3500 6500 60  0001 C CNN "Aukstis"
F 9 "-" H 2900 6500 60  0001 C CNN "Marke"
F 10 "Jungtis" H 2750 6600 60  0001 C CNN "Pavadinimas"
F 11 "-" H 3000 6500 60  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 3100 6500 60  0001 C CNN "Nuoroda"
F 13 "DIP" H 3750 6600 60  0001 C CNN "Korpuso tipas"
F 14 "-" H 3200 6500 60  0001 C CNN "Parametras"
F 15 "-" H 3300 6500 60  0001 C CNN "Komentaras"
	1    2150 6450
	-1   0    0    1   
$EndComp
Wire Wire Line
	2350 6550 2700 6550
Wire Wire Line
	2350 6450 3100 6450
Wire Wire Line
	2350 6350 2700 6350
Text Label 2700 6550 2    60   ~ 0
COM
Text Label 2700 6350 2    60   ~ 0
12VDC
$Sheet
S 3100 2250 1650 4450
U 5BB80B90
F0 "Control" 60
F1 "control.sch" 60
F2 "DATA" I L 3100 6450 60 
F3 "T0_IN" I L 3100 3850 60 
F4 "T1_IN" I L 3100 4200 60 
F5 "T2_IN" I L 3100 4550 60 
F6 "T3_IN" I L 3100 4900 60 
F7 "T4_IN" I L 3100 5250 60 
F8 "REL_P1" O R 4750 2700 60 
F9 "REL_P2" O R 4750 2850 60 
F10 "REL_R1" O R 4750 3000 60 
F11 "REL_HEA" O R 4750 3150 60 
F12 "REL_HEB" O R 4750 3300 60 
F13 "~nRST" I L 3100 2800 60 
F14 "BOOT" I L 3100 3100 60 
F15 "12VDC" I L 3100 2450 60 
F16 "SCL" O L 3100 5800 60 
F17 "SDA" B L 3100 5900 60 
F18 "TSC1" I R 4750 4000 60 
F19 "TSC2" I R 4750 4100 60 
F20 "TSC3" I R 4750 4200 60 
F21 "TSC4" I R 4750 4300 60 
F22 "ADC_EX1" I R 4750 4550 60 
F23 "ADC_EX2" I R 4750 4650 60 
$EndSheet
Wire Wire Line
	10450 2450 9900 2450
Text HLabel 2450 4100 2    60   Input ~ 0
3VDC
Text HLabel 2450 3750 2    60   Input ~ 0
3VDC
Wire Wire Line
	2350 4550 3100 4550
Wire Wire Line
	2350 4900 3100 4900
Wire Wire Line
	2350 5250 3100 5250
Wire Wire Line
	2450 2800 2100 2800
Text Label 2100 2800 0    60   ~ 0
COM
Wire Wire Line
	2450 3100 2100 3100
Text Label 2100 3100 0    60   ~ 0
3VDC
Wire Wire Line
	2850 3100 3100 3100
Wire Wire Line
	2850 2800 3100 2800
Text Label 9900 2450 0    60   ~ 0
REL_P1
Wire Wire Line
	4750 2700 5400 2700
Wire Wire Line
	4750 2850 5400 2850
Wire Wire Line
	4750 3000 5400 3000
Wire Wire Line
	4750 3150 5400 3150
Wire Wire Line
	4750 3150 4750 3300
Wire Wire Line
	4750 3300 5400 3300
Text Label 5400 2700 2    60   ~ 0
REL_P1
Text Label 5400 2850 2    60   ~ 0
REL_P2
Text Label 5400 3000 2    60   ~ 0
REL_R1
Text Label 5400 3150 2    60   ~ 0
REL_HEA
Text Label 5400 3300 2    60   ~ 0
REL_HEB
$EndSCHEMATC
