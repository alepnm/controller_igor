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
Sheet 2 2
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
L R R11
U 1 1 5BB81255
P 3300 8650
F 0 "R11" V 3400 8650 50  0000 C CNN
F 1 "1k" V 3300 8650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3230 8650 50  0001 C CNN
F 3 "" H 3300 8650 50  0001 C CNN
	1    3300 8650
	0    1    -1   0   
$EndComp
$Comp
L 1N4148 D1
U 1 1 5BB81256
P 3650 8650
F 0 "D1" H 3750 8450 50  0000 C CNN
F 1 "BAS85" H 3650 8550 50  0000 C CNN
F 2 "Diodes_SMD:D_MiniMELF_Handsoldering" H 3650 8475 50  0001 C CNN
F 3 "" H 3650 8650 50  0001 C CNN
	1    3650 8650
	1    0    0    1   
$EndComp
$Comp
L R R18
U 1 1 5BB81257
P 4150 8650
F 0 "R18" V 4250 8650 50  0000 C CNN
F 1 "10k" V 4150 8650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4080 8650 50  0001 C CNN
F 3 "" H 4150 8650 50  0001 C CNN
	1    4150 8650
	0    1    -1   0   
$EndComp
$Comp
L R R15
U 1 1 5BB81258
P 3450 8950
F 0 "R15" V 3550 8950 50  0000 C CNN
F 1 "100" V 3450 8950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3380 8950 50  0001 C CNN
F 3 "" H 3450 8950 50  0001 C CNN
	1    3450 8950
	0    1    -1   0   
$EndComp
$Comp
L R R24
U 1 1 5BB81259
P 4650 9250
F 0 "R24" V 4750 9250 50  0000 C CNN
F 1 "5.1k" V 4650 9250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4580 9250 50  0001 C CNN
F 3 "" H 4650 9250 50  0001 C CNN
	1    4650 9250
	0    1    -1   0   
$EndComp
$Comp
L C C2
U 1 1 5BB8125A
P 2300 9250
F 0 "C2" H 2325 9350 50  0000 L CNN
F 1 "1nF" H 2325 9150 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2338 9100 50  0001 C CNN
F 3 "" H 2300 9250 50  0001 C CNN
	1    2300 9250
	-1   0    0    -1  
$EndComp
$Comp
L R R17
U 1 1 5BB8125B
P 4150 8350
F 0 "R17" V 4250 8350 50  0000 C CNN
F 1 "1k" V 4150 8350 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4080 8350 50  0001 C CNN
F 3 "" H 4150 8350 50  0001 C CNN
	1    4150 8350
	0    1    -1   0   
$EndComp
$Comp
L C C15
U 1 1 5BB8125C
P 4400 8950
F 0 "C15" H 4425 9050 50  0000 L CNN
F 1 "100nF" H 4425 8850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4438 8800 50  0001 C CNN
F 3 "" H 4400 8950 50  0001 C CNN
	1    4400 8950
	-1   0    0    -1  
$EndComp
$Comp
L R R16
U 1 1 5BB8125D
P 4150 8100
F 0 "R16" V 4250 8100 50  0000 C CNN
F 1 "10k" V 4150 8100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4080 8100 50  0001 C CNN
F 3 "" H 4150 8100 50  0001 C CNN
	1    4150 8100
	0    1    -1   0   
$EndComp
Text HLabel 2000 8950 0    60   Input ~ 0
DATA
$Comp
L ULN2003A U3
U 1 1 5BB81264
P 7900 2350
F 0 "U3" H 7900 2875 50  0000 C CNN
F 1 "ULN2003A" H 7900 2800 50  0000 C CNN
F 2 "ics:SOIC-16_3.9x9.9mm_Pitch1.27mm" H 7950 1700 50  0001 L CNN
F 3 "" H 8000 2250 50  0001 C CNN
F 4 "ZELM049" H 8000 2975 60  0001 C CNN "ventcode"
F 5 "-" H 8100 3075 60  0001 C CNN "Nominalas"
F 6 "0.08" H 8200 3175 60  0001 C CNN "Kaina"
F 7 "0805" H 8300 3275 60  0001 C CNN "Korpusas"
F 8 "1" H 8400 3375 60  0001 C CNN "Aukstis"
F 9 "ULN2003ADR" H 8500 3475 60  0001 C CNN "Marke"
F 10 "Mikroschema" H 8600 3575 60  0001 C CNN "Pavadinimas"
F 11 "ULN2003ADR" H 8700 3675 60  0001 C CNN "Gamintojo Kodas"
F 12 "http://www.tme.eu/en/details/uln2003adr/drivers-integrated-circuits/texas-instruments/" H 8800 3775 60  0001 C CNN "Nuoroda"
F 13 "SMD" H 8900 3875 60  0001 C CNN "Korpuso tipas"
F 14 "-" H 9000 3975 60  0001 C CNN "Parametras"
F 15 "-" H 9100 4075 60  0001 C CNN "Komentaras"
	1    7900 2350
	1    0    0    -1  
$EndComp
$Comp
L C C17
U 1 1 5BB81265
P 8400 3050
F 0 "C17" H 8425 3150 50  0000 L CNN
F 1 "100nF" H 8425 2950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 8438 2900 50  0001 C CNN
F 3 "" H 8400 3050 50  0001 C CNN
	1    8400 3050
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 5BB81266
P 2400 5050
F 0 "R4" V 2500 5050 50  0000 C CNN
F 1 "10k" V 2400 5050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 5050 50  0001 C CNN
F 3 "" H 2400 5050 50  0001 C CNN
	1    2400 5050
	-1   0    0    1   
$EndComp
$Comp
L C C3
U 1 1 5BB81267
P 2650 5050
F 0 "C3" H 2675 5150 50  0000 L CNN
F 1 "100nF" H 2675 4950 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2688 4900 50  0001 C CNN
F 3 "" H 2650 5050 50  0001 C CNN
	1    2650 5050
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 5BB81268
P 3400 4800
F 0 "R12" V 3500 4800 50  0000 C CNN
F 1 "1k" V 3400 4800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3330 4800 50  0001 C CNN
F 3 "" H 3400 4800 50  0001 C CNN
	1    3400 4800
	0    -1   -1   0   
$EndComp
$Comp
L R R5
U 1 1 5BB8126B
P 2400 5950
F 0 "R5" V 2500 5950 50  0000 C CNN
F 1 "10k" V 2400 5950 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 5950 50  0001 C CNN
F 3 "" H 2400 5950 50  0001 C CNN
	1    2400 5950
	-1   0    0    1   
$EndComp
$Comp
L C C4
U 1 1 5BB8126C
P 2650 5950
F 0 "C4" H 2675 6050 50  0000 L CNN
F 1 "100nF" H 2675 5850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2688 5800 50  0001 C CNN
F 3 "" H 2650 5950 50  0001 C CNN
	1    2650 5950
	1    0    0    -1  
$EndComp
$Comp
L R R13
U 1 1 5BB8126D
P 3400 5700
F 0 "R13" V 3500 5700 50  0000 C CNN
F 1 "1k" V 3400 5700 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3330 5700 50  0001 C CNN
F 3 "" H 3400 5700 50  0001 C CNN
	1    3400 5700
	0    -1   -1   0   
$EndComp
$Comp
L R R6
U 1 1 5BB81270
P 2400 6900
F 0 "R6" V 2500 6900 50  0000 C CNN
F 1 "10k" V 2400 6900 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 6900 50  0001 C CNN
F 3 "" H 2400 6900 50  0001 C CNN
	1    2400 6900
	-1   0    0    1   
$EndComp
$Comp
L C C5
U 1 1 5BB81271
P 2650 6900
F 0 "C5" H 2675 7000 50  0000 L CNN
F 1 "100nF" H 2675 6800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 2688 6750 50  0001 C CNN
F 3 "" H 2650 6900 50  0001 C CNN
	1    2650 6900
	1    0    0    -1  
$EndComp
$Comp
L R R14
U 1 1 5BB81272
P 3400 6650
F 0 "R14" V 3500 6650 50  0000 C CNN
F 1 "1k" V 3400 6650 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3330 6650 50  0001 C CNN
F 3 "" H 3400 6650 50  0001 C CNN
	1    3400 6650
	0    -1   -1   0   
$EndComp
$Comp
L R R9
U 1 1 5BB81277
P 2750 3800
F 0 "R9" V 2850 3750 50  0000 C CNN
F 1 "1k" V 2750 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2680 3800 50  0001 C CNN
F 3 "" H 2750 3800 50  0001 C CNN
	1    2750 3800
	0    -1   -1   0   
$EndComp
$Comp
L R R10
U 1 1 5BB81278
P 2750 4050
F 0 "R10" V 2850 4050 50  0000 C CNN
F 1 "28R" V 2750 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2680 4050 50  0001 C CNN
F 3 "" H 2750 4050 50  0001 C CNN
	1    2750 4050
	0    -1   -1   0   
$EndComp
$Comp
L R R3
U 1 1 5BB81279
P 2400 4050
F 0 "R3" V 2500 4050 50  0000 C CNN
F 1 "3.24k" V 2400 4050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 4050 50  0001 C CNN
F 3 "" H 2400 4050 50  0001 C CNN
	1    2400 4050
	0    -1   -1   0   
$EndComp
$Comp
L C C10
U 1 1 5BB8127A
P 3200 4000
F 0 "C10" H 3225 4100 50  0000 L CNN
F 1 "100nF" H 3225 3900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3238 3850 50  0001 C CNN
F 3 "" H 3200 4000 50  0001 C CNN
	1    3200 4000
	1    0    0    -1  
$EndComp
$Comp
L CP C12
U 1 1 5BB8127B
P 3650 4000
F 0 "C12" H 3675 4100 50  0000 L CNN
F 1 "10uFx50V" H 3500 3700 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 3688 3850 50  0001 C CNN
F 3 "" H 3650 4000 50  0001 C CNN
	1    3650 4000
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 5BB8127E
P 2750 2200
F 0 "R7" V 2850 2150 50  0000 C CNN
F 1 "1k" V 2750 2200 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2680 2200 50  0001 C CNN
F 3 "" H 2750 2200 50  0001 C CNN
	1    2750 2200
	0    -1   -1   0   
$EndComp
$Comp
L R R8
U 1 1 5BB8127F
P 2750 2450
F 0 "R8" V 2850 2450 50  0000 C CNN
F 1 "28R" V 2750 2450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2680 2450 50  0001 C CNN
F 3 "" H 2750 2450 50  0001 C CNN
	1    2750 2450
	0    -1   -1   0   
$EndComp
$Comp
L R R2
U 1 1 5BB81280
P 2400 2450
F 0 "R2" V 2500 2450 50  0000 C CNN
F 1 "3.24k" V 2400 2450 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 2330 2450 50  0001 C CNN
F 3 "" H 2400 2450 50  0001 C CNN
	1    2400 2450
	0    -1   -1   0   
$EndComp
$Comp
L C C9
U 1 1 5BB81281
P 3200 2400
F 0 "C9" H 3225 2500 50  0000 L CNN
F 1 "100nF" H 3225 2300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 3238 2250 50  0001 C CNN
F 3 "" H 3200 2400 50  0001 C CNN
	1    3200 2400
	1    0    0    -1  
$EndComp
$Comp
L CP C11
U 1 1 5BB81282
P 3650 2400
F 0 "C11" H 3675 2500 50  0000 L CNN
F 1 "10uFx50V" H 3550 2100 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 3688 2250 50  0001 C CNN
F 3 "" H 3650 2400 50  0001 C CNN
	1    3650 2400
	1    0    0    -1  
$EndComp
$Comp
L C C14
U 1 1 5BB81283
P 4000 4000
F 0 "C14" H 4150 4000 50  0000 L CNN
F 1 "330nF" H 4150 3900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4038 3850 50  0001 C CNN
F 3 "" H 4000 4000 50  0001 C CNN
	1    4000 4000
	1    0    0    -1  
$EndComp
$Comp
L R R20
U 1 1 5BB81284
P 4250 3100
F 0 "R20" V 4350 3100 50  0000 C CNN
F 1 "12k" V 4250 3100 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4180 3100 50  0001 C CNN
F 3 "" H 4250 3100 50  0001 C CNN
	1    4250 3100
	0    -1   -1   0   
$EndComp
$Comp
L C C13
U 1 1 5BB81285
P 4000 2400
F 0 "C13" H 4150 2400 50  0000 L CNN
F 1 "330nF" H 4100 2300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 4038 2250 50  0001 C CNN
F 3 "" H 4000 2400 50  0001 C CNN
	1    4000 2400
	1    0    0    -1  
$EndComp
$Comp
L R R19
U 1 1 5BB81286
P 4250 1500
F 0 "R19" V 4350 1500 50  0000 C CNN
F 1 "12k" V 4250 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 4180 1500 50  0001 C CNN
F 3 "" H 4250 1500 50  0001 C CNN
	1    4250 1500
	0    -1   -1   0   
$EndComp
Text Label 6150 2650 0    60   ~ 0
CTRL_HE
Text Label 6150 2050 0    60   ~ 0
CTRL_P1
Text Label 6150 2250 0    60   ~ 0
CTRL_P2
Text Label 6150 2450 0    60   ~ 0
CTRL_R1
$Comp
L LM2904 U1
U 1 1 5BB8128B
P 4550 2100
F 0 "U1" H 4550 2300 50  0000 L CNN
F 1 "LM2904" H 4550 1900 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4550 2100 50  0001 C CNN
F 3 "" H 4550 2100 50  0001 C CNN
	1    4550 2100
	1    0    0    -1  
$EndComp
$Comp
L LM2904 U1
U 2 1 5BB8128C
P 4550 3700
F 0 "U1" H 4550 3900 50  0000 L CNN
F 1 "LM2904" H 4550 3500 50  0000 L CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 4550 3700 50  0001 C CNN
F 3 "" H 4550 3700 50  0001 C CNN
	2    4550 3700
	1    0    0    -1  
$EndComp
$Comp
L CP C6
U 1 1 5BB8128D
P 3100 5050
F 0 "C6" H 3250 5100 50  0000 L CNN
F 1 "10uFx50V" H 3250 5000 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 3138 4900 50  0001 C CNN
F 3 "" H 3100 5050 50  0001 C CNN
	1    3100 5050
	1    0    0    -1  
$EndComp
$Comp
L CP C7
U 1 1 5BB8128E
P 3100 5950
F 0 "C7" H 3250 6000 50  0000 L CNN
F 1 "10uFx50V" H 3250 5900 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 3138 5800 50  0001 C CNN
F 3 "" H 3100 5950 50  0001 C CNN
	1    3100 5950
	1    0    0    -1  
$EndComp
$Comp
L CP C8
U 1 1 5BB8128F
P 3100 6900
F 0 "C8" H 3250 6950 50  0000 L CNN
F 1 "10uFx50V" H 3250 6850 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D5.0mm_P2.50mm" H 3138 6750 50  0001 C CNN
F 3 "" H 3100 6900 50  0001 C CNN
	1    3100 6900
	1    0    0    -1  
$EndComp
Text Label 6650 5300 0    60   ~ 0
ADC_T0
Text Label 6650 5400 0    60   ~ 0
ADC_T1
Text Label 6650 5600 0    60   ~ 0
ADC_T2
Text Label 6650 5700 0    60   ~ 0
ADC_T3
Text Label 6650 5800 0    60   ~ 0
ADC_T4
Text Label 8500 7350 2    60   ~ 0
COM
Text Label 8150 4100 3    60   ~ 0
3VDC
Text Label 7800 4100 3    60   ~ 0
12VDC
Text Label 6200 6200 0    60   ~ 0
MCUTX
Text Label 6200 6300 0    60   ~ 0
MCURX
Text Label 6650 5500 0    60   ~ 0
PWM_T0
Text Label 9350 6400 2    60   ~ 0
PWM_T1
$Comp
L Unicon U2
U 1 1 5BB81292
P 8000 5850
F 0 "U2" H 7450 7050 60  0000 C CNN
F 1 "Unicon" H 8000 5850 60  0000 C CNN
F 2 "various:Unicon_Module" H 7750 5650 60  0001 C CNN
F 3 "" H 7750 5650 60  0001 C CNN
	1    8000 5850
	1    0    0    -1  
$EndComp
Text Label 9350 6800 2    60   ~ 0
CTRL_P1
Text Label 9350 6900 2    60   ~ 0
CTRL_P2
Text Label 9350 5800 2    60   ~ 0
CTRL_R1
Text Label 9350 6600 2    60   ~ 0
CTRL_HE
$Comp
L T_BC817 Q2
U 1 1 5BB81294
P 4000 9250
F 0 "Q2" H 4250 9350 59  0000 L CNN
F 1 "BC817" H 4250 9250 50  0000 L CNN
F 2 "disc:SOT-23" H 4800 9300 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BC817_BC817W_BC337.pdf" H 6100 9200 50  0001 C CNN
F 4 "ZELT008" H 4100 8950 60  0001 C CNN "ventcode"
F 5 "-" H 5150 9250 60  0001 C CNN "Nominalas"
F 6 "0.03" H 5300 9300 60  0001 C CNN "Kaina"
F 7 "SOT-23" H 6450 9400 60  0001 C CNN "Korpusas"
F 8 "1" H 5250 9350 60  0001 C CNN "Aukstis"
F 9 "BC817" H 3800 9350 60  0001 C CNN "Marke"
F 10 "Tranzistorius" H 4850 9400 60  0001 C CNN "Pavadinimas"
F 11 "BC817" H 3800 9350 59  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 5850 9400 60  0001 C CNN "Nuoroda"
F 13 "SMD" H 5750 9400 60  0001 C CNN "Korpuso tipas"
F 14 "45V/500mA" H 6350 9300 60  0001 C CNN "Parametras"
F 15 "markiravimas 6D*" H 5700 9300 60  0001 C CNN "Komentaras"
	1    4000 9250
	-1   0    0    -1  
$EndComp
$Comp
L T_BC817 Q1
U 1 1 5BB81295
P 2900 8650
F 0 "Q1" H 3150 8750 59  0000 L CNN
F 1 "BC817" H 3150 8650 50  0000 L CNN
F 2 "disc:SOT-23" H 3700 8700 50  0001 C CNN
F 3 "https://assets.nexperia.com/documents/data-sheet/BC817_BC817W_BC337.pdf" H 5000 8600 50  0001 C CNN
F 4 "ZELT008" H 3000 8350 60  0001 C CNN "ventcode"
F 5 "-" H 4050 8650 60  0001 C CNN "Nominalas"
F 6 "0.03" H 4200 8700 60  0001 C CNN "Kaina"
F 7 "SOT-23" H 5350 8800 60  0001 C CNN "Korpusas"
F 8 "1" H 4150 8750 60  0001 C CNN "Aukstis"
F 9 "BC817" H 2700 8750 60  0001 C CNN "Marke"
F 10 "Tranzistorius" H 3750 8800 60  0001 C CNN "Pavadinimas"
F 11 "BC817" H 2700 8750 59  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 4750 8800 60  0001 C CNN "Nuoroda"
F 13 "SMD" H 4650 8800 60  0001 C CNN "Korpuso tipas"
F 14 "45V/500mA" H 5250 8700 60  0001 C CNN "Parametras"
F 15 "markiravimas 6D*" H 4600 8700 60  0001 C CNN "Komentaras"
	1    2900 8650
	-1   0    0    -1  
$EndComp
$Comp
L LED D3
U 1 1 5BB81297
P 6750 2050
F 0 "D3" H 6750 1950 50  0000 C CNN
F 1 "LED" H 6750 1950 50  0001 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 6750 2050 50  0001 C CNN
F 3 "" H 6750 2050 50  0001 C CNN
	1    6750 2050
	-1   0    0    1   
$EndComp
$Comp
L LED D5
U 1 1 5BB81298
P 6950 2250
F 0 "D5" H 6950 2150 50  0000 C CNN
F 1 "LED" H 6950 2150 50  0001 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 6950 2250 50  0001 C CNN
F 3 "" H 6950 2250 50  0001 C CNN
	1    6950 2250
	-1   0    0    1   
$EndComp
$Comp
L LED D4
U 1 1 5BB81299
P 6750 2450
F 0 "D4" H 6750 2350 50  0000 C CNN
F 1 "LED" H 6750 2350 50  0001 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 6750 2450 50  0001 C CNN
F 3 "" H 6750 2450 50  0001 C CNN
	1    6750 2450
	-1   0    0    1   
$EndComp
$Comp
L LED D6
U 1 1 5BB8129A
P 6950 2650
F 0 "D6" H 6950 2550 50  0000 C CNN
F 1 "LED" H 6950 2550 50  0001 C CNN
F 2 "LEDs:LED_0805_HandSoldering" H 6950 2650 50  0001 C CNN
F 3 "" H 6950 2650 50  0001 C CNN
	1    6950 2650
	-1   0    0    1   
$EndComp
Connection ~ 3650 3800
Wire Wire Line
	2200 4050 2250 4050
Connection ~ 3200 4200
Wire Wire Line
	3200 4150 3200 4200
Wire Wire Line
	2200 4200 2200 4050
Wire Wire Line
	2200 4200 4800 4200
Wire Wire Line
	3650 4200 3650 4150
Connection ~ 3200 3800
Wire Wire Line
	3200 3800 3200 3850
Connection ~ 3000 3800
Wire Wire Line
	3650 3800 3650 3850
Wire Wire Line
	3000 4050 2900 4050
Wire Wire Line
	3000 3800 3000 4050
Wire Wire Line
	2900 3800 4250 3800
Wire Wire Line
	2550 4050 2600 4050
Wire Wire Line
	2000 3800 2600 3800
Connection ~ 2400 6650
Wire Wire Line
	3550 6650 4000 6650
Wire Wire Line
	2400 6750 2400 6650
Wire Wire Line
	2400 7150 3700 7150
Wire Wire Line
	2400 7050 2400 7150
Connection ~ 2400 5700
Wire Wire Line
	3550 5700 4000 5700
Wire Wire Line
	2000 5700 3250 5700
Wire Wire Line
	2400 5800 2400 5700
Wire Wire Line
	2400 6200 3700 6200
Wire Wire Line
	2400 6100 2400 6200
Connection ~ 2400 4800
Wire Wire Line
	3550 4800 4000 4800
Wire Wire Line
	2000 4800 3250 4800
Wire Wire Line
	2400 4900 2400 4800
Wire Wire Line
	2400 5300 3700 5300
Wire Wire Line
	2400 5200 2400 5300
Wire Wire Line
	8400 3250 8400 3200
Connection ~ 8400 2850
Wire Wire Line
	8400 2900 8400 2850
Wire Wire Line
	8300 2850 8800 2850
Wire Wire Line
	7600 3250 8400 3250
Wire Wire Line
	7900 3250 7900 3050
Wire Wire Line
	8300 2150 8600 2150
Connection ~ 2800 9550
Connection ~ 3900 8350
Wire Wire Line
	3900 8350 3900 8100
Wire Wire Line
	3900 8100 4000 8100
Wire Wire Line
	5200 8100 4300 8100
Connection ~ 4400 8650
Wire Wire Line
	4400 8800 4400 8650
Wire Wire Line
	4500 9250 4200 9250
Connection ~ 3900 9550
Wire Wire Line
	4400 9550 4400 9100
Wire Wire Line
	4300 8650 5200 8650
Wire Wire Line
	4300 8350 5200 8350
Connection ~ 2800 8350
Wire Wire Line
	2300 8350 2300 9100
Wire Wire Line
	2300 8350 4000 8350
Wire Wire Line
	2800 8350 2800 8450
Wire Wire Line
	2300 9550 2300 9400
Wire Wire Line
	3300 8950 2000 8950
Wire Wire Line
	3150 8650 3100 8650
Wire Wire Line
	3500 8650 3450 8650
Connection ~ 3900 8950
Wire Wire Line
	3600 8950 3900 8950
Wire Wire Line
	4800 9250 5200 9250
Wire Wire Line
	2800 9550 2800 8850
Connection ~ 3900 8650
Wire Wire Line
	3900 9550 3900 9450
Wire Wire Line
	2300 9550 4400 9550
Wire Wire Line
	3900 8650 3900 9050
Wire Wire Line
	3800 8650 4000 8650
Connection ~ 3650 4200
Connection ~ 3650 2200
Wire Wire Line
	2200 2450 2250 2450
Connection ~ 3200 2600
Wire Wire Line
	3200 2550 3200 2600
Wire Wire Line
	2200 2600 2200 2450
Wire Wire Line
	2200 2600 4800 2600
Wire Wire Line
	3650 2600 3650 2550
Connection ~ 3200 2200
Wire Wire Line
	3200 2200 3200 2250
Connection ~ 3000 2200
Wire Wire Line
	3650 2200 3650 2250
Wire Wire Line
	3000 2450 2900 2450
Wire Wire Line
	3000 2200 3000 2450
Wire Wire Line
	2900 2200 4250 2200
Wire Wire Line
	2550 2450 2600 2450
Wire Wire Line
	2000 2200 2600 2200
Connection ~ 3650 2600
Wire Wire Line
	4450 3300 4450 3400
Wire Wire Line
	4450 4200 4450 4000
Wire Wire Line
	4850 3700 5300 3700
Wire Wire Line
	4000 3100 4000 3850
Wire Wire Line
	4000 3600 4250 3600
Wire Wire Line
	4950 1700 4450 1700
Wire Wire Line
	4450 1700 4450 1800
Wire Wire Line
	4450 2600 4450 2400
Wire Wire Line
	4850 2100 5300 2100
Wire Wire Line
	4000 1500 4000 2250
Wire Wire Line
	4000 2000 4250 2000
Wire Wire Line
	4950 1500 4400 1500
Wire Wire Line
	4950 3100 4400 3100
Connection ~ 4450 2600
Wire Wire Line
	4000 1500 4100 1500
Connection ~ 4450 4200
Connection ~ 4000 3600
Wire Wire Line
	4000 4150 4000 4200
Connection ~ 4000 4200
Wire Wire Line
	4000 3100 4100 3100
Connection ~ 4000 2000
Wire Wire Line
	4000 2550 4000 2600
Connection ~ 4000 2600
Wire Wire Line
	4950 3300 4450 3300
Wire Wire Line
	7200 2150 7500 2150
Wire Wire Line
	7100 2250 7500 2250
Wire Wire Line
	7100 2350 7500 2350
Wire Wire Line
	7200 2450 7500 2450
Wire Wire Line
	7500 2550 7400 2550
Wire Wire Line
	8300 2250 8600 2250
Wire Wire Line
	8300 2350 8600 2350
Wire Wire Line
	8300 2450 8600 2450
Wire Wire Line
	8300 2550 8600 2550
Wire Wire Line
	7400 2550 7400 2450
Connection ~ 7400 2450
Wire Wire Line
	2000 6650 3250 6650
Wire Wire Line
	2650 4900 2650 4800
Connection ~ 2650 4800
Wire Wire Line
	2650 5200 2650 5300
Connection ~ 2650 5300
Wire Wire Line
	3100 4900 3100 4800
Connection ~ 3100 4800
Wire Wire Line
	3100 5200 3100 5300
Connection ~ 3100 5300
Wire Wire Line
	2650 5800 2650 5700
Connection ~ 2650 5700
Wire Wire Line
	2650 6100 2650 6200
Connection ~ 2650 6200
Wire Wire Line
	3100 5800 3100 5700
Connection ~ 3100 5700
Wire Wire Line
	3100 6100 3100 6200
Connection ~ 3100 6200
Wire Wire Line
	2650 6750 2650 6650
Connection ~ 2650 6650
Wire Wire Line
	2650 7050 2650 7150
Connection ~ 2650 7150
Wire Wire Line
	3100 6750 3100 6650
Connection ~ 3100 6650
Wire Wire Line
	3100 7050 3100 7150
Connection ~ 3100 7150
Wire Wire Line
	7850 7250 7850 7350
Wire Wire Line
	7850 7350 8500 7350
Wire Wire Line
	8150 7350 8150 7250
Wire Wire Line
	7950 7250 7950 7350
Connection ~ 7950 7350
Wire Wire Line
	8050 7250 8050 7350
Connection ~ 8050 7350
Connection ~ 8150 7350
Wire Wire Line
	8150 4500 8150 4100
Wire Wire Line
	7800 4100 7800 4500
Wire Wire Line
	7150 6200 6200 6200
Wire Wire Line
	7150 6300 6200 6300
Wire Wire Line
	6850 5050 7150 5050
Wire Wire Line
	6850 4900 7150 4900
Wire Wire Line
	8850 5500 9450 5500
Wire Wire Line
	8850 5600 9450 5600
Wire Wire Line
	6600 2050 6150 2050
Wire Wire Line
	6800 2250 6150 2250
Wire Wire Line
	6600 2450 6150 2450
Wire Wire Line
	6800 2650 6150 2650
Wire Wire Line
	6900 2050 7200 2050
Wire Wire Line
	7200 2050 7200 2150
Wire Wire Line
	6900 2450 7100 2450
Wire Wire Line
	7100 2450 7100 2350
Wire Wire Line
	7100 2650 7200 2650
Wire Wire Line
	7200 2650 7200 2450
Text Label 9350 5700 2    60   ~ 0
BUZZ
Wire Wire Line
	7500 2050 7350 2050
Wire Wire Line
	7350 2050 7350 1800
Wire Wire Line
	7350 1800 6150 1800
Text Label 6150 1800 0    60   ~ 0
BUZZ
$Comp
L Buzzer_CSS-J4B20 Z1
U 1 1 5BB8129B
P 7900 1500
F 0 "Z1" H 7900 1700 50  0000 C CNN
F 1 "CSS-J4B20" H 7950 1350 50  0000 C CNN
F 2 "disc:BUZZER" H 8650 1550 50  0001 C CNN
F 3 "http://www.cui.com/product/resource/css-j4b20-smt.pdf" H 9550 1350 50  0001 C CNN
F 4 "ZEL028" H 7900 1350 60  0001 C CNN "ventcode"
F 5 "-" H 8750 1450 60  0001 C CNN "Nominalas"
F 6 "0" H 8950 1450 60  0001 C CNN "Kaina"
F 7 "8.5x8.5" H 9450 1550 60  0001 C CNN "Korpusas"
F 8 "3.2" H 9750 1550 60  0001 C CNN "Aukstis"
F 9 "-" H 8450 1450 60  0001 C CNN "Marke"
F 10 "Signalizatorius" H 8750 1650 60  0001 C CNN "Pavadinimas"
F 11 "CSS-J4B20" H 9500 1650 60  0001 C CNN "Gamintojo Kodas"
F 12 "-" H 8650 1450 60  0001 C CNN "Nuoroda"
F 13 "SMD" H 9050 1550 60  0001 C CNN "Korpuso tipas"
F 14 "3.6V/2.73kHz/97dBA" H 9550 1450 60  0001 C CNN "Parametras"
F 15 "-" H 8550 1450 60  0001 C CNN "Komentaras"
	1    7900 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 2050 8400 2050
Wire Wire Line
	8400 2050 8400 1500
Wire Wire Line
	8400 1500 8100 1500
$Comp
L R R25
U 1 1 5BB8129C
P 7450 1500
F 0 "R25" V 7550 1500 50  0000 C CNN
F 1 "470" V 7450 1500 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 7380 1500 50  0001 C CNN
F 3 "" H 7450 1500 50  0001 C CNN
	1    7450 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7600 1500 7700 1500
Wire Wire Line
	7300 1500 6150 1500
Text Label 6150 1500 0    60   ~ 0
12VDC
Wire Wire Line
	7950 4500 7950 4100
Text Label 7950 4100 3    60   ~ 0
5VDC
Text HLabel 2000 2200 0    60   Input ~ 0
T0_IN
Text HLabel 2000 3800 0    60   Input ~ 0
T1_IN
Text HLabel 2000 4800 0    60   Input ~ 0
T2_IN
Text HLabel 2000 5700 0    60   Input ~ 0
T3_IN
Text HLabel 2000 6650 0    60   Input ~ 0
T4_IN
Text Label 5200 9250 2    60   ~ 0
MCUTX
Text Label 5200 8350 2    60   ~ 0
MCURX
Text Label 5200 8100 2    60   ~ 0
3VDC
Text Label 5200 8650 2    60   ~ 0
12VDC
Text Label 3400 9550 2    60   ~ 0
COM
Text Label 4950 1500 2    60   ~ 0
PWM_T0
Text Label 4950 3100 2    60   ~ 0
PWM_T1
Text Label 4950 3300 2    60   ~ 0
3VDC
Text Label 4950 1700 2    60   ~ 0
3VDC
Text Label 5300 2100 2    60   ~ 0
ADC_T0
Text Label 5300 3700 2    60   ~ 0
ADC_T1
Text Label 4800 2600 2    60   ~ 0
COM
Text Label 4800 4200 2    60   ~ 0
COM
Text Label 3700 5300 2    60   ~ 0
COM
Text Label 3700 6200 2    60   ~ 0
COM
Text Label 3700 7150 2    60   ~ 0
COM
Text Label 4000 4800 2    60   ~ 0
ADC_T2
Text Label 4000 5700 2    60   ~ 0
ADC_T3
Text Label 4000 6650 2    60   ~ 0
ADC_T4
Connection ~ 7900 3250
Text Label 7600 3250 0    60   ~ 0
COM
Text Label 8800 2850 2    60   ~ 0
12VDC
Text HLabel 8600 2150 2    60   Output ~ 0
REL_P1
Text HLabel 8600 2250 2    60   Output ~ 0
REL_P2
Text HLabel 8600 2350 2    60   Output ~ 0
REL_R1
Text HLabel 8600 2450 2    60   Output ~ 0
REL_HEA
Text HLabel 8600 2550 2    60   Output ~ 0
REL_HEB
Text HLabel 6850 4900 0    60   Input ~ 0
~nRST
Text HLabel 6850 5050 0    60   Input ~ 0
BOOT
Wire Wire Line
	7800 4400 6850 4400
Connection ~ 7800 4400
Text HLabel 6850 4400 0    60   Input ~ 0
12VDC
Text HLabel 9450 5500 2    60   Output ~ 0
SCL
Text HLabel 9450 5600 2    60   BiDi ~ 0
SDA
Wire Wire Line
	8850 5700 9350 5700
Wire Wire Line
	7150 5500 6650 5500
Wire Wire Line
	7150 5600 6650 5600
Wire Wire Line
	7150 5300 6650 5300
Wire Wire Line
	7150 5400 6650 5400
Wire Wire Line
	7150 5700 6650 5700
Wire Wire Line
	7150 5800 6650 5800
Wire Wire Line
	8850 6400 9350 6400
$Comp
L C C18
U 1 1 5BB9B6F8
P 10050 6000
F 0 "C18" V 10200 5850 50  0000 L CNN
F 1 "47nF" V 10200 6025 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10088 5850 50  0001 C CNN
F 3 "" H 10050 6000 50  0001 C CNN
	1    10050 6000
	0    1    1    0   
$EndComp
Wire Wire Line
	10200 6000 10450 6000
Text Label 10450 6000 2    60   ~ 0
COM
Wire Wire Line
	9900 6000 8850 6000
Wire Wire Line
	8850 6100 9450 6100
Wire Wire Line
	8850 6200 9450 6200
Wire Wire Line
	8850 6300 9450 6300
$Comp
L C C1
U 1 1 5BB9C5FA
P 10050 5200
F 0 "C1" V 10200 5050 50  0000 L CNN
F 1 "47nF" V 10200 5200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 10088 5050 50  0001 C CNN
F 3 "" H 10050 5200 50  0001 C CNN
	1    10050 5200
	0    1    1    0   
$EndComp
Wire Wire Line
	9900 5200 8850 5200
Wire Wire Line
	10200 5200 10450 5200
Text Label 10450 5200 2    60   ~ 0
COM
Wire Wire Line
	8850 5300 9450 5300
Wire Wire Line
	8850 6900 9350 6900
Wire Wire Line
	8850 6800 9350 6800
Wire Wire Line
	8850 6600 9350 6600
Wire Wire Line
	7150 5900 6650 5900
Wire Wire Line
	7150 6000 6650 6000
Wire Wire Line
	8850 5800 9350 5800
Text HLabel 9450 6100 2    60   Input ~ 0
TSC1
Text HLabel 9450 6200 2    60   Input ~ 0
TSC2
Text HLabel 9450 6300 2    60   Input ~ 0
TSC3
Text HLabel 9450 5300 2    60   Input ~ 0
TSC4
Text HLabel 6650 5900 0    60   Input ~ 0
ADC_EX1
Text HLabel 6650 6000 0    60   Input ~ 0
ADC_EX2
$EndSCHEMATC