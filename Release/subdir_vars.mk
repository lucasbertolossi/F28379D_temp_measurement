################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/headers/cmd/F2837xD_Headers_nonBIOS_cpu1.cmd 

LIB_SRCS += \
../SFO_v8_fpu_lib_build_c28_coff.lib 

ASM_SRCS += \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_CodeStartBranch.asm \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_usDelay.asm 

C_SRCS += \
../ADS124S08.c \
C:/Users/lbert/OneDrive/Documents/TCC/software/Software_final/F28379D_LCD1602-master/F28379D_LCD1602-master/F28379D_lcd.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_CpuTimers.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_DefaultISR.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_EPwm.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/headers/source/F2837xD_GlobalVariableDefs.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_Gpio.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_PieCtrl.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_PieVect.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_Spi.c \
C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_SysCtrl.c \
../Peripheral_Setup.c \
../adchal_tidrivers_adapted.c \
../crc.c \
../main.c \
../rtd.c 

C_DEPS += \
./ADS124S08.d \
./F28379D_lcd.d \
./F2837xD_CpuTimers.d \
./F2837xD_DefaultISR.d \
./F2837xD_EPwm.d \
./F2837xD_GlobalVariableDefs.d \
./F2837xD_Gpio.d \
./F2837xD_PieCtrl.d \
./F2837xD_PieVect.d \
./F2837xD_Spi.d \
./F2837xD_SysCtrl.d \
./Peripheral_Setup.d \
./adchal_tidrivers_adapted.d \
./crc.d \
./main.d \
./rtd.d 

OBJS += \
./ADS124S08.obj \
./F28379D_lcd.obj \
./F2837xD_CodeStartBranch.obj \
./F2837xD_CpuTimers.obj \
./F2837xD_DefaultISR.obj \
./F2837xD_EPwm.obj \
./F2837xD_GlobalVariableDefs.obj \
./F2837xD_Gpio.obj \
./F2837xD_PieCtrl.obj \
./F2837xD_PieVect.obj \
./F2837xD_Spi.obj \
./F2837xD_SysCtrl.obj \
./F2837xD_usDelay.obj \
./Peripheral_Setup.obj \
./adchal_tidrivers_adapted.obj \
./crc.obj \
./main.obj \
./rtd.obj 

ASM_DEPS += \
./F2837xD_CodeStartBranch.d \
./F2837xD_usDelay.d 

OBJS__QUOTED += \
"ADS124S08.obj" \
"F28379D_lcd.obj" \
"F2837xD_CodeStartBranch.obj" \
"F2837xD_CpuTimers.obj" \
"F2837xD_DefaultISR.obj" \
"F2837xD_EPwm.obj" \
"F2837xD_GlobalVariableDefs.obj" \
"F2837xD_Gpio.obj" \
"F2837xD_PieCtrl.obj" \
"F2837xD_PieVect.obj" \
"F2837xD_Spi.obj" \
"F2837xD_SysCtrl.obj" \
"F2837xD_usDelay.obj" \
"Peripheral_Setup.obj" \
"adchal_tidrivers_adapted.obj" \
"crc.obj" \
"main.obj" \
"rtd.obj" 

C_DEPS__QUOTED += \
"ADS124S08.d" \
"F28379D_lcd.d" \
"F2837xD_CpuTimers.d" \
"F2837xD_DefaultISR.d" \
"F2837xD_EPwm.d" \
"F2837xD_GlobalVariableDefs.d" \
"F2837xD_Gpio.d" \
"F2837xD_PieCtrl.d" \
"F2837xD_PieVect.d" \
"F2837xD_Spi.d" \
"F2837xD_SysCtrl.d" \
"Peripheral_Setup.d" \
"adchal_tidrivers_adapted.d" \
"crc.d" \
"main.d" \
"rtd.d" 

ASM_DEPS__QUOTED += \
"F2837xD_CodeStartBranch.d" \
"F2837xD_usDelay.d" 

C_SRCS__QUOTED += \
"../ADS124S08.c" \
"C:/Users/lbert/OneDrive/Documents/TCC/software/Software_final/F28379D_LCD1602-master/F28379D_LCD1602-master/F28379D_lcd.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_CpuTimers.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_DefaultISR.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_EPwm.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/headers/source/F2837xD_GlobalVariableDefs.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_Gpio.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_PieCtrl.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_PieVect.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_Spi.c" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_SysCtrl.c" \
"../Peripheral_Setup.c" \
"../adchal_tidrivers_adapted.c" \
"../crc.c" \
"../main.c" \
"../rtd.c" 

ASM_SRCS__QUOTED += \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_CodeStartBranch.asm" \
"C:/ti/c2000/C2000Ware_3_04_00_00/device_support/f2837xd/common/source/F2837xD_usDelay.asm" 


