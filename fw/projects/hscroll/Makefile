#******************************************************************************
#
# Makefile - Rules for building the USB device serial example.
#
# Copyright (c) 2012 Texas Instruments Incorporated.  All rights reserved.
# Software License Agreement
# 
# Texas Instruments (TI) is supplying this software for use solely and
# exclusively on TI's microcontroller products. The software is owned by
# TI and/or its suppliers, and is protected under applicable copyright
# laws. You may not combine this software with "viral" open-source
# software in order to form a larger program.
# 
# THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
# NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
# NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
# CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
# DAMAGES, FOR ANY REASON WHATSOEVER.
# 
# This is part of revision 9453 of the EK-LM4F120XL Firmware Package.
#
#******************************************************************************

#
# Defines the part type that this project uses.
#
PART=LM4F120H5QR

#
# Set the processor variant.
#
VARIANT=cm4f

#
# The base directory for StellarisWare.
#
#ROOT=/opt/stellarisware-ek-lm4f120xl
ROOT=/home/j/xfer/ek/

#
# Include the common make definitions.
#
include ${ROOT}/makedefs

#
# Where to find source files that do not live in this directory.
#
VPATH=${ROOT}/boards/ek-lm4f120xl/drivers
VPATH+=${ROOT}/utils

#
# Where to find header files that do not live in the source directory.
#
IPATH=${ROOT}/boards/ek-lm4f120xl
IPATH+=${ROOT}

all: ${COMPILER}
all: ${COMPILER}/hscroll.axf

#
# The rule to clean out all the build products.
#
clean:
	@rm -rf ${COMPILER} ${wildcard *~}

#
# The rule to create the target directory.
#
${COMPILER}:
	@mkdir -p ${COMPILER}

${COMPILER}/hscroll.axf: ${COMPILER}/startup_${COMPILER}.o
${COMPILER}/hscroll.axf: ${COMPILER}/uartstdio.o
${COMPILER}/hscroll.axf: ${COMPILER}/hscroll.o
${COMPILER}/hscroll.axf: ${COMPILER}/usb_mouse_structs.o
${COMPILER}/hscroll.axf: ${COMPILER}/ustdlib.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/usbmode.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/usbtick.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/usbdesc.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/device/usbdconfig.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/device/usbdhandler.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/device/usbdcdesc.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/device/usbdenum.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/device/usbdhid.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/device/usbdhidmouse.o
${COMPILER}/hscroll.axf: ${ROOT}/usblib/
${COMPILER}/hscroll.axf: ${ROOT}/driverlib/${COMPILER}-cm4f/libdriver-cm4f.a
${COMPILER}/hscroll.axf: hscroll.ld
SCATTERgcc_hscroll=hscroll.ld
ENTRY_hscroll=ResetISR
CFLAGSgcc=-DTARGET_IS_BLIZZARD_RA2 -DUART_BUFFERED -Dgcc

#
# Include the automatically generated dependency files.
#
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif
