/*
 
 Firefly.h :definitions specific to Firefly vision chip
 Centeye, Inc
 Created by Alison Leonard. August, 2011

 ===============================================================================
 Copyright (c) 2011, Centeye, Inc.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of Centeye, Inc. nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CENTEYE, INC. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ===============================================================================
*/

#ifndef FIREFLY_MAIN_H_
#define FIREFLY_MAIN_H_


// IO PIN DEFINITIONS
// NavIO12 CMD BUS
#define CMDIO_MASK      ((uint16_t)0x00FF)
#define CMDIO_PORT      GPIOA

// read not write pin
#define WNR_MASK        GPIO_Pin_4
#define WNR_PORT        GPIOC

// Chip select pin
#define CS_MASK         GPIO_Pin_5
#define CS_PORT         GPIOC


//BIASES AND VISION CHIP SETTINGS
#define NBIAS 0x0
#define VREF 0x1
#define NBIAS2 0x2
#define PRSUPPLY 0x3
#define ANALOGOUTBIAS 0x4

// amplification settings
#define AMPCONF 0x8
#define AMPOP 0x9
#define FPCONF 0xA

// column select
#define COLSEL 0x10

// ADC Configuration and Operation
#define ADCOP 0x18
#define RESB 0x19
#define REST 0x1A
#define YUKNBIAS 0x1B

//Vault open close Commands
#define CONNECTVDDA 0x1C
#define BIASSWITCH1 0x1D
#define BIASSWITCH2 0x1E
#define BIASSWITCH3 0x1F

// row select
#define ROWSEL 0x20
// horizontal and vertical binning switches
#define HSW 0x28
#define VSW 0x30

//Linear Regulator
#define LINREG1 0x38
#define LINREG2 0x39
#define LINREG3 0x3A
#define LINREG4 0x3B

//DATA SET DEFAULTS
#define RAW_ROWS_DEFAULT  16 //64
#define RAW_COLS_DEFAULT  32 //128

#define RAW_ROWS_MAX      128
#define RAW_COLS_MAX      256
#define RAW_COLS_TEMP_MAX 128

#define RAW_ROWS_MIN      16
#define RAW_COLS_MIN      32

#endif