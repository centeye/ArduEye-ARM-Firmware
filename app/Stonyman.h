/*
 
 Stonyman.h :definitions specific to Stonyman vision chip
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

#ifndef STONYMAN_H
#define STONYMAN_H

/// IO PIN DEFINITIONS
// reset register pointer
#define RESP_MASK       GPIO_Pin_6
#define RESP_PORT       GPIOA

// increment register pointer
#define INCP_MASK       GPIO_Pin_5
#define INCP_PORT       GPIOA

// reset value pointer
#define RESV_MASK       GPIO_Pin_4
#define RESV_PORT       GPIOA

// increment value pointer
#define INCV_MASK       GPIO_Pin_3
#define INCV_PORT       GPIOA

// amplification run pin (toggle pin to amplify pixel
#define INPHI_MASK      GPIO_Pin_2
#define INPHI_PORT      GPIOA

///REGISTER DEFINITIONS
#define COLSEL          0
#define ROWSEL          1
#define VSW             2
#define HSW             3
#define VREF            4
#define CONFIG          5
#define NBIAS           6
#define AOBIAS          7

// data set defaults
#define RAW_ROWS_DEFAULT  14 //64
#define RAW_COLS_DEFAULT  14 //128

#define RAW_ROWS_MAX      112
#define RAW_COLS_MAX      112

#define RAW_ROWS_MIN      14
#define RAW_COLS_MIN      14

// REGISTER DEFAULT SETTINGS
#define AMP_GAIN_DEFAULT  5
#define VREF_DEFAULT      30

#define NBIAS_DEFAULT     55
#define AOBIAS_DEFAULT    50

//TIMING DEFAULT SETTINGS
// time to wait between each pin toggle
#define CMD_DELAY         0
// time to wait before reading pixel value
#define SETTLING_TIME_DEFAULT 3


#endif

