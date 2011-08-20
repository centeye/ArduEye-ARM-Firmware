/***************************************************************************
 **
 **    Firefly VC INCLUDE FILE
 **
 **   Centeye, Inc 2011
 **
 **    $Revision: 1
 **
 ***************************************************************************/

#ifndef FIREFLY_MAIN_H_
#define FIREFLY_MAIN_H_

//Vaulted Biases
#define NBIAS 0x0
#define VREF 0x1
#define NBIAS2 0x2
#define PRSUPPLY 0x3
#define ANALOGOUTBIAS 0x4

#define AMPCONF 0x8
#define AMPOP 0x9
#define FPCONF 0xA

#define COLSEL 0x10

//ADC Configuration and Operation
#define ADCOP 0x18
#define RESB 0x19
#define REST 0x1A
#define YUKNBIAS 0x1B

//Vault Commands
#define CONNECTVDDA 0x1C
#define BIASSWITCH1 0x1D
#define BIASSWITCH2 0x1E
#define BIASSWITCH3 0x1F
//Focal Plane
#define ROWSEL 0x20
#define HSW 0x28
#define VSW 0x30
//Linear Regulator
#define LINREG1 0x38
#define LINREG2 0x39
#define LINREG3 0x3A
#define LINREG4 0x3B

#endif