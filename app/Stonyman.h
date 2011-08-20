/***************************************************************************
 **
 **    StoneyMan VC INCLUDE FILE
 **
 **   Centeye, Inc 2011
 **
 **    $Revision: 1
 **
 ***************************************************************************/

#ifndef STONYMAN_H
#define STONYMAN_H

/// IO pins

#define RESP_MASK       GPIO_Pin_6
#define RESP_PORT       GPIOA

#define INCP_MASK       GPIO_Pin_5
#define INCP_PORT       GPIOA

#define RESV_MASK       GPIO_Pin_4
#define RESV_PORT       GPIOA

#define INCV_MASK       GPIO_Pin_3
#define INCV_PORT       GPIOA

#define INPHI_MASK      GPIO_Pin_2
#define INPHI_PORT      GPIOA

/// Registers
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

#define AMP_GAIN_DEFAULT  5
#define VREF_DEFAULT      28

#define NBIAS_DEFAULT     52
#define AOBIAS_DEFAULT    35

#define CMD_DELAY         0

#define SETTLING_TIME_DEFAULT 3


#endif

