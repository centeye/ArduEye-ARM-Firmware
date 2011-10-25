/***************************************************************************
 **
 **   Common Definitions INCLUDE FILE
 **
 **   Centeye, Inc 2011
 **
 **    $Revision: 1
 **
 ***************************************************************************/

#ifndef COMMON_H
#define COMMON_H


#define RAW_IMG_DEF_SIZE    12544 //32768

//////////////// Command Default Values//////////////////////////
// default high pass filter level
#define HP_SHIFT_DEFAULT       5  
// smoothing rate default
#define OF_ALPHA_DEFAULT       1
// local winner search window size
#define LWTA_WINSIZE_DEFAULT   9
// local winner take all threshold (winner must be above threshold)
#define LWTA_THRESH_DEFAULT    120
// FPN_ON default
#define FPN_ON_DEFAULT  0
// DRC_ON default
#define DRC_ON_DEFAULT  0
// time to wait before reading pixel value (currently used for Stonyman only)
#define SETTLING_TIME_DEFAULT 3
// VISON CHIP REGISTER DEFAULT SETTINGS (Set for Stonyman chip)
#define AMP_GAIN_DEFAULT  5
#define VREF_DEFAULT      30
#define NBIAS_DEFAULT     55
#define AOBIAS_DEFAULT    50

#endif