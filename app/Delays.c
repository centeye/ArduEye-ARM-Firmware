/*************************************************************************
 *
 *    Used with ICCARM and AARM.
 *
 *    Delays
 *    Centeye,Inc
 *    Alison Leonard
 *    May 29, 2011
 **************************************************************************/

#include "Delays.h"

/*************************************************************************
 * Function Name: DelayResolution100us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay ~ (arg * 100us)
 *
 *************************************************************************/
void DelayResolution100us(Int32U Dly)
{
  for(; Dly; Dly--)
  {
    for(volatile Int32U j = DLY_100US; j; j--)
    {
    }
  }
}

/*************************************************************************
 * Function Name: DelayResolution10us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay ~ (arg * 10us)
 *
 *************************************************************************/
void DelayResolution10us(Int32U Dly)
{
  for(; Dly; Dly--)
  {
    for(volatile Int32U j = DLY_10US; j; j--)
    {
    }
  }
}

/*************************************************************************
 * Function Name: DelayResolution1us
 * Parameters: Int32U Dly
 *
 * Return: none
 *
 * Description: Delay ~ (arg * 1us)
 *
 *************************************************************************/
void DelayResolution1us(Int32U Dly)
{
  for(; Dly; Dly--)
  {
    for(volatile Int32U j = DLY_1US; j; j--)
    {
    }
  }
}