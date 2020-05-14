#include "../bt_inc/boot_crc8.h"
 
#define AL2_FCS_COEF ((1 << 7) + (1 << 6) + (1 << 5))

unsigned char BT_GetCRC8(unsigned char * data, int length)
{
   unsigned char cFcs = 0;
   int i, j;
 
   for( i = 0; i < length; i ++ )
   {
      cFcs ^= data[i];
      for(j = 0; j < 8; j ++)
      {
         if(cFcs & 1)
            cFcs = (unsigned char)((cFcs >> 1) ^ AL2_FCS_COEF);
         else
            cFcs >>= 1;
      }
   } 
   return cFcs;
}
