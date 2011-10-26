/***************************************************************/
/* Comandos TWI - ESC - Projeto Hiperion                       */
/* PLaTooN 2011                                                */
/*                                                             */
/***************************************************************/

#include "TWI_slave.h"
#include "TWI_Hiperion.h"

unsigned char TrataErroTransI2C ( unsigned char TWIerrorMsg )
{
// Se houver falha, reinicia a recepcao e acende LED
  TWI_Start_Transceiver();
  PD3 = 1;
  return TWIerrorMsg; 
}
