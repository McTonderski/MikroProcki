#include "frdm_bsp.h"
#include "tsi.h"
/**
* @brief The main loop.
*
* @return NULL
*/

int main (void) {

	TSI_Init (); /* initialize slider */
	DELAY(1000)
	while(1) {
		uint8_t slider = TSI_ReadSlider(); 
	}	
}
