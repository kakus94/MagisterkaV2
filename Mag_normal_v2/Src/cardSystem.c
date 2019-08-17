/*
 * cardSystem.c
 *
 *  Created on: 08.06.2019
 *      Author: kkarp
 */

#include "cardSystem.h"
#include "main.h"

void itemAdd(Stos_typeDef* box, uint8_t* card, uint8_t* sector) {
//	item thisItem;
//	uint8_t counter = 0;
//	uint8_t newPivot = box->pivot++;
//	if (newPivot > 30)
//		newPivot = 0;
//
//	memcpy(&thisItem.CardID, &card, 4);
//	memcpy(&thisItem.SectorID, &sector, 4);
//
//	if (box->pivot == 0) {
//		for (int i = 0; i <= 3; i++)
//			if (box->itemTab[box->pivot].SectorID == sector)
//				counter++;
//		if (counter == 4) {
//			thisItem.Iterator = box->itemTab[box->pivot].Iterator++;
//			memcpy(&box->itemTab[newPivot], &thisItem, sizeof(thisItem));
//		} else {
//			thisItem.Iterator = 1;
//			memcpy(&box->itemTab[newPivot], &thisItem, sizeof(thisItem));
//		}
//
//	} else {
//
//	}
}

uint8_t pushItem(Stos_typeDef** top, uint8_t* card, uint8_t* sector) {
	uint8_t counter =0 ;
	Stos_typeDef *nowy;
	    nowy = (Stos_typeDef *)malloc(sizeof(Stos_typeDef));
//	    nowy->key = liczba;
	    memcpy(nowy->object.CardID,card,4);
	    memcpy(nowy->object.SectorID,sector,4);
	    for (int i = 0; i <= 3; i++)
	    			if ((*top)->object.SectorID[i] == sector[i])
	    				counter++;
	    			else break;
	    if(counter == 4)
	    {
	    	  nowy->object.Iterator = (*top)->object.Iterator + 1;
	    }else{
	    	nowy->object.Iterator = 1;
	    }


	    nowy->previous=0;
	    if(*top == 0)
	    *top = nowy;
	    else {
	    nowy->previous=*top;
	    *top = nowy;
	         }
	return 1;
}

Stos_typeDef popItem(Stos_typeDef** top)
{
	Stos_typeDef *tmp;
	Stos_typeDef result ;
	result.object.Iterator = 255 ;

	     if (*top ==0)
	     {
			 printf("Stos pusty\n\r");
	     }
	     else{
	     tmp = *top;
	     (*top) =(*top)->previous;
//	     liczba =(*top)->key;
	     memcpy(&result,tmp,sizeof(Stos_typeDef));
	     free(tmp);
	     }
	     return result;
}

