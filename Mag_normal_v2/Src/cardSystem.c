/*
 * cardSystem.c
 *
 *  Created on: 08.06.2019
 *      Author: kkarp
 */

#include "cardSystem.h"

void itemAdd(Box_typeDef* box, uint8_t* card, uint8_t* sector) {
	item thisItem;
	uint8_t counter = 0;
	uint8_t newPivot = box->pivot++;
	if (newPivot > 30)
		newPivot = 0;

	memcpy(&thisItem.CardID, &card, 4);
	memcpy(&thisItem.SectorID, &sector, 4);

	if (box->pivot == 0) {
		for (int i = 0; i <= 3; i++)
			if (box->itemTab[box->pivot].SectorID == sector)
				counter++;
		if (counter == 4) {
			thisItem.Iterator = box->itemTab[box->pivot].Iterator++;
			memcpy(&box->itemTab[newPivot], &thisItem, sizeof(thisItem));
		} else {
			thisItem.Iterator = 1;
			memcpy(&box->itemTab[newPivot],&thisItem, sizeof(thisItem));
		}

	} else {

	}
}
