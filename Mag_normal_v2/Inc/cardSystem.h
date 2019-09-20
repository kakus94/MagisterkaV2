/*
 * cardSystem.h
 *
 *  Created on: 08.06.2019
 *      Author: kkarp
 */

#include "main.h"

#ifndef CARDSYSTEM_H_
#define CARDSYSTEM_H_

typedef 	struct{
	uint8_t CardID[4];
	uint8_t SectorID[4];
	uint16_t Iterator;
	}__packed item;

typedef struct stos{
	item object;
	struct stos * previous;

}Stos_typeDef;


void itemAdd(Stos_typeDef* box,uint8_t* card,uint8_t* sector);
uint8_t pushItem(Stos_typeDef** item,uint8_t* card,uint8_t* sector,uint16_t iterator);
Stos_typeDef popItem(Stos_typeDef** object);

#endif /* CARDSYSTEM_H_ */
