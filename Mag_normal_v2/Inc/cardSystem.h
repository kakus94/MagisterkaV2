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
	}item;

typedef struct{
	uint8_t pivot;
	uint8_t readLast;
	item itemTab[30];

}Box_typeDef;


void itemAdd(Box_typeDef* box,uint8_t* card,uint8_t* sector);

#endif /* CARDSYSTEM_H_ */
