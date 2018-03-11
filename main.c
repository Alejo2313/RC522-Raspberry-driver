/*
 * main.c
 *
 *  Created on: 10 mar. 2018
 *      Author: Alejo
 */
#include "RC522.h"
int main(void){
	uint8_t tmp = 0;
	uint8_t ID[16];
	int n, i;
	printf("hello world \n");
	if(RC522_Init() == STATUS_ERROR)
		return 1;

	//while(1){}
	printf("REG: %d \n ", RC522_ReadRegister(MFRC522_REG_T_RELOAD_L));
	while(1){

		if(RC522_Check(ID) == STATUS_OK){
			printf("found tag.! ");
			RC522_Anticoll(ID);
			for(i = 0; i <8 ; i++){
				printf("%d ", ID[i]);
			}
			printf("\n");
			printf("Size: %d",RC522_SelectTag(ID));

		}
	}
	return 0;
}
