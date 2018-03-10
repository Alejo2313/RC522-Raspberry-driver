/*
 * RC522.c
 *
 *  Created on: 10 mar. 2018
 *      Author: Alejo
 */


#include "RC522.h"

const uint8_t antenna_Gain[] = {18, 23, 18, 23, 33,38,43,48};

void RC522_Init(void) {
	#ifdef SPI_DEV
	 printf("Setup: %d \n",wiringPiSPISetup(spidev0, DEFAULT_SPEED));

	#endif

	RC522_Reset();

	RC522_WriteRegister(MFRC522_REG_T_MODE, 0x8D);
	RC522_WriteRegister(MFRC522_REG_T_PRESCALER, 0x3E);
	RC522_WriteRegister(MFRC522_REG_T_RELOAD_L, 30);
	RC522_WriteRegister(MFRC522_REG_T_RELOAD_H, 0);

	RC522_WriteRegister(MFRC522_REG_RF_CFG, 0x70);
	RC522_WriteRegister(MFRC522_REG_TX_AUTO, 0x40);
	RC522_WriteRegister(MFRC522_REG_MODE, 0x3D);

	RC522_AntennaOn();
}



Status_t RC522_Check(uint8_t* id) {
	Status_t status;

	status = RC522_Request(PICC_REQIDL, id);
	if (status == STATUS_OK) {
		status = RC522_Anticoll(id);
	}
	RC522_Halt();

	return status;
}


/*I/O Functions*/

void RC522_WriteRegister(uint8_t addr, uint8_t val) {
	uint8_t buffer[2];

	#ifdef SPI_DEV
		buffer[0] = (addr << 1) & 0x7E;
		buffer[1] = val;
		wiringPiSPIDataRW(spidev0,buffer,2);
	#endif
}


uint8_t RC522_ReadRegister(uint8_t addr) {
	uint8_t buffer[2];
	buffer[0]= ((addr << 1) & 0x7E) | 0x80;
	buffer[1]= 0x00;
	#ifdef SPI_DEV
		wiringPiSPIDataRW(spidev0, buffer, 2);
	#endif
	return buffer[1];
}



void RC522_SetBitMask(uint8_t reg, uint8_t mask) {
	RC522_WriteRegister(reg, RC522_ReadRegister(reg) | mask);
}


void RC522_ClearBitMask(uint8_t reg, uint8_t mask){
	RC522_WriteRegister(reg, RC522_ReadRegister(reg) & (~mask));
}




/*Antenna functions*/
void RC522_AntennaOn(void) {
	uint8_t temp;

	temp = RC522_ReadRegister(MFRC522_REG_TX_CONTROL);
	if ((temp & 0x03) != 0X03 ) {
		RC522_SetBitMask(MFRC522_REG_TX_CONTROL, 0x03);
	}
}

void RC522_AntennaOff(void) {
	RC522_ClearBitMask(MFRC522_REG_TX_CONTROL, 0x03);
}
uint8_t RC522_GetAntennaGain(){
	uint8_t temp;
	temp = (RC522_ReadRegister(MFRC522_REG_RF_CFG) &0x70)>>4;
	return antenna_Gain[temp];
}



void RC522_Reset(void) {
	RC522_WriteRegister(MFRC522_REG_COMMAND, PCD_RESETPHASE);
}

/*Tag functions*/

Status_t RC522_Request(uint8_t reqMode, uint8_t* TagType) {
	Status_t status;
	uint16_t backBits;			//The received data bits

	RC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x07);		//TxLastBists = BitFramingReg[2..0]	???

	TagType[0] = reqMode;
	status = RC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != STATUS_OK) || (backBits != 0x10)) {
		status = STATUS_ERROR;
	}

	return status;
}



Status_t RC522_ToCard(uint8_t command, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint16_t* backLen) {

	Status_t status = STATUS_ERROR;
	uint8_t irqEn = 0x00;
	uint8_t waitIRq = 0x00;
	uint8_t lastBits;


	uint8_t n;
	uint16_t i;

	switch (command) {
		case PCD_AUTHENT: {
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE: {
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
	}

	RC522_WriteRegister(MFRC522_REG_COMMAND, PCD_IDLE);

	RC522_WriteRegister(MFRC522_REG_COMM_IE_N, irqEn | 0x80);
	RC522_ClearBitMask(MFRC522_REG_COMM_IRQ, 0x80);
	RC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);

	for (i = 0; i < sendLen; i++) {
		RC522_WriteRegister(MFRC522_REG_FIFO_DATA, sendData[i]);
	}

	//Execute the command
	RC522_WriteRegister(MFRC522_REG_COMMAND, command);
	if (command == PCD_TRANSCEIVE) {
		RC522_SetBitMask(MFRC522_REG_BIT_FRAMING, 0x80);		//StartSend=1,transmission of data starts
	}

	i = 2000;	//i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
	do {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = RC522_ReadRegister(MFRC522_REG_COMM_IRQ);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	RC522_ClearBitMask(MFRC522_REG_BIT_FRAMING, 0x80);			//StartSend=0

	if (i != 0)  {
		if (!(RC522_ReadRegister(MFRC522_REG_ERROR) & 0x13)) {
			status = STATUS_OK;

			if (command == PCD_TRANSCEIVE) {
				n = RC522_ReadRegister(MFRC522_REG_FIFO_LEVEL);
				lastBits = RC522_ReadRegister(MFRC522_REG_CONTROL) & 0x07;
				if (lastBits) {
					*backLen = (n - 1) * 8 + lastBits;
				} else {
					*backLen = n * 8;
				}

				if (n == 0) {
					n = 1;
				}
				if (n > MFRC522_MAX_LEN) {
					n = MFRC522_MAX_LEN;
				}

				for (i = 0; i < n; i++) {
					backData[i] = RC522_ReadRegister(MFRC522_REG_FIFO_DATA);
				}
			}
		} else {
			status = STATUS_ERROR;
		}
	}

	return status;
}


Status_t RC522_Anticoll(uint8_t* serNum) {
	Status_t status;
	uint8_t i;
	uint8_t serNumCheck = 0;
	uint16_t unLen;

	RC522_WriteRegister(MFRC522_REG_BIT_FRAMING, 0x00);		//TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = RC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	if (status == STATUS_OK) {
		//Check card serial number
		for (i = 0; i < 4; i++) {
			serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i]) {
			status = STATUS_ERROR;
		}
	}
	return status;
}

void RC522_CalculateCRC(uint8_t*  pIndata, uint8_t len, uint8_t* pOutData) {
	uint8_t i, n;

	RC522_ClearBitMask(MFRC522_REG_DIV_IRQ, 0x04);			//CRCIrq = 0
	RC522_SetBitMask(MFRC522_REG_FIFO_LEVEL, 0x80);			//Clear the FIFO pointer
	//Write_MFRC522(CommandReg, PCD_IDLE);

	//Writing data to the FIFO
	for (i = 0; i < len; i++) {
		RC522_WriteRegister(MFRC522_REG_FIFO_DATA, *(pIndata+i));
	}
	RC522_WriteRegister(MFRC522_REG_COMMAND, PCD_CALCCRC);

	//Wait CRC calculation is complete
	i = 0xFF;
	do {
		n = RC522_ReadRegister(MFRC522_REG_DIV_IRQ);
		i--;
	} while ((i!=0) && !(n&0x04));			//CRCIrq = 1

	//Read CRC calculation result
	pOutData[0] = RC522_ReadRegister(MFRC522_REG_CRC_RESULT_L);
	pOutData[1] = RC522_ReadRegister(MFRC522_REG_CRC_RESULT_M);
}


uint8_t RC522_SelectTag(uint8_t* serNum) {
	uint8_t i;
	Status_t status;
	uint8_t size;
	uint16_t recvBits;
	uint8_t buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;
	for (i = 0; i < 5; i++) {
		buffer[i+2] = *(serNum+i);
	}
	RC522_CalculateCRC(buffer, 7, &buffer[7]);
	status = RC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == STATUS_OK) && (recvBits == 0x18)) {
		size = buffer[0];
	} else {
		size = 0;
	}

	return size;
}

Status_t RC522_Auth(uint8_t authMode, uint8_t BlockAddr, uint8_t* Sectorkey, uint8_t* serNum) {
	Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[12];

	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = BlockAddr;
	for (i = 0; i < 6; i++) {
		buff[i+2] = *(Sectorkey+i);
	}
	for (i=0; i<4; i++) {
		buff[i+8] = *(serNum+i);
	}
	status = RC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

	if ((status != STATUS_OK) || (!(RC522_ReadRegister(MFRC522_REG_STATUS2) & 0x08))) {
		status = STATUS_ERROR;
	}

	return status;
}

Status_t RC522_Read(uint8_t blockAddr, uint8_t* recvData) {
	Status_t status;
	uint16_t unLen;

	recvData[0] = PICC_READ;
	recvData[1] = blockAddr;
	RC522_CalculateCRC(recvData,2, &recvData[2]);
	status = RC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

	if ((status != STATUS_OK) || (unLen != 0x90)) {
		status = STATUS_ERROR;
	}

	return status;
}

Status_t RC522_Write(uint8_t blockAddr, uint8_t* writeData) {
	Status_t status;
	uint16_t recvBits;
	uint8_t i;
	uint8_t buff[18];

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	RC522_CalculateCRC(buff, 2, &buff[2]);
	status = RC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	if ((status != STATUS_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
		status = STATUS_ERROR;
	}

	if (status == STATUS_OK) {
		for (i = 0; i < 16; i++) {
			buff[i] = *(writeData+i);
		}
		RC522_CalculateCRC(buff, 16, &buff[16]);
		status = RC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != STATUS_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
			status = STATUS_ERROR;
		}
	}

	return status;
}



void RC522_Halt(void) {
	uint16_t unLen;
	uint8_t buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	RC522_CalculateCRC(buff, 2, &buff[2]);

	RC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}

