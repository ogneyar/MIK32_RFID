
#include "RFID.h"
#include "mik32_hal_scr1_timer.h"
#include <stddef.h>


Uid uid;								// Used by PICC_ReadCardSerial().
uint16_t _chipSelectPin;
GPIO_TypeDef * _chipSelectPort;
uint16_t _resetPowerDownPin;
GPIO_TypeDef * _resetPowerDownPort;
SCR1_TIMER_HandleTypeDef hscr1_timer; // необходимо для функции delay
typedef uint8_t bool;



// Возврат байта идентификатора
uint8_t get_uid(uint8_t number)
{
	return uid.uidByte[number];
}

// Инициализация таймера ядра scr1
static void Scr1_Timer_Init(void)
{
    hscr1_timer.Instance = SCR1_TIMER;
    hscr1_timer.ClockSource = SCR1_TIMER_CLKSRC_INTERNAL; /* Источник тактирования */
    hscr1_timer.Divider = 0;                              /* Делитель частоты 10-битное число */
    HAL_SCR1_Timer_Init(&hscr1_timer);
}

// Запуск таймера
void MILLIS_Start(uint32_t time)
{
	HAL_SCR1_Timer_Start(&hscr1_timer, time);
}

// Возврат статуса таймера
int MILLIS_GetFlag(void)
{
	return HAL_SCR1_Timer_GetFlagCMP(&hscr1_timer);
}

// Остановка таймера ядра
void MILLIS_Stop(void)
{
	HAL_SCR1_Timer_Disable(&hscr1_timer);
}

// Задержка в мс
void delay(uint32_t time)
{
	HAL_DelayMs(&hscr1_timer, time);
}

// Установка режима работы заданного пина
void pinMode(GPIO_TypeDef *Port, uint32_t Pin, uint8_t OutInput)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.DS = HAL_GPIO_DS_2MA;

	if (OutInput) {
		// Port->DIRECTION_OUT = (Pin);
		GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_OUTPUT;
		GPIO_InitStruct.Pull = HAL_GPIO_PULL_NONE;
	}else {
		// Port->DIRECTION_IN = (Pin);
		GPIO_InitStruct.Mode = HAL_GPIO_MODE_GPIO_INPUT;
		GPIO_InitStruct.Pull = HAL_GPIO_PULL_UP;
	}
	HAL_GPIO_Init(Port, &GPIO_InitStruct);
}

// Чтение состояния заданного пина
uint8_t digitalRead(GPIO_TypeDef *Port, uint32_t Pin)
{
	if (Port->STATE & Pin) return 1;	
	return 0;
}

// Установка состояния заданного пина
void digitalWrite(GPIO_TypeDef *Port, uint32_t Pin, uint8_t SetReset)
{
	HAL_GPIO_WritePin(Port, Pin, SetReset);
}

// Инициализация RFID
void RFID_init(GPIO_TypeDef *chipSelectPort, uint16_t chipSelectPin, GPIO_TypeDef *resetPowerDownPort, uint16_t resetPowerDownPin)
{
	_chipSelectPin = chipSelectPin;
	_chipSelectPort = chipSelectPort;
	_resetPowerDownPin = resetPowerDownPin;
	_resetPowerDownPort = resetPowerDownPort;

  	Scr1_Timer_Init(); // необходимо для функции delay	

    PCD_Init();                     // Инициализация модуля
    PCD_SetAntennaGain(RxGain_max); // Установка усиления антенны
    PCD_AntennaOff();               // Перезагружаем антенну
    PCD_AntennaOn();                // Включаем антенну
}


void PCD_WriteRegister(enum PCD_Register reg, byte value ) 
{
	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);		// Select slave
	SPI_SendData(reg);
	SPI_SendData(value);
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);		// Release slave again
} 


void PCD_WriteRegister_Array(enum PCD_Register reg, byte count, byte *values)
{
	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);		// Select slave
	SPI_SendData(reg);
	for (byte index = 0; index < count; index++) {
		SPI_SendData(values[index]);
	}
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);		// Release slave again
}


byte PCD_ReadRegister(enum PCD_Register reg	)
{
	byte value;
	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);			// Select slave
	SPI_SendData(0x80 | reg);
	value = SPI_ReceiveData();
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);			// Release slave again
	return value;
}


void PCD_ReadRegister_Array(enum PCD_Register reg, byte count, byte *values, byte rxAlign) {
	if (count == 0) {
		return;
	}
	byte address = 0x80 | reg;
	byte index = 0;

	digitalWrite(_chipSelectPort, _chipSelectPin, LOW);		// Select slave
	count--;
	SPI_transfer(address);
	if (rxAlign) {
		byte mask = (0xFF << rxAlign) & 0xFF;
		byte value = SPI_transfer(address);
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
	}
	while (index < count) {
		values[index] = SPI_transfer(address);
		index++;
	}
	values[index] = SPI_transfer(0);
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);	// Release slave again
}


void PCD_SetRegisterBitMask(enum PCD_Register reg, byte mask)
{ 
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp | mask);			// set bit mask
}


void PCD_ClearRegisterBitMask(enum PCD_Register reg, byte mask)
{
	byte tmp;
	tmp = PCD_ReadRegister(reg);
	PCD_WriteRegister(reg, tmp & (~mask));		// clear bit mask
}


enum StatusCode PCD_CalculateCRC(byte *data, byte length, byte *result) {
	PCD_WriteRegister(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_WriteRegister(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_WriteRegister(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_WriteRegister_Array(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_WriteRegister(CommandReg, PCD_CalcCRC);		// Start the calculation
	
    HAL_SCR1_Timer_Start(&hscr1_timer, 89);
	bool completed = false;

	do {
		byte n = PCD_ReadRegister(DivIrqReg);
		if (n & 0x04) {									// CRCIRq bit set - calculation done
			PCD_WriteRegister(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
			result[0] = PCD_ReadRegister(CRCResultRegL);
			result[1] = PCD_ReadRegister(CRCResultRegH);
			completed = true;
			break;
		}
	}
	while (HAL_SCR1_Timer_GetFlagCMP(&hscr1_timer) == 0);

    HAL_SCR1_Timer_Disable(&hscr1_timer);

	if (completed)
		return STATUS_OK;

	return STATUS_TIMEOUT;
}



/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

void PCD_Init() {
	bool hardReset = false;

	pinMode(_chipSelectPort, _chipSelectPin, _OUTPUT);
	digitalWrite(_chipSelectPort, _chipSelectPin, HIGH);
	
	if (_resetPowerDownPin != UNUSED_PIN) {
		pinMode(_resetPowerDownPort, _resetPowerDownPin, _INPUT);
	
		if (digitalRead(_resetPowerDownPort, _resetPowerDownPin) == LOW) {	// The MFRC522 chip is in power down mode.

			pinMode(_resetPowerDownPort, _resetPowerDownPin, _OUTPUT);		// Now set the resetPowerDownPin as digital output.
			digitalWrite(_resetPowerDownPort, _resetPowerDownPin, LOW);		// Make sure we have a clean LOW state.
			delay(1);
			digitalWrite(_resetPowerDownPort, _resetPowerDownPin, HIGH);		// Exit power down mode. This triggers a hard reset.
			delay(50);
			hardReset = true;
		}
	}

	if (!hardReset) { // Perform a soft reset if we haven't triggered a hard reset above.
		PCD_Reset();
	}
	
	// Reset baud rates
	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_WriteRegister(ModWidthReg, 0x26);

	PCD_WriteRegister(TModeReg, 0x80);			// TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
	PCD_WriteRegister(TPrescalerReg, 0xA9);		// TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
	PCD_WriteRegister(TReloadRegH, 0x03);		// Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
	PCD_WriteRegister(TReloadRegL, 0xE8);
	
	PCD_WriteRegister(TxASKReg, 0x40);		// Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
	PCD_WriteRegister(ModeReg, 0x3D);		// Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
	PCD_AntennaOn();						// Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
}


void PCD_Reset() {
	PCD_WriteRegister(CommandReg, PCD_SoftReset);
	
	uint8_t count = 0;
	do {
		delay(50);
	} while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
}


void PCD_AntennaOn() {
	byte value = PCD_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		PCD_WriteRegister(TxControlReg, value | 0x03);
	}
}


void PCD_AntennaOff() {
	PCD_ClearRegisterBitMask(TxControlReg, 0x03);
}


byte PCD_GetAntennaGain() {
	return PCD_ReadRegister(RFCfgReg) & (0x07<<4);
}


void PCD_SetAntennaGain(byte mask) {
	if (PCD_GetAntennaGain() != mask) {						// only bother if there is a change
		PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	}
}



/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

enum StatusCode PCD_TransceiveData( byte *sendData,		///< Pointer to the data to transfer to the FIFO.
									byte sendLen,		///< Number of bytes to transfer to the FIFO.
									byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
									byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
									byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits. Default nullptr.
									byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
									byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	byte waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}


enum StatusCode PCD_CommunicateWithPICC(byte command,		///< The command to execute. One of the PCD_Command enums.
										byte waitIRq,		///< The bits in the ComIrqReg register that signals successful completion of the command.
										byte *sendData,		///< Pointer to the data to transfer to the FIFO.
										byte sendLen,		///< Number of bytes to transfer to the FIFO.
										byte *backData,		///< nullptr or pointer to buffer if data should be read back after executing the command.
										byte *backLen,		///< In: Max number of bytes to write to *backData. Out: The number of bytes returned.
										byte *validBits,	///< In/Out: The number of valid bits in the last byte. 0 for 8 valid bits.
										byte rxAlign,		///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
										byte checkCRC		///< In: True => The last two bytes of the response is assumed to be a CRC_A that must be validated.
) {
	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;		// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
	
	PCD_WriteRegister(CommandReg, PCD_Idle);			// Stop any active command.
	PCD_WriteRegister(ComIrqReg, 0x7F);					// Clear all seven interrupt request bits
	PCD_WriteRegister(FIFOLevelReg, 0x80);				// FlushBuffer = 1, FIFO initialization

	PCD_WriteRegister_Array(FIFODataReg, sendLen, sendData);	// Write sendData to the FIFO

	PCD_WriteRegister(BitFramingReg, bitFraming);		// Bit adjustments
	PCD_WriteRegister(CommandReg, command);				// Execute the command
	if (command == PCD_Transceive) {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80);	// StartSend=1, transmission of data starts
	}	

    HAL_SCR1_Timer_Start(&hscr1_timer, 36);
	bool completed = false;

	do {
		byte n = PCD_ReadRegister(ComIrqReg);	// ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {					// One of the interrupts that signal success has been set.
			completed = true;
			break;
		}
		if (n & 0x01) {						// Timer interrupt - nothing received in 25ms
			break;
		}
	}
	while (HAL_SCR1_Timer_GetFlagCMP(&hscr1_timer) == 0);

    HAL_SCR1_Timer_Disable(&hscr1_timer);

	if (!completed) {
		return STATUS_TIMEOUT;
	}
	
	byte errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
  
	byte _validBits = 0;
	
	if (backData && backLen) {
		byte n = PCD_ReadRegister(FIFOLevelReg);	// Number of bytes in the FIFO
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;											// Number of bytes returned
		PCD_ReadRegister_Array(FIFODataReg, n, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_ReadRegister(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) {
			*validBits = _validBits;
		}
	}
	
	if (errorRegValue & 0x08) {		// CollErr
		return STATUS_COLLISION;
	}
	
	if (backData && backLen && checkCRC) {
		if (*backLen == 1 && _validBits == 4) {
			return STATUS_MIFARE_NACK;
		}
		if (*backLen < 2 || _validBits != 0) {
			return STATUS_CRC_WRONG;
		}
		byte controlBuffer[2];
		enum StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) {
			return STATUS_CRC_WRONG;
		}
	}

	return STATUS_OK;
} // End PCD_CommunicateWithPICC()


enum StatusCode PICC_RequestA(byte *bufferATQA, byte *bufferSize)
{
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}


enum StatusCode PICC_WakeupA(byte *bufferATQA, byte *bufferSize)
{
	return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
}


enum StatusCode PICC_REQA_or_WUPA(byte command, byte *bufferATQA, byte *bufferSize)
{
	byte validBits;
	enum StatusCode status;
	
	if (bufferATQA == NULL || *bufferSize < 2) {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, false);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
	return STATUS_OK;
}


enum StatusCode PICC_Select(Uid *uid, byte validBits)
{
	bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	byte cascadeLevel = 1;
	enum StatusCode result;
	byte count;
	byte checkBit;
	byte index;
	byte uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	byte buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	byte bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	byte rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	byte txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	byte *responseBuffer;
	byte responseLength;
	
	if (validBits > 80) {
		return STATUS_INVALID;
	}
	
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	
	uidComplete = false;
	while (!uidComplete) {
		
		switch (cascadeLevel) {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
			
			case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
			
			default:
				return STATUS_INTERNAL_ERROR;
				break;
		}
		
		currentLevelKnownBits = validBits - (8 * uidIndex);
		if (currentLevelKnownBits < 0) {
			currentLevelKnownBits = 0;
		}
		index = 2; // destination index in buffer[]
		if (useCascadeTag) {
			buffer[index++] = PICC_CMD_CT;
		}
		byte bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
		if (bytesToCopy) {
			byte maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
			if (bytesToCopy > maxBytes) {
				bytesToCopy = maxBytes;
			}
			for (count = 0; count < bytesToCopy; count++) {
				buffer[index++] = uid->uidByte[uidIndex + count];
			}
		}
		if (useCascadeTag) {
			currentLevelKnownBits += 8;
		}
		
		selectDone = false;
		while (!selectDone) {
			
			if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
				if (result != STATUS_OK) {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else { // This is an ANTICOLLISION.
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, false);
			if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
				byte valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				byte collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) {
				return result;
			}
			else { // STATUS_OK
				if (currentLevelKnownBits >= 32) { // This was a SELECT.
					selectDone = true; // No more anticollision 
				}
				else { // This was an ANTICOLLISION.
					currentLevelKnownBits = 32;
				}
			}
		} // End of while (!selectDone)
		
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	} // End of while (!uidComplete)
		
	uid->size = 3 * cascadeLevel + 1;

	return STATUS_OK;
} // End PICC_Select()




/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

bool PICC_IsNewCardPresent() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);

	PCD_WriteRegister(TxModeReg, 0x00);
	PCD_WriteRegister(RxModeReg, 0x00);
	PCD_WriteRegister(ModWidthReg, 0x26);

	enum StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);

	return (result == STATUS_OK || result == STATUS_COLLISION);
}


bool PICC_ReadCardSerial() {
	enum StatusCode result = PICC_Select(&uid, 0);
	return (result == STATUS_OK);
}
