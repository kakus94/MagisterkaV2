// Functions to manage the nRF24L01+ transceiver

#include "nrf24.h"
#include "stdio.h"s

// Read a register
// input:
//   reg - number of register to read
// return: value of register
static uint8_t nRF24_ReadReg(uint8_t reg) {
	uint8_t value;

	nRF24_CSN_L();
	nRF24_LL_RW(reg & nRF24_MASK_REG_MAP);
	value = nRF24_LL_RW(nRF24_CMD_NOP);
	nRF24_CSN_H();

	return value;
}

// Write a new value to register
// input:
//   reg - number of register to write
//   value - value to write
static void nRF24_WriteReg(uint8_t reg, uint8_t value) {
	nRF24_CSN_L();
	if (reg < nRF24_CMD_W_REGISTER) {
		// This is a register access
		nRF24_LL_RW(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP ));
		nRF24_LL_RW(value);
	} else {
		// This is a single byte command or future command/register
		nRF24_LL_RW(reg);
		if ((reg != nRF24_CMD_FLUSH_TX ) && (reg != nRF24_CMD_FLUSH_RX )
				&& (reg != nRF24_CMD_REUSE_TX_PL ) && (reg != nRF24_CMD_NOP )) {
			// Send register value
			nRF24_LL_RW(value);
		}
	}
	nRF24_CSN_H();
}

// Read a multi-byte register
// input:
//   reg - number of register to read
//   pBuf - pointer to the buffer for register data
//   count - number of bytes to read
static void nRF24_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--) {
		*pBuf++ = nRF24_LL_RW(nRF24_CMD_NOP);
	}
	nRF24_CSN_H();
}

// Write a multi-byte register
// input:
//   reg - number of register to write
//   pBuf - pointer to the buffer with data to write
//   count - number of bytes to write
static void nRF24_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--) {
		nRF24_LL_RW(*pBuf++);
	}
	nRF24_CSN_H();
}

// Set transceiver to it's initial state
// note: RX/TX pipe addresses remains untouched
void nRF24_Init(void) {
	// Write to registers their initial values
	nRF24_WriteReg(nRF24_REG_CONFIG, 0x08);
	nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F);
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x03);
	nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03);
	nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03);
	nRF24_WriteReg(nRF24_REG_RF_CH, 0x02);
	nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E);
	nRF24_WriteReg(nRF24_REG_STATUS, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00);
	nRF24_WriteReg(nRF24_REG_DYNPD, 0x00);
	nRF24_WriteReg(nRF24_REG_FEATURE, 0x00);

	// Clear the FIFO's
	nRF24_FlushRX();
	nRF24_FlushTX();

	// Clear any pending interrupt flags
	nRF24_ClearIRQFlags();

	// Deassert CSN pin (chip release)
	nRF24_CSN_H();
}

// Check if the nRF24L01 present
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nRF24_Check(void) {
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *) nRF24_TEST_ADDR;

	// Write test TX address and read TX_ADDR register
	nRF24_WriteMBReg(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	nRF24_ReadMBReg(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);

	// Compare buffers, return error on first mismatch
	for (i = 0; i < 5; i++) {
		if (rxbuf[i] != *ptr++)
			return 0;
	}

	return 1;
}

// Control transceiver power mode
// input:
//   mode - new state of power mode, one of nRF24_PWR_xx values
void nRF24_SetPowerMode(uint8_t mode) {
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP) {
		// Set the PWR_UP bit of CONFIG register to wake the transceiver
		// It goes into Stanby-I mode with consumption about 26uA
		reg |= nRF24_CONFIG_PWR_UP;
	} else {
		// Clear the PWR_UP bit of CONFIG register to put the transceiver
		// into power down mode with consumption about 900nA
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

// Set transceiver operational mode
// input:
//   mode - operational mode, one of nRF24_MODE_xx values
void nRF24_SetOperationalMode(uint8_t mode) {
	uint8_t reg;

	// Configure PRIM_RX bit of the CONFIG register
	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX );
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

// Configure transceiver CRC scheme
// input:
//   scheme - CRC scheme, one of nRF24_CRC_xx values
// note: transceiver will forcibly turn on the CRC in case if auto acknowledgment
//       enabled for at least one RX pipe
void nRF24_SetCRCScheme(uint8_t scheme) {
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC );
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

// Set frequency channel
// input:
//   channel - radio frequency channel, value from 0 to 127
// note: frequency will be (2400 + channel)MHz
// note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be reset
void nRF24_SetRFChannel(uint8_t channel) {
	nRF24_WriteReg(nRF24_REG_RF_CH, channel);
}

// Set automatic retransmission parameters
// input:
//   ard - auto retransmit delay, one of nRF24_ARD_xx values
//   arc - count of auto retransmits, value form 0 to 15
// note: zero arc value means that the automatic retransmission disabled
void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc) {
	// Set auto retransmit settings (SETUP_RETR register)
	nRF24_WriteReg(nRF24_REG_SETUP_RETR,
			(uint8_t) ((ard << 4) | (arc & nRF24_MASK_RETR_ARC )));
}

// Set of address widths
// input:
//   addr_width - RX/TX address field width, value from 3 to 5
// note: this setting is common for all pipes
void nRF24_SetAddrWidth(uint8_t addr_width) {
	nRF24_WriteReg(nRF24_REG_SETUP_AW, addr_width - 2);
}

// Set static RX address for a specified pipe
// input:
//   pipe - pipe to configure address, one of nRF24_PIPEx values
//   addr - pointer to the buffer with address
// note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
// note: buffer length must be equal to current address width of transceiver
// note: for pipes[2..5] only first byte of address will be written because
//       other bytes of address equals to pipe1
// note: for pipes[2..5] only first byte of address will be written because
//       pipes 1-5 share the four most significant address bytes
void nRF24_SetAddr(uint8_t pipe, const uint8_t *addr) {
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe) {
	case nRF24_PIPETX:
	case nRF24_PIPE0:
	case nRF24_PIPE1:
		// Get address width
		addr_width = nRF24_ReadReg(nRF24_REG_SETUP_AW) + 1;
		// Write address in reverse order (LSByte first)
		addr += addr_width;
		nRF24_CSN_L();
		nRF24_LL_RW(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
		do {
			nRF24_LL_RW(*addr--);
		} while (addr_width--);
		nRF24_CSN_H();
		break;
	case nRF24_PIPE2:
	case nRF24_PIPE3:
	case nRF24_PIPE4:
	case nRF24_PIPE5:
		// Write address LSBbyte (only first byte from the addr buffer)
		nRF24_WriteReg(nRF24_ADDR_REGS[pipe], *addr);
		break;
	default:
		// Incorrect pipe number -> do nothing
		break;
	}
}

// Configure RF output power in TX mode
// input:
//   tx_pwr - RF output power, one of nRF24_TXPWR_xx values
void nRF24_SetTXPower(uint8_t tx_pwr) {
	uint8_t reg;

	// Configure RF_PWR[2:1] bits of the RF_SETUP register
	reg = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

// Configure transceiver data rate
// input:
//   data_rate - data rate, one of nRF24_DR_xx values
void nRF24_SetDataRate(uint8_t data_rate) {
	uint8_t reg;

	// Configure RF_DR_LOW[5] and RF_DR_HIGH[3] bits of the RF_SETUP register
	reg = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}

// Configure a specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
//   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
//   payload_len - payload length in bytes
void nRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len) {
	uint8_t reg;

	// Enable the specified pipe (EN_RXADDR register)
	reg = (nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

	// Set auto acknowledgment for a specified pipe (EN_AA register)
	reg = nRF24_ReadReg(nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON) {
		reg |= (1 << pipe);
	} else {
		reg &= ~(1 << pipe);
	}
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

// Disable specified RX pipe
// input:
//   PIPE - number of RX pipe, value from 0 to 5
void nRF24_ClosePipe(uint8_t pipe) {
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
}

// Enable the auto retransmit (a.k.a. enhanced ShockBurst) for the specified RX pipe
// input:
//   pipe - number of the RX pipe, value from 0 to 5
void nRF24_EnableAA(uint8_t pipe) {
	uint8_t reg;

	// Set bit in EN_AA register
	reg = nRF24_ReadReg(nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

// Disable the auto retransmit (a.k.a. enhanced ShockBurst) for one or all RX pipes
// input:
//   pipe - number of the RX pipe, value from 0 to 5, any other value will disable AA for all RX pipes
void nRF24_DisableAA(uint8_t pipe) {
	uint8_t reg;

	if (pipe > 5) {
		// Disable Auto-ACK for ALL pipes
		nRF24_WriteReg(nRF24_REG_EN_AA, 0x00);
	} else {
		// Clear bit in the EN_AA register
		reg = nRF24_ReadReg(nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		nRF24_WriteReg(nRF24_REG_EN_AA, reg);
	}
}

// Get value of the STATUS register
// return: value of STATUS register
uint8_t nRF24_GetStatus(void) {
	return nRF24_ReadReg(nRF24_REG_STATUS);
}

// Get pending IRQ flags
// return: current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
uint8_t nRF24_GetIRQFlags(void) {
	return (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ );
}

// Get status of the RX FIFO
// return: one of the nRF24_STATUS_RXFIFO_xx values
uint8_t nRF24_GetStatus_RXFIFO(void) {
	return (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO );
}

// Get status of the TX FIFO
// return: one of the nRF24_STATUS_TXFIFO_xx values
// note: the TX_REUSE bit ignored
uint8_t nRF24_GetStatus_TXFIFO(void) {
	return ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO ) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t nRF24_GetRXSource(void) {
	return ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO ) >> 1);
}

// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t nRF24_GetRetransmitCounters(void) {
	return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
}

// Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
void nRF24_ResetPLOS(void) {
	uint8_t reg;

	// The PLOS counter is reset after write to RF_CH register
	reg = nRF24_ReadReg(nRF24_REG_RF_CH);
	nRF24_WriteReg(nRF24_REG_RF_CH, reg);
}

// Flush the TX FIFO
void nRF24_FlushTX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

// Flush the RX FIFO
void nRF24_FlushRX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

// Clear any pending IRQ flags
void nRF24_ClearIRQFlags(void) {
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits of the STATUS register
	reg = nRF24_ReadReg(nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	nRF24_WriteReg(nRF24_REG_STATUS, reg);
}

// Write TX payload
// input:
//   pBuf - pointer to the buffer with payload data
//   length - payload length in bytes
void nRF24_WritePayload(uint8_t *pBuf, uint8_t length) {
	nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}

// Read top level payload available in the RX FIFO
// input:
//   pBuf - pointer to the buffer to store a payload data
//   length - pointer to variable to store a payload length
// return: one of nRF24_RX_xx values
//   nRF24_RX_PIPEX - packet has been received from the pipe number X
//   nRF24_RX_EMPTY - the RX FIFO is empty
nRF24_RXResult nRF24_ReadPayload(uint8_t *pBuf, uint8_t *length) {
	uint8_t pipe;

	// Extract a payload pipe number from the STATUS register
	pipe = (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO ) >> 1;

	// RX FIFO empty?
	if (pipe < 6) {
		// Get payload length
		*length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]);

		// Read a payload from the RX FIFO
		if (*length) {
			nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nRF24_RXResult) pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}

nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length,
		uint32_t wait) {

	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT )) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}

	// Check the flags in STATUS register

	// Clear pending IRQ flags
	nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

//#########################################
void nRF24_EnableDynamicPayloads(void)
{
	nRF24_WriteReg(nRF24_REG_FEATURE,nRF24_ReadReg(nRF24_REG_FEATURE) | nRF24_EN_DPL);
	nRF24_WriteReg(nRF24_REG_DYNPD,0x1F);
}

void nRF24_DisableDynamicPayloads(void){
	nRF24_WriteReg(nRF24_REG_FEATURE,0);
	nRF24_WriteReg(nRF24_REG_DYNPD,0);
}


//#########################################

extern UART_HandleTypeDef huart1;

// Print nRF24L01+ current configuration (for debug purposes)
void nRF24_DumpConfig(void) {
	uint8_t i, j;
	uint8_t aw;
	uint8_t buf[5];
	uint8_t buffor[200];
	uint16_t size;
	// Dump nRF24L01+ configuration
	// CONFIG
	i = nRF24_ReadReg(nRF24_REG_CONFIG);
	size = sprintf((char*) buffor, (char*) "[0x%02X] 0x%02X ",
	nRF24_REG_CONFIG, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);

	i & 0x01 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "PRX ", sizeof("PRX "), 100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "PTX ", sizeof("PTX "), 100);
	i & 0x02 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "PowerUP ",
					sizeof("PowerUP "), 100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "PowerDown ",
					sizeof("PowerDown "), 100);
	i & 0x04 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "CRC2 ", sizeof("CRC2 "),
					100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "CRC1 ", sizeof("CRC1 "),
					100);
	i & 0x08 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "ENCRC=1 ",
					sizeof("ENCRC=1 "), 100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "ENCRC=0 ",
					sizeof("ENCRC=0 "), 100);
	i & 0x10 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "MaxRT=1 ",
					sizeof("MaxRT=1 "), 100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "MaxRT=0 ",
					sizeof("MaxRT=0 "), 100);
	i & 0x20 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "TX_DS=1 ",
					sizeof("TX_DS=1 "), 100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "TX_DS=0 ",
					sizeof("TX_DS=0 "), 100);
	i & 0x40 ?
			HAL_UART_Transmit(&huart1, (uint8_t*) "RX_DS=1\n\r",
					sizeof("RX_DS=1\n\r"), 100) :
			HAL_UART_Transmit(&huart1, (uint8_t*) "RX_DS=0\n\r",
					sizeof("RX_DS=0\n\r"), 100);

	// EN_AA
	i = nRF24_ReadReg(nRF24_REG_EN_AA);
	size = sprintf((char*) buffor, (char*) "[0x%02X] 0x%02X",
	nRF24_REG_EN_AA, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	for (int8_t q = 0; q <= 5; q++) {
		size = sprintf((char*) buffor, (char*) " P%d=", q);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
		(i >> 1) & 0x01 ?
				HAL_UART_Transmit(&huart1, (uint8_t*) "1", sizeof("1"), 100) :
				HAL_UART_Transmit(&huart1, (uint8_t*) "0", sizeof("0"), 100);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, 100);
	// EN_RXADDR
	i = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	size = sprintf((char*) buffor, (char*) "[0x%02X] 0x%02X",
	nRF24_REG_EN_RXADDR, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	for (int8_t q = 0; q <= 5; q++) {
		size = sprintf((char*) buffor, (char*) " P%d=", q);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
		(i >> 1) & 0x01 ?
				HAL_UART_Transmit(&huart1, (uint8_t*) "1", sizeof("1"), 100) :
				HAL_UART_Transmit(&huart1, (uint8_t*) "0", sizeof("0"), 100);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, 100);
	// SETUP_AW
	i = nRF24_ReadReg(nRF24_REG_SETUP_AW);
	size = sprintf((char*) buffor, (char*) "[0x%02X] 0x%02X ",
	nRF24_REG_SETUP_AW, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);

	switch (i) {
	case 1:
		HAL_UART_Transmit(&huart1, (uint8_t*) "AdressLenght=3bytes",
				sizeof("AdressLenght=3bytes"), 100);
		break;
	case 2:
		HAL_UART_Transmit(&huart1, (uint8_t*) "AdressLenght=4bytes",
				sizeof("AdressLenght=4bytes"), 100);
		break;
	case 3:
		HAL_UART_Transmit(&huart1, (uint8_t*) "AdressLenght=5bytes",
				sizeof("AdressLenght=5bytes"), 100);
		break;
	default:
		HAL_UART_Transmit(&huart1, (uint8_t*) "Error", sizeof("Error"), 100);
		break;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, 100);
	// SETUP_RETR
	i = nRF24_ReadReg(nRF24_REG_SETUP_RETR);
	uint8_t retransmitCount = i & 0x0F;
	uint8_t retransmitDelay = i >> 4 & 0x0F;
	size = sprintf((char*) buffor,
			"[0x%02X] 0x%02X ARD=%d ARC=%d (retr.delay=%uus, count=%u)\r\n",
			nRF24_REG_SETUP_RETR, i, retransmitDelay, retransmitCount,
			(retransmitDelay * 250) + 250, retransmitCount);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RF_CH
	i = nRF24_ReadReg(nRF24_REG_RF_CH);
	size = sprintf((char*) buffor, "[0x%02X] 0x%02X channel=%d (%.3uGHz)\r\n",
	nRF24_REG_RF_CH, i, i, 2400 + i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RF_SETUP
	i = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	size = sprintf((char*) buffor,
			"[0x%02X] 0x%02X CONT_WAVE:%s PLL_LOCK:%s DataRate=",
			nRF24_REG_RF_SETUP, i, (i & 0x80) ? "ON" : "OFF",
			(i & 0x80) ? "ON" : "OFF");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	switch ((i & 0x28) >> 3) {
	case 0x00:
		HAL_UART_Transmit(&huart1, (uint8_t*) "1M", 2, 100);
		break;
	case 0x01:
		HAL_UART_Transmit(&huart1, (uint8_t*) "2M", 2, 100);
		break;
	case 0x04:
		HAL_UART_Transmit(&huart1, (uint8_t*) "250M", 2, 100);
		break;
	default:
		HAL_UART_Transmit(&huart1, (uint8_t*) "????", 2, 100);
		break;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "pbs RF_PWR=", sizeof("pbs RF_PWR="),
			100);
	switch ((i & 0x06) >> 1) {
	case 0x00:
		HAL_UART_Transmit(&huart1, (uint8_t*) "-18", 3, 100);
		break;
	case 0x01:
		HAL_UART_Transmit(&huart1, (uint8_t*) "-12", 3, 100);
		break;
	case 0x02:
		HAL_UART_Transmit(&huart1, (uint8_t*) "-6", 2, 100);
		break;
	case 0x03:
		HAL_UART_Transmit(&huart1, (uint8_t*) "0", 1, 100);
		break;
	default:
		HAL_UART_Transmit(&huart1, (uint8_t*) "??", 2, 100);
		break;
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "dBm\r\n", sizeof("dBm\r\n"), 100);
	// STATUS
	i = nRF24_ReadReg(nRF24_REG_STATUS);
	size = sprintf((char*) buffor,
			"[0x%02X] 0x%02X IRQ:%03b RX_PIPE:%u TX_FULL:%s\r\n",
			nRF24_REG_STATUS, i, (i & 0x70) >> 4, (i & 0x0E) >> 1,
			(i & 0x01) ? "YES" : "NO");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// OBSERVE_TX
	i = nRF24_ReadReg(nRF24_REG_OBSERVE_TX);
	size = sprintf((char*) buffor, "[0x%02X] 0x%02X PLOS_CNT=%u ARC_CNT=%u\r\n",
	nRF24_REG_OBSERVE_TX, i, i >> 4, i & 0x0F);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RPD
	i = nRF24_ReadReg(nRF24_REG_RPD);
	size = sprintf((char*) buffor, "[0x%02X] 0x%02X RPD=%s\r\n", nRF24_REG_RPD,
			i, (i & 0x01) ? "YES" : "NO");
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_ADDR_P0
	aw = 5;
	nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P0, buf, aw);
	size = sprintf((char*) buffor, "[0x%02X] RX_ADDR_P0 \"0x",
	nRF24_REG_RX_ADDR_P0);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	for (i = 0; i < 5; i++) {
		size = sprintf((char*) buffor, "%02X", buf[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffor, 2, 100);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, 100);
	// RX_ADDR_P1
	nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P1, buf, aw);
	size = sprintf((char*) buffor, "[0x%02X] RX_ADDR_P1 \"0x",
	nRF24_REG_RX_ADDR_P0);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	for (i = 0; i < 5; i++) {
		size = sprintf((char*) buffor, "%02X", buf[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffor, 2, 100);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", 2, 100);
	// RX_ADDR_P2
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P2);
	size = sprintf((char*) buffor, "[0x%02X] RX_ADDR_P2 \"",
	nRF24_REG_RX_ADDR_P2);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	size = sprintf((char*) buffor, "0x%02X\"\r\n", i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);

	// RX_ADDR_P3
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P3);
	size = sprintf((char*) buffor, "[0x%02X] RX_ADDR_P3 \"",
	nRF24_REG_RX_ADDR_P3);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	size = sprintf((char*) buffor, "0x%02X\"\r\n", i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_ADDR_P4
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P4);
	size = sprintf((char*) buffor, "[0x%02X] RX_ADDR_P4 \"",
	nRF24_REG_RX_ADDR_P3);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	size = sprintf((char*) buffor, "0x%02X\"\r\n", i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_ADDR_P5
	i = nRF24_ReadReg(nRF24_REG_RX_ADDR_P5);
	size = sprintf((char*) buffor, "[0x%02X] RX_ADDR_P5 \"",
	nRF24_REG_RX_ADDR_P3);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	size = sprintf((char*) buffor, "0x%02X\"\r\n", i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// TX_ADDR
	nRF24_ReadMBReg(nRF24_REG_TX_ADDR, buf, aw);
	size = sprintf((char*) buffor, "[0x%02X] TX_ADDR \" 0x", nRF24_REG_TX_ADDR);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	for (i = 0; i < aw; i++) {
		size = sprintf((char*) buffor, "%02X", buf[i]);
		HAL_UART_Transmit(&huart1, (uint8_t*) buffor, 2, 100);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*) "\n\r", size, 100);
	// RX_PW_P0
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P0);
	size = sprintf((char*) buffor, "[0x%02X] RX_PW_P0=%u\r\n",
	nRF24_REG_RX_PW_P0, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_PW_P1
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P1);
	size = sprintf((char*) buffor, "[0x%02X] RX_PW_P1=%u\r\n",
	nRF24_REG_RX_PW_P1, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_PW_P2
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P2);
	size = sprintf((char*) buffor, "[0x%02X] RX_PW_P2=%u\r\n",
	nRF24_REG_RX_PW_P2, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_PW_P3
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P3);
	size = sprintf((char*) buffor, "[0x%02X] RX_PW_P3=%u\r\n",
	nRF24_REG_RX_PW_P3, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_PW_P4
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P4);
	size = sprintf((char*) buffor, "[0x%02X] RX_PW_P4=%u\r\n",
	nRF24_REG_RX_PW_P4, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
	// RX_PW_P5
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P5);
	size = sprintf((char*) buffor, "[0x%02X] RX_PW_P5=%u\r\n",
	nRF24_REG_RX_PW_P5, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);

	i = nRF24_ReadReg(nRF24_REG_DYNPD);
	size = sprintf((char*) buffor, "[0x%02X] DYNPD=%u\r\n",
			nRF24_REG_DYNPD, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);

	i = nRF24_ReadReg(nRF24_REG_FEATURE);
	size = sprintf((char*) buffor, "[0x%02X] FEATURE=%u\r\n",
			nRF24_REG_FEATURE, i);
	HAL_UART_Transmit(&huart1, (uint8_t*) buffor, size, 100);
}


void nRF24_PrintSetting(){
	uint8_t i;
	uint8_t buf[2][5];

	printf("================ NRF Configuration ================\r\n");

	i = nRF24_ReadReg(nRF24_REG_CONFIG);
	printf("STATUS    =  0x%02X RX_DR=%d TX_DS=%d MAX_RT=%d RX_P_NO=%d TX_FULL=%d\r\n", i , i & (1<<6), i & (1<<5), i & (1<<4),i & (3<<1),i & (1<<0) );

	nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P0, buf[0], 5);
	nRF24_ReadMBReg(nRF24_REG_RX_ADDR_P1, buf[1], 5);
	printf("RX_ADDR_P0-1	 = ");
	printf("0x");
	for(int i = 0 ; i < 5; i++) printf("%02X",buf[0][i]);
	printf("  0x");
	for(int i = 0 ; i < 5; i++) printf("%02X",buf[1][i]);
	printf("\r\n");

	buf[0][0] = nRF24_ReadReg(nRF24_REG_RX_ADDR_P2);
	buf[0][1] = nRF24_ReadReg(nRF24_REG_RX_ADDR_P3);
	buf[0][2] = nRF24_ReadReg(nRF24_REG_RX_ADDR_P4);
	buf[0][3] = nRF24_ReadReg(nRF24_REG_RX_ADDR_P5);
	printf("RX_ADDR_P2-5	 = 0x%02x 0x%02x 0x%02x 0x%02x",buf[0][0],buf[0][1],buf[0][2],buf[0][3]);
	printf("\r\n");

	nRF24_ReadMBReg(nRF24_REG_TX_ADDR, buf[0], 5);
	printf("TX_ADDR	 = ");
	printf("0x");
	for(int i = 0 ; i < 5; i++) printf("%02X",buf[0][i]);
	printf("\r\n");


	i = nRF24_ReadReg(nRF24_REG_RX_PW_P0);
	printf("RX_PW_P0-5	 = 0x%02X ",i);
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P1);
	printf("0x%02X ",i);
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P2);
	printf("0x%02X ",i);
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P3);
	printf("0x%02X ",i);
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P4);
	printf("0x%02X ",i);
	i = nRF24_ReadReg(nRF24_REG_RX_PW_P5);
	printf("0x%02X ",i);
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_EN_AA);
	printf("EN_AA       =  0x%02X", i );
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	printf("EN_RXADDR	 = 0x%02X", i );
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_RF_CH);
	printf("RF_CH		 = 0x%02X", i );
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	printf("RF_SETUP	 = 0x%02X", i );
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_CONFIG);
	printf("CONFIG		 = 0x%02X", i );
	printf("\r\n");

	buf[0][0] = nRF24_ReadReg(nRF24_REG_DYNPD);
	buf[0][1] = nRF24_ReadReg(nRF24_REG_FEATURE);
	printf("DYNPD/FEATURE	 = 0x%02X 0x%02X", buf[0][0],buf[0][1]);
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	printf("Data Rate	 = ");
	switch ((i & 0x28) ) {
		case 0x00:
			printf("1Mbps");
			break;
		case 0x08:
			printf("2Mbps");
			break;
		case 0x20:
			printf("250kbps");
			break;
		default:
			printf("???");
			break;
	}
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_CONFIG);
	if(i & (1<<2)) printf("CRC Length	 = 16bit");
	else printf("CRC Length	 = 8bit");
	printf("\r\n");

	i = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	switch ((i & 0x06) >> 1) {
		case 0x00:
			printf("PA Power	 = -18dBm");
			break;
		case 0x01:
			printf("PA Power	 = -12dBm");
			break;
		case 0x02:
			printf("PA Power	 = -6dBm");
			break;
		case 0x03:
			printf("PA Power	 = 0dBm");
			break;
		default:
			printf("PA Power	 = ??");
			break;
		}
	printf("\r\n");

}

void nRF24_setConfiguration()
{
	uint8_t ADDR1[] ={0x01,0x01,0x01,0x01,0x01}; // the TX address
		uint8_t ADDR2[] ={0xc2,0xc2,0xc2,0xc2,0xc2};
		nRF24_EnableAA(0x3f); // disable ShockBurst
		nRF24_SetRFChannel(0x4c); // set RF channel to 2490MHz
		nRF24_SetAddrWidth(5); // address width is 5 bytes
		nRF24_SetDataRate(nRF24_DR_250kbps); // 2Mbit/s data rate
		nRF24_SetCRCScheme(nRF24_CRC_2byte); // 1-byte CRC scheme
		nRF24_SetTXPower(nRF24_TXPWR_0dBm); // configure TX power
		nRF24_SetAutoRetr(nRF24_ARD_4000us, 15); // configure auto retransmit: 15 retransmissions with pause of 2500s in between

		nRF24_SetAddr(nRF24_PIPETX, ADDR1); // program TX address
		nRF24_SetAddr(nRF24_PIPE0, ADDR1); // program pipe#0 RX address, must be same as TX (for ACK packets)

		nRF24_SetAddr(nRF24_PIPE2, ADDR2); // program pipe address
		nRF24_SetRXPipe(nRF24_PIPE2, nRF24_AA_ON, 32); // enable RX pipe#1 with Auto-ACK: enabled, payload length: 10 bytes

		nRF24_SetOperationalMode(nRF24_MODE_RX); // switch transceiver to the TX mode
		nRF24_SetPowerMode(nRF24_PWR_UP); // wake-up transceiver (in case if it sleeping)
		nRF24_CE_H();
}
