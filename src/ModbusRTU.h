/*
Filename:	ModbusRTULib.h
Author:		norgor
Date:		27.03.2017
*/


#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#define BIT_SET(variable, bit) ((variable) |= (1<<(bit)))
#define BIT_RESET(variable, bit) ((variable) &= ~(1<<(bit)))
#define BIT_CHECK(variable, bit) (variable & (1 << bit))

#define MODBUS_MAX_FRAME_LENGTH 255

namespace ModbusRTU
{
	extern uint16_t crc16(const uint8_t *nData, uint16_t wLength);
	extern uint16_t endianSwap16(uint16_t a);
	extern uint32_t endianSwap32(uint32_t a);

	struct ModbusRegister
	{
		enum RegisterType : uint8_t
		{
			None,
			Coil,
			DiscreteInput,
			InputRegister,
			HoldingRegister
		};

		ModbusRegister()
		: m_RegisterType(None), m_RegisterNumber(0), m_pData(nullptr), m_MinValue(0), m_MaxValue(0), m_readOnly(1)
		{
		}

		RegisterType m_RegisterType;
		uint16_t m_RegisterNumber;
		uint8_t *m_pData;
		uint16_t m_MinValue; // Minimum allowed value
		uint16_t m_MaxValue; // Maximum allowed value
		uint8_t m_readOnly;
	};

	template <uint16_t registerCount>
	class ModbusRTUSlave
	{

		enum FunctionCode : uint8_t
		{
			ReadCoils = 1,
			ReadDiscreteInputs = 2,
			ReadMultipleHoldingRegisters = 3,
			ReadInputRegisters = 4,
			WriteSingleCoil = 5,
			WriteSingleRegister = 6,
			WriteMultipleCoils = 15,
			WriteMultipleRegisters = 16,
			Exception = 128,
		};

		enum ExceptionCode : uint8_t
		{
			IllegalFunction = 1,
			IllegalDataAddress = 2,
			IllegalDataValue = 3,
		};

		bool m_modbusEnabled = false;
		HardwareSerial *m_pHardwareSerial;
		uint8_t m_SlaveID;
		ModbusRegister m_RegisterArray[registerCount];
		uint16_t m_AssignedRegisters;
		uint32_t m_BaudRate;
		uint8_t m_InputFrame[MODBUS_MAX_FRAME_LENGTH];
		uint8_t m_OutputFrame[MODBUS_MAX_FRAME_LENGTH];
		uint8_t m_InputFrameLength;

		//Checks if frame has been corrupted
		//
		//
		bool isFrameCorrupted(uint8_t *frame, uint16_t frameLength)
		{
			// Validate input
			if (frame == nullptr || frameLength < 2) {
				return true; // Invalid frame is considered corrupted
			}

			// Extract received CRC (little-endian format)
			uint16_t receivedCrc = frame[frameLength - 2] | (frame[frameLength - 1] << 8);

			// Calculate CRC for the data portion of the frame
			uint16_t calculatedCrc = crc16(frame, frameLength - 2);

			// Check if CRC matches
			return (calculatedCrc != receivedCrc);
		}

		//Sends exception to master
		//Uses last received frame as function code
		//
		void throwException(ExceptionCode exceptionCode)
		{
			m_OutputFrame[0] = m_SlaveID;
			m_OutputFrame[1] = m_InputFrame[1] + FunctionCode::Exception;
			m_OutputFrame[2] = exceptionCode;
			sendFrame(m_OutputFrame, 3);
		}

		//Attempts to find register in register array
		//Returns pointer to register if register found
		//Returns nullptr if register not found
		ModbusRegister *findRegister(uint16_t _register)
		{
			for (uint16_t i = 0; i < m_AssignedRegisters; i++)
			{
				if (m_RegisterArray[i].m_RegisterNumber == _register && m_RegisterArray[i].m_pData && m_RegisterArray[i].m_RegisterType != ModbusRegister::None)
					return &m_RegisterArray[i];
			}

			return nullptr;
		}

		//Receives frame from serial buffer
		//Function is non-blocking
		//Returns true when frame is successfully received
		bool receiveFrame()
		{
			// Check if the input frame is already full
			if (m_InputFrameLength >= MODBUS_MAX_FRAME_LENGTH) {
				clearInputFrame();
				return false;
			}

			// Read available data into the input frame
			if (m_pHardwareSerial->available()) {
				m_InputFrameLength += m_pHardwareSerial->readBytes(&m_InputFrame[m_InputFrameLength], MODBUS_MAX_FRAME_LENGTH - m_InputFrameLength);

				// Check if at least 2 bytes (slave address + function code) are received
				if (m_InputFrameLength >= 2) {

					// Function length checks for different function codes
					if (m_InputFrame[1] == FunctionCode::ReadCoils ||
						m_InputFrame[1] == FunctionCode::ReadDiscreteInputs ||
						m_InputFrame[1] == FunctionCode::WriteSingleCoil ||
						m_InputFrame[1] == FunctionCode::WriteSingleRegister ||
						m_InputFrame[1] == FunctionCode::ReadMultipleHoldingRegisters ||
						m_InputFrame[1] == FunctionCode::ReadInputRegisters) {
						return m_InputFrameLength == 8;
					}
					else if (m_InputFrame[1] == FunctionCode::WriteMultipleCoils || 
							m_InputFrame[1] == FunctionCode::WriteMultipleRegisters) {
						return m_InputFrameLength == m_InputFrame[6] + 9;
					}
					else {
						// Throw exception for illegal function code
						throwException(ExceptionCode::IllegalFunction);
						DLN_MB("MB: receiveFrame - IllegalFunction");

						// Clear the current input frame
						clearInputFrame();
						return false;
					}
				}
			}

			return false;
		}

		//Send response frame to master
		//This function appends a CRC16 to the end of the frame
		//
		void sendFrame(uint8_t *pFrame, uint8_t frameLength)
		{
			*(uint16_t*)&pFrame[frameLength] = crc16(pFrame, frameLength);
			m_pHardwareSerial->write(pFrame, frameLength + 2);
		}

		//Set input frame length to zero
		//
		//
		void clearInputFrame()
		{
			m_InputFrameLength = 0;
		}

		//Parses input frame
		//
		//
		void parseFrame(uint8_t *frame, uint16_t frameLength)
		{
			if (frame[1] == ReadCoils)
			{
				uint16_t targetRegister = endianSwap16(*(uint16_t*)&frame[2]);
				uint16_t targetRegisterLength = endianSwap16(*(uint16_t*)&frame[4]);
				byte dataLength = (ceil(((float)targetRegisterLength) / 8.0f));

				//Write frame header
				m_OutputFrame[0] = m_SlaveID;
				m_OutputFrame[1] = frame[1];
				m_OutputFrame[2] = dataLength;

				//Loop through requested registers
				for (uint16_t i = 0; i < targetRegisterLength; i++)
				{
					//Find the register
					ModbusRegister *pRegister = findRegister(targetRegister + i);

					//Check if register is valid
					if (pRegister && pRegister->m_RegisterType == ModbusRegister::Coil)
					{
						//Write register value to response frame
						if (*pRegister->m_pData)
							BIT_SET(m_OutputFrame[3 + (i / 8)], i % 8); //Set corresponding register bit
						else
							BIT_RESET(m_OutputFrame[3 + (i / 8)], i % 8); //Reset corresponding register bit
					}
					else
					{
						//Throw exception if register is not valid
						throwException(ExceptionCode::IllegalDataAddress);
						DBF_MB("MB: ReadCoils - IllegalDataAddress at Addr %u\n", targetRegister);
						return;
					}
				}

				//Send response frame to master
				sendFrame(m_OutputFrame, 3 + dataLength);
			}
			else if (frame[1] == ReadDiscreteInputs)
			{
				uint16_t targetRegister = endianSwap16(*(uint16_t*)&frame[2]);
				uint16_t targetRegisterLength = endianSwap16(*(uint16_t*)&frame[4]);
				byte dataLength = (ceil(((float)targetRegisterLength) / 8.0f));

				//Write frame header
				m_OutputFrame[0] = m_SlaveID;
				m_OutputFrame[1] = frame[1];
				m_OutputFrame[2] = dataLength;

				//Loop through requested registers
				for (uint16_t i = 0; i < targetRegisterLength; i++)
				{
					//Find the register
					ModbusRegister *pRegister = findRegister(targetRegister + i);

					//Check if register is valid
					if (pRegister && pRegister->m_RegisterType == ModbusRegister::DiscreteInput)
					{
						//Write register value to response frame
						if (*pRegister->m_pData)
							BIT_SET(m_OutputFrame[3 + (i / 8)], i % 8); //Set corresponding register bit
						else
							BIT_RESET(m_OutputFrame[3 + (i / 8)], i % 8); //Reset corresponding register bit
					}
					else
					{
						//Throw exception if register is not valid
						throwException(ExceptionCode::IllegalDataAddress);
						DBF_MB("MB: ReadDiscreteInputs - IllegalDataAddress at Addr %u\n", targetRegister);
						return;
					}
				}

				//Send response frame to master
				sendFrame(m_OutputFrame, dataLength + 3);
			}
			else if (frame[1] == ReadMultipleHoldingRegisters)
			{
				uint16_t targetRegister = endianSwap16(*(uint16_t*)&frame[2]);
				uint16_t targetRegisterLength = endianSwap16(*(uint16_t*)&frame[4]);
				uint8_t dataLength = targetRegisterLength * 2;

				//Write frame header
				m_OutputFrame[0] = m_SlaveID;
				m_OutputFrame[1] = frame[1];
				m_OutputFrame[2] = dataLength;

				//Loop through requested registers
				for (uint16_t i = 0; i < targetRegisterLength; i++)
				{
					//Find the register
					ModbusRegister *pRegister = findRegister(targetRegister + i);

					//Check if register is valid
					if (pRegister && pRegister->m_RegisterType == ModbusRegister::HoldingRegister)
					{
						*(uint16_t*)&m_OutputFrame[3 + (i * 2)] = endianSwap16(*(uint16_t*)pRegister->m_pData);
						DBF_MB("MB: Read - Addr %u Value %u\n", targetRegister + i, *(uint16_t*)pRegister->m_pData);
					}
					else
					{
						//Throw exception if register is not valid
						throwException(ExceptionCode::IllegalDataAddress);
						DBF_MB("MB: ReadMultipleHoldingRegisters - IllegalDataAddress at Addr %u\n", targetRegister + i);
						return;
					}
				}

				//Send response frame to master
				sendFrame(m_OutputFrame, dataLength + 3);
			}
			else if (frame[1] == ReadInputRegisters)
			{
				uint16_t targetRegister = endianSwap16(*(uint16_t*)&frame[2]);
				uint16_t targetRegisterLength = endianSwap16(*(uint16_t*)&frame[4]);
				byte dataLength = targetRegisterLength * 2;

				//Write frame header
				m_OutputFrame[0] = m_SlaveID;
				m_OutputFrame[1] = frame[1];
				m_OutputFrame[2] = dataLength;

				//Loop through requested registers
				for (uint16_t i = 0; i < targetRegisterLength; i++)
				{
					//Find the register
					ModbusRegister *pRegister = findRegister(targetRegister + i);

					//Check if register is valid
					if (pRegister && pRegister->m_RegisterType == ModbusRegister::InputRegister)
					{
						*(uint16_t*)&m_OutputFrame[3 + (i * 2)] = endianSwap16(*(short*)pRegister->m_pData);
					}
					else
					{
						//Throw exception if register is not valid
						throwException(ExceptionCode::IllegalDataAddress);
						DBF_MB("MB: ReadInputRegisters - IllegalDataAddress at Addr %u\n", targetRegister);
						return;
					}
				}

				//Send response frame to master
				sendFrame(m_OutputFrame, dataLength + 3);
			}
			else if (frame[1] == WriteSingleCoil)
			{
				ModbusRegister *pRegister = findRegister(endianSwap16(*(uint16_t*)&frame[2]));

				//Check if register exists and if it is of correct type
				if (pRegister && pRegister->m_RegisterType == ModbusRegister::Coil)
				{
					//Write the target value to coil
					*(bool*)pRegister->m_pData = ((*(uint16_t*)&frame[4]) == true);
				}
				else
				{
					//Throw exception if exister is non-existent or incorrect type
					throwException(ExceptionCode::IllegalDataAddress);
					DLN_MB("MB: WriteSingleCoil - IllegalDataAddress");
					return;
				}

				m_pHardwareSerial->write(frame, 8);
			}
			else if (frame[1] == WriteSingleRegister)
			{	
				const uint16_t targetAddr = endianSwap16(*(uint16_t*)&frame[2]);
				ModbusRegister *pRegister = findRegister(endianSwap16(*(uint16_t*)&frame[2]));
				
				if (pRegister && pRegister->m_readOnly==0 && pRegister->m_RegisterType == ModbusRegister::HoldingRegister)
				{
					uint16_t value = endianSwap16(*(uint16_t*)&frame[4]);
					if (value < pRegister->m_MinValue || value > pRegister->m_MaxValue)
					{
						throwException(ExceptionCode::IllegalDataValue);
						DBF_MB("MB: WriteSingleRegister - IllegalDataValue at Addr %u: Value=%u (Allowed: %u..%u)\n", targetAddr, value, pRegister->m_MinValue, pRegister->m_MaxValue);
						return;
					}
					*(uint16_t*)pRegister->m_pData = value;
				}
				else
				{
					throwException(ExceptionCode::IllegalDataAddress);
					DBF_MB("MB: WriteSingleRegister - IllegalDataAddress at Addr %u\n", targetAddr);
					return;
				}

				m_pHardwareSerial->write(frame, 8);
			}
			else if (frame[1] == WriteMultipleCoils)
			{
				uint16_t targetRegister = endianSwap16(*(uint16_t*)&frame[2]);
				uint16_t targetRegisterLength = endianSwap16(*(uint16_t*)&frame[4]);

				for (uint16_t i = 0; i < targetRegisterLength; i++)
				{
					ModbusRegister *pRegister = findRegister(targetRegister + i);

					//Check if register exists and if it is of correct type
					if (pRegister && pRegister->m_RegisterType == ModbusRegister::Coil)
					{
						//Write the target value to coil
						*(bool*)pRegister->m_pData = BIT_CHECK(frame[7 + (i / 8)], i % 8);
					}
					else
					{
						//Throw exception if register is non-existent or incorrect type
						throwException(ExceptionCode::IllegalDataAddress);
						DBF_MB("MB: WriteMultipleCoils - IllegalDataAddress at Addr %u\n", targetRegister);
						return;
					}
				}

				uint16_t lastCrc = *(uint16_t*)&frame[6];

				*(uint16_t*)&frame[6] = crc16(frame, 6);
				m_pHardwareSerial->write(frame, 8);
				*(uint16_t*)&frame[6] = lastCrc;
			}
			else if (frame[1] == WriteMultipleRegisters)
			{
				uint16_t targetRegister = endianSwap16(*(uint16_t*)&frame[2]);
				uint16_t targetRegisterLength = endianSwap16(*(uint16_t*)&frame[4]);
				uint8_t byteCount = frame[6];

				// Validate that the byte count matches the expected data length
				if (byteCount != targetRegisterLength * 2) {
					throwException(ExceptionCode::IllegalDataValue);
					DLN_MB("MB: WriteMultipleRegisters - Mismatched byte count");
					return;
				}

				for (uint16_t i = 0; i < targetRegisterLength; i++)
				{
					ModbusRegister *pRegister = findRegister(targetRegister + i);

					// Check if register exists and if it is of correct type
					if (pRegister && pRegister->m_readOnly == 0 && pRegister->m_RegisterType == ModbusRegister::HoldingRegister)
					{
						// Calculate the offset for the current register's value
						uint16_t valueOffset = 7 + i * 2;
						uint16_t value = endianSwap16(*(uint16_t*)&frame[valueOffset]);

						// Validate the value against register constraints
						if (value < pRegister->m_MinValue || value > pRegister->m_MaxValue)
						{
							throwException(ExceptionCode::IllegalDataValue);
							DBF_MB("MB: WriteMultipleRegisters - IllegalDataValue at Addr %u: Value=%u (Allowed: %u..%u)\n", targetRegister + i, value, pRegister->m_MinValue, pRegister->m_MaxValue);
							return;
						}

						// Write the value to the register
						*(uint16_t*)pRegister->m_pData = value;
						DBF_MB("MB: Write - Addr %u Value %u\n", targetRegister + i, value);
					}
					else
					{
						// Throw exception if register is non-existent or incorrect type
						throwException(ExceptionCode::IllegalDataAddress);
						DBF_MB("MB: WriteMultipleRegisters - IllegalDataAddress at Addr %u\n", targetRegister + i);
        				return;
					}
				}

				// Construct response frame
				m_OutputFrame[0] = frame[0];
				m_OutputFrame[1] = frame[1];
				m_OutputFrame[2] = frame[2];
				m_OutputFrame[3] = frame[3];
				m_OutputFrame[4] = frame[4];
				m_OutputFrame[5] = frame[5];
				sendFrame(m_OutputFrame, 6);
			}
		}

		//Add register to register array
		//Length of data type depends on register type
		//Input registers and holding registers are two-byte
		//Discrete inputs and coils are one-byte
		int32_t addRegister(byte *pData, uint16_t _register, ModbusRegister::RegisterType registerType, uint16_t minValue = 0, uint16_t maxValue = 0xFFFF, uint8_t readOnly = 1)
		{
			if (m_AssignedRegisters >= registerCount || findRegister(_register))
				return -1;

			m_RegisterArray[m_AssignedRegisters].m_RegisterType = registerType;
			m_RegisterArray[m_AssignedRegisters].m_RegisterNumber = _register;
			m_RegisterArray[m_AssignedRegisters].m_pData = pData;
			m_RegisterArray[m_AssignedRegisters].m_MinValue = minValue;
			m_RegisterArray[m_AssignedRegisters].m_MaxValue = maxValue;
			m_RegisterArray[m_AssignedRegisters].m_readOnly = readOnly;

			m_AssignedRegisters++;

			return 0;
		}

	public:

		ModbusRTUSlave() : m_pHardwareSerial(nullptr),
						m_SlaveID(1),
						m_AssignedRegisters(0),
						m_BaudRate(0),
						m_InputFrameLength(0)
		{}

		// Adds coil to register list and returns register number
		// Returns -1 when no more registers available
		//
		int32_t addCoil(bool *coil, uint16_t _register, uint8_t readOnly = 0)
		{
			return addRegister((byte*)coil, _register, ModbusRegister::Coil, 0, 1, readOnly);
		}

		// Adds discrete input to register list and returns register number
		// Returns -1 when no more registers available
		//
		int32_t addDiscreteInput(const bool *discreteInput, uint16_t _register, uint8_t readOnly = 1)
		{
			return addRegister((byte*)discreteInput, _register, ModbusRegister::DiscreteInput, 0, 1, readOnly);
		}

		//Adds input register to register list and returns register number'
		//Returns -1 when no more registers available
		//
		int32_t addInputRegister(const uint16_t *inputRegister, uint16_t _register, uint16_t minValue = 0, uint16_t maxValue = 0xFFFF, uint8_t readOnly = 1)
		{
			return addRegister((byte*)inputRegister, _register, ModbusRegister::InputRegister, minValue, maxValue, readOnly);
		}

		//Adds holding register to register list and returns register 
		//Returns -1 when no more registers available
		//
		int32_t addHoldingRegister(uint16_t *holdingRegister, uint16_t _register, uint16_t minValue = 0, uint16_t maxValue = 0xFFFF, uint8_t readOnly = 1)
		{
			return addRegister((byte*)holdingRegister, _register, ModbusRegister::HoldingRegister, minValue, maxValue, readOnly);
		}

		//Initializes Modbus and serial data transmission
		//
		//
		void begin(uint32_t baud, HardwareSerial *pHardwareSerial = &Serial, uint8_t slaveId = 1)
		{

			DLN_MB("MB: Starting Modbus module...");

			//Clear all registers
			for (uint16_t i = 0; i < registerCount; i++)
			{
				m_RegisterArray[i].m_RegisterType = ModbusRegister::None;
				m_RegisterArray[i].m_pData = nullptr;
			}
			m_AssignedRegisters = 0; 

			//Initialize variables
			m_BaudRate = baud;
			m_SlaveID = slaveId;

			clearInputFrame();

			//Initialize serial
			m_pHardwareSerial = pHardwareSerial;
			pHardwareSerial->begin(m_BaudRate);
			pHardwareSerial->setTimeout(0);

			//Empty the serial buffer
			m_pHardwareSerial->readBytes(m_InputFrame, MODBUS_MAX_FRAME_LENGTH);
		}

		void updateSettings(uint32_t baud, HardwareSerial *pHardwareSerial = &Serial, uint8_t slaveId = 1)
		{

			DLN_MB("MB: Updating Modbus settings...");

			//Initialize variables
			m_BaudRate = baud;
			m_SlaveID = slaveId;

			clearInputFrame();

			//Initialize serial
			m_pHardwareSerial = pHardwareSerial;
			pHardwareSerial->begin(m_BaudRate);
			pHardwareSerial->setTimeout(0);

			//Empty the serial buffer
			m_pHardwareSerial->readBytes(m_InputFrame, MODBUS_MAX_FRAME_LENGTH);
		}

		//Checks for incoming frames from master
		//If frame has been received then it is parsed
		//
		void update()
		{
			//Check if a new frame is available
			if (receiveFrame())
			{
				//Check slave ID
				if (m_InputFrame[0] != m_SlaveID)
				{
					clearInputFrame();
					DLN_MB("MB: update - wrong id");
					return;
				}

				//Check if frame is corrupted
				if (isFrameCorrupted(m_InputFrame, m_InputFrameLength))
				{
					clearInputFrame();
					DLN_MB("MB: update - frame corrupted");
					return;
				}
				else {
					//DLN_MB("MB: update - frame okay");
				}

				//Parses the incoming frame when enabled
				if(m_modbusEnabled) {
					parseFrame(m_InputFrame, m_InputFrameLength);
				}

				clearInputFrame();
			}
		}

		void enable() {

			m_modbusEnabled = true;

		}

		void disable() {

			m_modbusEnabled = false;

		}

	};
}
#endif
