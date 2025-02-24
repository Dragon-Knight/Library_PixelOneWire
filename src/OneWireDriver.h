#pragma once
#include <inttypes.h>

class OneWireDriver
{
	public:
		
		static constexpr uint8_t CMD_SEARCH_ROM = 0xF0;
		static constexpr uint8_t CMD_READ_ROM = 0x33;
		static constexpr uint8_t CMD_MATCH_ROM = 0x55;
		static constexpr uint8_t CMD_SKIP_ROM = 0xCC;
		static constexpr uint8_t CMD_ALARM_SEARCH = 0xEC;
		
		struct __attribute__((packed)) rom_t
		{
			union
			{
				uint8_t raw[8];
				struct __attribute__((packed))
				{
					uint8_t code;
					uint8_t sn[6];
					uint8_t crc;
				};
			};
		};
		
		enum error_t :int8_t
		{
			ERROR_NONE = 0,
		};
		
		
		OneWireDriver(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim) : 
			_gpio_port(port), _gpio_pin(pin), _htim(htim),
			_gpio_cfg{pin, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}
		{
			// Инициализация пина как открытый сток
			_SetPinAsOutput();
			HAL_GPIO_WritePin(_gpio_port, _gpio_pin, GPIO_PIN_SET);
			
			return;
		}
		
		template <uint8_t N> 
		uint8_t SearchROM(rom_t (&roms)[N], bool alarm = false)
		{
			uint8_t rom_idx = 0;
			uint8_t lastDiscrepancy = 0;
			uint8_t romByte[8];
			
			memset(romByte, 0x00, sizeof(romByte));
			
			bool searchComplete = false;
			while(searchComplete == false && rom_idx < N)
			{
				if(Reset() == false)
					break;
				
				WriteByte( (alarm ? CMD_ALARM_SEARCH : CMD_SEARCH_ROM) );
				
				uint8_t discrepancyMarker = 0;
				
				uint8_t bitIndex = 0;
				while(bitIndex < 64)
				{
					uint8_t direction;
					uint8_t bit = _ReadBit();
					uint8_t complementBit = _ReadBit();
					
					if(bit && complementBit)
					{
						searchComplete = true;
						
						break;
					}
					
					if(!bit && !complementBit)
					{
						if(bitIndex + 1 < lastDiscrepancy)
						{
							direction = (romByte[bitIndex / 8] >> (bitIndex % 8)) & 0x01;
						}
						else
						{
							direction = (bitIndex + 1 == lastDiscrepancy) ? 1 : 0;
							discrepancyMarker = (direction == 0) ? bitIndex + 1 : discrepancyMarker;
						}
					}
					else
					{
						direction = bit;
					}
					
					if(direction)
						romByte[bitIndex / 8] |= (1 << (bitIndex % 8));
					else
						romByte[bitIndex / 8] &= ~(1 << (bitIndex % 8));
					
					_WriteBit(direction);
					
					bitIndex++;
				}
				
				lastDiscrepancy = discrepancyMarker;
				if(lastDiscrepancy == 0)
					searchComplete = true;
				
				if(crc8(romByte, sizeof(romByte)) != 0)
					break;
				
				memcpy(&roms[rom_idx].raw, romByte, sizeof(romByte));
				rom_idx++;
			}
			
			return rom_idx;
		}
		
		void CMD_MatchRom(const rom_t &romCode)
		{
			WriteByte(CMD_MATCH_ROM);
			WriteBytes(romCode.raw, sizeof(romCode.raw));
			
			return;
		}
		
		void CMD_SkipRom()
		{
			WriteByte(CMD_SKIP_ROM);

			return;
		}
		
		
		// Сброс шины
		bool Reset()
		{
			_SetPinAsOutput();
			
			HAL_GPIO_WritePin(_gpio_port, _gpio_pin, GPIO_PIN_RESET); // Сигнал RESET
			_DelayUs(480);
			
			_SetPinAsInput();
			_DelayUs(70);
			
			bool presence = (HAL_GPIO_ReadPin(_gpio_port, _gpio_pin) == GPIO_PIN_RESET);
			_DelayUs(410);
			
			return presence;
		}
		
		// Записать байт в шину
		void WriteByte(uint8_t byte)
		{
			for(uint8_t i = 0; i < 8; ++i)
			{
				_WriteBit(byte & 0x01);
				byte >>= 1;
			}
			
			return;
		}
		
		// Прочитать байт из шины
		uint8_t ReadByte()
		{
			uint8_t byte = 0x00;
			for(uint8_t i = 0; i < 8; ++i)
			{
				byte |= (_ReadBit() << i);
			}
			
			return byte;
		}
		
		// Записать `length` байт в шину из массива `data`
		void WriteBytes(const uint8_t *data, uint8_t length)
		{
			for(uint8_t i = 0; i < length; ++i)
			{
				WriteByte(data[i]);
			}
			
			return;
		}
		
		// Прочитать `length` байт из шины в массив `data`
		void ReadBytes(uint8_t *data, uint8_t length)
		{
			for(uint8_t i = 0; i < length; ++i)
			{
				data[i] = ReadByte();
			}
			
			return;
		}
		
		// Посчитать контрольную сумму для массива `data`
		uint8_t crc8(const uint8_t *data, uint8_t length)
		{
			uint8_t crc = 0x00;
			
			while(length--)
			{
				uint8_t inbyte = *data++;
				for(uint8_t i = 8; i; i--)
				{
					uint8_t mix = (crc ^ inbyte) & 0x01;
					crc >>= 1;
					if(mix) crc ^= 0x8C;
					inbyte >>= 1;
				}
			}
			
			return crc;
		}
		
	private:
		
		void _DelayUs(uint16_t us)
		{
			__HAL_TIM_SET_COUNTER(_htim, 0);
			while(__HAL_TIM_GET_COUNTER(_htim) < us)
			{
				__NOP();
			}
			
			return;
		}
		
		void _SetPinAsOutput()
		{
			_gpio_cfg.Mode = GPIO_MODE_OUTPUT_OD;
			HAL_GPIO_Init(_gpio_port, &_gpio_cfg);
			
			return;
		}
		
		void _SetPinAsInput()
		{
			_gpio_cfg.Mode = GPIO_MODE_INPUT;
			HAL_GPIO_Init(_gpio_port, &_gpio_cfg);
			
			return;
		}
		
		void _WriteBit(uint8_t bit)
		{
			const uint16_t delay_low[2] = {60, 6};
			const uint16_t delay_high[2] = {10, 64};
			
			_SetPinAsOutput();

			HAL_GPIO_WritePin(_gpio_port, _gpio_pin, GPIO_PIN_RESET);
			_DelayUs( delay_low[bit] );
			
			HAL_GPIO_WritePin(_gpio_port, _gpio_pin, GPIO_PIN_SET);
			_DelayUs( delay_high[bit] );
			
			return;
		}
		
		uint8_t _ReadBit()
		{
			uint8_t bit;
			
			__disable_irq();
			
			_SetPinAsOutput();
			HAL_GPIO_WritePin(_gpio_port, _gpio_pin, GPIO_PIN_RESET);
			_DelayUs(6);
			
			_SetPinAsInput();
			_DelayUs(9);
			
			bit = HAL_GPIO_ReadPin(_gpio_port, _gpio_pin);
			
			__enable_irq();
			
			_DelayUs(55);
			
			return bit;
		}
		
		
		GPIO_TypeDef *_gpio_port;
		uint16_t _gpio_pin;
		TIM_HandleTypeDef *_htim;
		GPIO_InitTypeDef _gpio_cfg;
};
