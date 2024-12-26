#pragma once
#include <inttypes.h>
#include "stm32f1xx_hal.h" // Замените на соответствующую версию для вашей STM32


extern TIM_HandleTypeDef htim1;

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
		
		
		OneWireDriver(GPIO_TypeDef *port, uint16_t pin) : 
			_gpio_port(port), _gpio_pin(pin), 
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
			
			bool searchComplete = false;
			while(searchComplete == false)
			{
				if(Reset() == false) break;
				
				memset(romByte, 0x00, sizeof(romByte));
				
				WriteByte( ((alarm == false) ? CMD_SEARCH_ROM : CMD_ALARM_SEARCH) );
				
				uint8_t discrepancyMarker = 0;
				for(uint8_t bitIndex = 0; bitIndex < 64; ++bitIndex)
				{
					uint8_t bit = _ReadBit();
					uint8_t complementBit = _ReadBit();
					
					// Линия в некорректном состоянии
					if(bit && complementBit)
					{
						searchComplete = true;
						break;
					}
					
					uint8_t direction = 0;
					
					if(!bit && !complementBit)
					{
						// Конфликт: выбираем направление
						
						if(bitIndex + 1 == lastDiscrepancy){
							direction = 1;
						}
						else if(bitIndex + 1 > lastDiscrepancy){
							direction = 0;
						}
						else{
							direction = (romByte[bitIndex / 8] >> (bitIndex % 8)) & 0x01;
						}
						
						if(direction == 0)
						{
							discrepancyMarker = bitIndex + 1;
						}
					
					} else {
						// Нет конфликта, используем значение бита
						
						direction = bit;
					}
					
					// Записываем бит в текущий байт ROM-кода
					if(direction)
					{
						romByte[bitIndex / 8] |= (1 << (bitIndex % 8));
					} else {
						romByte[bitIndex / 8] &= ~(1 << (bitIndex % 8));
					}
					
					_WriteBit(direction);
				}
				
				lastDiscrepancy = discrepancyMarker;
				if(!lastDiscrepancy)
					searchComplete = true;
				
				if( crc8(romByte, sizeof(romByte)-1) != romByte[7])
					break;
				
				// Избегаем переполнения массива
				if(rom_idx >= N)
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
			__HAL_TIM_SET_COUNTER(&htim1, 0);
			while(__HAL_TIM_GET_COUNTER(&htim1) < us)
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
		GPIO_InitTypeDef _gpio_cfg;
};


#include "OneWireTSens.h"
#include "OneWireTSensEx.h"




// https://cdn-shop.adafruit.com/datasheets/DS18B20.pdf

OneWireDriver oneWire(GPIOB, GPIO_PIN_9);
OneWireTSensEx sensors(oneWire);

// Пример использования
int main1111() {





	sensors.RegReadyCallback([](OneWireTSensEx::sensor_t *obj, uint8_t count) -> void
	{
		for(uint8_t i = 0; i < count; ++i)
		{
			Logger.Printf("Rom: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X, Temp: %05d°C, Valid: %d, Min: %d, Mid: %d, Max: %d", 
			obj[i].rom->raw[0], obj[i].rom->raw[1], obj[i].rom->raw[2], obj[i].rom->raw[3], 
			obj[i].rom->raw[4], obj[i].rom->raw[5], obj[i].rom->raw[6], obj[i].rom->raw[7], 
			obj[i].temp, obj[i].valid, sensors.GetMinTemp(), sensors.GetMidTemp(), sensors.GetMaxTemp()).PrintNewLine();
		}
	});

	while(1)
	{
		uint32_t time = HAL_GetTick();
		sensors.Processing(time);
	}

/*
	uint8_t fcount = sensors.Search();
		Logger.Printf("Found count: %d", fcount).PrintNewLine();
	
    // Вывод ROM-кодов
	OneWireDriver::rom_t *roms = {};
	uint8_t roms_count = sensors.GetRomsPtr(roms);
    for(uint8_t i = 0; i < roms_count; ++i)
	{
		DEBUG_LOG_ARRAY_HEX("1W ROM", roms[i].raw, 8);
		DEBUG_LOG_NEW_LINE();
	}
	
	// Чтение температур с найденных устройств
	uint8_t read_state = 0;
    while(1)
	{
		switch(read_state)
		{
			case 0:
				sensors.Convert();
				read_state = 1;
				break;
			
			case 1:
				int16_t temp[20];
				uint8_t count = sensors.Read(temp, 20);
				for(uint8_t i = 0; i < count; ++i)
				{
					Logger.Printf("Temperature %d°C", temp[i]).PrintNewLine();
				}
				read_state = 0;
				break;
		}
		
		HAL_Delay(1000);
	}
*/





/*
	OneWireDriver::rom_t roms[16];
	uint8_t fcount = oneWire.SearchROM(roms, 16);


    // Поиск всех ROM-кодов на линии
    //searchROM(oneWire);
	Logger.Printf("Found count: %d", fcount).PrintNewLine();
	

    // Вывод ROM-кодов
    for (uint8_t i = 0; i < fcount; i++) {
        //printf("Found ROM: 0x%016llX\n", rom);
		Logger.Printf("Found ROM: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", roms[i].code,roms[i].sn[0],roms[i].sn[1],roms[i].sn[2],roms[i].sn[3],roms[i].sn[4],roms[i].sn[5],roms[i].crc).PrintNewLine();
    }

    // Чтение температур с найденных устройств
    while (1) {
        for (uint8_t i = 0; i < fcount; i++) {
            int16_t temperature = sensors.ConvertAndRead(roms[i]);
				Logger.Printf("Temperature [ROM 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]: %d°C", roms[i].code,roms[i].sn[0],roms[i].sn[1],roms[i].sn[2],roms[i].sn[3],roms[i].sn[4],roms[i].sn[5],roms[i].crc, temperature).PrintNewLine();
        }
        HAL_Delay(1000); // Задержка между чтениями
    }
*/

}


