#pragma once
#include <inttypes.h>
//#include "OneWireDriver.h"

// https://cdn-shop.adafruit.com/datasheets/DS18B20.pdf
// https://www.analog.com/media/en/technical-documentation/data-sheets/DS18S20.pdf

//template <uint8_t _max_sensor> 
class OneWireTSens
{
	static constexpr uint8_t _max_sensor = 16;
	
	public:
		
		static constexpr uint8_t CMD_CONVERT_T = 0x44;
		static constexpr uint8_t CMD_WRITE_SCRATCHPAD = 0x4E;
		static constexpr uint8_t CMD_READ_SCRATCHPAD = 0xBE;
		static constexpr uint8_t CMD_COPY_SCRATCHPAD = 0x48;
		static constexpr uint8_t CMD_RECALL_E2 = 0xB8;
		static constexpr uint8_t CMD_READ_POWER_SUPPLY = 0xB4;
		
		static constexpr int16_t NO_VALID_TEMP = INT16_MAX;
		
		struct __attribute__((packed)) scratchpad_t
		{
			union
			{
				uint8_t raw[9];
				struct __attribute__((packed))
				{
					int16_t temp;
					uint8_t temp_h;
					uint8_t temp_l;
					uint8_t cfg;
					uint8_t _reserved[3];
					uint8_t crc;
				};
			};
		};
		
		
		OneWireTSens(OneWireDriver &driver) : _driver(driver), _roms_count(0)
		{}
		
		uint8_t SearchFiltered()
		{
			memset(_roms, 0x00, sizeof(_roms));
			_roms_count = 0;
			
			OneWireDriver::rom_t tmp_roms[_max_sensor] = {};
			uint8_t tmp_roms_count = _driver.SearchROM(tmp_roms);
			for(uint8_t i = 0; i < tmp_roms_count; ++i)
			{
				if(tmp_roms[i].code != 0x10 && tmp_roms[i].code != 0x28) continue;
				
				_roms[_roms_count++] = tmp_roms[i];
			}
			
			return _roms_count;
		}
		
		uint8_t Search()
		{
			memset(_roms, 0x00, sizeof(_roms));
			_roms_count = _driver.SearchROM(_roms);
			
			return _roms_count;
		}
		
		
		// Запустить конвертацию температуры по индексу датчика.
		void Convert(uint8_t idx)
		{
			if(idx >= _roms_count)
				return;					// Ошибка: индекс ненайденного датчика
			
			Convert(_roms[idx]);
			
			return;
		}
		
		// Запустить конвертацию температуры по ROM
		void Convert(const OneWireDriver::rom_t &rom)
		{
			if(_driver.Reset() == false)
				return;					// Ошибка: устройство не отвечает
			
			_driver.CMD_MatchRom(rom);
			_driver.WriteByte(CMD_CONVERT_T);

			return;
		}
		
		// Запустить конвертацию температуры на всех датчиках
		void Convert()
		{
			if(_driver.Reset() == false)
				return;					// Ошибка: устройство не отвечает
			
			_driver.CMD_SkipRom();
			_driver.WriteByte(CMD_CONVERT_T);
			
			return;
		}
		
		
		// Прочитать температуру с датчика по индексу
		int16_t Read(uint8_t idx)
		{
			if(idx >= _roms_count)
				return NO_VALID_TEMP;	// Ошибка: индекс ненайденного датчика
			
			return Read(_roms[idx]);
		}
		
		// Прочитать температуру с датчика по ROM
		int16_t Read(const OneWireDriver::rom_t &rom)
		{
			scratchpad_t scratchpad = {};

			if(_driver.Reset() == false)
				return NO_VALID_TEMP;	// Ошибка: устройство не отвечает
			
			_driver.CMD_MatchRom(rom);
			_driver.WriteByte(CMD_READ_SCRATCHPAD);
			_driver.ReadBytes(scratchpad.raw, sizeof(scratchpad.raw));
			
			if(_driver.crc8(scratchpad.raw, 8) != scratchpad.crc)
			{
				DEBUG_LOG_ARRAY_HEX("ERR-CRC", scratchpad.raw, 9);
				DEBUG_LOG_NEW_LINE();
				return NO_VALID_TEMP;	// Ошибка: CRC не верное
			}
			
			return _ConvertTemp(rom, scratchpad);
		}

		// Прочитать все датчики, найденные через `Search()`
		uint8_t Read(int16_t *temp, uint8_t size)
		{
			if(size < _roms_count)
				return 0;
			
			uint8_t count = 0;
			for(uint8_t i = 0; i < _roms_count; ++i)
			{
				temp[i] = Read(_roms[i]);
				count++;
			}

			return count;
		}
		
		
		// Прочитать температуру с известным ROM в блокирующем режиме
		int16_t ConvertAndRead(const OneWireDriver::rom_t &rom)
		{
			Convert(rom);
			
			HAL_Delay(750);
			
			return Read(rom);
		}


		// Получить список найденных ROM
		uint8_t GetRomsPtr(OneWireDriver::rom_t *&roms, uint8_t &count)
		{
			roms = _roms;
			count = _roms_count;
			return _roms_count;
		}
		
		
		const OneWireDriver::rom_t * const roms = _roms;
		const uint8_t &roms_count = _roms_count;
		
	private:

		int16_t _ConvertTemp(const OneWireDriver::rom_t &rom, const scratchpad_t &scratchpad)
		{
			int16_t temp = NO_VALID_TEMP;
			
			switch(rom.code)
			{
				// DS1820, DS18S20
				case 0x10:
				{
					temp = ((int32_t)scratchpad.temp * 100) / 2;
					break;
				}

				// DS18B20, DS1822
				case 0x28:
				{
					temp = ((int32_t)scratchpad.temp * 100) / 16;
					break;
				}
			}
			
			return temp;
		}
		
		
		OneWireDriver &_driver;
		OneWireDriver::rom_t _roms[_max_sensor];
		uint8_t _roms_count;
};
