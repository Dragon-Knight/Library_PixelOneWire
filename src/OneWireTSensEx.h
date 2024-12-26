#pragma once
#include <inttypes.h>
#include <CUtils.h>
//#include "OneWireTSens.h"

class OneWireTSensEx : /*private*/public OneWireTSens
{
	static constexpr uint8_t _max_sensor = 16;
	static constexpr uint16_t _tick = 30;
	
	public:

		struct sensor_t
		{
			const OneWireDriver::rom_t *rom;	// 
			bool active;						// Флаг найденного устройтва
			bool valid;							// Флаг верных данных
			//uint8_t id;							// ID устройства после поиска (для операций OneWireTSens)
			int16_t temp;						// Текущая температура в сотых градуса
		};
		
		using callback_ready_t = void (*)(sensor_t *sensors, uint8_t count);
		
		
		OneWireTSensEx(OneWireDriver &driver) : OneWireTSens(driver), 
			_callback_ready(nullptr), _state(STATE_INIT)
		{}

		void RegReadyCallback(callback_ready_t callback)
		{
			_callback_ready = callback;

			return;
		}

		
		int16_t GetMinTemp()
		{
			int16_t min = OneWireTSens::NO_VALID_TEMP;
			for(uint8_t i = 0; i < OneWireTSens::roms_count; ++i)
			{
				if(_obj[i].valid == false) continue;
				if(_obj[i].temp > min) continue;
				
				min = _obj[i].temp;
			}
			
			return min;
		}
		
		int16_t GetMidTemp()
		{
			int32_t mid = 0;
			uint8_t div = 0;
			for(uint8_t i = 0; i < OneWireTSens::roms_count; ++i)
			{
				if(_obj[i].valid == false) continue;

				mid += _obj[i].temp;
				div++;
			}
			
			return (div > 0) ? (mid / div) : OneWireTSens::NO_VALID_TEMP;
		}
		
		int16_t GetMaxTemp()
		{
			int16_t max = OneWireTSens::NO_VALID_TEMP;
			for(uint8_t i = 0; i < OneWireTSens::roms_count; ++i)
			{
				if(_obj[i].valid == false) continue;
				if(_obj[i].temp < max && max != OneWireTSens::NO_VALID_TEMP) continue;
				
				max = _obj[i].temp;
			}
			
			return max;
		}

		uint8_t GetAllTemp(sensor_t *obj, uint8_t size)
		{
			if(size < OneWireTSens::roms_count)
				return 0;
			
			uint8_t count = 0;
			for(uint8_t i = 0; i < OneWireTSens::roms_count; ++i)
			{
				obj[i] = _obj[i];
				count++;
			}
			
			return count;
		}
		
		void Processing(uint32_t &time)
		{
			if(time - _last_tick < _tick) return;
			_last_tick = time;
			
			switch(_state)
			{
				case STATE_INIT:
				{
					OneWireTSens::SearchFiltered();
					for(uint8_t i = 0; i < OneWireTSens::roms_count; ++i)
					{
						_obj[i].active = true;
						_obj[i].rom = &OneWireTSens::roms[i];
					}
					
					_state = (OneWireTSens::roms_count > 0) ? STATE_CONVERT : STATE_NO_SENSOR;
					break;
				}
				
				case STATE_CONVERT:
				{
					OneWireTSens::Convert();
					
					_delay = ((800 + (_tick / 2)) / _tick);
					
					_state = STATE_CONVERT_WAIT;
					break;
				}
				
				case STATE_CONVERT_WAIT:
				{
					if(--_delay == 0)
					{
						_obj_idx = 0;
						_state = STATE_READ;
					}
					
					break;
				}
				
				case STATE_READ:
				{
					int16_t temp = OneWireTSens::Read(_obj_idx);
					_obj[_obj_idx].temp = temp;
					_obj[_obj_idx].valid = (temp == OneWireTSens::NO_VALID_TEMP) ? false : true;
					
					if(++_obj_idx == OneWireTSens::roms_count)
					{
						if(_callback_ready != nullptr)
							_callback_ready(_obj, OneWireTSens::roms_count);
						
						_delay = ((1000 + (_tick / 2)) / _tick);
						_state = STATE_READ_WAIT;
					}

					break;
				}
				
				case STATE_READ_WAIT:
				{
					if(--_delay == 0)
					{
						_state = STATE_CONVERT;
					}
					
					break;
				}
				
				default:
				{
					break;
				}
			}
			
			return;
		}

	private:

		enum state_t : uint8_t 
		{
			STATE_NONE = 0,			// Не нициализирован
			STATE_INIT,				// Нужно выполнить инициализацию
			STATE_CONVERT,			// Нужно отправить команду конвертации
			STATE_CONVERT_WAIT,		// Ожидание конвертации
			STATE_READ,				// Нужно прочитать температуру
			STATE_READ_WAIT,		// Ожидание после чтения
			STATE_NO_SENSOR,		// Датчики не найдены
		};

		callback_ready_t _callback_ready;

		int16_t _temp[_max_sensor];
		uint8_t _temp_count = 0;
		state_t _state;
		int16_t _delay;

		

		uint32_t _last_tick;

		sensor_t _obj[_max_sensor] = {};
		uint8_t _obj_idx;

};
