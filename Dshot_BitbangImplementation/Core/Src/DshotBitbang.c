
#include "DshotBitbang.h"

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

void DshotOutputMode_init(){

	HAL_TIM_Base_Start(&htim1);
	//HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	//HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)DMA_Buffer, (uint32_t)&(GPIOE->BSRR), FRAME_BIT_NUMBER * BIT_SECTION_NUMBER);
}
void MemBuffer_init(uint32_t *MemoryBuffer){
	for (uint16_t i = 0; i < FRAME_BIT_NUMBER * BIT_SECTION_NUMBER; i++){
		MemoryBuffer[i] = 0x00;
	}
	for (uint8_t i = 0; i < 16; i++){
		MemoryBuffer[i * BIT_SECTION_NUMBER] = RESET_GPIO_PIN_9;

		MemoryBuffer[i * BIT_SECTION_NUMBER + BIT_1_HIGH_SECTION_NUMBER] = SET_GPIO_PIN_9;
	}
}
uint16_t calculateCRCandTelemtryBit(uint16_t value){
	value = value << 1;
	return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
}
uint16_t getDshotFrame(uint16_t value){
	return ((value << 5) | calculateCRCandTelemtryBit(value));
}

void DshotFrameToMemBuffer(uint16_t DshotFrame, uint32_t *MemoryBuffer){
	for (uint8_t i = 0; i <16; i++){
        if (1 << (15 - i) & DshotFrame)
        {
        	MemoryBuffer[i * BIT_SECTION_NUMBER + BIT_0_HIGH_SECTION_NUMBER] = 0x00;
        }
        else
        {
        	MemoryBuffer[i * BIT_SECTION_NUMBER + BIT_0_HIGH_SECTION_NUMBER] = SET_GPIO_PIN_9;
        }
	}
}

uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift)
{
    uint32_t* buffer_end = raw_buffer + 200;
    while (raw_buffer < buffer_end)
    {
        // reception starts just after transmission, so there is a lot of HIGH samples. Find first LOW bit:
        if (__builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
            __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
            __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
            __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0))
        {   // if LOW edge was detected:
            uint32_t* buffer_previous = raw_buffer - 1;
            // set buffer end as current buffer + length of BDshot response:
            buffer_end = raw_buffer + BDSHOT_RESPONSE_LENGTH * 6;
            uint32_t motor_response = 0;
            uint8_t bits = 0;
            while (raw_buffer <= buffer_end)
            {
                // look for the high edge:
                if (__builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0) ||
                    __builtin_expect((*raw_buffer++ & (1 << motor_shift)), 0))
                {
                    if (raw_buffer <= buffer_end) {
                        uint8_t len = MAX((raw_buffer - buffer_previous) / 6, 1); // how many bits has the same value (for int rounding 1 is added)
                        bits += len;
                        motor_response <<= len;
                        buffer_previous = raw_buffer - 1;
                        // then look for the low edge:
                        while (raw_buffer < buffer_end)
                        {
                            if (__builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0) ||
                                __builtin_expect(!(*raw_buffer++ & (1 << motor_shift)), 0)) {
                                if (raw_buffer <= buffer_end) {
                                    len = MAX((raw_buffer - buffer_previous) / 6, 1); // how many bits has the same value (for int rounding 1 is added)
                                    bits += len;
                                    motor_response <<= len;
                                    motor_response |= 0x1FFFFF >> (BDSHOT_RESPONSE_LENGTH - len);
                                    buffer_previous = raw_buffer - 1;
                                }
                                break;
                            }
                        }
                    }
                }
            }
            // if last bits were 1 they were not added so far
            motor_response <<= (BDSHOT_RESPONSE_LENGTH - bits);
            if (*buffer_previous & (1 << motor_shift)) {
                motor_response |= 0x1FFFFF >> bits; // 21 ones right-shifted
            }
            return motor_response;
        }
    }
    // if LOW edge was not found return incorrect motor response:
    return 0xFFFFFFFF;
}
static uint8_t BDshot_check_checksum(uint16_t value)
{
    // BDshot frame has 4 last bits CRC:
    if (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) == 0x0F)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint16_t read_BDshot_response(uint32_t value)
{
    // BDshot frame contain 21 bytes but first is always 0 (used only for detection).
    // Next 20 bits are 4 sets of 5-bits which are mapped with 4-bits real value.
    // After all, value is 16-bit long with 12-bit eRPM value (actually it is a period of eRPM) and 4-bit CRC.
    // 12-bit eRPM value has 3 first bits od left shifting and 9-bit mantisa.

    // put nibbles in the array in places of mapped values (to reduce empty elements smallest mapped value will always be subtracted)
    // now it is easy to create real value - mapped value indicate array element which contain nibble value:
#define iv 0xFFFFFFFF
    static const uint32_t GCR_table[32] = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv};

    value = (value ^ (value >> 1)); // now we have GCR value
    uint16_t motors_rpm;
    uint32_t decoded_value = GCR_table[(value & 0x1F)];
    decoded_value |= GCR_table[((value >> 5) & 0x1F)] << 4;
    decoded_value |= GCR_table[((value >> 10) & 0x1F)] << 8;
    decoded_value |= GCR_table[((value >> 15) & 0x1F)] << 12;

    // if wrongly decoded decoded_value will be bigger than uint16_t:
    if (decoded_value < 0xFFFF && BDshot_check_checksum(decoded_value))
    {
        // if checksum is correct real save real RPM.
        // value sent by ESC is a period between each pole changes [us].
        // to achive eRPM we need to find out how many of these changes are in one minute.
        // eRPM = (60*1000 000)/T_us next RPM can be achived -> RPM = eRPM/(poles/2):

    	motors_rpm = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);      // cut off CRC and add shifting - this is period in [us]
        motors_rpm = 60 * 1000000 / motors_rpm * 2 / MOTOR_POLES_NUMBER; // convert to RPM

    }
    return motors_rpm;
}

