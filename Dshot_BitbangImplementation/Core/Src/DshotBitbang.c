
#include "DshotBitbang.h"


uint32_t DMA_Buffer[FRAME_BIT_NUMBER * BIT_SECTION_NUMBER] = {0};


void DshotOutputMode_init(){

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_1);
	HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)DMA_Buffer, (uint32_t)&(GPIOE->BSRR), FRAME_BIT_NUMBER * BIT_SECTION_NUMBER);
}
void MemBuffer_init(uint32_t *MemoryBuffer){
	for (uint16_t i = 0; i < FRAME_BIT_NUMBER * BIT_SECTION_NUMBER; i++){
		MemoryBuffer[i] = 0x00;
	}
	for (uint8_t i = 0; i < FRAME_BIT_NUMBER - 2; i++){
		MemoryBuffer[i * BIT_SECTION_NUMBER] = RESET_GPIO_PIN_9;

		DMA_Buffer[i * BIT_SECTION_NUMBER + BIT_1_HIGH_SECTION_NUMBER] = SET_GPIO_PIN_9;
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
	for (uint8_t i = 0; i < FRAME_BIT_NUMBER - 2; i++){
        if (1 << (FRAME_BIT_NUMBER  - 3 - i) & DshotFrame)
        {
        	MemoryBuffer[i * BIT_SECTION_NUMBER + BIT_0_HIGH_SECTION_NUMBER] = 0x00;
        }
        else
        {
        	MemoryBuffer[i * BIT_SECTION_NUMBER + BIT_0_HIGH_SECTION_NUMBER] = SET_GPIO_PIN_9;
        }
	}
}


