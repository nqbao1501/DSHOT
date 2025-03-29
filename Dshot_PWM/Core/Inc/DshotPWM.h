#ifndef DSHOTPWM_H
#define DSHOTPWM_H


#include "stm32f4xx_hal.h"
/*
 * Ý tưởng của thư viện: Sử dụng chế độ sinh xung PWM của TIM để sinh ra xung DSHOT.
 * 1 bit trong xung DSHOT dù là 0 hay 1 thì đều có độ dài như nhau (VD: DSHOT 300 có độ dài xung là 3.333us)
 * Bit 0 và 1 khác nhau ở điểm phần TIME LOW. Với bit 1, phần này rộng 2.5us; với bit 0 phần này rộng 1,25us
 * Code này cố gắng điều khiển giá trị của thanh ghi ARR. Khi TIM đếm đến số của thanh ghi này, xung PWM sinh ra chuyển từ low->high.
 * Các giá trị cuả thanh ghi ARR được điều khiển bởi DMA theo chế độ memory to peripheral. Code sử dụng 2 buffer để nạp dữ liệu và cập nhật dữ liệu vào DMA
 */
#define DMA_BUFFER_LENGTH 75
#define MEM_BUFFER_LENGTH 16
#define BIT_0_CCR_REG_VALUE	210
#define BIT_1_CCR_REG_VALUE	420

extern uint32_t Memory_Buffer[MEM_BUFFER_LENGTH];
extern uint32_t DMA_Buffer[DMA_BUFFER_LENGTH];
extern uint8_t swapFlag;

//Callback được gọi khi DMA đã truyền hết 1 nửa bộ nhớ
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);

//Phươngh trình khởi tạo 2 buffer dùng cho DMA
void Dshot_MemoryBuffer_init(uint32_t *MemoryBuffer);
void Dshot_DMABuffer_init(uint32_t *MemoryBuffer);

//Các phương trình chuyển giá trị động cơ thành frame DSHOT (16bit)
uint16_t Dshot_CalculateCRCandTelemtryBit(uint16_t value);
uint16_t Dshot_GetDshotFrame(uint16_t value);

//Các phương trình nạp frame DSHOT vào DMA
void Dshot_DshotFrame_to_buffer(uint16_t DshotFrame, uint32_t *MemoryBuffer);

#endif
