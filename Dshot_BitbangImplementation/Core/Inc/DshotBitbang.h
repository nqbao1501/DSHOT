#ifndef INC_DSHOTBITBANG_H_
#define INC_DSHOTBITBANG_H_

#include "stm32f4xx_hal.h"
/*
 * Ý tưởng của thư viện: Sử dụng nhịp của TIM để set bit vào thanh ghi BSRR với dữ liệu từ DMAl, từ đó thay đổi trạng thái HIGH LOW của GPIO output.
 * 1 bit trong xung DSHOT dù là 0 hay 1 thì đều có độ dài như nhau (VD: DSHOT 300 có độ dài xung là 3.333us)
 * Bit 0 và 1 khác nhau ở điểm phần TIME LOW. Với bit 1, phần này rộng 2.5us; với bit 0 phần này rộng 1,25us
 * ta lấy 1.25/3/33, được 3/8. Lấy 2.5/3.333 được 6/8. Vì vậy, code quy định phần TIME LOW của bit 0 xảy ra trong 3 xung nhịp của TIM,
 * TIME LOW của bit 1 xảy ra trong 6 xung nhịp của TIM, tất cả BIT thì là 8 xung nhịp của TIM
 * Do bộ nhớ DMA ở chế độ memory to peripheral -> cứ 1 xung nhịp clock trôi qua sẽ nhả 1 phần tử trong array DMA
 * -> array của DMA sẽ có 8 * 18 phần tử. 8 là số xung nhịp cần thiết để qua 1 bit, 18 là sô bit trong 1 frame (thực tế là 16 nhưng lấy dôi ra 18).
 *
 * Thư viện đã thành công trong việc gửi tín hiệu từ fc->esc, chuyển dma, timer, gpio của fc thành input để nhận giá trị eRPM từ esc, nhưng chưa giải mã được tín hiệu đi về chính xác
 */

#define SET_GPIO_PIN_9 GPIO_PIN_9
#define RESET_GPIO_PIN_9 (GPIO_PIN_9 << 16)


#define BIT_SECTION_NUMBER  8
#define BIT_1_HIGH_SECTION_NUMBER  6 - 1
#define BIT_0_HIGH_SECTION_NUMBER  3 - 1
#define FRAME_BIT_NUMBER  18

#define MOTOR_POLES_NUMBER				14
#define DSHOT_BITRATE					300
#define BDSHOT_RESPONSE_BITRATE			DSHOT_BITRATE * (5/4) //400 kbit
#define BDSHOT_RESPONSE_LENGTH			21
#define TIMER_TICK_PER_SECOND			BIT_SECTION_NUMBER * DSHOT_BITRATE //2.4 MHz
#define TIMER_TICK_PER_BIT				TIMER_TICK_PER_SECOND / BDSHOT_RESPONSE_BITRATE //6

extern DMA_HandleTypeDef hdma_tim1_up;
extern TIM_HandleTypeDef htim1;
void DshotOutputMode_init();
void MemBuffer_init(uint32_t *MemoryBuffer);

uint16_t calculateCRCandTelemtryBit(uint16_t value);
uint16_t getDshotFrame(uint16_t value);
void DshotFrameToMemBuffer(uint16_t DshotFrame, uint32_t *MemoryBuffer);

uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift);
uint16_t read_BDshot_response(uint32_t value);

#endif /* INC_DSHOTBITBANG_H_ */
