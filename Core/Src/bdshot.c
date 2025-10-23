#include "bdshot.h"

#include <stm32f767xx.h>

#include "global_constants.h"
#include "global_variables.h"
#include "stm32f7xx.h"

// static void fill_bb_BDshot_buffer(uint16_t m1_value);
static void update_motors_rpm();
static uint32_t get_BDshot_response(uint32_t raw_buffer[],
									const uint8_t motor_shift);
static void read_BDshot_response(uint32_t value, uint8_t motor);
static bool BDshot_check_checksum(uint16_t value);
static uint16_t prepare_BDshot_package(uint16_t value);
static uint16_t calculate_BDshot_checksum(uint16_t value);
static uint16_t prepare_BDshot_package_3D(uint16_t throttle, bool reverse);
int32_t bdshot_value_to_rpm(uint32_t value, uint8_t motor_poles);

// flags for reception or transmission:
static bool bdshot_reception_1 = true;

void DMA2_Stream1_IRQHandler(void) {
	if (DMA2->LISR & DMA_LISR_TCIF1) {
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;

		if (bdshot_reception_1) {
			// set GPIOs as inputs:
			GPIOC->MODER &= ~GPIO_MODER_MODER9;
			// set pull up for those pins:
			GPIOC->PUPDR |= GPIO_PUPDR_PUPDR9_0;

			// set timer:
			TIM1->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE /
							BDSHOT_RESPONSE_BITRATE /
							BDSHOT_RESPONSE_OVERSAMPLING -
						1;
			TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE /
						 BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING;

			DMA2_Stream1->CR &= ~(DMA_SxCR_DIR);
			DMA2_Stream1->PAR = (uint32_t)(&(GPIOC->IDR));
			DMA2_Stream1->M0AR = (uint32_t)(dshot_bb_buffer_1_r);
			// Main idea:
			// After sending DShot frame to ESC start receiving GPIO values.
			// Capture data (probing longer than ESC response).
			// There is ~33 [us] gap before the response so it is necessary to
			// add more samples:
			DMA2_Stream1->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 +
										BDSHOT_RESPONSE_LENGTH + 1) *
								  BDSHOT_RESPONSE_OVERSAMPLING);

			DMA2_Stream1->CR |= DMA_SxCR_EN;
			bdshot_reception_1 = false;
		}
	}

	uint32_t isr = DMA2->LISR;
	if (isr &
		(DMA_LISR_TCIF1 | DMA_LISR_HTIF1 | DMA_LISR_TEIF1 | DMA_LISR_DMEIF1)) {
		DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 |
					  DMA_LIFCR_CDMEIF1;
	}
}

void update_dma(void) {
	GPIOC->MODER |= GPIO_MODER_MODER9_0;

	TIM1->CR1 &= ~TIM_CR1_CEN;
	TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

	DMA2_Stream1->CR &= ~DMA_SxCR_EN;
	// while (DMA2_Stream1->CR & DMA_SxCR_EN);
	DMA2_Stream1->CR |= DMA_SxCR_DIR_0;
	DMA2_Stream1->PAR = (uint32_t)&GPIOC->BSRR;
	DMA2_Stream1->M0AR = (uint32_t)dshot_bb_buffer_1;
	DMA2_Stream1->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;
	DMA2_Stream1->CR |= DMA_SxCR_EN;

	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;

	bdshot_reception_1 = true;
}

void preset_bb_Dshot_buffer_single() {
	// Очистить буфер
	for (uint16_t i = 0; i < DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;
		 i++) {
		dshot_bb_buffer_1[i] = 0x00;
	}

	// Задать шаблон сигнала
	for (uint8_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) {
		// Начало бита — низкий уровень
		dshot_bb_buffer_1[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0
														 << MOTOR_1;

		// Подъём через DSHOT_BB_1_LENGTH тактов
		dshot_bb_buffer_1[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_1_LENGTH] =
			GPIO_BSRR_BS_0 << MOTOR_1;
	}
}

void fill_bb_BDshot_buffer(uint16_t m1_value, bool mode3D, bool reverse)

{
	if (mode3D) {
		m1_value = prepare_BDshot_package_3D(m1_value, reverse);
	} else {
		m1_value = prepare_BDshot_package(m1_value);
	}

	for (uint8_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2;
		 i++)  // последние 2 бита — всегда high
	{
		// Проверяем, равен ли текущий бит 1 или 0
		if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m1_value) {
			// ==== Бит = 1 ====
			// Ничего не делаем — оставляем 0x00, GPIO остаётся LOW до конца
			// DSHOT_BB_1_LENGTH
			dshot_bb_buffer_1[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH] =
				0x00;
		} else {
			// ==== Бит = 0 ====
			// Поднимаем сигнал раньше (через DSHOT_BB_0_LENGTH)
			dshot_bb_buffer_1[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH] =
				GPIO_BSRR_BS_0 << MOTOR_1;
		}
	}
}

int32_t get_motors_rpm(viod) {
	// BDshot bit banging reads whole GPIO register.
	// Now it's time to create BDshot responses from all motors (made of
	// individual bits).
	uint32_t motor_1_response =
		get_BDshot_response(dshot_bb_buffer_1_r, MOTOR_1);

	int32_t rpm = bdshot_value_to_rpm(motor_1_response, 14);
	return rpm;
}

static uint32_t get_BDshot_response(uint32_t raw_buffer[],
									const uint8_t motor_shift) {
	// Reception starts just after transmission, so there is a lot of HIGH
	// samples. Find first LOW bit:

	uint16_t i = 0;
	uint16_t previous_i = 0;
	uint16_t end_i = 0;
	uint32_t previous_value = 1;
	uint32_t motor_response = 0;
	uint8_t bits = 0;

	while (i < (int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 *
					 BDSHOT_RESPONSE_OVERSAMPLING)) {
		if (!(raw_buffer[i] & (1 << motor_shift))) {
			previous_value = 0;
			previous_i = i;
			end_i = i + BDSHOT_RESPONSE_LENGTH * BDSHOT_RESPONSE_OVERSAMPLING;
			break;
		}
		i++;
	}
	// if LOW edge was detected:
	if (previous_value == 0) {
		while (i < end_i) {
			// then look for changes in bits values and compute BDSHOT bits:
			if ((raw_buffer[i] & (1 << motor_shift)) != previous_value) {
				const uint8_t len =
					(i - previous_i) / BDSHOT_RESPONSE_OVERSAMPLING > 1
						? (i - previous_i) / BDSHOT_RESPONSE_OVERSAMPLING
						: 1;  // how many bits had the same value
				bits += len;
				motor_response <<= len;
				if (previous_value != 0) {
					motor_response |=
						(0x1FFFFF >>
						 (21 - len));  // 21 ones right-shifted by 20 or less
				}
				previous_value = raw_buffer[i] & (1 << motor_shift);
				previous_i = i;
			}
			i++;
		}
		// if last bits were 1 they were not added so far
		motor_response <<= (BDSHOT_RESPONSE_LENGTH - bits);
		motor_response |= 0x1FFFFF >> bits;	 // 21 ones right-shifted

		return motor_response;
	} else {  // if LOW edge was not found return incorrect motor response:
		return 0xFFFFFFFF;
	}
}

int32_t bdshot_value_to_rpm(uint32_t value, uint8_t motor_poles) {
	const uint32_t IV = 0xFFFFFFFFu;
	static const uint32_t GCR_table[32] = {
		IV, IV, IV, IV, IV, IV, IV, IV, IV, 9, 10, 11, IV, 13, 14, 15,
		IV, IV, 2,	3,	IV, 5,	6,	7,	IV, 0, 8,  1,  IV, 4,  12, IV};

	if (motor_poles == 0) return -1;  // защита от деления на ноль

	// Gray → Binary
	value = (value ^ (value >> 1));

	// Раскодировать 4 ниббла по таблице
	uint32_t nib0 = GCR_table[(value & 0x1Fu)];
	if (nib0 == IV) return -1;
	uint32_t nib1 = GCR_table[((value >> 5) & 0x1Fu)];
	if (nib1 == IV) return -1;
	uint32_t nib2 = GCR_table[((value >> 10) & 0x1Fu)];
	if (nib2 == IV) return -1;
	uint32_t nib3 = GCR_table[((value >> 15) & 0x1Fu)];
	if (nib3 == IV) return -1;

	uint32_t decoded_value = nib0 | (nib1 << 4) | (nib2 << 8) | (nib3 << 12);

	// Проверка CRC
	if (decoded_value >= 0xFFFFu) return -1;
	if (!BDshot_check_checksum((uint16_t)decoded_value)) return -1;

	// Извлекаем мантиссу и сдвиг
	uint32_t mantissa = (decoded_value & 0x1FF0u) >> 4;	 // 9 бит
	uint32_t shift = (decoded_value >> 13) & 0x7u;		 // 3 бита

	if (mantissa == 0) return -1;

	// Период в микросекундахstatic void setup_GPIOB();	// GPIOB (pin 0 - motor;
	// pin 1 - motor)
	uint32_t period_us = mantissa << shift;
	if (period_us == 0) return -1;

	// eRPM = (60 * 1_000_000) / period_us
	// RPM = eRPM * 2 / motor_poles
	// без использования float — безопасно для микроконтроллера
	uint64_t eRPM = (60ULL * 1000000ULL) / period_us;
	uint64_t rpm = (eRPM * 2ULL) / motor_poles;

	// ограничим диапазон чтобы не переполнить int32_t
	if (rpm > 2147483647ULL) rpm = 2147483647ULL;

	return (int32_t)rpm;
}

static bool BDshot_check_checksum(uint16_t value) {
	// BDshot frame has 4 last bits CRC:
	if (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) ==
		0x0F) {
		return true;
	} else {
		return false;
	}
}

static uint16_t prepare_BDshot_package(uint16_t value) {
	if (value > 0 && value < 48) value = 48;

	uint16_t packet = value << 1;
	uint16_t checksum = calculate_BDshot_checksum(packet);

	return (packet << 4) | checksum;
}

static uint16_t prepare_BDshot_package_3D(uint16_t throttle, bool reverse) {
	// Минимальное значение — 48 (как в обычном DShot)
	if (throttle == 0) {
		uint16_t packet = throttle << 1;
		uint16_t checksum = calculate_BDshot_checksum(packet);

		return (packet << 4) | checksum;
	}
	if (throttle > 0 && throttle < 48) throttle = 48;
	if (throttle > 1047) throttle = 1047;  // защита от выхода за диапазон

	// В 3D режиме: если reverse == true → добавляем 1000
	uint16_t value = reverse ? (throttle + 1000) : throttle;

	// Формирование пакета
	uint16_t packet = value << 1;
	uint16_t checksum = calculate_BDshot_checksum(packet);

	return (packet << 4) | checksum;
}

static uint16_t calculate_BDshot_checksum(uint16_t packet) {
	uint16_t csum = 0;
	uint16_t csum_data = packet;

	for (int i = 0; i < 3; i++) {  // берём три группы по 4 бита
		csum ^= (csum_data & 0xF);
		csum_data >>= 4;
	}

	return ~csum & 0x0F;
}