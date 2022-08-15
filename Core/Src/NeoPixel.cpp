/*
 * NeoPixel.cpp
 *
 *  Created on: Mar 23, 2022
 *      Author: paripal
 */

#include "NeoPixel.h"

namespace ws2812 {

void NeoPixel::update_write_buffer(){
	for(uint8_t i = 0; i < pixel_num; i++){
		for(uint8_t j = 0; j < color_num; j++){
			uint8_t color = 	(j == 0) ? NeoPixel::colors[i].green
							: 	(j == 1) ? NeoPixel::colors[i].red
							: 	(j == 2) ? NeoPixel::colors[i].blue : 0;
			for(uint8_t k = 0; k < byte; k++){
				NeoPixel::write_buffer[(i * color_num + j) * byte + k] = ((color & (0x80 >> k)) > 0) ? NeoPixel::high : NeoPixel::low;
			}
		}
	}
	NeoPixel::write_buffer[data_len] = 0;
}

void NeoPixel::rend(){
	update_write_buffer();
	if(NeoPixel::hdma->State != HAL_DMA_STATE_READY) {
    	HAL_TIM_PWM_Stop_DMA(NeoPixel::htim, NeoPixel::Channel);
  	}
	HAL_TIM_PWM_Start_DMA(NeoPixel::htim, NeoPixel::Channel, (uint32_t*)NeoPixel::write_buffer, data_len + reset_bit);
}

NeoPixel::NeoPixel(TIM_HandleTypeDef *htim, uint32_t Channel, DMA_HandleTypeDef *hdma, uint8_t high_level_pulse_len, uint8_t low_level_pulse_len) {
	// TODO Auto-generated constructor stub
	NeoPixel::htim = htim;
	NeoPixel::Channel = Channel;
	NeoPixel::hdma = hdma;
	NeoPixel::high = high_level_pulse_len;
	NeoPixel::low = low_level_pulse_len;
}

} /* namespace ws2812 */
