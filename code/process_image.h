#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void process_image_start(void);
uint16_t get_line_position(void);
//void find_black_line(uint8_t img_buff_ptr[IMAGE_BUFFER_SIZE], uint8_t* lowBorder, uint8_t* highBorder);
uint16_t extract_line_width(uint8_t *buffer);

#endif /* PROCESS_IMAGE_H */
