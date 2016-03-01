/*
 * stereoprotocol.h
 *
 *  Created on: Sep 23, 2015
 *      Author: roland
 */

#ifndef SW_AIRBORNE_SUBSYSTEMS_GPS_STEREOPROTOCOL_H_
#define SW_AIRBORNE_SUBSYSTEMS_GPS_STEREOPROTOCOL_H_

#include <inttypes.h>
struct MsgProperties {
  uint16_t positionImageStart;
  uint16_t width;
  uint16_t height;
} ;


typedef struct MsgProperties MsgProperties;
// function primitives

/**
 * Increment circular buffer counter by i
 */
uint16_t stereoprot_add(uint16_t counter, uint16_t i, uint16_t buffer_size);

/**
 * Decrement circular buffer counter by i
 */
uint16_t stereoprot_diff(uint16_t counter, uint16_t i,uint16_t buffer_size);

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
uint8_t stereoprot_isEndOfMsg(uint8_t *stack, uint16_t i,uint16_t buffer_size);

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
uint8_t stereoprot_isStartOfMsg(uint8_t *stack, uint16_t i,uint16_t buffer_size);
//void stereoprot_get_msg_properties(uint8_t *, MsgProperties *, uint16_t,uint16_t);
uint8_t stereoprot_isStartOfLine(uint8_t *stack, uint16_t i,uint16_t buffer_size);
uint8_t stereoprot_isEndOfLine(uint8_t *stack, uint16_t i,uint16_t buffer_size);
uint8_t handleStereoPackageSmall(uint8_t newByte, uint16_t buffer_size, uint16_t *insert_loc, uint16_t *extract_loc,
                                 uint16_t *msg_start, uint8_t *msg_buf, uint8_t *ser_read_buf, uint8_t *stereocam_datadata_new,
                                 uint16_t *stereocam_datalen, uint16_t *stereocam_data_matrix_width, uint16_t *stereocam_data_matrix_height);

/**
 * Get all available data from stereo com link and decode any complete messages.
 * Returns as soon as a complete message is found. Messages placed in msg_buf
 */
uint8_t handleStereoPackage(uint8_t newByte, uint16_t buffer_size, uint16_t *insert_loc, uint16_t *extract_loc,
                            uint16_t *msg_start, uint8_t *msg_buf, uint8_t *ser_read_buf, uint8_t *stereocam_datadata_new,
                            uint16_t *stereocam_datalen, uint16_t *stereocam_data_matrix_width, uint16_t *stereocam_data_matrix_height);


/**
 * Retrieve size of image from message
 */
void stereoprot_get_msg_properties(uint8_t *raw, MsgProperties *properties, uint16_t start,uint16_t buffer_size);




#endif /* SW_AIRBORNE_SUBSYSTEMS_GPS_STEREOPROTOCOL_H_ */
