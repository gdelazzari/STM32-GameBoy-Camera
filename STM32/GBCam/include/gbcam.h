/*
 * gbcam.h
 *
 *  Created on: 30/nov/2014
 *      Author: giacomo
 */

#ifndef GBCAM_H_
#define GBCAM_H_


/*
 * Configuration
 */
#define IMAGE_WIDTH	16
#define IMAGE_HEIGHT	16

/*
 * Defines
 */
#define	 IMAGE_SIZE	IMAGE_WIDTH * IMAGE_HEIGHT
#define PIXSKIP		16128 / IMAGE_SIZE


/*
 * Typedefs
 */
typedef enum {
	SLOW = 0,
	FAST = 1
} CLOCKSPEED;

/*
 * Function prototypes
 */
void CAM_init(void);

void CAM_reset(void);

void CAM_loadRegister(uint8_t address, uint8_t data);

void CAM_startCapture(void);

void CAM_pauseCapture(void);
void CAM_resumeCapture(void);

int CAM_imageAvailable(void);
uint8_t* CAM_readImage(void);


#endif /* GBCAM_H_ */
