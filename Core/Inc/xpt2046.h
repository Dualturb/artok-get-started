#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

 // The following values are used for touch panel validation.
 // They represent a wide, generic range that should not be changed for different screens.
 #define XPT_XMIN 		200
 #define XPT_XMAX 		3900
 #define XPT_YMIN 		200
 #define XPT_YMAX 		3900

#define XPT_WIDTH 		(XPT_XMAX - XPT_XMIN)
#define XPT_HEIGHT 		(XPT_YMAX - XPT_YMIN)

// Threshold for touch pressure (Z-axis)
#define Z_THRESHOLD 	400

// Public function prototypes
void XPT2046_Init(void);
uint16_t XPT2046_ReadZ(void);
bool XPT2046_ReadXY(uint16_t *x, uint16_t *y);
#ifdef __cplusplus
}
#endif

#endif /* INC_TOUCH_H_ */
