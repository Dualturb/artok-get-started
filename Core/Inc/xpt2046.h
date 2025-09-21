#ifndef INC_TOUCH_H_
#define INC_TOUCH_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

 // XPT2046 Touch Controller Constants
 #define TP_PRES_DOWN 0x80
 #define TP_CATH_PRES 0x40
 #define CMD_RDX 0xD0
 #define CMD_RDY 0x90

 // Raw touch coordinate boundaries and pressure threshold.
 // These are specific to your hardware and may require calibration.
 #define XPT_XMIN 350
 #define XPT_YMIN 350
 #define XPT_XMAX 3800
 #define XPT_YMAX 3600
 #define XPT_WIDTH (XPT_XMAX - XPT_XMIN)
 #define XPT_HEIGHT (XPT_YMAX - XPT_YMIN)
 #define Z_THRESHOLD 400

 void XPT2046_Init(void);
 bool XPT2046_Read(uint16_t *x, uint16_t *y);
 uint8_t XPT2046_IsReasonable(uint16_t x, uint16_t y);
#ifdef __cplusplus
}
#endif

#endif /* INC_TOUCH_H_ */
