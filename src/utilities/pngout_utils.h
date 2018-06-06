/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef PNGOUT_UTILS_H
#define PNGOUT_UTILS_H

#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>
#include <stdbool.h>

//Why do we need to re-prototype this function?
ILboolean ilutGLScreen();

/* If PNGOutPic is true, then a singe image will be saved
 */
static bool PNGOutPic = false;

/* How many video-frame sets have been saved? 
 */
static int VidNum = 0;

/* Take a screenshot 
 */
void TakeScreenshot(const char *screenshotFile);

#endif
