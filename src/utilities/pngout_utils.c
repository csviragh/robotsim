//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#include "pngout_utils.h"

/* Take a screenshot */

void TakeScreenshot(const char *screenshotFile) {

    ILuint imageID = ilGenImage();
    ilBindImage(imageID);
    ilutGLScreen();
    ilEnable(IL_FILE_OVERWRITE);
    ilSaveImage(screenshotFile);
    ilDeleteImage(imageID);

}
