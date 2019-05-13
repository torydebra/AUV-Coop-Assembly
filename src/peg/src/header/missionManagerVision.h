#ifndef MISSIONMANAGERVISION_H
#define MISSIONMANAGERVISION_H

#include <iostream>
#include <fstream>
#include "../rosInterfaces/header/robotVisionInterface.h"
#include "../rosInterfaces/header/worldInterface.h"
#include "../helper/header/infosVision.h"
#include "../helper/header/logger.h"

#include "../vision/header/detector.h"
#include "../vision/header/tracker.h"

#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>

int main(int, char**);


#endif // MISSIONMANAGERVISION_H
