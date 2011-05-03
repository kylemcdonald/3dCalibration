#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#define SHARED_RESOURCE_PREFIX string("../../../SharedBin/data/")
//#define CALIBRATION_PREFIX string("calibration-old/")
#define DATA_PREFIX string("old_sequence/")
using namespace cv;
using namespace ofxCv;

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
	Calibration kinectCalibration, colorCalibration;
};
