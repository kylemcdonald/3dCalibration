#pragma once

#include "ofMain.h"

#include "ofxCv.h"
using namespace cv;
using namespace ofxCv;

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	
	Calibration leftCalibration, rightCalibration;
};
