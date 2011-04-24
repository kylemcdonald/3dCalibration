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
	void keyPressed(int key);
	
	Calibration leftCalibration, rightCalibration;
	Mat rotationLR, translationLR;
	Mat rotationRL, translationRL;
	ofEasyCam cam;
	
	int curImage;
};
