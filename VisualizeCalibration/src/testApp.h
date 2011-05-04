#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#define SHARED_RESOURCE_PREFIX string("../../../SharedBin/data/")
#define DATA_PREFIX string("triple/")

using namespace cv;
using namespace ofxCv;

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
	Calibration kinectCalibration, colorCalibration;
	Mat rotationKinectToColor, translationKinectToColor;
	Mat rotationColorToKinect, translationColorToKinect;
	ofEasyCam cam;
	
	int curImage;
};
