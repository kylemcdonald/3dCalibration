#pragma once

#include "ofMain.h"
#include "ofxKinect.h"

#include "ofxCv.h"
using namespace cv;
using namespace ofxCv;

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();
	
	void keyPressed(int key);
	
	ofxKinect kinect;
	bool saveIr, saveDepth;
	
	ofImage cur;
};
