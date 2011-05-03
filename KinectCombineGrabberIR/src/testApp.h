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
	ofVideoGrabber color;
	
	vector<FloatImage*> kinectBuffer;
	vector<ofPixels*> irBuffer;
	vector<ofPixels*> colorBuffer;
	
	vector<unsigned long> kinectTime;
	vector<unsigned long> colorTime;
	
	int curImageColor;
	int curImageKinect;
	int totalImages;
	bool recordingKinect, recordingColor, recordingIr;
	bool needToSave;
};
