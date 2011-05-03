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
	
	vector<FloatImage*> kinectBuffer;
	vector<ofImage*> colorBuffer;
	
	vector<int> kinectTime;
	vector<int> colorTime;
	
	vector<Point2f> colorCenter;
	vector<cv::Rect> kinectRoi;
	vector<float> colorDistance;
	vector<float> kinectDistance;
	
	Calibration calibration;
};
