#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#define SHARED_RESOURCE_PREFIX string("../../../SharedBin/data/")

using namespace cv;
using namespace ofxCv;

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
	void updatePointCloud();
	void updateColors();
	
	Calibration kinectCalibration, colorCalibration;
	Mat rotation, translation;
	ofEasyCam cam;
	
	ofImage curKinect, curColor;
	ofDirectory kinectList, colorList;
	
	vector<Point2f> imagePoints;
	vector<Point3f> pointCloud;
	vector<Point3f> pointCloudColors;
	
	int curImage;
	bool reloadImage;
};
