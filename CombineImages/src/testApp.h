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
	
	void updatePointCloud();
	void updateColors();
	
	Calibration leftCalibration, rightCalibration;
	Mat rotation, translation;
	ofEasyCam cam;
	
	ofImage curLeft, curRight;
	ofDirectory leftList, rightList;
	
	vector<Point2f> imagePoints;
	vector<Point3f> pointCloud;
	vector<Point3f> pointCloudColors;
	
	int curImage;
	bool reloadImage;
};
