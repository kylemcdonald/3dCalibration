#pragma once

#include "ofMain.h"

#include "ofxCv.h"
#define SHARED_RESOURCE_PREFIX string("../../../SharedBin/data/")
#define DATA_PREFIX string("old_sequence/")
//#define SEQUENCE_PREFIX string("sequence/")

using namespace cv;
using namespace ofxCv;

#define USE_GAMECAM

#ifdef USE_GAMECAM
#include "ofxGameCamera.h"
#endif

class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void keyPressed(int key);
	
	void updatePointCloud();
	void updateColors();
	
	Calibration kinectCalibration, colorCalibration;    
    Mat rotationKinectToColor, translationKinectToColor;
	Mat rotationColorToKinect, translationColorToKinect;

	Mat rotation, translation;
	#ifdef USE_GAMECAM
	ofxGameCamera cam;
	#else
	ofEasyCam cam;
	#endif
	ofImage curKinect, curColor;
	ofDirectory kinectList, colorList, irList;
	
	vector<Point2f> imagePoints;
	vector<Point3f> pointCloud;
	vector<Point3f> pointCloudColors;
	float xfudge;
	float yfudge;

	int curImage;
	bool reloadImage;
	

	bool drawCheckboards;
	bool checkboardsLoaded;
	void loadCalibrationFromImages();
	void loadCalibrationFromFile();
	void showCalibrationCheckboards();
};
