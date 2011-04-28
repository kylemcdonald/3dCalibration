#include "testApp.h"

void saveTransformation(Calibration& from, Calibration& to, string filename) {
	Mat rotation, translation;
	
	from.getTransformation(to, rotation, translation);
	FileStorage fs(ofToDataPath(filename), FileStorage::WRITE);
    
	fs << "rotation" << rotation;
	fs << "translation" << translation;
		
	cout << "rotation:" << endl << rotation << endl;
	cout << "translation:" << endl << translation << endl;
}

void testApp::setup() {
	kinectCalibration.setBoardSize(10, 7);
	kinectCalibration.setSquareSize(2.5); // same units as focal length and real world space (cm)
	kinectCalibration.calibrateFromDirectory(SHARED_RESOURCE_PREFIX + "calibration/kinect/");
	kinectCalibration.save(SHARED_RESOURCE_PREFIX + "calibration/kinect.yml");
	
	colorCalibration.setBoardSize(10, 7);
	colorCalibration.setSquareSize(2.5);
	colorCalibration.calibrateFromDirectory(SHARED_RESOURCE_PREFIX + "calibration/color/");
	colorCalibration.save(SHARED_RESOURCE_PREFIX + "calibration/color.yml");
	
	saveTransformation(kinectCalibration, colorCalibration, SHARED_RESOURCE_PREFIX + "calibration/kinectToColor.yml");
	saveTransformation(colorCalibration, kinectCalibration, SHARED_RESOURCE_PREFIX + "calibration/colorToKinect.yml");
}

void testApp::update() {
}

void testApp::draw() {
}
