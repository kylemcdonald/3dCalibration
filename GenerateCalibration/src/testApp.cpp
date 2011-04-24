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
	leftCalibration.setBoardSize(10, 7);
	leftCalibration.setSquareSize(2.54); // same units as focal length and real world space
	leftCalibration.calibrateFromDirectory("left/");
	leftCalibration.save("left.yml");
	
	rightCalibration.setBoardSize(10, 7);
	rightCalibration.setSquareSize(2.54);
	rightCalibration.calibrateFromDirectory("right/");
	rightCalibration.save("right.yml");
	
	saveTransformation(leftCalibration, rightCalibration, "leftToRight.yml");
	saveTransformation(rightCalibration, leftCalibration, "rightToLeft.yml");
}

void testApp::update() {
}

void testApp::draw() {
}
