#include "testApp.h"

void calibrate(Calibration& calib, string dir) {
	
	ofDirectory dirList;
	ofImage cur;
	
	dirList.listDir(dir);
	calib.setBoardSize(10, 7);
	calib.setSquareSize(10);
	for(int i = 0; i < dirList.size(); i++) {
		cout << "loading " << dirList.getPath(i) << endl;
		cur.loadImage(dirList.getPath(i));
		bool found = calib.add(toCv(cur));
		cout << "pattern found: " << found << endl;
	}
	cout << "calibrating for " << calib.size() << " good images out of " << dirList.size() << endl;
	calib.calibrate();
}

void testApp::setup() {
	ofSetVerticalSync(true);
	
	ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
	
	calibrate(kinectCalibration, SHARED_RESOURCE_PREFIX + "calibration/kinect/");
	calibrate(colorCalibration, SHARED_RESOURCE_PREFIX + "calibration/color/");
	
	kinectCalibration.save(SHARED_RESOURCE_PREFIX + "calibration/kinect.yml");
	colorCalibration.save(SHARED_RESOURCE_PREFIX + "calibration/color.yml");
	
	kinectCalibration.getTransformation(colorCalibration, rotationColorToKinect, translationColorToKinect);
	colorCalibration.getTransformation(kinectCalibration, rotationKinectToColor, translationKinectToColor);
	
	cout << "Kinect to Color:" << endl << rotationKinectToColor << endl << translationKinectToColor << endl;
	cout << "Color to Kinect:" << endl << rotationColorToKinect << endl << translationColorToKinect << endl;
	
	curImage = -1;
}

void testApp::update() {
}

void testApp::draw() {
	
	cam.begin();
	glEnable(GL_DEPTH_TEST);
	ofDrawAxis(100);
	Calibration* curCalibration;
	if(mouseX < ofGetWidth() / 2) {
		curCalibration = &kinectCalibration;
	} else {		
		curCalibration = &colorCalibration;
		applyMatrix(makeMatrix(rotationKinectToColor, translationKinectToColor));
	}
	
	if(curImage == -1) {
		curCalibration->draw3d();
	} else {
		curCalibration->draw3d(curImage);
	}
	
	glDisable(GL_DEPTH_TEST);
	cam.end();
}

void testApp::keyPressed(int key) {
	switch(key) {
		case OF_KEY_UP: curImage++; break;
		case OF_KEY_DOWN: curImage--; break;
	}
	curImage = ofClamp(curImage, -1, kinectCalibration.size() - 1);
}