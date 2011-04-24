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
	
	calibrate(leftCalibration, "calibration/left/");
	calibrate(rightCalibration, "calibration/right/");
	
	leftCalibration.save("calibration/left.yml");
	rightCalibration.save("calibration/right.yml");
	
	rightCalibration.getTransformation(leftCalibration, rotationRL, translationRL);
	leftCalibration.getTransformation(rightCalibration, rotationLR, translationLR);
	
	cout << "LR:" << endl << rotationLR << endl << translationLR << endl;
	cout << "RL:" << endl << rotationRL << endl << translationRL << endl;
	
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
		curCalibration = &leftCalibration;
	} else {		
		if(true || mouseY > ofGetHeight() / 2) {
			curCalibration = &rightCalibration;
			cout << "norm ";
			applyMatrix(makeMatrix(rotationRL, translationRL));
		} else {
			curCalibration = &leftCalibration;
			cout << "inv ";
			Mat rotationInvLR = rotationLR.inv();
			applyMatrix(makeMatrix(rotationInvLR, -translationLR));
		}
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
	curImage = ofClamp(curImage, -1, leftCalibration.size() - 1);
}