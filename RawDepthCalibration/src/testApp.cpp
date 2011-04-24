#include "testApp.h"

void testApp::setup() {
	ofSetVerticalSync(true);
	
	kinect.init(true);
	kinect.open();
	
	cur.allocate(640, 480, OF_IMAGE_GRAYSCALE);
}

void testApp::update() {
	if(kinect.isConnected()) {
		kinect.update();
		if(kinect.isFrameNew()) {
			if(saveIr) {
				memcpy(cur.getPixels(), kinect.getPixels(), 640 * 480);
				cur.saveImage("ir.png");
				saveIr = false;
			}
			if(saveDepth) {
				ofxKinectRecorder recorder;
				recorder.init("depth.raw");
				recorder.newFrame(kinect.getRawDepthPixels());
				saveDepth = false;
			}
		}
	}
}

void testApp::draw() {
	ofBackground(0);
	
	int tw = 640;
	int th = 480;
	
	ofSetColor(255);
	try {
		kinect.draw(0, 0, tw, th);
		kinect.drawDepth(0, th, tw, th);
	} catch (exception& e) {
		cout << ofGetTimestampString("%H:%M:%S:%i") << " kinect.draw() failed: " << e.what() << endl;
	}
	
	ofSetColor(ofColor::red);
	ofLine(tw / 2, 0, tw / 2, ofGetHeight());
	ofLine(0, th / 2, ofGetWidth(), th / 2);
	ofLine(0, th + th / 2, ofGetWidth(), th + th / 2);
}

void testApp::exit() {
	kinect.close();
}

void testApp::keyPressed(int key) {
	if(key == 'i') {
		saveIr = true;
	}
	if(key == 'd') {
		saveDepth = true;
	}
}