#include "testApp.h"

void testApp::setup() {
	color.initGrabber(640, 480);
	
	kinect.init(false, false);
	kinect.open();
	
	totalImages = 128;
	curImageColor = 0;
	curImageKinect = 0;
	recording = false;
	needToSave = true;
	
	kinectBuffer.resize(totalImages);
	irBuffer.resize(totalImages);
	colorBuffer.resize(totalImages);
	
	kinectTime.resize(totalImages);
	colorTime.resize(totalImages);
	
	for(int i = 0; i < totalImages; i++) {
		kinectBuffer[i] = new FloatImage();
		kinectBuffer[i]->allocate(640, 480);
		
		irBuffer[i] = new ofPixels();
		irBuffer[i]->allocate(640, 480, OF_IMAGE_GRAYSCALE);
		
		colorBuffer[i] = new ofPixels();
		colorBuffer[i]->allocate(640, 480, OF_IMAGE_COLOR);
	}
}

#include "Poco/NumberFormatter.h"
void testApp::update() {	
	if(kinect.isConnected()) { 
		kinect.update();
		if(kinect.isFrameNew()) {
			if(recording && curImageKinect < totalImages) {
				kinectTime[curImageKinect] = ofGetSystemTime();
				unsigned short* pixels = kinect.getRawDepthPixels();
				float* curBuffer = kinectBuffer[curImageKinect]->getPixels();
				int n = 640 * 480;
				for(int i = 0; i < n; i++) {
					curBuffer[i] = pixels[i];
				}
				curImageKinect++;
			}
		}
	}
	
	color.update();
	if(color.isFrameNew()) {
		if(recording && curImageColor < totalImages) {
			colorTime[curImageColor] = ofGetSystemTime();
			*colorBuffer[curImageColor] = color.getPixelsRef();
			curImageColor++;
		}
	}
	
	if(curImageColor == totalImages && curImageKinect == totalImages && needToSave) {
		ofFile time;
		time.open("time.csv", ofFile::WriteOnly);
		for(int i = 0; i < totalImages; i++) {
			string padded;
			Poco::NumberFormatter::append0(padded, i, 3);
			kinectBuffer[i]->saveRaw("kinect-" + padded + ".raw");
			ofSaveImage(*irBuffer[i], "ir-" + padded + ".png");
			ofSaveImage(*colorBuffer[i], "color-" + padded + ".png");
			time << colorTime[i] << "\t" << kinectTime[i] << endl;
		}
		time.close();
		needToSave = false;
	}
}

void testApp::draw() {
	ofBackground(0);
	
	kinect.drawDepth(0, 0);
	color.draw(640, 0);
}

void testApp::exit() {
	kinect.close();
}

void testApp::keyPressed(int key) {
	if(key == ' ') {
		recording = !recording;
	}
}