#include "testApp.h"

void testApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	
	ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
	
	leftCalibration.load("calibration/left.yml");
	rightCalibration.load("calibration/right.yml");
	
	leftList.listDir("video/left/");
	leftList.sort();
	rightList.listDir("video/right/");
	rightList.sort();
	
	FileStorage fs(ofToDataPath("calibration/rightToLeft.yml"), FileStorage::READ);
	fs["rotation"] >> rotation;
	fs["translation"] >> translation;
	
	cout << "rotation:" << endl << rotation << endl;
	cout << "translation:" << endl << translation << endl;
	
	cout << "left: " << leftCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	cout << "right: " << rightCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	
	
	curImage = 0;
	reloadImage = true;
}

void testApp::updatePointCloud() {
	pointCloud.clear();
	
	const unsigned int Xres = 640;
	const unsigned int Yres = 480;
	
	Point2d fov = leftCalibration.getUndistortedIntrinsics().getFov();
	float fx = tanf(ofDegToRad(fov.x) / 2) * 2;
	float fy = tanf(ofDegToRad(fov.y) / 2) * 2;
	
	Point2d principalPoint = leftCalibration.getUndistortedIntrinsics().getPrincipalPoint();
	cv::Size imageSize = leftCalibration.getUndistortedIntrinsics().getImageSize();
	
	cout << "principal point is " << principalPoint << endl;
	
	cout << "loading point cloud" << endl;
	
	int w = curLeft.getWidth();
	int h = curLeft.getHeight();
	unsigned char* pixels = curLeft.getPixels();
	int i = 0;
	float depthNear = 40;
	float depthRange = 90 - depthNear;
	for(int y = 0; y < h; y++) {
		for(int j = 0; j < w; j++) {
			if(pixels[i] != 0 && pixels[i] != 255) {
				// from disk, we need to recover the distance
				int x = Xres - j - 1; // x axis is flipped from depth image
				float z = ((float) pixels[i] / 255) * depthRange + depthNear;
				
				// is this projective to real world transform correct?
				// what about the principal point?
				// then do projective to real world transform
				float xReal = (((float) x - principalPoint.x) / imageSize.width) * z * fx;
				float yReal = (((float) y - principalPoint.y) / imageSize.height) * z * fy;
				
				// add each point into pointCloud
				pointCloud.push_back(Point3f(xReal, yReal, z));
			}									
			i++;
		}
	}
	
	cout << "point cloud size: " << pointCloud.size() << endl;
}

void testApp::updateColors() {
	imagePoints.clear();
	
	// rotate, translate the points to fit the rightCalibration perspective
	// and project them onto the rightCalibration image space
	// and undistort them
	projectPoints(Mat(pointCloud),
								rotation, translation,
								rightCalibration.getDistortedIntrinsics().getCameraMatrix(),
								rightCalibration.getDistCoeffs(),
								imagePoints);
	
	// get the color at each of the projectedPoints inside curRight
	// add them into pointCloudColors
	pointCloudColors.clear();
	cv::Size curSize = rightCalibration.getUndistortedIntrinsics().getImageSize();
	int w = curSize.width;
	int h = curSize.height;
	int n = w * h - 4;
	unsigned char* pixels = curRight.getPixels();
	for(int i = 0; i < imagePoints.size(); i++) {
		int j = 3 * ((int) imagePoints[i].y * w + (int) imagePoints[i].x);
		j = ofClamp(j, 0, n);
		pointCloudColors.push_back(Point3f(pixels[j + 0] / 255.f,
																			 pixels[j + 1] / 255.f,
																			 pixels[j + 2] / 255.f));
	}
}

void testApp::update() {
	if(reloadImage) {
		curLeft.loadImage(leftList.getPath(curImage));
		curRight.loadImage(rightList.getPath(curImage));
		
		leftCalibration.undistort(toCv(curLeft));
		//rightCalibration.undistort(curRight); // projectPoints will undistort for us
		
		curLeft.update();
		curRight.update();
		
		updatePointCloud();
		updateColors();
		
		reloadImage = false;
	}
}

void testApp::draw() {
	ofBackground(0);
	
	cam.begin();
	glEnable(GL_DEPTH_TEST);
	ofDrawAxis(100);
	
	ofSetColor(255);
	ofPushMatrix();
	ofTranslate(0, 0, -10);
	curLeft.draw(0, -480);
	curRight.draw(0, 0);
	ofPopMatrix();
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glColorPointer(3, GL_FLOAT, sizeof(Point3f), &(pointCloudColors[0].x));
	glVertexPointer(3, GL_FLOAT, sizeof(Point3f), &(pointCloud[0].x));
	glDrawArrays(GL_POINTS, 0, pointCloud.size());
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	
	ofSetColor(255);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, sizeof(Point2f), &(imagePoints[0].x));
	glDrawArrays(GL_POINTS, 0, pointCloud.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glDisable(GL_DEPTH_TEST);
	cam.end();
}

void testApp::keyPressed(int key) {
	switch(key) {
		case OF_KEY_UP: curImage++; break;
		case OF_KEY_DOWN: curImage--; break;
	}
	curImage = ofClamp(curImage, 0, leftList.size() - 1);
	reloadImage = true;
}