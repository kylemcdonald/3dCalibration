#include "testApp.h"

void testApp::setup() {	
	ofSetVerticalSync(true);
	
	ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
	
	kinectList.listDir(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "depth/");
	colorList.listDir(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "color/");
	irList.listDir(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "ir/");
	
	kinectList.sort();
	colorList.sort();
	irList.sort();
	
	kinectCalibration.setSquareSize(2.5);
	kinectCalibration.setPatternSize(10, 7);
	
	colorCalibration.setSquareSize(2.5);
	colorCalibration.setPatternSize(10, 7);
	
	kinectCalibration.calibrateFromDirectory(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "ir/");
	colorCalibration.calibrateFromDirectory(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "color/");
	
	kinectCalibration.getTransformation(colorCalibration, rotationKinectToColor, translationKinectToColor);
	colorCalibration.getTransformation(kinectCalibration, rotationColorToKinect, translationColorToKinect);
	
	cout << "Kinect to Color:" << endl << rotationKinectToColor << endl << translationKinectToColor << endl;
	cout << "Color to Kinect:" << endl << rotationColorToKinect << endl << translationColorToKinect << endl;
	
#ifdef USE_GAMECAM
	cam.speed = 5;
	cam.autosavePosition = true;
	cam.useArrowKeys = false;
	cam.loadCameraPosition();
#endif
	
	curImage = 0;
	reloadImage = true;
}

const float
k1 = 0.1236,
k2 = 2842.5,
k3 = 1.1863,
k4 = 0.0370;

inline float rawToCentimeters(float raw) {
	return 100 * (k1 * tan((raw / k2) + k3) - k4);
}

void testApp::updatePointCloud() {
	pointCloud.clear();
	
	const unsigned int Xres = 640;
	const unsigned int Yres = 480;
	
	Point2d fov = kinectCalibration.getUndistortedIntrinsics().getFov();
	float fx = tanf(ofDegToRad(fov.x) / 2) * 2;
	float fy = tanf(ofDegToRad(fov.y) / 2) * 2;
	
	Point2d principalPoint = kinectCalibration.getUndistortedIntrinsics().getPrincipalPoint();
	cv::Size imageSize = kinectCalibration.getUndistortedIntrinsics().getImageSize();
	
	int w = 640;
	int h = 480;
//	int w = curKinect.getWidth();
//	int h = curKinect.getHeight();
//	float* pixels = curKinect.getPixels();
	Mat pixels = curKinect;
	int i = 0;
	
	/*
	 principalPoint.x += ofMap(mouseX, 0, ofGetWidth(), -4, 4);
	 principalPoint.y += ofMap(mouseY, 0, ofGetHeight(), -4, 4);
	 cout << "fudge: " << ofMap(mouseX, 0, ofGetWidth(), -4, 4) << "x" << ofMap(mouseY, 0, ofGetHeight(), -4, 4) << endl;
	 */
	
	for(int y = 0; y < h; y++) {
		for(int j = 0; j < w; j++) {
			float pixel = curKinect.at<float>(y, j);
			if(pixel < 1000) { // the rest is basically noise
				int x = Xres - j - 1; // x axis is flipped from depth image
				float z = rawToCentimeters(pixel);
				
				float xReal = (((float) x - principalPoint.x) / imageSize.width) * z * fx;
				float yReal = (((float) y - principalPoint.y) / imageSize.height) * z * fy;
				
				// add each point into pointCloud
				pointCloud.push_back(Point3f(xReal, yReal, z));
			}									
			i++;
		}
	}
}

void testApp::updateColors() {
	imagePoints.clear();
	
	// rotate, translate the points to fit the colorCalibration perspective
	// and project them onto the colorCalibration image space
	// and undistort them
	projectPoints(Mat(pointCloud),
								rotationKinectToColor, translationKinectToColor,
								colorCalibration.getDistortedIntrinsics().getCameraMatrix(),
								colorCalibration.getDistCoeffs(),
								imagePoints);
	
	// get the color at each of the projectedPoints inside curColor
	// add them into pointCloudColors
	pointCloudColors.clear();
	int w = curColor.getWidth();
	int h = curColor.getHeight();
	int n = w * h;
	unsigned char* pixels = curColor.getPixels();
	for(int i = 0; i < imagePoints.size(); i++) {
		int j = (int) imagePoints[i].y * w + (int) imagePoints[i].x;
		if(j < 0 || j >= n) {
			pointCloudColors.push_back(Point3f(1, 1, 1));
		} else {
			j *= 3;
			pointCloudColors.push_back(Point3f(pixels[j + 0] / 255.f, pixels[j + 1] / 255.f, pixels[j + 2] / 255.f));
		}
	}
}

void testApp::update() {
	if(reloadImage) {		
		curKinect = loadRaw(kinectList.getPath(curImage), 640, 480);
//		curKinect.loadRaw(kinectList.getPath(curImage), 640, 480);	

		kinectCalibration.undistort(toCv(curKinect));
		updatePointCloud();
		
		curColor.loadImage(colorList.getPath(curImage));
		curColor.update();
		updateColors();
		
		reloadImage = false;
	}	
}

void testApp::draw() {
	ofBackground(128);
	
	cam.begin();
	glEnable(GL_DEPTH_TEST);
	
	glPushMatrix();
	glScaled(1, -1, -1);
	
	ofDrawAxis(100);
	
	ofSetColor(255);
	curColor.draw(0, 0, curColor.getWidth(), curColor.getHeight());
	
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);
	glColorPointer(3, GL_FLOAT, sizeof(Point3f), &(pointCloudColors[0].x));
	glVertexPointer(3, GL_FLOAT, sizeof(Point3f), &(pointCloud[0].x));
	glDrawArrays(GL_POINTS, 0, pointCloud.size());
	glDisableClientState(GL_COLOR_ARRAY);
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glDisable(GL_DEPTH_TEST);
	
	ofSetColor(255);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, sizeof(Point2f), &(imagePoints[0].x));
	glDrawArrays(GL_POINTS, 0, pointCloud.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	
	Calibration* curCalibration;
	if(mouseX < ofGetWidth() / 2) {
		curCalibration = &kinectCalibration;
	} else {		
		curCalibration = &colorCalibration;
		applyMatrix(makeMatrix(rotationColorToKinect, translationColorToKinect));
	}
	
	curCalibration->draw3d(curImage);
	
	glPopMatrix();
	
	cam.end();
	
}

void testApp::keyPressed(int key) {
	switch(key) {
		case OF_KEY_UP: curImage++; break;
		case OF_KEY_DOWN: curImage--; break;
	}
	
	curImage = ofClamp(curImage, 0, kinectList.size() - 1);
	reloadImage = true;
}