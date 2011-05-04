#include "testApp.h"

void calibrate(Calibration& calib, string dir) {
	
	ofDirectory dirList;
	ofImage cur;
	
	dirList.listDir(dir);
	calib.setBoardSize(10, 7);
	calib.setSquareSize(2.5);
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
	
	loadCalibrationFromFile();
	
	kinectList.listDir(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "depth/");
	kinectList.sort();
	colorList.listDir(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "color/");
	colorList.sort();
	irList.listDir(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "ir/");
	irList.sort();
	
	FileStorage fs(ofToDataPath(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "kinectToColor.yml"), FileStorage::READ);
	fs["rotation"] >> rotation;
	fs["translation"] >> translation;
	
	cout << "rotation:" << endl << rotation << endl;
	cout << "translation:" << endl << translation << endl;
	
	cout << "kinect: " << kinectCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	cout << "color: " << colorCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	
#ifdef USE_GAMECAM
	cam.speed = 5;
	cam.autosavePosition = true;
	cam.useArrowKeys = false;
	cam.loadCameraPosition();
#endif
	
	drawChessboards = false;
	chessboardsLoaded = false;
	
	curImage = 0;
	reloadImage = true;
	
	useColor = true;
}

// these are for converting centimeters to/from raw values
// using equation from http://openkinect.org/wiki/Imaging_Information
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
	
	int w = curKinect.getWidth();
	int h = curKinect.getHeight();
	float* pixels = curKinect.getPixels();
	int i = 0;
	
	for(int y = 0; y < h; y++) {
		for(int j = 0; j < w; j++) {
			if(pixels[i] != 2048) {
				int x = Xres - j - 1; // x axis is flipped from depth image
				float z = rawToCentimeters(pixels[i]);
				
				float yflipped = Yres - y - 1;
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
								rotation, translation,
								colorCalibration.getDistortedIntrinsics().getCameraMatrix(),
								colorCalibration.getDistCoeffs(),
								imagePoints);
	
	// get the color at each of the projectedPoints inside curColor
	// add them into pointCloudColors
	pointCloudColors.clear();
	cv::Size curSize = colorCalibration.getUndistortedIntrinsics().getImageSize();
	int w = curSize.width;
	int h = curSize.height;
	int n = (w * h) * 3;
	unsigned char* pixels = curColor.getPixels();
	for(int i = 0; i < imagePoints.size(); i++) {
		int j = 3 * ((int) imagePoints[i].y * w + (int) imagePoints[i].x);
		j = ofClamp(j, 0, n-1);
		pointCloudColors.push_back(Point3f(pixels[j + 0] / 255.f, 
																			 pixels[j + 1] / 255.f, 
																			 pixels[j + 2] / 255.f));
	}
}

void testApp::update() {
	if(reloadImage) {
		curKinect.loadRaw(kinectList.getPath(curImage), 640, 480);
		
		curColor.loadImage(colorList.getPath(curImage));
		
		kinectCalibration.undistort(toCv(curKinect));
		
		curColor.update();
		
		reloadImage = false;
	}
	
	updatePointCloud();
	updateColors();
	
}

void testApp::loadCalibrationFromFile() {
	kinectCalibration.load(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "kinect.yml");
	colorCalibration.load(SHARED_RESOURCE_PREFIX +  DATA_PREFIX + "color.yml");
}

void testApp::loadCalibrationFromImages() {
	calibrate(kinectCalibration, SHARED_RESOURCE_PREFIX + DATA_PREFIX + "ir/");
	calibrate(colorCalibration, SHARED_RESOURCE_PREFIX + DATA_PREFIX + "color/");
	
	kinectCalibration.save(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "kinect.yml");
	colorCalibration.save(SHARED_RESOURCE_PREFIX + DATA_PREFIX + "color.yml");
	
	kinectCalibration.getTransformation(colorCalibration, rotationKinectToColor, translationKinectToColor);
	colorCalibration.getTransformation(kinectCalibration, rotationColorToKinect, translationColorToKinect);
	
	cout << "Kinect to Color:" << endl << rotationKinectToColor << endl << translationKinectToColor << endl;
	cout << "Color to Kinect:" << endl << rotationColorToKinect << endl << translationColorToKinect << endl;
	
	curImage = -1;	
}

void testApp::showCalibrationChessboards() {
	if(!chessboardsLoaded){
		loadCalibrationFromImages();
		chessboardsLoaded = true;
	}
	
	ofDrawAxis(100);
	Calibration* curCalibration;
	if(mouseX < ofGetWidth() / 2) {
		curCalibration = &kinectCalibration;
	} else {		
		curCalibration = &colorCalibration;
		applyMatrix(makeMatrix(rotationColorToKinect, translationColorToKinect));
	}
	
	if(curImage == -1) {
		curCalibration->draw3d();
	} else {
		curCalibration->draw3d(curImage);
	}
}

void testApp::draw() {
	ofBackground(128);
	
	cam.begin();
	glEnable(GL_DEPTH_TEST);
	
	glPushMatrix();
	glScaled(1, -1, 1);
	
	ofDrawAxis(100);
	
	ofSetColor(255);
	curColor.draw(0, 0, curColor.getWidth(), curColor.getHeight());
	
	glEnableClientState(GL_VERTEX_ARRAY);
	if(useColor) {
		glEnableClientState(GL_COLOR_ARRAY);
		glColorPointer(3, GL_FLOAT, sizeof(Point3f), &(pointCloudColors[0].x));
	}
	glVertexPointer(3, GL_FLOAT, sizeof(Point3f), &(pointCloud[0].x));
	glDrawArrays(GL_POINTS, 0, pointCloud.size());
	if(useColor) {
		glDisableClientState(GL_COLOR_ARRAY);
	}
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glDisable(GL_DEPTH_TEST);
	
	ofSetColor(255);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(2, GL_FLOAT, sizeof(Point2f), &(imagePoints[0].x));
	glDrawArrays(GL_POINTS, 0, pointCloud.size());
	glDisableClientState(GL_VERTEX_ARRAY);
	
	glPopMatrix();
	
	if(drawChessboards){
		showCalibrationChessboards();
	}
	
	cam.end();
	
}

void testApp::keyPressed(int key) {
	switch(key) {
		case OF_KEY_UP: curImage++; break;
		case OF_KEY_DOWN: curImage--; break;
	}
	
	if(key == 'c'){
		drawChessboards = !drawChessboards;
	}
	
	if(key == 'v') {
		useColor = !useColor;
	}
	
	curImage = ofClamp(curImage, 0, kinectList.size() - 1);
	reloadImage = true;
}