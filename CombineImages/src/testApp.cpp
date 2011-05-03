#include "testApp.h"

void testApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	
	ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
	
	kinectCalibration.load(SHARED_RESOURCE_PREFIX + CALIBRATION_PREFIX + "kinect.yml");
	colorCalibration.load(SHARED_RESOURCE_PREFIX + CALIBRATION_PREFIX + "color.yml");
	
//	kinectList.listDir("video/left/");
//	kinectList.sort();
//	colorList.listDir("video/right/");
//	colorList.sort();

	kinectList.listDir(SHARED_RESOURCE_PREFIX + SEQUENCE_PREFIX + "kinect/");
	kinectList.sort();
	colorList.listDir(SHARED_RESOURCE_PREFIX + SEQUENCE_PREFIX + "color/");
	colorList.sort();
	
//	kinectCalibration.getTransformation(colorCalibration, rotationKinectToColor, translationKinectToColor);
//	colorCalibration.getTransformation(kinectCalibration, rotationColorToKinect, translationColorToKinect);    

//    cout << "rotation:" << endl << rotationKinectToColor << endl;
//	cout << "translation:" << endl << translationKinectToColor << endl;

//	FileStorage fs(ofToDataPath(SHARED_RESOURCE_PREFIX+"calibration/colorToKinect.yml"), FileStorage::READ);
    FileStorage fs(ofToDataPath(SHARED_RESOURCE_PREFIX + CALIBRATION_PREFIX + "kinectToColor.yml"), FileStorage::READ);
	fs["rotation"] >> rotation;
	fs["translation"] >> translation;
	
	cout << "rotation:" << endl << rotation << endl;
	cout << "translation:" << endl << translation << endl;
	
	cout << "kinect: " << kinectCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	cout << "color: " << colorCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	
	
	#ifdef USE_GAMECAM
	cam.speed = 10;
	cam.autosavePosition = true;
	cam.useArrowKeys = false;
	cam.loadCameraPosition();
//	cam.minimumY = 0;
//	cam.maximumY = 360;
	#endif
	
	curImage = 0;
	reloadImage = true;
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
//	
//    cout << "size " << imageSize.width << " " << imageSize.height << endl;
//	cout << "principal point is " << principalPoint << endl;
//	cout << "loading point cloud" << endl;
	
	int w = curKinect.getWidth();
	int h = curKinect.getHeight();
	unsigned char* pixels = curKinect.getPixels();
	int i = 0;
	float depthNear = 40;
	float depthFar = 90;
//	float depthNear = ofMap(mouseX, 0, 1000, 0, 80);
//	float depthFar = depthNear + mouseY;
	float depthRange = depthFar - depthNear;
//	xfudge = mouseX/50.0;
//	yfudge = mouseY*2.0;
	xfudge = 0;//-25.72;
	yfudge = 0;//-40;
	for(int y = 0; y < h; y++) {
		for(int j = 0; j < w; j++) {
			if(pixels[i] != 0 && pixels[i] != 255) {
				// from disk, we need to recover the distance
				int x = Xres - j - 1; // x axis is flipped from depth image
                //int x = j; // x axis is flipped from depth image
				//float z = ((float) pixels[i] / 255) * depthRange + depthNear;
				float z = ofMap(pixels[i], 255, 0, depthNear, depthFar);

				// is this projective to real world transform correct?
				// what about the principal point?
				// then do projective to real world transform
				//x: 5.45 y: -0.43
				float xReal = (((float) x - principalPoint.x - xfudge) / imageSize.width) * z * fx;
				float yReal = (((float) y - principalPoint.y - yfudge) / imageSize.height) * z * fy;
                
				// add each point into pointCloud
				pointCloud.push_back(Point3f(xReal, yReal, z));
			}									
			i++;
		}
	}
	
	
	
	//cout << "point cloud size: " << pointCloud.size() << endl;
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
		curKinect.loadImage(kinectList.getPath(curImage));

		curColor.loadImage(colorList.getPath(curImage));
		
		kinectCalibration.undistort(toCv(curKinect));
		//colorCalibration.undistort(curColor); // projectPoints will undistort for us
		
		curKinect.update();
		curColor.update();
		
		reloadImage = false;
	}
	
	updatePointCloud();
	updateColors();
		
}

void testApp::draw() {
	ofBackground(0);
	
	cam.begin();
	
	ofDrawAxis(100);
	
	ofSetColor(255);
	curKinect.draw(0, curKinect.getHeight(), curKinect.getWidth(), -curKinect.getHeight());
	curColor.draw(0, 0, curColor.getWidth(), -curColor.getHeight());	
	
	glPushMatrix();
	glScaled(1, -1, 1);
	glEnable(GL_DEPTH_TEST);		
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
	
	glPopMatrix();
	
	glDisable(GL_DEPTH_TEST);
	cam.end();
		
}

void testApp::keyPressed(int key) {
	switch(key) {
		case OF_KEY_UP: curImage++; break;
		case OF_KEY_DOWN: curImage--; break;
	}
	if(key == ' '){
		cout << "shift is x: " << xfudge << " y: " << yfudge << endl;
	}
	curImage = ofClamp(curImage, 0, kinectList.size() - 1);
	reloadImage = true;
}