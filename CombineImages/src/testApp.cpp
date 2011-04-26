#include "testApp.h"

void testApp::setup() {
	ofSetVerticalSync(true);
	ofSetFrameRate(30);
	
	ofSetDrawBitmapMode(OF_BITMAPMODE_MODEL_BILLBOARD);
	
	kinectCalibration.load(SHARED_RESOURCE_PREFIX + "calibration/kinect.yml");
	colorCalibration.load(SHARED_RESOURCE_PREFIX + "calibration/color.yml");
	
//	kinectList.listDir("video/left/");
//	kinectList.sort();
//	colorList.listDir("video/right/");
//	colorList.sort();

	kinectList.listDir(SHARED_RESOURCE_PREFIX + "sequence/kinect/");
	kinectList.sort();
	colorList.listDir(SHARED_RESOURCE_PREFIX + "sequence/color/");
	colorList.sort();
	
//	kinectCalibration.getTransformation(colorCalibration, rotationKinectToColor, translationKinectToColor);
//	colorCalibration.getTransformation(kinectCalibration, rotationColorToKinect, translationColorToKinect);    

//    cout << "rotation:" << endl << rotationKinectToColor << endl;
//	cout << "translation:" << endl << translationKinectToColor << endl;

	//FileStorage fs(ofToDataPath(SHARED_RESOURCE_PREFIX+"calibration/colorToKinect.yml"), FileStorage::READ);
    FileStorage fs(ofToDataPath(SHARED_RESOURCE_PREFIX+"calibration/kinectToColor.yml"), FileStorage::READ);
	fs["rotation"] >> rotation;
	fs["translation"] >> translation;
	
	cout << "rotation:" << endl << rotation << endl;
	cout << "translation:" << endl << translation << endl;
	
	cout << "kinect: " << kinectCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	cout << "color: " << colorCalibration.getUndistortedIntrinsics().getCameraMatrix() << endl;
	
	
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
	
    cout << "size " << imageSize.width << " " << imageSize.height << endl;
	cout << "principal point is " << principalPoint << endl;
	cout << "loading point cloud" << endl;
	
	int w = curKinect.getWidth();
	int h = curKinect.getHeight();
	unsigned char* pixels = curKinect.getPixels();
	int i = 0;
	float depthNear = 40;
	float depthRange = 90 - depthNear;
	for(int y = 0; y < h; y++) {
		for(int j = 0; j < w; j++) {
			if(pixels[i] != 0 && pixels[i] != 255) {
				// from disk, we need to recover the distance
				//int x = Xres - j - 1; // x axis is flipped from depth image
                int x = j; // x axis is flipped from depth image
				float z = ((float) pixels[i] / 255) * depthRange + depthNear;
				
				// is this projective to real world transform correct?
				// what about the principal point?
				// then do projective to real world transform
				float xReal = (((float) x - principalPoint.x) / imageSize.width) * z * fx;
				float yReal = (((float) y - principalPoint.y) / imageSize.height) * z * fy;
//				float xReal = ofMap(x - principalPoint.x, -w/2, w/2, -imageSize.width/2, imageSize.width/2);
//                float yReal = ofMap(y - principalPoint.y, -h/2, h/2, -imageSize.height/2, imageSize.height/2);
                
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
	curKinect.draw(0, -480);
	curColor.draw(0, 0);
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
	curImage = ofClamp(curImage, 0, kinectList.size() - 1);
	reloadImage = true;
}