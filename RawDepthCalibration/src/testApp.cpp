#include "testApp.h"

void testApp::setup() {
	ofFile time;
	time.open("time.csv");
	if(time.is_open()) {
		int i = 0;
		while(!time.eof()) {
			int curKinectTime, curColorTime;
			time >> curColorTime;
			time >> curKinectTime;
			kinectTime.push_back(curKinectTime);
			colorTime.push_back(curColorTime);
		}
		time.close();
	}
	
	cout << "loading color images" << endl;	
	ofDirectory colorDir;
	colorDir.listDir("color");	
	colorBuffer.resize(colorDir.size());
	for(int i = 0; i < colorDir.size(); i++) {
		colorBuffer[i] = new ofImage();
		colorBuffer[i]->loadImage(colorDir.getPath(i));
		colorBuffer[i]->setImageType(OF_IMAGE_GRAYSCALE);
	}
	
	cout << "loading kinect images" << endl;
	ofDirectory kinectDir;
	kinectDir.listDir("kinect");	
	kinectBuffer.resize(kinectDir.size());
	for(int i = 0; i < kinectDir.size(); i++) {
		kinectBuffer[i] = new FloatImage();
		kinectBuffer[i]->loadRaw(kinectDir.getPath(i), 640, 480);
	}
	
	cout << "analyzing" << endl;
	calibration.load("ps3eye-zoomed.yml");
	Mat cameraMatrix = calibration.getDistortedIntrinsics().getCameraMatrix();
	Mat distCoeffs = calibration.getDistCoeffs();
	
	vector<Point3f> objectPoints;
	cv::Size boardSize(4, 3);
	float boardScale = 50; // mm
	for(int y = 0; y < boardSize.height; y++) {
		for(int x = 0; x < boardSize.width; x++) {
			objectPoints.push_back(Point3f((x - ((boardSize.width - 1) / 2)) * boardScale,
																		 (y - ((boardSize.height - 1) / 2)) * boardScale, 0));
		}
	}
	
	cout << "finding chessboards" << endl;
	colorCenter.resize(colorBuffer.size());
	colorDistance.resize(colorBuffer.size());
	for(int i = 0; i < colorBuffer.size(); i++) {
		Mat camMat = toCv(*colorBuffer[i]);
		vector<Point2f> corners;
		
		bool locationFound = findChessboardCorners(camMat, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH);
		if(locationFound) {			
			cornerSubPix(camMat, corners, cv::Size(4, 4), cv::Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			
			Mat rotation, translation;
			solvePnP(Mat(objectPoints), Mat(corners), cameraMatrix, distCoeffs, rotation, translation);
			
			for(int j = 0; j < corners.size(); j++) {
				colorCenter[i] += corners[j];
			}
			colorCenter[i] *= 1. / corners.size();
			colorDistance[i] = Point3f(translation).z;
		}
	}
	
	cout << "averaging depth images" << endl;
	kinectRoi.resize(kinectBuffer.size());
	kinectDistance.resize(kinectBuffer.size());
	for(int i = 0; i < kinectBuffer.size(); i++) {
		int curKinectTime = kinectTime[i];
		int bestDiff = 0;
		int bestColor = 0;
		for(int j = 0; j < colorTime.size(); j++) {
			int curColorTime = colorTime[j];
			int curDiff = abs(curColorTime - curKinectTime);
			if(j == 0 || curDiff < bestDiff) {
				bestColor = j;
				bestDiff = curDiff;
			}
		}
		
		Point2f roiSize(1000, 1000);
		roiSize *= 20 / colorDistance[bestColor];
		Point2f roiCenter(colorCenter[bestColor] + Point2f(40, 40));
		cv::Rect roiRect(roiCenter - roiSize, cv::Size(roiSize * 2));
		Mat roi = Mat(kinectBuffer[i]->toCv(), roiRect);
		Scalar curMean = mean(roi);
		
		kinectRoi[i] = roiRect;
		
		kinectDistance[i] = curMean[0];
		
		cout << bestColor << "\t" << kinectDistance[i] << "\t" << colorDistance[bestColor] << endl;
		
		kinectBuffer[i]->normalizeToMax();
		kinectBuffer[i]->update();
	}
}

void testApp::update() {
}

void testApp::draw() {
	int offset = ofClamp(mouseX, 0, colorBuffer.size() - 1);
	colorBuffer[offset]->draw(0, 0);
	ofCircle(toOf(colorCenter[offset]), 4);
	
	ofTranslate(640, 0);
	kinectBuffer[offset]->draw(0, 0);
	ofRect(toOf(kinectRoi[offset]));
}
