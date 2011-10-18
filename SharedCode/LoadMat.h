
#pragma once
#include "ofMain.h"
#include "ofxCv.h"
using namespace cv;
using namespace ofxCv;

static  Mat loadRaw(string filename, unsigned int width, unsigned int height) {
	unsigned int n = width * height;
	ofFile file(filename, ofFile::ReadOnly, true);
	ofBuffer buffer = file.readToBuffer();
	Mat mat(height, width, CV_32FC1);
	memcpy(mat.ptr(), buffer.getBinaryBuffer(), n * sizeof(float));
	return mat;
}
