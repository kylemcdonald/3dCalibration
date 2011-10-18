#include "testApp.h"

using namespace ofxCv;
using namespace cv;

Mat loadRaw(string filename, unsigned int width, unsigned int height) {
	unsigned int n = width * height;
	ofFile file(filename, ofFile::ReadOnly, true);
	ofBuffer buffer = file.readToBuffer();
	Mat mat(height, width, CV_32FC1);
	memcpy(mat.ptr(), buffer.getBinaryBuffer(), n * sizeof(float));
	return mat;
}

void testApp::setup() {
	int width = 640;
	int height = 480;
	Mat raw = loadRaw("depth.raw", width, height);
	img.allocate(width, height, OF_IMAGE_GRAYSCALE);
	float maxDistance = 4000;
	for(int y = 0; y < height; y++) {
		for(int x = 0; x < width; x++) {
			img.getPixels()[y * width + x] = 255 * raw.at<float>(y, x) / maxDistance;
		}
	}
	img.update();
}

void testApp::update() {
}

void testApp::draw() {
	img.draw(0, 0);
}
