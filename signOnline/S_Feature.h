#pragma once
#include <vector>
#include <opencv2\opencv.hpp>
using namespace std;
using namespace cv;

#define SIZE 64

class S_CFeature
{
public:
	S_CFeature(void);
	~S_CFeature(void);
	bool GetHOGHistogram_Patch(IplImage* img, vector<double> &hog_hist);
	double Histogram(vector<double> vec1, vector<double> vec2);
};

