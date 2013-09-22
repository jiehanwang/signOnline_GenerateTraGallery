#pragma once
#include <vector>
#include <atlstr.h>
#include <iostream>
#include <fstream>
#include <atlstr.h>
#include <string>
#include <opencv2\opencv.hpp>
#include "globalDefine.h"
#include "S_Feature.h"
#include <direct.h>
using namespace std;
using namespace cv;

struct Std
{
	IplImage * pic_route;//Pointer to the key frame sequence.
	double std_distance; //Distance to the mean image.
};

class S_CPostures
{
public:
	S_CPostures(void);
	~S_CPostures(void);
public:
	vector<double> keyFeatureStream;      //Feature computed from keyFrames[LRB]
	IplImage*      keyFrames;             //The best key frame.
	S_CFeature     P_myFeature;          //The object of CFeature class.
	vector<Std>    choose_pic;           //Store the best picture.
	
	IplImage* leftImage;
	IplImage* rightImage;
	IplImage* bothImage;

public:
		//Select the best key frame from key frame candidates.
	void keyFrameSelect(IplImage** rightKeyFrames, IplImage** leftKeyFrames, IplImage** bothKeyFrames,
		int rightKeyNum, int leftKeyNum, int bothKeyNum,int frameIndexStart, int frameIndexEnd, int folderIndex);
	IplImage* keyFrameSelect_sub(IplImage** KeyFrames, int KeyNum);
		//Feature calculation for the best key frame
	void featureCalPosture(void);
		//
	double img_distance(IplImage* dst1, IplImage* dst2);
	IplImage* Resize(IplImage* _img);
	bool cmp(Std pp, Std qq);
	int bestFrame(vector<Std> choose_pic);
	
	void saveFrames(int folderIndex, IplImage* image, int lrb, int frameIndexStart, int frameIndexEnd);
};

