#pragma once
#include <vector>
#include <atlstr.h>
#include <iostream>
#include <fstream>
#include "globalDefine.h"
using namespace std;

class S_CGalleryCreate
{
public:
	S_CGalleryCreate(void);
	~S_CGalleryCreate(void);
private:
	vector<double> ****feature;//[Gallery_num][Posture_num][LRB][25];//feature feature for each key frame.
	int ***ikeyFrameNo;//[Gallery_num][Posture_num][LRB];
	bool testFlag[Posture_num];     //The mask to choose the testing samples.
	int posture_num_used;     //The number of chosen gestures.

public:
	void galleryReadFromDat();
	void setDataMask();
	vector<double> getGalleryValue(int wordIndex, int lrb, int keyFrameIndex, int GalleryIndex);
	void getGalleryValue2(int wordIndex, int lrb, int GalleryIndex, vector<double> ifeature[]);
	int getKeyFrameNo(int galleryIndex, int postureIndex, int lrb);
	void galleryReadFromDat_Combine(CString route);
	bool getTestFlag(int index);
};

