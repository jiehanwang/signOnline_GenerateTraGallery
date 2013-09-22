#include<iostream>/////////////////////////////////////注意释放内存空间，delete!!!
#include<fstream>
#include <opencv2\opencv.hpp>
#include "globalDefine.h"

using namespace std;
using namespace cv;

void GetPostureScore(HandSegment leftHands,HandSegment rightHands,HandSegment bothHands,double* score);
void Arrayrankingindex(double a[],int length,int index[]);
void ReadData();
