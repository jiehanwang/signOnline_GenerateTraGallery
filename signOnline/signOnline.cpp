// signOnline.cpp : Defines the entry point for the console application.
//Hanjie Wang, 2013-04, VIPL, ICT
#include "stdafx.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <string>
#include <opencv2\opencv.hpp>
#include <atlstr.h>
#include <math.h>
#include <direct.h>
#include "globalDefine.h"
#include "S_Matching.h"
#include "Readvideo.h"
#include <windows.h>	
#include "S_TrajectoryRecognition.h"
#include "windows.h"
using namespace std;
using namespace cv;

	//For objects of classes.
S_CMatching      myMatching;


int _tmain(int argc,char *argv[])
{
		//Definition
	int             i,j,k,sentenceIndex;
	SLR_ST_Skeleton skeletonCurrent;    //The 3 current data.
	Mat             depthCurrent;
	IplImage        *frameCurrent;
	CvPoint3D32f    headPoint;
	clock_t         start, durTime;      //Clock
	CString         s_FileName;
	CString         s_filefolder;
	CString         videoFileName;
	int             framSize;            //The total frame number.
	int             detectNum;
	depthCurrent.create(480,640,CV_16UC1);

		//Read gallery. 3 in total.
		//Posture: Two routes are in the S_GalleryCreate.cpp. 
		//Trajectory: One route is in the S_TrajectoryRecognition.cpp, InitData().
		//To adapt the framework, I quit passing route form main function.
	//myMatching.CreateGallery();
	//for (int se = 1; se<5; se++)
	{
		sentenceIndex = 0;//atoi(argv[1]);//181
		myMatching.getSentenceIndex(sentenceIndex);

		//Read video for testing. That will be captured by Kinect on-line in the future.
		Readvideo      myReadVideo;
		if (sentenceIndex<10)
		{
			videoFileName.Format("D:\\iData\\isolatedWord\\P50\\P50_000%d_1_0_20121002.oni",sentenceIndex);
		}
		else if (sentenceIndex<100)
		{
			videoFileName.Format("D:\\iData\\isolatedWord\\P50\\P50_00%d_1_0_20121002.oni",sentenceIndex);
		}
		else if (sentenceIndex<239)
		{
			videoFileName.Format("D:\\iData\\isolatedWord\\P50\\P50_0%d_1_0_20121002.oni",sentenceIndex);
		}
		else if (sentenceIndex<1000)
		{
			videoFileName.Format("D:\\iData\\isolatedWord\\P50\\P50_0%d_1_0_20121208.oni",sentenceIndex);
		}

		string  s   =   (LPCTSTR)videoFileName;
		myReadVideo.readvideo(s);
		framSize = myReadVideo.vColorData.size();

		bool isLast = true;
		int maxY  = min<int>(myReadVideo.vSkeletonData[0]._2dPoint[7].y, myReadVideo.vSkeletonData[0]._2dPoint[11].y);
		myMatching.initial(maxY);

		//The loop
		for (i=0; i<framSize; i++)
		{

			skeletonCurrent = myReadVideo.vSkeletonData[i];
			depthCurrent    = myReadVideo.vDepthData[i];
			frameCurrent    = myReadVideo.vColorData[i];
			headPoint       = myReadVideo.headPoint3D;
			myMatching.pushSkeletonData(skeletonCurrent,headPoint);
 			int rank[40][topXValue];
 			double score[40][topXValue];
 			myMatching.onlineDetect(skeletonCurrent, depthCurrent, frameCurrent, headPoint, rank, score);
		}

		myMatching.outPutTrajectoryGallery();
		myMatching.release();

	}
	
 	cout<<endl<<"done"<<endl;
//	getchar();
	return 0;
}

