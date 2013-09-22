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
		sentenceIndex = atoi(argv[1]);//181
		myMatching.getSentenceIndex(sentenceIndex);

		//Read video for testing. That will be captured by Kinect on-line in the future.
		Readvideo      myReadVideo;
		if (sentenceIndex<10)
		{
			videoFileName.Format("D:\\iData\\p08_02\\S08_000%d_1_0_20130412.oni",sentenceIndex);
		}
		else if (sentenceIndex<100)
		{
			videoFileName.Format("D:\\iData\\p08_02\\S08_00%d_1_0_20130412.oni",sentenceIndex);
		}
		else if (sentenceIndex<226)
		{
			videoFileName.Format("D:\\iData\\p08_02\\S08_0%d_1_0_20130412.oni",sentenceIndex);
		}
		else if (sentenceIndex<1000)
		{
			//videoFileName.Format("D:\\iData\\p08_02\\S08_0%d_1_0_20130919.oni",sentenceIndex);
			videoFileName.Format("D:\\iData\\ydd-s\\S05_0%d_5_0_20130919.oni",sentenceIndex);
		}

		string  s   =   (LPCTSTR)videoFileName;
		myReadVideo.readvideo(s);
		framSize = myReadVideo.vColorData.size();

		bool isLast = true;
		int maxY  = min<int>(myReadVideo.vSkeletonData[0]._2dPoint[7].y, myReadVideo.vSkeletonData[0]._2dPoint[11].y);
		myMatching.initial(maxY);

		//The loop
		//The hidden code is use for generating galleries.
		//vector<KeyFrameSegment> vKeyFrameAll;

		//ofstream outfile;
		//outfile.open("..\\time.txt",ios::out);
		//start = clock();
// 		ofstream outfile;
// 		outfile.open("..\\test.txt",ios::out | ios::app);
// 		outfile<<"sentenceIndex: -----------------"<<sentenceIndex<<endl;
		ofstream outfile_short;
		outfile_short.open("..\\totalInfo.csv",ios::out | ios::app);
		for (i=0; i<framSize; i++)
		{
			//start = clock();
			//cout<<i<<" ";
			skeletonCurrent = myReadVideo.vSkeletonData[i];
			//depthCurrent    = myReadVideo.vDepthData[i];
			//frameCurrent    = myReadVideo.vColorData[i];
			headPoint       = myReadVideo.headPoint3D;
			myMatching.pushSkeletonData(skeletonCurrent,headPoint);
			//myMatching.doMatch_record();
			//myMatching.doMatch_SG();
// 			int rank[40][topXValue];
// 			double score[40][topXValue];
// 
// 			detectNum = myMatching.onlineDetect(skeletonCurrent, depthCurrent, frameCurrent, headPoint, rank, score);
// 			
// 			for (int i=0; i<detectNum; i++)
// 			{
// 				cout<<"-----------Got it"<<endl;
// 				for (int r=0; r<5; r++)
// 				{
// 					cout<<rank[i][r]<<" ";
// 					outfile<<rank[i][r]<<" ";
// 				}
// 				cout<<endl;
// 				outfile<<endl;
// 			}
// 			if (detectNum > 0)
// 			{
// 				outfile<<endl;
// 			}

			//durTime=clock()-start;
			//outfile<<"FrameID: "<<i<<" One frame cost:	"<<durTime<<endl;
		}
		//myMatching.doMatch_SG_last();
		myMatching.outPutTrajectoryGallery();

		//outfile.close();

// 		int rank[40][topXValue];
// 		double score[40][topXValue];
// 		int mask[40];
// 		detectNum = myMatching.release(rank, score, mask);
// 		cout<<endl<<"The last one or two classes: "<<endl;
// 		for (int i=0; i<detectNum; i++)
// 		{
// 			cout<<"-----------Got it_Final result "<<mask[i]<<" ";
// 			for (int r=0; r<5; r++)
// 			{
// 				cout<<rank[i][r]<<" ";
// 			}
// 			cout<<endl;
// 		}
// 
// 		//durTime=clock()-start;
// 		//double rate = durTime/framSize;
// 		//cout<<"One frame cost:	"<<durTime<<"/"<<framSize<<" rate "<<rate<<endl;
// 
// 		int recall1 = 0;
// 		int recall5 = 0;
// 		int ground = 0;
// 		int preci = 0;
// 
// 		ground = myMatching.groundTruth[sentenceIndex].size();
// 		preci = detectNum;
// 
// 		for (int i=0; i<detectNum; i++)
// 		{
// 			if (mask[i] == 0)
// 			{
// 				preci--;
// 			}
// 		}
// 
// 		for (int i=0; i<detectNum; i++)
// 		{
// 			for (int j=0; j<ground; j++)
// 			{
// 				if (mask[i] == 1
// 					&& rank[i][0] == myMatching.groundTruth[sentenceIndex][j])
// 				{
// 					recall1++;
// 				}
// 				for (int k=0; k<topXValue; k++)
// 				{
// 					if (mask[i] == 1
// 						&& rank[i][k] == myMatching.groundTruth[sentenceIndex][j])
// 					{
// 						recall5++;
// 						break;
// 					}
// 				}
// 				
// 			}
// // 			if (mask[i] == 1)
// // 			{
// // 				recall1++;
// // 			}
// // 			if (mask[i]>0)
// // 			{
// // 				recall5++;
// // 			}
// 		}
// 		outfile_short<<sentenceIndex<<","<<recall1<<","<<(recall5>ground?ground:recall5)<<","<<ground<<","<<preci<<endl;
// 		outfile_short.close();
	}
	
 	cout<<endl<<"done"<<endl;
//	getchar();
	return 0;
}

