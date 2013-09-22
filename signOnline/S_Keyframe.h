#pragma once
#include <opencv2\opencv.hpp>
#include <vector>
#include "globalDefine.h"
#include "typedefs.h"
#include "connexe.h"
#include <direct.h>
#include <fstream>


#define consistThres	5
#define velThres	20
//#define heightThres	370

using namespace std;
using namespace cv;

struct FrameSegment
{
	bool both;
	bool left;
	bool right;

	int bothCoor[2];
	int leftCoor[2];
	int rightCoor[2];

	IplImage* bothImage;
	IplImage* leftImage;
	IplImage* rightImage;
};

const int bufferSize=50;

class S_Keyframe
{
	
	int leftBegin;
	int leftEnd;
	int rightBegin;
	int rightEnd;
	bool* leftFlag;
	bool* rightFlag;
	bool* bothFlag;
	int rightLastFrame;
	int leftLastFrame;
	int bothLastFrame;
	
	ColorModel skinColorModel;
	CRITICAL_SECTION csFrameData;
	CRITICAL_SECTION csFragmentData;
	CRITICAL_SECTION csMessageData;
	int bufferBeginPointer;
	int bufferEndPointer;
	
	vector<CvPoint3D32f> rightHands;
	vector<CvPoint3D32f> leftHands; 
	//vector<SLR_ST_Skeleton> vSkeletonData;
	//vector<Mat> vDepthData;
	//vector<IplImage*> vColorData;
	SLR_ST_Skeleton vSkeletonData[bufferSize];
	Mat vDepthData[bufferSize];
	IplImage* vColorData[bufferSize];
	Rect initFaceRect;
	short initMinDepth;
	double* rightVel;
	double* leftVel;
	double meanRightVel;
	double meanLeftVel;
	vector<KeyFrameUnit> LeftKeyFrames;
	vector<KeyFrameUnit> RightKeyFrames;
	double calVel(vector<CvPoint3D32f> Hands,double* vel,int &beginFrame,int &endFrame,int handIdx);
	double calDist(CvPoint3D32f p1,CvPoint3D32f p2);
	double calDist(CvPoint2D32f p1,CvPoint2D32f p2);
	void keyframesJudge(double* vel,int beginFrame,int endFrame,double meanVel,vector<KeyFrameUnit> &KeyFrames);
	void mergeKeyframes(vector<KeyFrameUnit> &KeyFrames);
	void getFaceRect(int frameID);
	void getMinDepth(int frameID,Rect imageRect);
	void depthRestrict(int frameID,IplImage* grayImage);
	void skinColorProcess(IplImage* colorImage);
	//获取连通区域坐标，参数（图片，连通区域个数，联通区域中心坐标，连通区域矩形坐标，连通区域像素点阈值）
	void getConnexeCenterBox(IplImage* image, int& nMaxRect, int* theCent, int* theBox, int& nThreshold);
	void getHandImage(int frameID,IplImage* binaryImage, int& nMaxRect, int* theCent, int* theBox,IplImage* grayImage);
	void getHandImage(int frameID,IplImage* binaryImage, int& nMaxRect, int* theCent, int* theBox,IplImage* grayImage,FrameSegment& fs);
	
	void releaseHandStruct(HandSegment &handStruct);
	void saveHandImage(IplImage* handImage, int frameID, HANDTYPE hType, Point coor);
	void saveHandImage(IplImage* handImage, int frameID, HANDTYPE hType, Point coor,FrameSegment& fs);
	IplImage* getConnextImage(IplImage* grayImage,IplImage* binaryImage,int* theBox,int connIdx,int frameID);
	bool isSkinColor(IplImage* colorImage,int i,int j);
	bool isSkinColorModel(IplImage* colorImage,int i,int j);
	void getSkinColorModel(IplImage* faceImage);
	bool mergeFragment(KeyFrameSegment &Fragment);
	double coverProp(Rect leftRect,Rect rightRect, Rect bothRect);
	void getCurrStatus(double* vel,int &beginFrame, int endFrame ,bool &currStatus,bool* flag);
	void CreateNewFragment(KeyFrameSegment &Fragment);
	void putInFragment(KeyFrameSegment &Fragment,FrameSegment &fs ,int frameID);
	bool isInFragment(KeyFrameSegment Fragment,FrameSegment fs);
	void putInSegmentVector(KeyFrameSegment &Fragment);
	void addInLastFragment(KeyFrameSegment &Fragment);
	void mergeFragment(KeyFrameSegment& firstFragment,KeyFrameSegment& secondFragment);
	IplImage* mergeHands(IplImage* leftHand,Point leftPoint,IplImage* rightHand,Point rightPoint,Point& bothPoint);
	bool isSameFragment(KeyFrameSegment Fragment1,KeyFrameSegment Fragment2,int index);
	double Img_distance(IplImage *dst1,IplImage *dst2);
	double coverProp(IplImage* image1,IplImage* image2);
public:
	int myHeightThres;
	vector<KeyFrameSegment> v_kfSegment;
	KeyFrameSegment kfSegmentBuffer[bufferSize];
	int framesNum;
	int processedFrameNum;
	bool getDataOver;
	bool segmentOver;
	void KeyframeExtraction();
	vector<KeyFrameUnit> getRightKeyFrames();
	vector<KeyFrameUnit> getLeftKeyFrames();
	S_Keyframe(void);
	S_Keyframe(vector<SLR_ST_Skeleton> SkeletonData,vector<Mat> DepthData, vector<IplImage*> ColorData);
	~S_Keyframe(void);
	void handSegment(int frameID);
	void getFrameSegment(int frameID,FrameSegment& fs);
	void saveHandStruct(string filePath);
	void saveHandStruct(string filePath,HandSegment right,HandSegment left,HandSegment both);
	void saveKeyFrameSegment(string filePath);
	void saveKeyFrameSegment(string filePath,vector<KeyFrameSegment> vKeyFrame);
	void pushImageData(SLR_ST_Skeleton SkeletonData,Mat DepthData, IplImage* ColorData);
	void KeyframeExtractionOnline();
	void releaseMemory();
	void releaseFragment(KeyFrameSegment& fs);
	void putInBuffer(KeyFrameSegment& fs);
	KeyFrameSegment getFragment();
	bool isThereFragment();
	bool processOver();
	void setGetDataOver(bool dataOver);
	bool getGetDataOver();
	bool getSegmentOver();
	void deleteHandStruct(HandSegment handStruct);
	void setHeightThres(int height);
};