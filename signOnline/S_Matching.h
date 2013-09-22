#pragma once
#include <vector>
#include <atlstr.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <opencv2\opencv.hpp>
#include "globalDefine.h"
#include "S_Feature.h"
#include "S_Postures.h"
#include "S_GalleryCreate.h"
#include "S_TrajectoryRecognition.h"
#include <algorithm>
#include "S_Keyframe.h"
#include "winnt.h"
#include "shellapi.h"
using namespace std;
using namespace cv;

struct RECVPARAM
{
	S_Keyframe   douKeyframe;
	int a;
};

typedef struct scoreAndIndex
{
	double score;
	int index;
}scoreAndIndex;

struct streamMatch
{
	double weight[MaxKeyFrameNumber][Posture_num];
	scoreAndIndex weightSort[MaxKeyFrameNumber][Posture_num];
	int maxLengh;
	int frameIndex_start;
	int frameIndex_end;
	int *topX;
	double posScoreDetail[MaxKeyFrameNumber][Gallery_num][Posture_num];
	double posScore[MaxKeyFrameNumber][Posture_num];
	double traScore[MaxKeyFrameNumber][Posture_num];
	int    pairSum[MaxKeyFrameNumber][Posture_num];
	int    postureMatchedIndex[MaxKeyFrameNumber][Gallery_num][Posture_num][2]; //0: start. 1: end. 
	                                                                            //It will be used if trajectory is extracted by posture. 
	int    trajectoryMatchedIndex[MaxKeyFrameNumber][Gallery_num][Posture_num][2];
};
struct traMatch 
{
	int startFrameID;
	int endFrameID;
	int startFrameID_match;
	int endFrameID_match;
	double score[Posture_num];
	float bestScore;
	bool isMatch;
	int matchedClass;
	int rankIndex[5];
	double rankScore[5];

};

struct Pair
{
	int man;
	int woman;
	double love;
	int married;  //1: married. 0: unmarried. 2: may be
};
struct topClass
{
	int classIndex;
	int hit;   //1: hit; 0: lost
	double score;
	double scorePre;
	double scorePrePre;
};

struct currentRank
{
	int index[topXValue];
	double score[topXValue];
	int flag[topXValue];  //-1: invalid. 1: valid
};
struct manuallySeg
{
	int sentenceID;
	int wordNum;
	int beginFrame[20];
	int endFrame[20];

};

struct traGallery
{
	int sentenceID;
	int wordNum;
	CvPoint3D32f         head;
	vector<CvPoint3D32f> *leftHand;
	vector<CvPoint3D32f> *rightHand;
};

class S_CMatching
{
public:
	S_CMatching(void);
	~S_CMatching(void);
//private:
	int sentenceIndex;
#ifdef saveFiles
	CString  s_FileName;
	CString  s_filefolder;

	ofstream outfile;
	ofstream outfile_topX;
	ofstream outfile_short;
	ofstream outfile_all;
	ofstream outfile_languageModel;

	vector<int>     groundTruth[300];    //At most 300 words.
	int             sentenceNumber;
	int             sentenceLength[250];
#endif
#ifdef generateGalleryKey
	vector<KeyFrameSegment> vKeyFrameAll;
#endif
#ifdef svmTrainKey
	vector<KeyFrameSegment> vKeyFrameAllSVM;
#endif
public:
	S_CGalleryCreate M_myGallery; //Read gallery in.
	S_CFeature       M_myFeature;      //For feature computing and comparing.
	S_CPostures      M_myPostures;
	//S_Trajectory     M_myTrajectory;

	vector<double> **M_keyFeatureStream;//[MaxKeyFrameNumber][LRB]; //The posture stream in memory.
	//double ***M_keyFeatureStream;
	int M_keyFrameNumber;     //The posture number in stream.
	int M_firstKeyFrameIndex; //The first frame. It usually in the position 0. If a sign is 
	                          //detected, it will move forward.
	streamMatch *M_myStreamMatch;//[MaxKeyFrameNumber];  //The matching result

		//Global variants used in doMatch makes it faster.
 	vector<double> featureToCom[25];
 	vector<double> **featureToBeCom;//[LRB][MaxKeyFrameNumber];  

	bool isDetectSign;  //Flag of sign detection.

	vector<int>     wordClassResult;
	vector<int>     *wordClassResultPotential;//[topXValue];
	vector<double>  *wordClassScorePotential;
	vector<int>     *wordClassResultPotentialLengh;

	topClass *myTopClass;//[topXValue];

	vector<SLR_ST_Skeleton> M_SkeletonData;
	vector<CvPoint3D32f> M_HeadPoint3D;
	//vector<int> keyFrameStart;
	//vector<int> keyFrameEnd;
	vector<traMatch> M_traMatchResult;
	int previousEndFrameIndex;
	int traDetectNo;
	int start_tra_pointer;

	manuallySeg myManuallySeg[sentenceNum];  //Used for creating gallery in successive sign sentences.

	RECVPARAM       *pRecvParam;        //Handle and parameter for dual-thread.
	DWORD           thredID;
	HANDLE          hThread;
	
	currentRank myCurrentRank[3];   //0,current. 1, pre. 2, prepre
	int alarm;

		//The probability of priors of words and transitions between words.
	float *firstProbability;//[Posture_num];
	float *priorProbability;//[Posture_num];
	float **transProbability;//[Posture_num][Posture_num];
	float **transProbability_sec;//[Posture_num][Posture_num];
	bool isFisrtWord;
	bool isFirstAvilable;

	traGallery M_traGallery[sentenceNum];
	int gapFrame;
	vector<scoreAndIndex> competeSen;
	int sentenceMask[sentenceNum];
	vector<int> *rankFlush;
	vector<int> *rankFlushSentece;
	vector<int>  rankFlushMask;
	bool isFirstReadySG;

	double Posture_Distance(vector<double> xx[], int x, vector<double> yy[], int y, int *pairSum);
	int doMatch();
	void CreateGallery();
	void matchUpdate(vector<double> keyFeatureStream, int frameIndex_start, int frameIndex_end);
	double Posture_Distance_new(vector<double> probe[], int probeNo, vector<double> gallery[], 
		int galleryNo, int* pairSum, int *firstLady, int *lastLady);
	double Posture_Distance_new_lastProbe(vector<double> probe[], int probeNo,
		vector<double> gallery[], int galleryNo, int* pairSum, int *firstLady, int *lastLady);
	int release(int rank[][topXValue], double score[][topXValue], int mask[]);
	void release();
	int getWordclassSize(void);
	int getWordclassResult(int index);
	int trajectoryInital(void);
	void pushSkeletonData(SLR_ST_Skeleton skeletonData, CvPoint3D32f headPoint3D);
	
	int signWordDetect();
	int signWordDetect_forLast();
	int signWordDetect_forLast_delay2();
	int signWordDetect_forTraOnly(ofstream &outfile, ofstream &outfile_short);
	int signWordDetect_delay2();
	int signWordDetect_delay2_0616(ofstream &outfile, ofstream &outfile_short);
	void topXclass();
	
	void doMatch_tra(int sentenceID, int skeleton_start, int skeleton_end);
	static bool comp(scoreAndIndex dis_1, scoreAndIndex dis_2);
	void manuallySegTest(int sentenceID);
	void readstr(FILE *f,char *string);
	void readstrLong(FILE *f,char *string);
	float calSpeed(vector<SLR_ST_Skeleton> skeletonData_temp);
	int signWordDetect_0614();
	int signWordDetect_0614_forLast(ofstream &outfile, ofstream &outfile_short, ofstream &outfile_topX);
	int signWordDetect_0620();
	int signWordDetect_0620_forLast();

	void initial(int maxY);
	static DWORD WINAPI RecvProc(LPVOID lpParameter);
	int onlineDetect(SLR_ST_Skeleton skeletonCurrent, Mat depthCurrent, IplImage* frameCurrent,  CvPoint3D32f headPoint,
		int rank[][topXValue], double score[][topXValue]);
	void getSentenceIndex(int isentenceIndex);
	void readInGroundTruth(void);
	void outputForLanguageModel(void);
	int generateGallery(void);
	bool MyDeleteFile(char *lpszPath);
	void forSVMTraining(void);
	bool isAllFail(int rank[]);
	void releaseAfterDetect(void);
	void readInProbability(void);
	void readInProbability_sec(void);
	bool probabilityCheck(currentRank myCurrentRank, int length);
	bool probabilityCheck_oldFramwork(topClass myTopClass[], int length, int style);
	currentRank rankReRank(streamMatch src, int style);
	void outPutTrajectoryGallery(void);
	int doMatch_SG(int rank[][topXValue], double score[][topXValue], int Nindex);
	int doMatch_SG_last(void);
	void creatTraGallery_SG(void);
	float traLength(vector<CvPoint3D32f> leftHand, vector<CvPoint3D32f> rightHand);
	int doMatch_record(void);
	static bool comp2(scoreAndIndex dis_1, scoreAndIndex dis_2);
	void readinSentenceMask(void);
};

