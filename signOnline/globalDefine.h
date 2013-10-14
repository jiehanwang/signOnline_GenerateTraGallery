#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <hash_map>
//#include <cv.h>
#include <opencv2\opencv.hpp>

#define ZHIHAOPC
//#define CURVE2D
#define CURVE3D

using namespace std;

const int K = 15; ///K for KNN

const int num=5;
const int dim=3;//3D
const int WordsNum=370;
const int probenum=0;
const double PI= 3.1415;
const double eps=0.0001;
const int F=1500;
const int rsnum=1000;

struct HandSegment
{
	int keyframe_num;
	int* keyframe_no;
	IplImage*** keyframe_pic;
	int** keyframe_ID;
	int** keyframe_coor;
};

struct KeyFrameSegment
{
	int BeginFrameID;		//�ؼ�Ƭ�ο�ʼ֡��
	int EndFrameID;			//�ؼ�Ƭ��jie'shu֡��

	int BothLabel;			//0-��	1-��
	int LeftLabel;			//0-��	1-��
	int RightLabel;			//0-��	1-��
	
	int BothNum;			//˫��ͼƬ��Ŀ
	int LeftNum;			//����ͼƬ��Ŀ
	int RightNum;			//����ͼƬ��Ŀ

	int* BothID;			//ÿ��˫�ַָ�ͼƬ��֡��
	int* LeftID;			//ÿ�����ַָ�ͼƬ��֡��
	int* RightID;			//ÿ�����ַָ�ͼƬ��֡��

	int* BothCoor;			//ÿ��˫�ַָ�ͼƬ���Ͻǵ�����
	int* LeftCoor;			//ÿ�����ַָ�ͼƬ���Ͻǵ�����
	int* RightCoor;			//ÿ�����ַָ�ͼƬ���Ͻǵ�����

	IplImage** BothImages;	//˫��ͼƬ
	IplImage** LeftImages;	//˫��ͼƬ
	IplImage** RightImages;	//˫��ͼƬ

	int* BothDepth;			//ÿ��˫�ַָ�ͼƬ���
	int* LeftDepth;			//ÿ�����ַָ�ͼƬ���
	int* RightDepth;		//ÿ�����ַָ�ͼƬ���

};

enum HANDTYPE
{
	LEFT,
	RIGHT,
	BOTH,
};
//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct Result
/// @brief recognition result struct
//////////////////////////////////////////////////////////////////////////
struct Result
{
	double posturePro[5];  ///< probability of posture recognition result δ��
	int postureWord[5];    ///< index of posture recognition result  δ��
	double curvePro[5];    ///< probability of curve recognition result  δ��
	int curveWord[5];      ///< index of curve recognition result  δ��
	double fusionPro[5];    ///< probability of curve recognition result
	int fusionWord[5];      ///< index of curve recognition result
};

//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct Score
/// @brief word index and score, used to sort by score
//////////////////////////////////////////////////////////////////////////
struct Score
{
	int index;    ///< index of recognition result
	double score; ///< probability of recognition result

	bool operator < (const Score& s) const
	{
		return score < s.score;
	}
	bool operator > (const Score& s) const
	{
		return score > s.score;
	}
};

//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct Posture
/// @brief score left and right hand posture, hand and wrist point
//////////////////////////////////////////////////////////////////////////
struct Posture
{
	IplImage *leftHandImg;    ///< left hand image
	IplImage *rightHandImg;   ///< right hand image
	CvPoint leftHandPt;       ///< left hand point
	CvPoint leftWristPt;      ///< left wrist point
	CvPoint rightHandPt;      ///< right hand point
	CvPoint rightWristPt;     ///< right wrist point

	Posture():leftHandImg(NULL),rightHandImg(NULL) {};
};

//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct ColorModel
/// @brief skin color model, mean and variance of cr cb
//////////////////////////////////////////////////////////////////////////
struct ColorModel
{
	double mean_cr;     ///< mean of cr
	double mean_cb;     ///< mean of cb
	double d_cr;        ///< variance of cr
	double d_cb;        ///< variance of cb
	ColorModel():mean_cr(0),mean_cb(0),d_cr(0),d_cb(0){};
};

//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct _Vector2i
/// @brief skeleton data transformed data
//////////////////////////////////////////////////////////////////////////
struct _Vector2i
{
	int x;
	int y;
};

//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct SLR_ST_Skeleton
/// @brief skeleton data  real data
//////////////////////////////////////////////////////////////////////////
struct _Vector4f
{
	float x;
	float y;
	float z;
	float w;
};

//////////////////////////////////////////////////////////////////////////
/// @author xu zhihao
/// @struct SLR_ST_Skeleton
/// @brief skeleton data  real data and transformed data
//////////////////////////////////////////////////////////////////////////
struct SLR_ST_Skeleton
{
	_Vector4f _3dPoint[20];    ///< real point
	_Vector2i _2dPoint[20];    ///< pix in color image
}; 

struct Fragment{
	int fragmentCount;
	map<int, float> classWeightHit;
	multimap<float, int> weightHitClass;
	Fragment() : fragmentCount(0) {};
};
struct SignWordDescriptor{
	int id;
	vector<int> hand[2];
};

enum STAT_TYPE{
	E_AVER,   
	E_MIN,
	E_MEDIUM
};

struct KeyFrameUnit
{
    int StartFrame;
    int EndFrame;
};

typedef multimap<double, int, greater<double> > RecogValueIDPair;

#define SIZE 64
//#define Gallery_num 4            //Gallery����
//#define Posture_num 370       //����ʻ����


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Add by Hanjie Wang
#define LRB 3    //Left, right, both
#define FusedLRB 1
#define Gallery_num 5
#define Posture_num 370
#define feature_dimension 1764
#define MaxKeyFrameNumber 1
#define delay2
#define ThreForOnline 0.5
#define MarryThre 0.0
#define topXValue   5   //The allowed top class index to be considered in doMatche.
#define keyFrameGap 10
#define top2Tore   0.08   //used in function "doMatch" in Matching class. 
#define pairSumPenalty 0.02
#define finalResultThre 0//0.58 is for posture only.
#define allowedTraFrameSize 30
#define consistThres	5
#define MaxKeyPostureNum 10
#define Theda  0.0000001
#define maxTraDiff 10
#define lamda_diff 0.07
#define transitionFrameThre 200//0.02
#define iexp 2.7183  //for the trajectory factor
#define lamda 0.3   //for the trajectory factor. The larger, the more important of trajectory.
#define sentenceNum 226

//#define onLineProcess
#define saveFiles              //To record some information.
#define groundTruthKey         //Need "sentence.txt"
//#define generateGalleryKey  //Open the key. It will close the sign detect module.
#define trajectoryKey
//#define svmTrainKey        //Open the key. The code should run in the .bat mood.
#define useNewJudge
//#define laguageModelByWang  //It is only active d in the NewJudge.
//#define useSegmentation
