#include "PostureRecognition.h"
#include "S_Keyframe.h"
#include <time.h>
//#include "WriteGallery.h"

using namespace std;
using namespace cv;

class Readvideo
{
public:
	HANDLE hKeyFrameThread;
	DWORD keyFrameThreadID;	
	vector<LONGLONG> vDepthFrame;
	vector<LONGLONG> vColorFrame;
	vector<LONGLONG> vSkeletonFrame;
	CvPoint headPoint;				//ͷ���������꣬��ά
	CvPoint3D32f headPoint3D;		//ͷ���������꣬��ά
	vector<SLR_ST_Skeleton> vSkeletonData;		//һ������ʻ�ĹǼܵ㼯��
	vector<Mat> vDepthData;
	vector<IplImage*> vColorData;
	bool bBegin;					 //����ʻ��Ƿ�ʼ  - for video
	bool bEnd;						 //����ʻ��Ƿ����   - for video
	S_Keyframe keyFrames;
public:
	static unsigned long WINAPI keyFrameSegmentThread(LPVOID pParam);
	void keyFrameSegment();
	void readvideo(string filePath);
	bool readColorFrame(string filename);
	bool readDepthFrame(string filename);
	bool readSkeletonFrame(string filename);
	Readvideo(void);
	~Readvideo(void);
};

