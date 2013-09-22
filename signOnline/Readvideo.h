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
	CvPoint headPoint;				//头部中心坐标，二维
	CvPoint3D32f headPoint3D;		//头部中心坐标，三维
	vector<SLR_ST_Skeleton> vSkeletonData;		//一个手语词汇的骨架点集合
	vector<Mat> vDepthData;
	vector<IplImage*> vColorData;
	bool bBegin;					 //手语词汇是否开始  - for video
	bool bEnd;						 //手语词汇是否结束   - for video
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

