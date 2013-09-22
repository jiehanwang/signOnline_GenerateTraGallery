#include "StdAfx.h"
#include "Readvideo.h"


Readvideo::Readvideo(void)
{
}


Readvideo::~Readvideo(void)
{
	vSkeletonData.clear();

	for(int i=0;i<vDepthData.size();i++)
		vDepthData[i].release();
	vDepthData.clear();

	for(int i=0;i<vColorData.size();i++)
		cvReleaseImage(&(vColorData[i]));
	vColorData.clear();
}

void Readvideo::readvideo(string filePath)
{
	clock_t start, durTime;
	start=clock();
	if( !( readColorFrame(filePath) && readDepthFrame(filePath) && readSkeletonFrame(filePath) ))
	{
		return;
	}	

	if( !( vSkeletonFrame.size() == vColorFrame.size() && vSkeletonFrame.size() == vDepthFrame.size() ))
	{
		return;
	}

	string str = filePath.substr(filePath.length()-25,21);
	cout<<str<<endl;
	const char* savefilename=str.c_str();
	double disLeft, disRight;
	CvPoint lPoint1,lPoint2;
	CvPoint rPoint1,rPoint2;
	CvPoint lwPoint, rwPoint;

	//��ȡ��ɫ����Ⱥ͹Ǽ�����
	const char* filePathChar=filePath.c_str();
	char filePathName[100];
	strcpy(filePathName,filePathChar);
	strcat(filePathName,"\\color.avi");
	CvCapture *capture = cvCreateFileCapture(filePathName);
	if( NULL == capture )
		return;

	int x,y;
	ifstream depthFileReader;
	depthFileReader.open(filePath+"\\depth.dat",ios::binary);
	if(depthFileReader == NULL)
		return;

	ifstream skeletonFileReader;
	skeletonFileReader.open(filePath+"\\skeleton.dat",ios::binary);
	if(skeletonFileReader == NULL)
		return;

	//read first color depth and skeleton data
	IplImage *frame = cvQueryFrame(capture);
	vColorData.push_back(cvCloneImage(frame));

	Mat depthMat;
	depthMat.create(480,640,CV_16UC1);
	ushort *depthData = new ushort[640*480];
	depthFileReader.read((char*)depthData,640*480*sizeof(ushort));
	if( !depthFileReader.fail() )
	{
		for(y=0; y<480; y++)
		{
			for(x=0; x<640; x++)
			{
				depthMat.at<ushort>(y,x) = depthData[y*640+x];
			}
		}
	}	
	delete [] depthData;
	vDepthData.push_back(depthMat.clone());

	SLR_ST_Skeleton mSkeleton;
	skeletonFileReader.read((char*)&mSkeleton,sizeof(mSkeleton));
	

	SLR_ST_Skeleton sLast = mSkeleton;
	SLR_ST_Skeleton sCurrent;
	int index = 0;

	//��ʼ��ͷ���������꣬��ά����ά
	headPoint = cvPoint(mSkeleton._2dPoint[3].x,mSkeleton._2dPoint[3].y);
	headPoint3D = cvPoint3D32f(mSkeleton._3dPoint[3].x,mSkeleton._3dPoint[3].y,mSkeleton._3dPoint[3].z);


	//��ʼ�ֲ���͵㣬����֮���ж���̧��һ���߶ȿ�ʼһ������ʻ�
	int maxY  = min<int>(mSkeleton._2dPoint[7].y,mSkeleton._2dPoint[11].y);
	//keyFrames.setHeightThres(maxY-20);
	int handDepth = max<int>((int)(mSkeleton._3dPoint[7].z*1000+0.5),(int)(mSkeleton._3dPoint[11].z*1000+0.5));
	vSkeletonData.clear();
	vSkeletonData.push_back(mSkeleton);
	
	int stillCount = 0;
	bBegin = false;
	bEnd = false;
	//////////////////////��ʼ�ֲ��ָ��߳�///////////////////////////
//	keyFrameSegment();
	while(index < vSkeletonFrame.size()-1)
	{
		//��ȡ��һ֡�Ĳ�ɫ����Ⱥ͹Ǽ�����
		index++;
		frame = cvQueryFrame(capture);
		vColorData.push_back(cvCloneImage(frame));
		
		depthData = new ushort[640*480];
		depthFileReader.read((char*)depthData,640*480*sizeof(ushort));
		if( !depthFileReader.fail() )
		{
			for(y=0; y<480; y++)
			{
				for(x=0; x<640; x++)
				{
					depthMat.at<ushort>(y,x) = depthData[y*640+x];
				}
			}
		}	
		delete [] depthData;
		vDepthData.push_back(depthMat.clone());
		skeletonFileReader.read((char*)&sCurrent,sizeof(sCurrent));


		lPoint1 = cvPoint(sLast._2dPoint[7].x,sLast._2dPoint[7].y);
		lPoint2 = cvPoint(sCurrent._2dPoint[7].x,sCurrent._2dPoint[7].y);
		rPoint1 = cvPoint(sLast._2dPoint[11].x,sLast._2dPoint[11].y);
		rPoint2 = cvPoint(sCurrent._2dPoint[11].x,sCurrent._2dPoint[11].y);
		lwPoint = cvPoint(sCurrent._2dPoint[6].x,sCurrent._2dPoint[6].y);
		rwPoint = cvPoint(sCurrent._2dPoint[10].x,sCurrent._2dPoint[10].y);
		int lDepth = (int)(sCurrent._3dPoint[7].z*1000+0.5);
		int rDepth = (int)(sCurrent._3dPoint[11].z*1000+0.5);

		//�ж�һ������ʱ���Ƿ�ʼ������һ���ֲ�̧��ĸ߶�����ʼ�߶���ֵС��30ʱ
		if( !bBegin )
		{
			if(lPoint2.y < maxY - 30 || rPoint2.y < maxY - 30)
			{
				bBegin = true;
				bEnd = false;			
			}
		}

		CvPoint outLeftPoint,outRightPoint;
		//if begin, judge end time and do hand sgementation
		vSkeletonData.push_back(sCurrent);
		sLast = sCurrent;
		//////////////////////����һ֡����///////////////////////////
//		keyFrames.pushImageData(sCurrent,depthMat,frame);
	}

	
	bBegin = false;
	bEnd = true;

	durTime=clock()-start;
	cout<<"Read Data Time:	"<<durTime<<endl;

	start=clock();
// 	//////////////////////�������ݽ�����־///////////////////////////
// 	keyFrames.getDataOver=true;
// 
// 
// 	//////////////////////ѭ����ȡ����///////////////////////////
// 	while(keyFrames.segmentOver==false || keyFrames.isThereFragment())
// 	{
// 		//////////////////////���пɶ�����///////////////////////////
// 		if( keyFrames.isThereFragment() )
// 		{
// 			//////////////////////��ȡ����///////////////////////////
// 			KeyFrameSegment tempSegment=keyFrames.getFragment();
// 			
// 			//////////////////////�����ݽ��в���///////////////////////////
// 
// 
// 			//////////////////////����ʹ������ͷ��ڴ�///////////////////////////
// 			keyFrames.releaseFragment(tempSegment);
// 		}
// 		Sleep(10);
// 	}

	//////////////////////ȫ��������ͷ��ڴ�///////////////////////////
//	keyFrames.releaseMemory();
//	vSkeletonData.clear();


// 	for(int i=0;i<vDepthData.size();i++)
// 		vDepthData[i].release();
// 	vDepthData.clear();

// 	for(int i=0;i<vColorData.size();i++)
// 		cvReleaseImage(&(vColorData[i]));
// 	vColorData.clear();

// 	vColorFrame.clear();
// 	vDepthFrame.clear();
// 	vSkeletonFrame.clear();
// 	vSkeletonData.clear();

	depthFileReader.close();
	skeletonFileReader.close();
	cvReleaseCapture(&capture);
}

bool Readvideo::readColorFrame(string filename)
{
	vColorFrame.clear();
	ifstream colorFrameReader;
	const char* filePathChar=filename.c_str();
	char filePathName[100];
	strcpy(filePathName,filePathChar);
	strcat(filePathName,"\\color.frame");
	colorFrameReader.open(filePathName,ios::binary);
	if(colorFrameReader == NULL)
		return false;
	while( !colorFrameReader.eof() )
	{
		LONGLONG colorframeno;
		colorFrameReader.read((char*)&colorframeno,sizeof(LONGLONG));
		if( colorFrameReader.fail() )
			break;
		vColorFrame.push_back(colorframeno);
	}
	colorFrameReader.close();
	return true;
}

bool Readvideo::readDepthFrame(string filename)
{
	vDepthFrame.clear();
	ifstream depthFrameReader;
	const char* filePathChar=filename.c_str();
	char filePathName[100];
	strcpy(filePathName,filePathChar);
	strcat(filePathName,"\\depth.frame");
	depthFrameReader.open(filePathName,ios::binary);
	if(depthFrameReader == NULL)
		return false;
	while( !depthFrameReader.eof() )
	{
		LONGLONG depthframeno;
		depthFrameReader.read((char*)&depthframeno,sizeof(LONGLONG));
		if( depthFrameReader.fail() )
			break;
		vDepthFrame.push_back(depthframeno);
	}
	depthFrameReader.close();
	return true;
}

bool Readvideo::readSkeletonFrame(string filename)
{
	vSkeletonFrame.clear();
	ifstream skeletonFrameReader;
	const char* filePathChar=filename.c_str();
	char filePathName[100];
	strcpy(filePathName,filePathChar);
	strcat(filePathName,"\\skeleton.frame");
	skeletonFrameReader.open(filePathName,ios::binary);
	if(skeletonFrameReader == NULL)
		return false;
	while(!skeletonFrameReader.eof())
	{
		LONGLONG skeletonframeno;
		skeletonFrameReader.read((char*)&skeletonframeno,sizeof(LONGLONG));
		if( skeletonFrameReader.fail() )
			break;
		vSkeletonFrame.push_back(skeletonframeno);
	}
	skeletonFrameReader.close();
	return true;
}

unsigned long WINAPI Readvideo::keyFrameSegmentThread(LPVOID pParam)
{
	Readvideo* myVideo=(Readvideo*)pParam;
	myVideo->keyFrames.KeyframeExtractionOnline();
	return 0;
}

void Readvideo::keyFrameSegment()
{
	hKeyFrameThread = CreateThread(NULL,0,this->keyFrameSegmentThread,this,0,&keyFrameThreadID);
}