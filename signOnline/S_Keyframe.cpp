#include "StdAfx.h"
#include "S_Keyframe.h"


S_Keyframe::S_Keyframe()
{
	bufferBeginPointer=0;
	bufferEndPointer=0;
	framesNum=0;
	processedFrameNum=0;
	//int maxFrameNum=5000;
	getDataOver=false;
	segmentOver=false;
	rightLastFrame=-1;
	leftLastFrame=-1;
	bothLastFrame=-1;
	myHeightThres=370;
	InitializeCriticalSection(&csFrameData);
	InitializeCriticalSection(&csFragmentData);
	InitializeCriticalSection(&csMessageData);

	rightVel=new double[bufferSize];
	leftVel=new double[bufferSize];
	
	leftFlag=new bool[bufferSize];
	rightFlag=new bool[bufferSize];
	bothFlag=new bool[bufferSize];

	for(int i=0;i<bufferSize;i++)
	{
		leftFlag[i]=false;
		rightFlag[i]=false;
		bothFlag[i]=false;
	}
}

S_Keyframe::S_Keyframe(vector<SLR_ST_Skeleton> SkeletonData,vector<Mat> DepthData, vector<IplImage*> ColorData)
{
	//InitializeCriticalSection(&csFrameData);
	//framesNum=SkeletonData.size();

	//vSkeletonData=SkeletonData;
	//vDepthData=DepthData;
	//vColorData=ColorData;
	//
	//rightVel=NULL;
	//leftVel=NULL;
	//for(int i=0;i<framesNum;i++)
	//{
	//	CvPoint3D32f tempLeft,tempRight;
	//	tempLeft.x = 1000 * vSkeletonData[i]._3dPoint[7].x;
	//	tempLeft.y = 1000 * vSkeletonData[i]._3dPoint[7].y;
	//	tempLeft.z = 1000 * vSkeletonData[i]._3dPoint[7].z;
	//	leftHands.push_back(tempLeft);

	//	tempRight.x = 1000 * vSkeletonData[i]._3dPoint[11].x;
	//	tempRight.y = 1000 * vSkeletonData[i]._3dPoint[11].y;
	//	tempRight.z = 1000 * vSkeletonData[i]._3dPoint[11].z;
	//	rightHands.push_back(tempRight);
	//}
	//leftFlag=new bool[framesNum];
	//rightFlag=new bool[framesNum];
	//bothFlag=new bool[framesNum];

	//for(int i=0;i<framesNum;i++)
	//{
	//	leftFlag[i]=false;
	//	rightFlag[i]=false;
	//	bothFlag[i]=false;
	//}

	//RightHandSegment.keyframe_num=0;
	//RightHandSegment.keyframe_no=new int[20];
	//RightHandSegment.keyframe_pic=new IplImage**[20];
	//RightHandSegment.keyframe_ID=new int*[20];
	//RightHandSegment.keyframe_coor=new int*[20];
	//for(int i=0;i<20;i++)
	//{
	//	RightHandSegment.keyframe_pic[i]=new IplImage*[framesNum];
	//	RightHandSegment.keyframe_ID[i]=new int[framesNum];
	//	RightHandSegment.keyframe_coor[i]=new int[framesNum*2];
	//}

	//LeftHandSegment.keyframe_num=0;
	//LeftHandSegment.keyframe_no=new int[20];
	//LeftHandSegment.keyframe_pic=new IplImage**[20];
	//LeftHandSegment.keyframe_ID=new int*[20];
	//LeftHandSegment.keyframe_coor=new int*[20];
	//for(int i=0;i<20;i++)
	//{
	//	LeftHandSegment.keyframe_pic[i]=new IplImage*[framesNum];
	//	LeftHandSegment.keyframe_ID[i]=new int[framesNum];
	//	LeftHandSegment.keyframe_coor[i]=new int[framesNum*2];
	//}

	//BothHandSegment.keyframe_num=0;
	//BothHandSegment.keyframe_no=new int[20];
	//BothHandSegment.keyframe_pic=new IplImage**[20];
	//BothHandSegment.keyframe_ID=new int*[20];
	//BothHandSegment.keyframe_coor=new int*[20];
	//for(int i=0;i<20;i++)
	//{
	//	BothHandSegment.keyframe_pic[i]=new IplImage*[framesNum];
	//	BothHandSegment.keyframe_ID[i]=new int[framesNum];
	//	BothHandSegment.keyframe_coor[i]=new int[framesNum*2];
	//}
}

S_Keyframe::~S_Keyframe(void)
{
	framesNum=0;
	//for(int i=0;i<vColorData.size();i++)
	//	cvReleaseImage(&vColorData[i]);
	if(leftFlag!=NULL)
	{
		delete[] leftFlag;
		leftFlag=NULL;
	}
	if(rightFlag!=NULL)
	{
		delete[] rightFlag;
		rightFlag=NULL;
	}
	if(bothFlag!=NULL)
	{
		delete[] bothFlag;
		bothFlag=NULL;
	}
	if(rightVel!=NULL)
	{
		delete[] rightVel;
		rightVel=NULL;
	}
	if(leftVel!=NULL)
	{
		delete[] leftVel;
		leftVel=NULL;
	}

	//deleteHandStruct(RightHandSegment);
	//deleteHandStruct(LeftHandSegment);
	//deleteHandStruct(BothHandSegment);
	for(int i=0;i<bufferSize && i<framesNum;i++)
		cvReleaseImage(&(vColorData[i]));
	for(int i=0;i<bufferSize && i<framesNum;i++)
		vDepthData[i].release();
	DeleteCriticalSection(&csFrameData);
	DeleteCriticalSection(&csFragmentData);
	DeleteCriticalSection(&csMessageData);
}

void S_Keyframe::deleteHandStruct(HandSegment handStruct)
{
	if(handStruct.keyframe_pic!=NULL)
	{
		for(int i=0;i<handStruct.keyframe_num;i++)
		{
			for(int j=0;j<handStruct.keyframe_no[i];j++)
				cvReleaseImage(&handStruct.keyframe_pic[i][j]);
		}
		for(int i=0;i<20;i++)
		{
			delete[] handStruct.keyframe_pic[i];
			delete[] handStruct.keyframe_coor[i];
			delete[] handStruct.keyframe_ID[i];
		}
		delete[] handStruct.keyframe_no;
		delete[] handStruct.keyframe_pic;
		delete[] handStruct.keyframe_coor;
		delete[] handStruct.keyframe_ID;

		handStruct.keyframe_pic=NULL;
	}
}

void S_Keyframe::releaseHandStruct(HandSegment &handStruct)
{
	if(handStruct.keyframe_pic!=NULL)
	{
		for(int i=0;i<handStruct.keyframe_num;i++)
		{
			for(int j=0;j<handStruct.keyframe_no[i];j++)
				cvReleaseImage(&handStruct.keyframe_pic[i][j]);
		}
	}
	handStruct.keyframe_num=0;
}

void S_Keyframe::pushImageData(SLR_ST_Skeleton SkeletonData,Mat DepthData, IplImage* ColorData)
{
	EnterCriticalSection(&csFrameData);
	CvPoint3D32f tempLeft,tempRight;
	tempLeft.x = 1000 * SkeletonData._3dPoint[7].x;
	tempLeft.y = 1000 * SkeletonData._3dPoint[7].y;
	tempLeft.z = 1000 * SkeletonData._3dPoint[7].z;
	leftHands.push_back(tempLeft);

	tempRight.x = 1000 * SkeletonData._3dPoint[11].x;
	tempRight.y = 1000 * SkeletonData._3dPoint[11].y;
	tempRight.z = 1000 * SkeletonData._3dPoint[11].z;
	rightHands.push_back(tempRight);

	if(framesNum >= bufferSize)
	{
		cvReleaseImage(&(vColorData[framesNum%bufferSize]));
		vDepthData[framesNum%bufferSize].release();
	}
	vSkeletonData[framesNum%bufferSize]=SkeletonData;
	vDepthData[framesNum%bufferSize]=DepthData.clone();
	vColorData[framesNum%bufferSize]=cvCloneImage(ColorData);
	
	framesNum++;

//	cout<<"push in data:"<<framesNum<<endl;
	LeaveCriticalSection(&csFrameData);
}

void S_Keyframe::KeyframeExtractionOnline()
{
	bool currFragmentFlag=0;
	bool prevFragmentFlag=0;
	KeyFrameSegment prevFragment;
	KeyFrameSegment currFragment;
	prevFragment.BothNum=0;
	prevFragment.LeftNum=0;
	prevFragment.RightNum=0;

	int leftBegin=0;
	int rightBegin=0;
	bool rightCurrStatus=1;	//0-static 1-dynamic
	bool leftCurrStatus=1;	//0-static 1-dynamic
	while( (!getGetDataOver()) || processedFrameNum != framesNum)
	{
		if(processedFrameNum == framesNum)
		{
			Sleep(30);
			continue;
		}
		//cout<<processedFrameNum<<endl;
		EnterCriticalSection(&csFrameData);
		if(processedFrameNum==0)
		{
			getFaceRect(0);
			rightVel[0]=0;
			leftVel[0]=0;
			processedFrameNum++;
			LeaveCriticalSection(&csFrameData);
			continue;
		}
		rightVel[processedFrameNum%bufferSize]=calDist(rightHands[processedFrameNum-1],rightHands[processedFrameNum]);
		if(rightBegin == 0)
		{
			if(vSkeletonData[processedFrameNum%bufferSize]._2dPoint[11].y < myHeightThres)
			{
				rightBegin=processedFrameNum;
				rightCurrStatus=1;
			}
		}
		else
		{
			getCurrStatus(rightVel,rightBegin,processedFrameNum,rightCurrStatus,rightFlag);
			if(rightCurrStatus==1)
				rightFlag[processedFrameNum%bufferSize]=false;
			else
				rightFlag[processedFrameNum%bufferSize]=true;
		}
		leftVel[processedFrameNum%bufferSize]=calDist(leftHands[processedFrameNum-1],leftHands[processedFrameNum]);
		if(leftBegin == 0)
		{
			if(vSkeletonData[processedFrameNum%bufferSize]._2dPoint[7].y < myHeightThres)
			{
				leftBegin=processedFrameNum;
				leftCurrStatus=1;
			}
		}
		else
		{
			getCurrStatus(leftVel,leftBegin,processedFrameNum,leftCurrStatus,leftFlag);
			if(leftCurrStatus==1)
				leftFlag[processedFrameNum%bufferSize]=false;
			else
				leftFlag[processedFrameNum%bufferSize]=true;
		}


		if(leftFlag[processedFrameNum%bufferSize]==true && rightFlag[processedFrameNum%bufferSize]==false
			&& vSkeletonData[processedFrameNum%bufferSize]._2dPoint[11].y <= myHeightThres)
		{
			if(rightVel[processedFrameNum%bufferSize]<=10)
				rightFlag[processedFrameNum%bufferSize]=true;
			//else
			//{
			//	if(rightVel[processedFrameNum%bufferSize]<=20)
			//	{
			//		double sum=0;
			//		for(int i=processedFrameNum-1;i>processedFrameNum-4;i--)
			//			sum+=rightVel[i%bufferSize];
			//		sum/=3;
			//		if(rightVel[processedFrameNum%bufferSize] < sum*1.5)
			//			rightFlag[processedFrameNum%bufferSize]=true;
			//	}
			//}
		}

		if(leftFlag[processedFrameNum%bufferSize]==false && rightFlag[processedFrameNum%bufferSize]==true
			&& vSkeletonData[processedFrameNum%bufferSize]._2dPoint[7].y <= myHeightThres)
		{
			if(leftVel[processedFrameNum%bufferSize]<=10)
				leftFlag[processedFrameNum%bufferSize]=true;
			//else
			//{
			//	if(leftVel[processedFrameNum%bufferSize]<=20)
			//	{
			//		double sum=0;
			//		for(int i=processedFrameNum-1;i>processedFrameNum-4;i--)
			//			sum+=leftVel[i%bufferSize];
			//		sum/=3;
			//		if(leftVel[processedFrameNum%bufferSize] < sum*1.5)
			//			leftFlag[processedFrameNum%bufferSize]=true;
			//	}
			//}
		}

		if(vSkeletonData[processedFrameNum%bufferSize]._2dPoint[11].y > myHeightThres)
			rightFlag[processedFrameNum%bufferSize]=false;
		if(vSkeletonData[processedFrameNum%bufferSize]._2dPoint[7].y > myHeightThres)
			leftFlag[processedFrameNum%bufferSize]=false;
		//if(processedFrameNum==185)
		//	cout<<"185----------------------------------"<<endl;
		FrameSegment fs;
		if(rightFlag[processedFrameNum%bufferSize]==true  ||  leftFlag[processedFrameNum%bufferSize]==true)
		{
			getFrameSegment(processedFrameNum,fs);
			if(currFragmentFlag==1)
			{
				if(! isInFragment(currFragment,fs) )
				{
					//putInSegmentVector(currFragment);
					if(prevFragmentFlag==1)
						mergeFragment(prevFragment,currFragment);
					else 
						prevFragment=currFragment;
					prevFragmentFlag=1;
					currFragmentFlag=0;
				}
				//else if(processedFrameNum-currFragment.EndFrameID==2)
				//{
				//	if(currFragment.RightLabel && rightVel[processedFrameNum%bufferSize]>5)
				//	{
				//		int meanVel=0;
				//		int count=0;
				//		for(int i=currFragment.RightNum-1;i>=0 && count<=3;i--)
				//		{
				//			meanVel+=rightVel[currFragment.RightID[i]%bufferSize];
				//			count++;
				//		}
				//		meanVel/=count;
				//		if(rightVel[processedFrameNum%bufferSize]>meanVel*1.7)
				//		{
				//			if(prevFragmentFlag==1)
				//				mergeFragment(prevFragment,currFragment);
				//			else 
				//				prevFragment=currFragment;
				//			prevFragmentFlag=1;
				//			currFragmentFlag=0;
				//		}
				//	}
				//	else if(currFragment.LeftLabel && leftVel[processedFrameNum%bufferSize]>5)
				//	{
				//		int meanVel=0;
				//		int count=0;
				//		for(int i=currFragment.LeftNum-1;i>=0 && count<=3;i--)
				//		{
				//			meanVel+=leftVel[currFragment.LeftID[i]%bufferSize];
				//			count++;
				//		}
				//		meanVel/=count;
				//		if(leftVel[processedFrameNum%bufferSize]>meanVel*1.7)
				//		{
				//			if(prevFragmentFlag==1)
				//				mergeFragment(prevFragment,currFragment);
				//			else 
				//				prevFragment=currFragment;
				//			prevFragmentFlag=1;
				//			currFragmentFlag=0;
				//		}
				//	}
				//}
			}
			if(currFragmentFlag==0)
			{
				CreateNewFragment(currFragment);
				currFragment.BeginFrameID=processedFrameNum;
				if(fs.both)
				{
					currFragment.BothLabel=1;
					currFragment.LeftLabel=0;
					currFragment.RightLabel=0;
				}
				else
				{
					currFragment.BothLabel=0;
					if(leftFlag[processedFrameNum%bufferSize]==true)
						currFragment.LeftLabel=1;
					else
						currFragment.LeftLabel=0;

					if(rightFlag[processedFrameNum%bufferSize]==true)
						currFragment.RightLabel=1;
					else
						currFragment.RightLabel=0;
				}
			}

			putInFragment(currFragment,fs,processedFrameNum);
			currFragmentFlag=1;
		}
		else
		{
			if(currFragmentFlag!=0)
			{
				if(processedFrameNum-currFragment.EndFrameID>=2)
				{
					if(currFragment.EndFrameID-currFragment.BeginFrameID>=0)
					{
						//putInSegmentVector(currFragment);
						
						if(prevFragmentFlag==1)
						{
							mergeFragment(prevFragment,currFragment);
							putInBuffer(prevFragment);
						}
						else
							putInBuffer(currFragment);
					}
					else
					{
						if(prevFragmentFlag==1)
						{
							mergeFragment(prevFragment,currFragment);
							putInBuffer(prevFragment);
						}
						else
							releaseFragment(currFragment);
					}
					currFragmentFlag=0;
					prevFragmentFlag=0;
				}
			}
		}

		processedFrameNum++;
		//cout<<"process frame:"<<processedFrameNum<<endl;
		LeaveCriticalSection(&csFrameData);
	}
	if(currFragmentFlag!=0)
		putInBuffer(currFragment);


	//saveKeyFrameSegment("Data//KeyFrames//Test//");
	//mergeFragment(RightHandSegment);
	//mergeFragment(LeftHandSegment);
	//mergeFragment(BothHandSegment);

	EnterCriticalSection(&csMessageData);
	segmentOver=true;
	LeaveCriticalSection(&csMessageData);
	
}

void S_Keyframe::putInSegmentVector(KeyFrameSegment &Fragment)
{
	if(Fragment.BothNum+Fragment.LeftNum+Fragment.RightNum==0)
		return;
	if(v_kfSegment.size()==0)
	{
		v_kfSegment.push_back(Fragment);
		return;
	}
	//while(Fragment.BeginFrameID-v_kfSegment[v_kfSegment.size()-1].EndFrameID <=2)
	//{
	//	//if()
	//	if(v_kfSegment[v_kfSegment.size()-1].EndFrameID - v_kfSegment[v_kfSegment.size()-1].BeginFrameID<=2)
	//	{
	//		releaseFragment(v_kfSegment[v_kfSegment.size()-1]);
	//		v_kfSegment.pop_back();
	//	}
	//}
	
	v_kfSegment.push_back(Fragment);
	return;
}

void S_Keyframe::addInLastFragment(KeyFrameSegment &Fragment)
{
	KeyFrameSegment newFragment;
	CreateNewFragment(newFragment);
	newFragment.BothLabel=Fragment.BothLabel;
	newFragment.LeftLabel=Fragment.LeftLabel;
	newFragment.RightLabel=Fragment.RightLabel;

	if(newFragment.BothLabel==1)
	{
		newFragment.BothNum=v_kfSegment[v_kfSegment.size()-1].BothNum+Fragment.BothNum;
		for(int i=0;i<v_kfSegment[v_kfSegment.size()-1].BothNum;i++)
		{
			newFragment.BothID[i]=v_kfSegment[v_kfSegment.size()-1].BothID[i];
			newFragment.BothCoor[i*2]=v_kfSegment[v_kfSegment.size()-1].BothCoor[i*2];
			newFragment.BothCoor[i*2+1]=v_kfSegment[v_kfSegment.size()-1].BothCoor[i*2+1];
			newFragment.BothImages[i]=v_kfSegment[v_kfSegment.size()-1].BothImages[i];
		}
		for(int i=0;i<Fragment.BothNum;i++)
		{
			int j=v_kfSegment[v_kfSegment.size()-1].BothNum+i;
			newFragment.BothID[j]=Fragment.BothID[i];
			newFragment.BothCoor[j*2]=Fragment.BothCoor[i*2];
			newFragment.BothCoor[j*2+1]=Fragment.BothCoor[i*2+1];
			newFragment.BothImages[j]=Fragment.BothImages[i];
		}
	}
	else
	{
		if(newFragment.LeftLabel==1)
		{
			newFragment.LeftNum=v_kfSegment[v_kfSegment.size()-1].LeftNum+Fragment.LeftNum;
			for(int i=0;i<v_kfSegment[v_kfSegment.size()-1].BothNum;i++)
			{
				newFragment.LeftID[i]=v_kfSegment[v_kfSegment.size()-1].LeftID[i];
				newFragment.LeftCoor[i*2]=v_kfSegment[v_kfSegment.size()-1].LeftCoor[i*2];
				newFragment.LeftCoor[i*2+1]=v_kfSegment[v_kfSegment.size()-1].LeftCoor[i*2+1];
				newFragment.LeftImages[i]=v_kfSegment[v_kfSegment.size()-1].LeftImages[i];
			}
			for(int i=0;i<Fragment.LeftNum;i++)
			{
				int j=v_kfSegment[v_kfSegment.size()-1].LeftNum+i;
				newFragment.LeftID[j]=Fragment.LeftID[i];
				newFragment.LeftCoor[j*2]=Fragment.LeftCoor[i*2];
				newFragment.LeftCoor[j*2+1]=Fragment.LeftCoor[i*2+1];
				newFragment.LeftImages[j]=Fragment.LeftImages[i];
			}
		}

		if(newFragment.RightLabel==1)
		{
			newFragment.RightNum=v_kfSegment[v_kfSegment.size()-1].RightNum+Fragment.RightNum;
			for(int i=0;i<v_kfSegment[v_kfSegment.size()-1].RightNum;i++)
			{
				newFragment.RightID[i]=v_kfSegment[v_kfSegment.size()-1].RightID[i];
				newFragment.RightCoor[i*2]=v_kfSegment[v_kfSegment.size()-1].RightCoor[i*2];
				newFragment.RightCoor[i*2+1]=v_kfSegment[v_kfSegment.size()-1].RightCoor[i*2+1];
				newFragment.RightImages[i]=v_kfSegment[v_kfSegment.size()-1].RightImages[i];
			}
			for(int i=0;i<Fragment.RightNum;i++)
			{
				int j=v_kfSegment[v_kfSegment.size()-1].RightNum+i;
				newFragment.RightID[j]=Fragment.RightID[i];
				newFragment.RightCoor[j*2]=Fragment.RightCoor[i*2];
				newFragment.RightCoor[j*2+1]=Fragment.RightCoor[i*2+1];
				newFragment.RightImages[j]=Fragment.RightImages[i];
			}
		}
	}
	releaseFragment(Fragment);
	releaseFragment(v_kfSegment[v_kfSegment.size()-1]);
	v_kfSegment.pop_back();
	v_kfSegment.push_back(newFragment);
}

bool S_Keyframe::isInFragment(KeyFrameSegment Fragment,FrameSegment fs)
{
	if((int)fs.both==true && Fragment.BothLabel==1)
		return true;
	if(Fragment.RightLabel==1  &&  Fragment.LeftLabel==1 && fs.right==true && fs.left==true)
		return true;
	if(Fragment.RightLabel==1  &&  Fragment.LeftLabel!=1 && fs.right==true && fs.left==false)
		return true;
	if(Fragment.RightLabel!=1  &&  Fragment.LeftLabel==1 && fs.right==false && fs.left==true)
		return true;
	return false;
}

void S_Keyframe::putInFragment(KeyFrameSegment &Fragment,FrameSegment &fs ,int frameID)
{
	if(frameID > Fragment.EndFrameID)
		Fragment.EndFrameID=frameID;
	if(frameID < Fragment.BeginFrameID)
		Fragment.BeginFrameID=frameID;

	if(fs.both)
	{
		Fragment.BothCoor[Fragment.BothNum*2]=fs.bothCoor[0];
		Fragment.BothCoor[Fragment.BothNum*2+1]=fs.bothCoor[1];
		Fragment.BothID[Fragment.BothNum]=frameID;
		Fragment.BothImages[Fragment.BothNum]=fs.bothImage;
		Fragment.BothNum++;
	}

	if(fs.left)
	{
		Fragment.LeftCoor[Fragment.LeftNum*2]=fs.leftCoor[0];
		Fragment.LeftCoor[Fragment.LeftNum*2+1]=fs.leftCoor[1];
		Fragment.LeftID[Fragment.LeftNum]=frameID;
		Fragment.LeftImages[Fragment.LeftNum]=fs.leftImage;
		Fragment.LeftNum++;
	}

	if(fs.right)
	{
		Fragment.RightCoor[Fragment.RightNum*2]=fs.rightCoor[0];
		Fragment.RightCoor[Fragment.RightNum*2+1]=fs.rightCoor[1];
		Fragment.RightID[Fragment.RightNum]=frameID;
		Fragment.RightImages[Fragment.RightNum]=fs.rightImage;
		Fragment.RightNum++;
	}
	Fragment.EndFrameID=frameID;
}

void S_Keyframe::CreateNewFragment(KeyFrameSegment &Fragment)
{
	int maxFrameNum=300;
	Fragment.BothID=new int[maxFrameNum];
	Fragment.LeftID=new int[maxFrameNum];
	Fragment.RightID=new int[maxFrameNum];

	Fragment.BothCoor=new int[maxFrameNum*2];
	Fragment.LeftCoor=new int[maxFrameNum*2];
	Fragment.RightCoor=new int[maxFrameNum*2];

	Fragment.BothImages=new IplImage*[maxFrameNum];
	Fragment.LeftImages=new IplImage*[maxFrameNum];
	Fragment.RightImages=new IplImage*[maxFrameNum];

	Fragment.BothNum=0;
	Fragment.LeftNum=0;
	Fragment.RightNum=0;
}

void S_Keyframe::getCurrStatus(double* vel,int &beginFrame, int endFrame ,bool &currStatus,bool* flag)
{
	double alph=0.6;
	double beta=1.5;
	double sum=0;
	int count=0;
	int begin=0;
	if(currStatus==0)
	{
		for(int i=endFrame-1;i>=beginFrame && count<3;i--)
		{
			if(flag[i%bufferSize]==true)
			{
				sum+=vel[i%bufferSize];
				count++;
			}
		}
		sum/=count;
	}
	else
	{

		for(int i=endFrame-1;i>=beginFrame && count<3;i--)
		{
			sum+=vel[i%bufferSize];
			count++;
		}
		sum/=count;
	}

	if(currStatus==1)
	{
		if(vel[endFrame%bufferSize] < sum*alph)
		{
			beginFrame=endFrame;
			currStatus=0;
		}
	}
	else
	{
		if(vel[endFrame%bufferSize] > sum*beta)
		{
			beginFrame=endFrame;
			currStatus=1;
		}
		else
			currStatus=0;
	}
	if(vel[endFrame%bufferSize]<5 && currStatus==1)
	{
		beginFrame=endFrame;
		currStatus=0;
	}
	if(vel[endFrame%bufferSize]>20 && currStatus==0)
	{
		beginFrame=endFrame;
		currStatus=1;
	}
}

void S_Keyframe::KeyframeExtraction()
{
	//if(framesNum>0)
	//{
	//	rightVel=new double[framesNum];
	//	leftVel=new double[framesNum];
	//}
	//else
	//	return;


	//meanRightVel=calVel(rightHands,rightVel,rightBegin,rightEnd,11);
	//meanLeftVel=calVel(leftHands,leftVel,leftBegin,leftEnd,7);

	//keyframesJudge(rightVel,rightBegin,rightEnd,meanRightVel,RightKeyFrames);
	//keyframesJudge(leftVel,leftBegin,leftEnd,meanLeftVel,LeftKeyFrames);

	//mergeKeyframes(RightKeyFrames);
	//mergeKeyframes(LeftKeyFrames);

	//for(int i=0;i<RightKeyFrames.size();i++)
	//{
	//	for(int j=RightKeyFrames[i].StartFrame;j<=RightKeyFrames[i].EndFrame;j++)
	//		rightFlag[j]=true;
	//}

	//for(int i=0;i<LeftKeyFrames.size();i++)
	//{
	//	for(int j=LeftKeyFrames[i].StartFrame;j<=LeftKeyFrames[i].EndFrame;j++)
	//		leftFlag[j]=true;
	//}

	//getFaceRect(0);

	//rightLastFrame=-1;
	//leftLastFrame=-1;
	//bothLastFrame=-1;

	//
	//for(int i=0;i<framesNum;i++)
	//	if(leftFlag[i] || rightFlag[i])
	//	{
	//		handSegment(i);
	//	}
	//
	//delete[] rightVel;
	//delete[] leftVel;
}

double S_Keyframe::calVel(vector<CvPoint3D32f> Hands,double* vel,int &beginFrame,int &endFrame,int handIdx)
{
	CvPoint3D32f lastPoint=Hands[0];
	CvPoint3D32f currPoint;
	vel[0]=0;
	
	for(int i=1;i<Hands.size();i++)
	{
		currPoint=Hands[i];
		vel[i]=calDist(lastPoint,currPoint);
		lastPoint=currPoint;
	}
	int i;
	for(i=0;i<Hands.size();i++)
		if(vSkeletonData[i]._2dPoint[handIdx].y < myHeightThres)
			break;
	beginFrame=i;
	int j=Hands.size()-1;
	for(j;j>=0;j--)
		if(vSkeletonData[j]._2dPoint[handIdx].y < myHeightThres)
			break;
	endFrame=j;

	double sum=0;
	for(i=beginFrame;i<=endFrame;i++)
		sum+=vel[i];
	sum/=Hands.size();
//	sum/=(endFrame-beginFrame+1);
	return sum;
}

double S_Keyframe::calDist(CvPoint3D32f p1,CvPoint3D32f p2)
{
	double dist=0;
	dist+=(p1.x-p2.x)*(p1.x-p2.x);
	dist+=(p1.y-p2.y)*(p1.y-p2.y);
	dist+=(p1.z-p2.z)*(p1.z-p2.z);
	dist=sqrt(dist);
	return dist;
}

double S_Keyframe::calDist(CvPoint2D32f p1,CvPoint2D32f p2)
{
	double dist=0;
	dist+=(p1.x-p2.x)*(p1.x-p2.x);
	dist+=(p1.y-p2.y)*(p1.y-p2.y);
	dist=sqrt(dist);
	return dist;
}

void S_Keyframe::keyframesJudge(double* vel,int beginFrame,int endFrame,double meanVel,vector<KeyFrameUnit> &KeyFrames)
{
	for(int i=beginFrame;i<=endFrame;i++)
	{
		if(vel[i] > meanVel)
			continue;
		int j;
		for(j=i+1;j<=endFrame;j++)
		{
			if(vel[j] > meanVel)
				break;
		}
		if(j-i > consistThres)
		{
			KeyFrameUnit currUnit;
			currUnit.StartFrame=i;
			currUnit.EndFrame=j-1;
			KeyFrames.push_back(currUnit);
		}
		i=j;
	}
}

vector<KeyFrameUnit> S_Keyframe::getLeftKeyFrames()
{
	return LeftKeyFrames;
}

vector<KeyFrameUnit> S_Keyframe::getRightKeyFrames()
{
	return RightKeyFrames;
}

void S_Keyframe::mergeKeyframes(vector<KeyFrameUnit> &KeyFrames)
{
	int size=KeyFrames.size();
	if(size < 2)
		return;

	vector<KeyFrameUnit> tempKeyframes=KeyFrames;
	KeyFrames.clear();
	
	for(int i=0;i<size;i++)
	{
		int currBegin=tempKeyframes[i].StartFrame;
		int currEnd=tempKeyframes[i].EndFrame;
		int j;
		for(j=i+1;j<size;j++)
		{
			if( (tempKeyframes[j].StartFrame-currEnd) <= 2)
				currEnd=tempKeyframes[j].EndFrame;
			else
				break;
		}
		KeyFrameUnit newUnit;
		newUnit.StartFrame=currBegin;
		newUnit.EndFrame=currEnd;
		KeyFrames.push_back(newUnit);
		i=j-1;
	}
}

void S_Keyframe::handSegment(int frameID)
{
	IplImage* grayImage;
	grayImage=cvCreateImage(cvSize(vColorData[frameID%bufferSize]->width,vColorData[frameID%bufferSize]->height),vColorData[frameID%bufferSize]->depth,1);
	cvCvtColor(vColorData[frameID%bufferSize],grayImage,CV_RGB2GRAY);
	depthRestrict(frameID,grayImage);
	IplImage* binaryImage;
	binaryImage=cvCreateImage(cvSize(vColorData[frameID%bufferSize]->width,vColorData[frameID%bufferSize]->height),vColorData[frameID%bufferSize]->depth,1);
	cvThreshold( grayImage, binaryImage, 20 ,255 , CV_THRESH_BINARY );

	int nMaxRect;
	int theCent[100];
	int theBox[100];
	int nThreshold=300;
	getConnexeCenterBox(binaryImage, nMaxRect, theCent, theBox, nThreshold);
	cvDilate(binaryImage,binaryImage);
	cvErode(binaryImage,binaryImage);
	cvCvtColor(vColorData[frameID%bufferSize],grayImage,CV_RGB2GRAY);

	getHandImage(frameID,binaryImage,nMaxRect,theCent,theBox,grayImage);

	cvReleaseImage(&grayImage);
	cvReleaseImage(&binaryImage);
}

void S_Keyframe::getFrameSegment(int frameID,FrameSegment& fs)
{
	IplImage* grayImage;
	grayImage=cvCreateImage(cvSize(vColorData[frameID%bufferSize]->width,vColorData[frameID%bufferSize]->height),vColorData[frameID%bufferSize]->depth,1);
	//if(frameID==351)
	//{
	//	cout<<grayImage<<endl;
	//}
	cvCvtColor(vColorData[frameID%bufferSize],grayImage,CV_RGB2GRAY);
	depthRestrict(frameID,grayImage);
	IplImage* binaryImage;
	binaryImage=cvCreateImage(cvSize(vColorData[frameID%bufferSize]->width,vColorData[frameID%bufferSize]->height),vColorData[frameID%bufferSize]->depth,1);
	cvThreshold( grayImage, binaryImage, 20 ,255 , CV_THRESH_BINARY );
	//if(frameID==351)
	//	cvSaveImage("binaryImage1.jpg",binaryImage);
	int nMaxRect;
	int theCent[100];
	int theBox[100];
	int nThreshold=300;
	getConnexeCenterBox(binaryImage, nMaxRect, theCent, theBox, nThreshold);
	/*if(frameID==351)
		cvSaveImage("binaryImage2.jpg",binaryImage);*/
	cvDilate(binaryImage,binaryImage);
	/*if(frameID==351)
		cvSaveImage("binaryImage3.jpg",binaryImage);*/
	cvErode(binaryImage,binaryImage);
	/*if(frameID==351)
		cvSaveImage("binaryImage4.jpg",binaryImage);*/
	cvCvtColor(vColorData[frameID%bufferSize],grayImage,CV_RGB2GRAY);
	//if(frameID==351)
	//	cvSaveImage("grayImage1.jpg",grayImage);

	fs.both=false;
	fs.left=false;
	fs.right=false;
	//if(frameID==351)
	//	cout<<"get hand image begin"<<endl;
	getHandImage(frameID,binaryImage,nMaxRect,theCent,theBox,grayImage,fs);
	//if(frameID==351)
	//{
	//	cout<<"get hand image end"<<endl;
	//	if(fs.right)
	//		cvSaveImage("rightImage.jpg",fs.rightImage);
	//	if(fs.left)
	//		cvSaveImage("leftImage.jpg",fs.leftImage);
	//}

	//if(frameID==351)
	//{
	//	cout<<"release gray image begin"<<endl;
	//	cout<<grayImage<<endl;
	//}
	//if(frameID==351)
	//	cvSaveImage("grayImage2.jpg",grayImage);
	cvReleaseImage(&grayImage);
	//if(frameID==351)
	//	cout<<"release gray image end"<<endl;

	//if(frameID==351)
	//	cout<<"release binary image begin"<<endl;
	cvReleaseImage(&binaryImage);
	//if(frameID==351)
	//	cout<<"release binary image end"<<endl;
}

void S_Keyframe::getFaceRect(int frameID)
{
	//IplImage* colorImage;
	//colorImage=cvCreateImage(cvSize(vColorData[frameID]->width,vColorData[frameID]->height),IPL_DEPTH_8U,1);
	//cvCvtColor(vColorData[frameID],colorImage,CV_BGR2GRAY);
	//LANDMARKINFO* lands;
	//lands=new LANDMARKINFO[103];
	//typedef void(* FaceDec)(IplImage *pImage, IplImage *outImage, LANDMARKINFO* lands); 

 //   //定义一个函数指针变量
 //   FaceDec pfFuncInDll = NULL; 

 //   //加载dll
 //   HINSTANCE hinst=LoadLibrary(LPCSTR("FaceDetector.dll")); 
 //           
 //   if ( hinst != NULL  )
	//{
	//	//找到dll的clFun函数
	//	pfFuncInDll = (FaceDec)GetProcAddress(hinst, "GetFaceFeaturePoints"); 

	//	IplImage* tempImage;
	//	tempImage=cvCreateImage(cvSize(colorImage->width,colorImage->height),colorImage->depth,3);
	//	//调用dll里的函数
	//	if (pfFuncInDll != NULL )
	//	{ 
	//		pfFuncInDll(colorImage,tempImage,lands);
	//	}
	//	cvReleaseImage(&tempImage);
	//	FreeLibrary(hinst);
 //   }
	//
	//cvReleaseImage(&colorImage);

	//int XMin = 999;
 //   int XMax = -999;
 //   int YMin = 999;
 //   int YMax = -999;

 //   for (int i = 0; i < 103; i++)
	//{
	//	Point pt(lands[i].x,lands[i].y);
	//	if (pt.x > XMax) XMax = (int)pt.x;
 //       if (pt.x < XMin) XMin = (int)pt.x;
 //       if (pt.y > YMax) YMax = (int)pt.y;
 //       if (pt.y < YMin) YMin = (int)pt.y;
 //   }



 //   int topX = XMin - (XMax - XMin) / 2;
 //   int topY = YMin - (YMax - YMin);
 //   int bottomX = XMax + 15;
 //   int bottomY = YMax + 5;
 //   int width = bottomX - topX + 1;
 //   int height = bottomY - topY + 1;

	//delete[] lands;

	Point headPoint = Point(vSkeletonData[frameID%bufferSize]._2dPoint[3].x, vSkeletonData[frameID%bufferSize]._2dPoint[3].y);
    Point neckPoint = Point(vSkeletonData[frameID%bufferSize]._2dPoint[2].x, vSkeletonData[frameID%bufferSize]._2dPoint[2].y);

    Point LShoulder = Point(vSkeletonData[frameID%bufferSize]._2dPoint[4].x, vSkeletonData[frameID%bufferSize]._2dPoint[4].y);
    Point RShoulder = Point(vSkeletonData[frameID%bufferSize]._2dPoint[8].x, vSkeletonData[frameID%bufferSize]._2dPoint[8].y);
    int width = (int)(3  * abs(LShoulder.x - RShoulder.x)/4);
    int height = (int)abs(headPoint.y - neckPoint.y) * 2;
    int topX = (int)(headPoint.x - width / 2);
    int topY = (int)(headPoint.y - height / 2);
	if(topY < 0)
	{
		height-=0-topY;
		topY=0;
	}

	//cvRectangle(vColorData[frameID],cvPoint(topX,topY),cvPoint(topX+width,topY+height),cvScalar(0,0,0));
	//cvSaveImage("headDect1.jpg",vColorData[frameID]);

    initFaceRect= Rect(topX, topY, width, height);
	getMinDepth(frameID,initFaceRect);

	int depthThres=vDepthData[frameID%bufferSize].at<u16>(neckPoint.y,neckPoint.x)-initMinDepth-20;
	if(depthThres < 20)
		depthThres=20;
	for(int i=initFaceRect.y;i<initFaceRect.y+initFaceRect.height;i++)
	{
		bool loopBreak=false;
		for(int j=initFaceRect.x;j<initFaceRect.x+initFaceRect.width;j++)
			if((vDepthData[frameID%bufferSize].at<u16>(i,j) >initMinDepth) && (vDepthData[frameID%bufferSize].at<u16>(i,j) -initMinDepth) <depthThres   &&  isSkinColor(vColorData[frameID%bufferSize],i,j) )
			{
				loopBreak=true;
				break;
			}
		if(loopBreak)
			break;
		topY++;
		height--;
	}

	for(int i=initFaceRect.y+initFaceRect.height-1;i>=initFaceRect.y;i--)
	{
		bool loopBreak=false;
		for(int j=initFaceRect.x;j<initFaceRect.x+initFaceRect.width;j++)
			if((vDepthData[frameID%bufferSize].at<u16>(i,j) >initMinDepth) && (vDepthData[frameID%bufferSize].at<u16>(i,j) -initMinDepth) <depthThres   &&  isSkinColor(vColorData[frameID%bufferSize],i,j) )
			{
				loopBreak=true;
				break;
			}
		if(loopBreak)
			break;
		height--;
	}

	for(int j=initFaceRect.x;j<initFaceRect.x+initFaceRect.width;j++)
	{
		bool loopBreak=false;
		for(int i=initFaceRect.y;i<initFaceRect.y+initFaceRect.height;i++)
			if((vDepthData[frameID%bufferSize].at<u16>(i,j) >initMinDepth) && (vDepthData[frameID%bufferSize].at<u16>(i,j) -initMinDepth) <depthThres   &&  isSkinColor(vColorData[frameID%bufferSize],i,j) )
			{
				loopBreak=true;
				break;
			}
		if(loopBreak)
			break;
		topX++;
		width--;
	}

	for(int j=initFaceRect.x+initFaceRect.width-1;j>=initFaceRect.x;j--)
	{
		bool loopBreak=false;
		for(int i=initFaceRect.y;i<initFaceRect.y+initFaceRect.height;i++)
			if((vDepthData[frameID%bufferSize].at<u16>(i,j) >initMinDepth) && (vDepthData[frameID%bufferSize].at<u16>(i,j) -initMinDepth) <depthThres   &&  isSkinColor(vColorData[frameID%bufferSize],i,j) )
			{
				loopBreak=true;
				break;
			}
		if(loopBreak)
			break;
		width--;
	}

	if(height > 20 && width > 20)
		initFaceRect= Rect(topX, topY, width, height);

// 	cvSetImageROI(vColorData[frameID%bufferSize],cvRect(topX,topY,width,height));
// 	IplImage* faceImage=cvCreateImage(cvSize(width,height),vColorData[frameID%bufferSize]->depth,vColorData[frameID%bufferSize]->nChannels);
// 	cvCopy(vColorData[frameID%bufferSize],faceImage);
// 	getSkinColorModel(faceImage);
// 	cvReleaseImage(&faceImage);
// 	cvResetImageROI(vColorData[frameID%bufferSize]);

	//cvRectangle(vColorData[frameID],cvPoint(topX,topY),cvPoint(topX+width,topY+height),cvScalar(0,0,0));
	//cvSaveImage("headDect.jpg",vColorData[frameID]);
	
}

void S_Keyframe::getMinDepth(int frameID,Rect imageRect)
{
	short depthMin = 9999;
	for(int i=imageRect.y;i<imageRect.y+imageRect.height;i++)
	{
		short* data=vDepthData[frameID%bufferSize].ptr<short>(i);
		for(int j=imageRect.x;j<imageRect.x+imageRect.width;j++)
			if(data[j] >400 && data[j]<depthMin)
				depthMin=data[j];
	}
	initMinDepth= depthMin;
}

void S_Keyframe::depthRestrict(int frameID, IplImage* grayImage)
{
	int headDepth=vSkeletonData[frameID%bufferSize]._3dPoint[3].z*1000;
	int disThres=headDepth-initMinDepth;
	int headThres=headDepth-disThres-5;
//	int neckThres=headThres+30;
	int bodyThres=headThres+50;
	int leftHandThres=headThres+150;
	int rightHandThres=leftHandThres;
	int legsHandThres=headThres;

	int nrow=vDepthData[frameID%bufferSize].rows;
	int ncol=vDepthData[frameID%bufferSize].cols*vDepthData[frameID%bufferSize].channels();
	for(int i=0;i<initFaceRect.y+initFaceRect.height;i++)
	{
		for(int j=0;j<initFaceRect.x;j++)
		{
			if(vDepthData[frameID%bufferSize].at<u16>(i,j) <20 || vDepthData[frameID%bufferSize].at<u16>(i,j)>leftHandThres)
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
			else if(!isSkinColorModel(vColorData[frameID%bufferSize],i,j))
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
		}
	}
	//cvSaveImage("1testColorLeftHand.jpg",grayImage);

	for(int i=0;i<initFaceRect.y+initFaceRect.height;i++)
	{
		for(int j=initFaceRect.x;j<initFaceRect.x+initFaceRect.width;j++)
		{
			if(vDepthData[frameID%bufferSize].at<u16>(i,j)<20 || vDepthData[frameID%bufferSize].at<u16>(i,j)>headThres)
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
			else if(!isSkinColorModel(vColorData[frameID%bufferSize],i,j))
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
		}
	}
	//cvSaveImage("2testColorHead.jpg",grayImage);

	for(int i=0;i<initFaceRect.y+initFaceRect.height;i++)
	{
		for(int j=initFaceRect.x+initFaceRect.width;j<ncol;j++)
		{
			if(vDepthData[frameID%bufferSize].at<u16>(i,j) <20 || vDepthData[frameID%bufferSize].at<u16>(i,j)>rightHandThres)
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
			else if(!isSkinColorModel(vColorData[frameID%bufferSize],i,j))
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
		}
	}
	//cvSaveImage("3testColorRightHand.jpg",grayImage);

	for(int i=initFaceRect.y+initFaceRect.height;i<initFaceRect.y+initFaceRect.height+30;i++)
	{
		for(int j=0;j<ncol;j++)
		{
			if(vDepthData[frameID%bufferSize].at<u16>(i,j) <20 || vDepthData[frameID%bufferSize].at<u16>(i,j)>bodyThres)
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
			else if(!isSkinColorModel(vColorData[frameID%bufferSize],i,j))
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
		}
	}
	//cvSaveImage("4testColorNeck.jpg",grayImage);

	for(int i=initFaceRect.y+initFaceRect.height+30;i<myHeightThres+60 && i<nrow;i++)
	{
		for(int j=0;j<ncol;j++)
		{
			if(vDepthData[frameID%bufferSize].at<u16>(i,j) <20 || vDepthData[frameID%bufferSize].at<u16>(i,j)>bodyThres)
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
			else if(!isSkinColorModel(vColorData[frameID%bufferSize],i,j))
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
		}
	}
	//cvSaveImage("5testColorBody.jpg",grayImage);

	for(int i=myHeightThres+60;i<myHeightThres+100 && i<nrow;i++)
	{
		for(int j=0;j<ncol;j++)
		{
			if(vDepthData[frameID%bufferSize].at<u16>(i,j)<20 || vDepthData[frameID%bufferSize].at<u16>(i,j)>legsHandThres)
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
			else if(!isSkinColorModel(vColorData[frameID%bufferSize],i,j))
			{
				((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
			}
		}
	}
	//cvSaveImage("6testColorLegs.jpg",grayImage);

	for(int i=myHeightThres+100;i<nrow;i++)
	{
		for(int j=0;j<ncol;j++)
		{
			((char*)(grayImage->imageData + grayImage->widthStep*i))[j] = 0;
		}
	}
	//cvSaveImage("7testColor.jpg",grayImage);
}

void S_Keyframe::skinColorProcess(IplImage* colorImage)
{
	for(int i=0;i<colorImage->height;i++)
		for(int j=0;j<colorImage->width;j++)
		{
			int B=((char*)(colorImage->imageData + colorImage->widthStep*i))[j*3];
			int G=((char*)(colorImage->imageData + colorImage->widthStep*i))[j*3+1];
			int R=((char*)(colorImage->imageData + colorImage->widthStep*i))[j*3+2];
			
			if((double)R-1.0905*B>=0)
			{
				double sum=B+G+R;
				if (3 * (double)(R - G) * (double)(R) + sum * (sum - 3 * 0.9498 * (double)R) > 0)
				{
					((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3] = 0;
					((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3+1] = 0;
					((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3+2] = 0;
				}
			}
			else
			{
				((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3] = 0;
				((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3+1] = 0;
				((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3+2] = 0;
			}
		}
}

bool S_Keyframe::isSkinColor(IplImage* colorImage, int i, int j)
{
	int B=((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3];
	int G=((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3+1];
	int R=((u8*)(colorImage->imageData + colorImage->widthStep*i))[j*3+2];
			
	if((double)R-1.0905*B>=0)
	{
		double sum=B+G+R;
		if (3 * (double)(R - G) * (double)(R) + sum * (sum - 3 * 0.9498 * (double)R) > 0)
		{
			return false;
		}
		return true;
	}
	return false;
}

void S_Keyframe::getConnexeCenterBox(IplImage* image, int& nMaxRect, int* theCent, int* theBox, int& nThreshold)
{
	int nconnectSrcArea = image->width*image->height;
	void *bufferOut =NULL;
	unsigned char *connectSrc =NULL;

	bufferOut  = (void*) malloc(nconnectSrcArea*sizeof(unsigned char));
	connectSrc = (unsigned char*) malloc(nconnectSrcArea*sizeof(unsigned char));
	memset(connectSrc,0,nconnectSrcArea*sizeof(unsigned char));
	int i,j;
	for(j = 0; j < image->height; j++)
	{
		for(i = 0; i < image->width; i++)
		{
			connectSrc[j*image->width+i]=(unsigned char)(image->imageData+image->widthStep*j)[i];		
		}
	}

	int nMax = 2;
	int bufferDims[3] = { image->width,image->height, 1 };
	Connexe_SetMinimumSizeOfComponents( nThreshold);		// original is set to 1;
	Connexe_SetMaximumNumberOfComponents( nMax );		// original is set to 1;

	int countConnect =  CountConnectedComponents( (void*)connectSrc, CONN_UCHAR,bufferOut, CONN_UCHAR,bufferDims );//bufferOut包含位源图像区域标号;

	free(connectSrc);
	connectSrc = (unsigned char*) bufferOut;

	for(j = 0; j < image->height; j ++)
	{
		for(i = 0; i < image->width; i ++)
		{
			(image->imageData+image->widthStep*j)[i] = connectSrc[j*image->width+i];
			//if(connectSrc[j*image->width+i] != 0)
			//{
			//	(image->imageData+image->widthStep*j)[i] = char (255);
			//}
		}
	}
	nMaxRect = GetCenterofComponentWithBoxNew(bufferOut, bufferDims, theCent, theBox, nMax, FALSE, nThreshold);

	free(bufferOut);
	bufferOut = NULL;
	connectSrc = NULL;
}

void S_Keyframe::getHandImage(int frameID,IplImage* binaryImage, int& nMaxRect, int* theCent, int* theBox,IplImage* grayImage)
{
	if(nMaxRect <1 || nMaxRect>2)
		return;

	double rightDis=calDist(rightHands[0],rightHands[frameID]);
	double leftDis=calDist(leftHands[0],leftHands[frameID]);

	CvPoint2D32f rightPoint,leftPoint,connexePoint;
	rightPoint.x=vSkeletonData[frameID%bufferSize]._2dPoint[11].x;
	rightPoint.y=vSkeletonData[frameID%bufferSize]._2dPoint[11].y;

	leftPoint.x=vSkeletonData[frameID%bufferSize]._2dPoint[7].x;
	leftPoint.y=vSkeletonData[frameID%bufferSize]._2dPoint[7].y;

	bool LFlag = false; //true: too close
    bool RFlag = false;

    if (leftDis < 80)
		LFlag = true;
    else 
		LFlag = false;

    if (rightDis < 80)
		RFlag = true;
    else
		RFlag = false;

	int handStatusKey = -1;
    // 0 -> Left&Right stay at the original region
    // 1 -> Left stay at the orignal region,Right leave
    // 2 -> Right stays at the orignal region,Left leave
    // 3 -> Left and Right leaves

    if (LFlag && RFlag)
		handStatusKey = 0;
	else if (LFlag && !RFlag)
		handStatusKey = 1;
	else if (!LFlag && RFlag)
		handStatusKey = 2;
	else if (!LFlag && !RFlag)
		handStatusKey = 3;

	if(nMaxRect==1)
	{		
		CvPoint2D32f connexePoint;
		connexePoint.x=theCent[0];
		connexePoint.y=theCent[1];

		rightDis=calDist(rightPoint,connexePoint);
		leftDis=calDist(leftPoint,connexePoint);
		IplImage* handImage=getConnextImage(grayImage, binaryImage,theBox,1,frameID);
		switch(handStatusKey)
		{
		case 0:
			if (rightDis < 80 && (rightDis < leftDis))
				saveHandImage(handImage, frameID, RIGHT, Point(*theBox,*(theBox+1)) );
            else if (leftDis < 80 && (rightDis > leftDis))
                saveHandImage(handImage, frameID, LEFT, Point(*theBox,*(theBox+1)) );
			break;
        case 1:
            if (rightDis < 80 && (rightDis < leftDis))
				saveHandImage(handImage, frameID, RIGHT, Point(*theBox,*(theBox+1)) );
            else if (leftDis < 80 && (rightDis > leftDis))
				saveHandImage(handImage, frameID, LEFT, Point(*theBox,*(theBox+1)) );
			break;
		case 2:
			if (rightDis < 80 && (rightDis < leftDis))
				saveHandImage(handImage, frameID,RIGHT, Point(*theBox,*(theBox+1)) );
            else if (leftDis < 80 && (rightDis > leftDis))
                saveHandImage(handImage, frameID, LEFT, Point(*theBox,*(theBox+1)) );
			break;
        case 3:
			if ( (theBox[3]-theBox[1])*(theBox[2]-theBox[0]) > 1200)
            {
				if (abs(leftDis - rightDis) < 150)
					saveHandImage(handImage, frameID, BOTH, Point(*theBox,*(theBox+1)) );
            }
            else
            {
				if (rightDis < 80 && leftDis < 80)
					saveHandImage(handImage, frameID, BOTH, Point(*theBox,*(theBox+1)) );
				else if (rightDis < 80 && (rightDis < leftDis))
					saveHandImage(handImage, frameID,RIGHT, Point(*theBox,*(theBox+1)) );
                else if (leftDis < 80 && (rightDis > leftDis))
					saveHandImage(handImage, frameID, LEFT, Point(*theBox,*(theBox+1)) );
			}
			break;
         default:
            break;
		}
		cvReleaseImage(&handImage);
	}

	if(nMaxRect==2)
	{
		int HandStatus = -1;
        //HandStatus=0; left 0 ,right 1
        //HandStatus=1; left 1; right=0;
        //HandStatus=2; together
        if (theCent[0] < theCent[2])
			HandStatus = 0;
        else
			HandStatus = 1;

        double leftDis0 = -999;
        double rightDis0 = -999;

        double leftDis1 = -999;
        double rightDis1 = -999;

		CvPoint2D32f connexePoint[2];
		connexePoint[0].x=theCent[0];
		connexePoint[0].y=theCent[1];
		connexePoint[1].x=theCent[2];
		connexePoint[1].y=theCent[3];

        leftDis0 = calDist(leftPoint, connexePoint[0]);
        rightDis0 = calDist(rightPoint, connexePoint[0]);
        leftDis1 = calDist(leftPoint, connexePoint[1]);
        rightDis1 = calDist(rightPoint, connexePoint[1]);

		int handsDis=calDist(connexePoint[0],connexePoint[1]);

        CvPoint2D32f headPoint;
		headPoint.x=vSkeletonData[frameID%bufferSize]._2dPoint[3].x;
		headPoint.y=vSkeletonData[frameID%bufferSize]._2dPoint[3].y;
        double head0 = calDist(headPoint, connexePoint[0]);
        double head1 = calDist(headPoint, connexePoint[1]);

		IplImage* handImage[2];
		handImage[0]=getConnextImage(grayImage, binaryImage,theBox,1,frameID);
		handImage[1]=getConnextImage(grayImage, binaryImage,theBox+4,2,frameID);

        double RegionThresold = 200;
		switch (handStatusKey)
        {
        case 0:
			//if (HandStatus == 1)
   //         {
			//	if (leftDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, LEFT);
   //             if (rightDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, RIGHT);
			//}
   //         else if (HandStatus == 0)
   //         {
			//	if (leftDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, LEFT);
			//	if (rightDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, RIGHT);
   //         }
			//break;
		case 1:
			//if (HandStatus == 1)
   //         {
			//	if (leftDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, LEFT);
   //             if (rightDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, RIGHT);
   //         }
   //         else if (HandStatus == 0)
   //         {
			//	if (leftDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, LEFT);
			//	if (rightDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, RIGHT);
			//}
   //             break;
		case 2:
			//if (HandStatus == 1)
   //         {
			//	if (leftDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, LEFT);
   //             if (rightDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, RIGHT);
   //         }
   //         else if (HandStatus == 0)
   //         {
			//	if (leftDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, LEFT);
			//	if (rightDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, RIGHT);
   //         }
   //         break;
			if(handsDis>100)
			{
				if(leftDis0<leftDis1 && rightDis0>rightDis1)
				{
					if (rightDis1 < 80)
						saveHandImage(handImage[1], frameID, RIGHT, Point(*(theBox+4),*(theBox+5)) );
					if (leftDis0 < 80 )
						saveHandImage(handImage[0], frameID, LEFT, Point(*theBox,*(theBox+1)) );
					break;
				}
				if(leftDis0>leftDis1 && rightDis0<rightDis1)
				{
					if (rightDis0 < 80)
						saveHandImage(handImage[0], frameID, RIGHT, Point(*theBox,*(theBox+1)) );
					if (leftDis1 < 80 )
						saveHandImage(handImage[1], frameID, LEFT, Point(*(theBox+4),*(theBox+5)) );
					break;
				}
				if(leftDis0<leftDis1 && rightDis0<rightDis1)
				{
					if (rightDis0 < leftDis0 && rightDis0<80)
					{
						saveHandImage(handImage[0], frameID, RIGHT, Point(*theBox,*(theBox+1)) );
						if(leftDis1 < 80)
							saveHandImage(handImage[1], frameID, LEFT, Point(*(theBox+4),*(theBox+5)) );
					}
					if (rightDis0 > leftDis0 && leftDis0<80)
					{
						saveHandImage(handImage[0], frameID, LEFT, Point(*theBox,*(theBox+1)) );
						if(rightDis1 < 80)
							saveHandImage(handImage[1], frameID, RIGHT, Point(*(theBox+4),*(theBox+5)) );
					}
					break;
				}
				if(leftDis0>leftDis1 && rightDis0>rightDis1)
				{
					if (rightDis1 < leftDis1 && rightDis1<80)
					{
						saveHandImage(handImage[1], frameID, RIGHT, Point(*(theBox+4),*(theBox+5)) );
						if(leftDis0 < 80)
							saveHandImage(handImage[0], frameID, LEFT, Point(*theBox,*(theBox+1)) );
					}
					if (rightDis1 > leftDis1 && leftDis1<80)
					{
						saveHandImage(handImage[1], frameID, LEFT, Point(*(theBox+4),*(theBox+5)) );
						if(rightDis0 < 80)
							saveHandImage(handImage[0], frameID, RIGHT, Point(*theBox,*(theBox+1)) );
					}
					break;
				}
			}
			else
			{
				if (HandStatus == 1)
	            {
					if (leftDis1 < RegionThresold)
						saveHandImage(handImage[1], frameID, LEFT, Point(*(theBox+4),*(theBox+5)) );
	                if (rightDis0 < RegionThresold)
						saveHandImage(handImage[0], frameID, RIGHT, Point(*theBox,*(theBox+1)) );
	            }
	            else if (HandStatus == 0)
	            {
					if (leftDis0 < RegionThresold)
						saveHandImage(handImage[0], frameID, LEFT, Point(*theBox,*(theBox+1)) );
					if (rightDis1 < RegionThresold)
						saveHandImage(handImage[1], frameID, RIGHT, Point(*(theBox+4),*(theBox+5)) );
	            }
			}
			break;
		case 3:
			if (head0 < 20 || head1 < 20)
            {
				if ((leftDis0 - leftDis1 > 8) && (rightDis0 - rightDis1 > 8))
                {
					//Left and Right together
					// Image<Bgr, Byte> imageResultColorB = getCropColorHandsFromWhole(colorImage, convexs[1].Image, convexs[1].CenterPoint, convexs[1].Rect);
					saveHandImage(handImage[1], frameID, BOTH, Point(*(theBox+4),*(theBox+5)) );
					break;
				}
				if ((leftDis1 - leftDis0 > 8 && (rightDis1 - rightDis0 > 8)))
                {
					//Left and Right together
					// Image<Bgr, Byte> imageResultColorB = getCropColorHandsFromWhole(colorImage, convexs[0].Image, convexs[0].CenterPoint, convexs[0].Rect);
					saveHandImage(handImage[0], frameID, BOTH, Point(*theBox,*(theBox+1)) );
					break;
				}
			}	
            else
            {
				if (HandStatus == 1)
                {
					//right hand is closer than left hand
					//1-> left hand, 0->right hand
                    if (leftDis1 < RegionThresold)
						saveHandImage(handImage[1], frameID, LEFT, Point(*(theBox+4),*(theBox+5)) );
					if (rightDis0 < RegionThresold)
						saveHandImage(handImage[0], frameID, RIGHT, Point(*theBox,*(theBox+1)) );
				}
                else if (HandStatus == 0)
                {
					//right hand is closer than left hand
                    //1-> left hand, 0->right hand
                    if (leftDis1 < RegionThresold)
						saveHandImage(handImage[0], frameID, LEFT, Point(*theBox,*(theBox+1)) );
					if (rightDis0 < RegionThresold)
						saveHandImage(handImage[1], frameID, RIGHT, Point(*(theBox+4),*(theBox+5)) );
				}
			}
			break;
		default:
            break;
        }
	}
}

void S_Keyframe::getHandImage(int frameID,IplImage* binaryImage, int& nMaxRect, int* theCent, int* theBox,IplImage* grayImage,FrameSegment& fs)
{
	if(nMaxRect <1 || nMaxRect>2)
		return;

	double rightDis=calDist(rightHands[0],rightHands[frameID]);
	double leftDis=calDist(leftHands[0],leftHands[frameID]);

	CvPoint2D32f rightPoint,leftPoint,connexePoint;
	rightPoint.x=vSkeletonData[frameID%bufferSize]._2dPoint[11].x;
	rightPoint.y=vSkeletonData[frameID%bufferSize]._2dPoint[11].y;

	leftPoint.x=vSkeletonData[frameID%bufferSize]._2dPoint[7].x;
	leftPoint.y=vSkeletonData[frameID%bufferSize]._2dPoint[7].y;

	bool LFlag = false; //true: too close
    bool RFlag = false;

    if (leftDis < 80)
		LFlag = true;
    else 
		LFlag = false;

    if (rightDis < 80)
		RFlag = true;
    else
		RFlag = false;

	int handStatusKey = -1;
    // 0 -> Left&Right stay at the original region
    // 1 -> Left stay at the orignal region,Right leave
    // 2 -> Right stays at the orignal region,Left leave
    // 3 -> Left and Right leaves

    if (LFlag && RFlag)
		handStatusKey = 0;
	else if (LFlag && !RFlag)
		handStatusKey = 1;
	else if (!LFlag && RFlag)
		handStatusKey = 2;
	else if (!LFlag && !RFlag)
		handStatusKey = 3;

	if(nMaxRect==1)
	{		
		CvPoint2D32f connexePoint;
		connexePoint.x=theCent[0];
		connexePoint.y=theCent[1];

		rightDis=calDist(rightPoint,connexePoint);
		leftDis=calDist(leftPoint,connexePoint);
		IplImage* handImage=getConnextImage(grayImage, binaryImage,theBox,1,frameID);
		switch(handStatusKey)
		{
		case 0:
			if (rightDis < 80 && (rightDis < leftDis))
				saveHandImage(handImage,frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
            else if (leftDis < 80 && (rightDis > leftDis))
                saveHandImage(handImage,frameID, LEFT, Point(*theBox,*(theBox+1)) ,fs);
			break;
        case 1:
            if (rightDis < 80 && (rightDis < leftDis))
				saveHandImage(handImage,frameID, RIGHT, Point(*theBox,*(theBox+1)) ,fs);
            else if (leftDis < 80 && (rightDis > leftDis))
				saveHandImage(handImage,frameID, LEFT, Point(*theBox,*(theBox+1)),fs );
			break;
		case 2:
			if (rightDis < 80 && (rightDis < leftDis))
				saveHandImage(handImage,frameID, RIGHT, Point(*theBox,*(theBox+1)) ,fs);
            else if (leftDis < 80 && (rightDis > leftDis))
                saveHandImage(handImage,frameID, LEFT, Point(*theBox,*(theBox+1)) ,fs);
			break;
        case 3:
			if ( (theBox[3]-theBox[1])*(theBox[2]-theBox[0]) > 1200)
            {
				if (abs(leftDis - rightDis) < 150)
					saveHandImage(handImage,frameID, BOTH, Point(*theBox,*(theBox+1)),fs );
            }
            else
            {
				if (rightDis < 80 && leftDis < 80)
					saveHandImage(handImage,frameID, BOTH, Point(*theBox,*(theBox+1)),fs );
				else if (rightDis < 80 && (rightDis < leftDis))
					saveHandImage(handImage,frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
                else if (leftDis < 80 && (rightDis > leftDis))
					saveHandImage(handImage,frameID, LEFT, Point(*theBox,*(theBox+1)),fs );
			}
			break;
         default:
            break;
		}
		cvReleaseImage(&handImage);
	}

	if(nMaxRect==2)
	{
		int HandStatus = -1;
        //HandStatus=0; left 0 ,right 1
        //HandStatus=1; left 1; right=0;
        //HandStatus=2; together
        if (theCent[0] < theCent[2])
			HandStatus = 0;
        else
			HandStatus = 1;

        double leftDis0 = -999;
        double rightDis0 = -999;

        double leftDis1 = -999;
        double rightDis1 = -999;

		CvPoint2D32f connexePoint[2];
		connexePoint[0].x=theCent[0];
		connexePoint[0].y=theCent[1];
		connexePoint[1].x=theCent[2];
		connexePoint[1].y=theCent[3];

        leftDis0 = calDist(leftPoint, connexePoint[0]);
        rightDis0 = calDist(rightPoint, connexePoint[0]);
        leftDis1 = calDist(leftPoint, connexePoint[1]);
        rightDis1 = calDist(rightPoint, connexePoint[1]);

		int handsDis=calDist(connexePoint[0],connexePoint[1]);

        CvPoint2D32f headPoint;
		headPoint.x=vSkeletonData[frameID%bufferSize]._2dPoint[3].x;
		headPoint.y=vSkeletonData[frameID%bufferSize]._2dPoint[3].y;
        double head0 = calDist(headPoint, connexePoint[0]);
        double head1 = calDist(headPoint, connexePoint[1]);

		IplImage* handImage[2];
		handImage[0]=getConnextImage(grayImage, binaryImage,theBox,1,frameID);
		handImage[1]=getConnextImage(grayImage, binaryImage,theBox+4,2,frameID);

        double RegionThresold = 200;
		switch (handStatusKey)
        {
        case 0:
			//if (HandStatus == 1)
   //         {
			//	if (leftDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, LEFT);
   //             if (rightDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, RIGHT);
			//}
   //         else if (HandStatus == 0)
   //         {
			//	if (leftDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, LEFT);
			//	if (rightDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, RIGHT);
   //         }
			//break;
		case 1:
			//if (HandStatus == 1)
   //         {
			//	if (leftDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, LEFT);
   //             if (rightDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, RIGHT);
   //         }
   //         else if (HandStatus == 0)
   //         {
			//	if (leftDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, LEFT);
			//	if (rightDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, RIGHT);
			//}
   //             break;
		case 2:
			//if (HandStatus == 1)
   //         {
			//	if (leftDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, LEFT);
   //             if (rightDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, RIGHT);
   //         }
   //         else if (HandStatus == 0)
   //         {
			//	if (leftDis0 < RegionThresold)
			//		saveHandImage(handImage[0], frameID, LEFT);
			//	if (rightDis1 < RegionThresold)
			//		saveHandImage(handImage[1], frameID, RIGHT);
   //         }
   //         break;
			if(handsDis>100)
			{
				if(leftDis0<leftDis1 && rightDis0>rightDis1)
				{
					if (rightDis1 < 80)
						saveHandImage(handImage[1],frameID, RIGHT, Point(*(theBox+4),*(theBox+5)),fs );
					if (leftDis0 < 80 )
						saveHandImage(handImage[0],frameID, LEFT, Point(*theBox,*(theBox+1)),fs );
					break;
				}
				if(leftDis0>leftDis1 && rightDis0<rightDis1)
				{
					if (rightDis0 < 80)
						saveHandImage(handImage[0],frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
					if (leftDis1 < 80 )
						saveHandImage(handImage[1],frameID, LEFT, Point(*(theBox+4),*(theBox+5)),fs );
					break;
				}
				if(leftDis0<leftDis1 && rightDis0<rightDis1)
				{
					if (rightDis0 < leftDis0 && rightDis0<80)
					{
						saveHandImage(handImage[0],frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
						if(leftDis1 < 80)
							saveHandImage(handImage[1],frameID, LEFT, Point(*(theBox+4),*(theBox+5)),fs );
					}
					if (rightDis0 > leftDis0 && leftDis0<80)
					{
						saveHandImage(handImage[0],frameID, LEFT, Point(*theBox,*(theBox+1)),fs );
						if(rightDis1 < 80)
							saveHandImage(handImage[1],frameID, RIGHT, Point(*(theBox+4),*(theBox+5)),fs );
					}
					break;
				}
				if(leftDis0>leftDis1 && rightDis0>rightDis1)
				{
					if (rightDis1 < leftDis1 && rightDis1<80)
					{
						saveHandImage(handImage[1],frameID, RIGHT, Point(*(theBox+4),*(theBox+5)),fs );
						if(leftDis0 < 80)
							saveHandImage(handImage[0],frameID, LEFT, Point(*theBox,*(theBox+1)),fs );
					}
					if (rightDis1 > leftDis1 && leftDis1<80)
					{
						saveHandImage(handImage[1],frameID, LEFT, Point(*(theBox+4),*(theBox+5)),fs );
						if(rightDis0 < 80)
							saveHandImage(handImage[0],frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
					}
					break;
				}
			}
			else
			{
				if (HandStatus == 1)
	            {
					if (leftDis1 < RegionThresold)
						saveHandImage(handImage[1],frameID, LEFT, Point(*(theBox+4),*(theBox+5)),fs );
	                if (rightDis0 < RegionThresold)
						saveHandImage(handImage[0],frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
	            }
	            else if (HandStatus == 0)
	            {
					if (leftDis0 < RegionThresold)
						saveHandImage(handImage[0],frameID, LEFT, Point(*theBox,*(theBox+1)) ,fs);
					if (rightDis1 < RegionThresold)
						saveHandImage(handImage[1],frameID, RIGHT, Point(*(theBox+4),*(theBox+5)),fs );
	            }
			}
			break;
		case 3:
			if (head0 < 20 || head1 < 20)
            {
				if ((leftDis0 - leftDis1 > 8) && (rightDis0 - rightDis1 > 8))
                {
					//Left and Right together
					// Image<Bgr, Byte> imageResultColorB = getCropColorHandsFromWhole(colorImage, convexs[1].Image, convexs[1].CenterPoint, convexs[1].Rect);
					saveHandImage(handImage[1],frameID, BOTH, Point(*(theBox+4),*(theBox+5)) ,fs);
					break;
				}
				if ((leftDis1 - leftDis0 > 8 && (rightDis1 - rightDis0 > 8)))
                {
					//Left and Right together
					// Image<Bgr, Byte> imageResultColorB = getCropColorHandsFromWhole(colorImage, convexs[0].Image, convexs[0].CenterPoint, convexs[0].Rect);
					saveHandImage(handImage[0],frameID, BOTH, Point(*theBox,*(theBox+1)) ,fs);
					break;
				}
			}	
            else
            {
				if (HandStatus == 1)
                {
					//right hand is closer than left hand
					//1-> left hand, 0->right hand
                    if (leftDis1 < RegionThresold)
						saveHandImage(handImage[1],frameID, LEFT, Point(*(theBox+4),*(theBox+5)) ,fs);
					if (rightDis0 < RegionThresold)
						saveHandImage(handImage[0],frameID, RIGHT, Point(*theBox,*(theBox+1)),fs );
				}
                else if (HandStatus == 0)
                {
					//right hand is closer than left hand
                    //1-> left hand, 0->right hand
                    if (leftDis1 < RegionThresold)
						saveHandImage(handImage[0],frameID, LEFT, Point(*theBox,*(theBox+1)),fs );
					if (rightDis0 < RegionThresold)
						saveHandImage(handImage[1],frameID, RIGHT, Point(*(theBox+4),*(theBox+5)),fs );
				}
			}
			break;
		default:
            break;
        }
	}
}

void S_Keyframe::saveHandImage(IplImage* handImage, int frameID, HANDTYPE hType, Point coor)
{
	//if(hType == LEFT)
	//{
	//	if(leftFlag[frameID]==false)
	//		return;
	//	if( leftLastFrame==-1 )
	//	{
	//		LeftHandSegment.keyframe_num++;
	//		LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]=0;
	//		LeftHandSegment.keyframe_ID[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]]=frameID;
	//		LeftHandSegment.keyframe_coor[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]*2]=coor.x;
	//		LeftHandSegment.keyframe_coor[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]*2+1]=coor.y;
	//		LeftHandSegment.keyframe_pic[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]]
	//																						=cvCloneImage(handImage);
	//		LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]++;
	//		leftLastFrame=frameID;
	//		return;
	//	}
	//	for(int i=leftLastFrame;i<frameID;i++)
	//		if(leftFlag[i]==false)
	//			{
	//				LeftHandSegment.keyframe_num++;
	//				LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]=0;
	//				LeftHandSegment.keyframe_pic[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]]
	//																								=cvCloneImage(handImage);
	//				LeftHandSegment.keyframe_ID[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]]=frameID;
	//				LeftHandSegment.keyframe_coor[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]*2]=coor.x;
	//				LeftHandSegment.keyframe_coor[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]*2+1]=coor.y;
	//				LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]++;
	//				leftLastFrame=frameID;
	//				return;
	//			}
	//	LeftHandSegment.keyframe_ID[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]]=frameID;
	//	LeftHandSegment.keyframe_coor[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]*2]=coor.x;
	//	LeftHandSegment.keyframe_coor[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]*2+1]=coor.y;
	//	LeftHandSegment.keyframe_pic[LeftHandSegment.keyframe_num-1][LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]]
	//																						=cvCloneImage(handImage);
	//	LeftHandSegment.keyframe_no[LeftHandSegment.keyframe_num-1]++;
	//	leftLastFrame=frameID;
	//	return;
	//}

	//if(hType == RIGHT)
	//{
	//	if(rightFlag[frameID]==false)
	//		return;
	//	if( rightLastFrame==-1 )
	//	{
	//		RightHandSegment.keyframe_num++;
	//		RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]=0;
	//		RightHandSegment.keyframe_ID[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]]=frameID;
	//		RightHandSegment.keyframe_coor[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]*2]=coor.x;
	//		RightHandSegment.keyframe_coor[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]*2+1]=coor.y;
	//		RightHandSegment.keyframe_pic[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]]
	//																						=cvCloneImage(handImage);
	//		RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]++;
	//		rightLastFrame=frameID;
	//		return;
	//	}
	//	for(int i=rightLastFrame;i<frameID;i++)
	//		if(rightFlag[i]==false)
	//		{
	//			RightHandSegment.keyframe_num++;
	//			RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]=0;
	//			RightHandSegment.keyframe_ID[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]]=frameID;
	//			RightHandSegment.keyframe_coor[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]*2]=coor.x;
	//			RightHandSegment.keyframe_coor[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]*2+1]=coor.y;
	//			RightHandSegment.keyframe_pic[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]]
	//																							=cvCloneImage(handImage);
	//			RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]++;
	//			rightLastFrame=frameID;
	//			return;
	//		}
	//	RightHandSegment.keyframe_ID[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]]=frameID;
	//	RightHandSegment.keyframe_coor[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]*2]=coor.x;
	//	RightHandSegment.keyframe_coor[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]*2+1]=coor.y;
	//	RightHandSegment.keyframe_pic[RightHandSegment.keyframe_num-1][RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]]
	//																						=cvCloneImage(handImage);
	//	RightHandSegment.keyframe_no[RightHandSegment.keyframe_num-1]++;
	//	rightLastFrame=frameID;
	//	return;
	//}

	//if(hType == BOTH)
	//{
	//	bothFlag[frameID]=true;
	//	leftFlag[frameID]=false;
	//	rightFlag[frameID]=false;
	//	if( bothLastFrame==-1 )
	//	{
	//		BothHandSegment.keyframe_num++;
	//		BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]=0;
	//		BothHandSegment.keyframe_ID[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]]=frameID;
	//		BothHandSegment.keyframe_coor[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]*2]=coor.x;
	//		BothHandSegment.keyframe_coor[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]*2+1]=coor.y;
	//		BothHandSegment.keyframe_pic[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]]
	//																						=cvCloneImage(handImage);
	//		BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]++;
	//		bothLastFrame=frameID;
	//		return;
	//	}
	//	for(int i=bothLastFrame;i<frameID;i++)
	//		if(bothFlag[i]==false)
	//		{
	//			BothHandSegment.keyframe_num++;
	//			BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]=0;
	//			BothHandSegment.keyframe_ID[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]]=frameID;
	//			BothHandSegment.keyframe_coor[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]*2]=coor.x;
	//			BothHandSegment.keyframe_coor[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]*2+1]=coor.y;
	//			BothHandSegment.keyframe_pic[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]]
	//																							=cvCloneImage(handImage);
	//			BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]++;
	//			bothLastFrame=frameID;
	//			return;
	//		}
	//	BothHandSegment.keyframe_ID[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]]=frameID;
	//	BothHandSegment.keyframe_coor[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]*2]=coor.x;
	//	BothHandSegment.keyframe_coor[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]*2+1]=coor.y;
	//	BothHandSegment.keyframe_pic[BothHandSegment.keyframe_num-1][BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]]
	//																						=cvCloneImage(handImage);
	//	BothHandSegment.keyframe_no[BothHandSegment.keyframe_num-1]++;
	//	bothLastFrame=frameID;
	//	return;
	//}
}

void S_Keyframe::saveHandImage(IplImage* handImage, int frameID, HANDTYPE hType, Point coor,FrameSegment& fs)
{
	if(hType == LEFT)
	{
		if(leftFlag[frameID%bufferSize]==false)
			return;

		fs.left=true;
		fs.leftCoor[0]=coor.x;
		fs.leftCoor[1]=coor.y;
		fs.leftImage=cvCloneImage(handImage);
		return;
	}

	if(hType == RIGHT)
	{
		if(rightFlag[frameID%bufferSize]==false)
			return;
		fs.right=true;
		fs.rightCoor[0]=coor.x;
		fs.rightCoor[1]=coor.y;
		fs.rightImage=cvCloneImage(handImage);
		return;
	}

	if(hType == BOTH)
	{
		bothFlag[frameID%bufferSize]=true;
		leftFlag[frameID%bufferSize]=false;
		rightFlag[frameID%bufferSize]=false;
		fs.both=true;
		fs.bothCoor[0]=coor.x;
		fs.bothCoor[1]=coor.y;
		fs.bothImage=cvCloneImage(handImage);
		return;
	}
}

IplImage* S_Keyframe::getConnextImage(IplImage* grayImage,IplImage* binaryImage,int* theBox,int connIdx,int frameID)
{
	CvRect ConnexeRect = cvRect(theBox[0],theBox[1],theBox[2]-theBox[0],theBox[3]-theBox[1]);
	cvSetImageROI(vColorData[frameID%bufferSize],ConnexeRect);
	IplImage* colorOutput=cvCreateImage(cvSize(ConnexeRect.width,ConnexeRect.height),vColorData[frameID%bufferSize]->depth,vColorData[frameID%bufferSize]->nChannels);
	cvCopy(vColorData[frameID%bufferSize],colorOutput);
	cvResetImageROI(vColorData[frameID%bufferSize]);
	cvCvtColor(colorOutput,colorOutput,CV_BGR2HSV);
	IplImage* equalizeImageH=cvCreateImage(cvSize(ConnexeRect.width,ConnexeRect.height),vColorData[frameID%bufferSize]->depth,1);
	IplImage* equalizeImageS=cvCreateImage(cvSize(ConnexeRect.width,ConnexeRect.height),vColorData[frameID%bufferSize]->depth,1);
	IplImage* equalizeImageV=cvCreateImage(cvSize(ConnexeRect.width,ConnexeRect.height),vColorData[frameID%bufferSize]->depth,1);
	cvSplit(colorOutput,equalizeImageH,equalizeImageS,equalizeImageV,NULL);
	cvEqualizeHist(equalizeImageV,equalizeImageV);
	cvMerge(equalizeImageH,equalizeImageS,equalizeImageV,NULL,colorOutput);
	//cvSetImageROI(grayImage,ConnexeRect);
	IplImage* outputImage = cvCreateImage(cvSize(ConnexeRect.width,ConnexeRect.height),grayImage->depth,grayImage->nChannels);
//	IplImage* resizeImage = cvCreateImage(cvSize(64,64),grayImage->depth,grayImage->nChannels);
	//cvCopy(grayImage,outputImage);
	//cvResetImageROI(grayImage);
	cvCvtColor(colorOutput,colorOutput,CV_HSV2BGR);
	cvCvtColor(colorOutput,outputImage,CV_RGB2GRAY);
	cvReleaseImage(&colorOutput);
	cvReleaseImage(&equalizeImageH);
	cvReleaseImage(&equalizeImageS);
	cvReleaseImage(&equalizeImageV);

	//for(int i=0;i<outputImage->height;i++)
	//{
	//	for(int j=0;j<outputImage->width;j++)
	//	{
	//		int s=((u8*)(binaryImage->imageData + binaryImage->widthStep*(i+theBox[1])))[j+theBox[0]];
	//		if(s!=connIdx)
	//			((u8*)(outputImage->imageData + outputImage->widthStep*i))[j]=0;
	//	}
	//}
//	cvResize(outputImage,resizeImage);
//	cvSmooth(resizeImage,resizeImage);
//	cvReleaseImage(&outputImage);
//	return resizeImage;
	return outputImage;
}

void S_Keyframe::getSkinColorModel(IplImage* faceImage)
{
	int i, j, k;
	int width = faceImage->width;
	int height = faceImage->height;

    //脸部颜色聚类，采用Cr，Cb颜色空间

	IplImage *m_pResultImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,faceImage->nChannels);
	cvCvtColor(faceImage,m_pResultImage,CV_BGR2RGB);
	CvMat *bufferMotionRegionImg = cvCreateMat(width*height,2,CV_32FC1);

	unsigned char r, g, b;
	for(j=0; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			k = j*width + i;

			r = (m_pResultImage->imageData + m_pResultImage->widthStep * j)[i*3+0];
			g = (m_pResultImage->imageData + m_pResultImage->widthStep * j)[i*3+1];
			b = (m_pResultImage->imageData + m_pResultImage->widthStep * j)[i*3+2];

			 ((float*)(bufferMotionRegionImg->data.ptr + bufferMotionRegionImg->step*k))[0]=
                0.5000*r-0.4187*g-0.0813*b+128; // cr
            ((float*)(bufferMotionRegionImg->data.ptr + bufferMotionRegionImg->step*k))[1]=
                -0.1687*r-0.3313*g+0.5000*b+128; // cb
		}
	}

	int maxIndex = 0;
	CvMat *clusterIndex = cvCreateMat(width*height,1,CV_32SC1);
	cvKMeans2(bufferMotionRegionImg,3,clusterIndex,
			cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 0.01 ));

	int indexCount[3] = {0,0,0};
	
	//获得最多类别标签，然后对最多类别求Cr、Cb均值和方差
	for(j=0; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			k = j*width + i;
			indexCount[clusterIndex->data.i[k]]++;
		}
	}

	for(i=0; i<3; i++)
	{
		if(indexCount[i] > indexCount[maxIndex])
			maxIndex = i;
	}
	int count = indexCount[maxIndex];

	double dCr = 0;
	double dCb = 0;
	double sumCr = 0;
	double sumCb = 0;
	int index = 0;
	float tempCr;
	float tempCb;
	for(j=0; j<height; j++)
	{
		for(i=0; i<width; i++)
		{
			k = j * width + i;
			index = clusterIndex->data.i[k];
			if(index == maxIndex )
			{
				tempCr = ((float*)(bufferMotionRegionImg->data.ptr + bufferMotionRegionImg->step*k))[0];
				tempCb = ((float*)(bufferMotionRegionImg->data.ptr + bufferMotionRegionImg->step*k))[1];
				sumCr += tempCr;
                sumCb += tempCb;
        	}
		}
	}
	skinColorModel.mean_cr = sumCr/count;
	skinColorModel.mean_cb = sumCb/count;

	for(j = 0; j < height; j ++)
        for(i = 0; i < width; i ++)
        {
            k = j * width + i;
            index = clusterIndex->data.i[k];
            if (index == maxIndex)
            {
				tempCr = ((float*)(bufferMotionRegionImg->data.ptr + bufferMotionRegionImg->step*k))[0];
				tempCb = ((float*)(bufferMotionRegionImg->data.ptr + bufferMotionRegionImg->step*k))[1];
                dCr += (tempCr - skinColorModel.mean_cr)*(tempCr - skinColorModel.mean_cr);//Cr
                dCb += (tempCb - skinColorModel.mean_cb)*(tempCb - skinColorModel.mean_cb);//Cb
            }
        }
    skinColorModel.d_cr=sqrt(dCr/count);
    skinColorModel.d_cb=sqrt(dCb/count);

	cvReleaseMat(&bufferMotionRegionImg);
	cvReleaseMat(&clusterIndex);
	cvReleaseImage(&m_pResultImage);
}

void S_Keyframe::saveHandStruct(string filePath)
{
	//const char* filePathChar=filePath.c_str();
	//mkdir(filePathChar);
	//if(RightHandSegment.keyframe_num > 0)
	//{
	//	for(int i=0;i<RightHandSegment.keyframe_num;i++)
	//	{
	//		for(int j=0;j<RightHandSegment.keyframe_no[i];j++)
	//		{
	//			char filePathName[100];
	//			char fileNameNum[100];
	//			sprintf(fileNameNum,"\\Right%d_%d.jpg",i,j);
	//			strcpy(filePathName,filePathChar);
	//			strcat(filePathName,fileNameNum);
	//			cvSaveImage(filePathName,RightHandSegment.keyframe_pic[i][j]);
	//		}
	//	}
	//}

	//if(LeftHandSegment.keyframe_num > 0)
	//{
	//	for(int i=0;i<LeftHandSegment.keyframe_num;i++)
	//	{
	//		for(int j=0;j<LeftHandSegment.keyframe_no[i];j++)
	//		{
	//			char filePathName[100];
	//			char fileNameNum[100];
	//			sprintf(fileNameNum,"\\Left%d_%d.jpg",i,j);
	//			strcpy(filePathName,filePathChar);
	//			strcat(filePathName,fileNameNum);
	//			cvSaveImage(filePathName,LeftHandSegment.keyframe_pic[i][j]);
	//		}
	//	}
	//}

	//if(BothHandSegment.keyframe_num > 0)
	//{
	//	for(int i=0;i<BothHandSegment.keyframe_num;i++)
	//	{
	//		for(int j=0;j<BothHandSegment.keyframe_no[i];j++)
	//		{
	//			char filePathName[100];
	//			char fileNameNum[100];
	//			sprintf(fileNameNum,"\\Both%d_%d.jpg",i,j);
	//			strcpy(filePathName,filePathChar);
	//			strcat(filePathName,fileNameNum);
	//			cvSaveImage(filePathName,BothHandSegment.keyframe_pic[i][j]);
	//		}
	//	}
	//}
}

void S_Keyframe::saveHandStruct(string filePath,HandSegment right,HandSegment left,HandSegment both)
{
	const char* filePathChar=filePath.c_str();
	mkdir(filePathChar);
	if(right.keyframe_num > 0)
	{
		for(int i=0;i<right.keyframe_num;i++)
		{
			for(int j=0;j<right.keyframe_no[i];j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\Right%d_%d.jpg",i,j);
				strcpy(filePathName,filePathChar);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,right.keyframe_pic[i][j]);
			}
		}
	}

	if(left.keyframe_num > 0)
	{
		for(int i=0;i<left.keyframe_num;i++)
		{
			for(int j=0;j<left.keyframe_no[i];j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\Left%d_%d.jpg",i,j);
				strcpy(filePathName,filePathChar);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,left.keyframe_pic[i][j]);
			}
		}
	}

	if(both.keyframe_num > 0)
	{
		for(int i=0;i<both.keyframe_num;i++)
		{
			for(int j=0;j<both.keyframe_no[i];j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\Both%d_%d.jpg",i,j);
				strcpy(filePathName,filePathChar);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,both.keyframe_pic[i][j]);
			}
		}
	}
}

void S_Keyframe::saveKeyFrameSegment(string filePath)
{
	const char* filePathChar=filePath.c_str();
	mkdir(filePathChar);

	for(int i=0;i<v_kfSegment.size();i++)
	{
		if(v_kfSegment[i].BothLabel==1)
		{
			for(int j=0;j<v_kfSegment[i].BothNum;j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\%03d_%03d_Both_%03d.jpg",i,v_kfSegment[i].BothID[j],j);
				strcpy(filePathName,filePathChar);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,v_kfSegment[i].BothImages[j]);
			}
			continue;
		}
		if(v_kfSegment[i].LeftLabel==1)
		{
			for(int j=0;j<v_kfSegment[i].LeftNum;j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\%03d_%03d_Left_%03d.jpg",i,v_kfSegment[i].LeftID[j],j);
				strcpy(filePathName,filePathChar);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,v_kfSegment[i].LeftImages[j]);
			}
		}
		if(v_kfSegment[i].RightLabel==1)
		{
			for(int j=0;j<v_kfSegment[i].RightNum;j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\%03d_%03d_Right_%03d.jpg",i,v_kfSegment[i].RightID[j],j);
				strcpy(filePathName,filePathChar);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,v_kfSegment[i].RightImages[j]);
			}
		}
	}
}

void S_Keyframe::saveKeyFrameSegment(string filePath,vector<KeyFrameSegment> vKeyFrame)
{
	const char* filePathChar=filePath.c_str();
	mkdir(filePathChar);
	char bothPath[100];
	char leftPath[100];
	char rightPath[100];
	strcpy(bothPath,filePathChar);
	strcat(bothPath,"\\KeyPosture");
	mkdir(bothPath);
	strcat(bothPath,"\\Both");
	mkdir(bothPath);
	strcpy(rightPath,filePathChar);
	strcat(rightPath,"\\KeyPosture");
	strcat(rightPath,"\\Right");
	mkdir(rightPath);
	strcpy(leftPath,filePathChar);
	strcat(leftPath,"\\KeyPosture");
	strcat(leftPath,"\\Left");
	mkdir(leftPath);

	int bothNum=0;
	int rightNum=0;
	int leftNum=0;
	int bothIdx[1000];
	int rightIdx[1000];
	int leftIdx[1000];

	int i,j;
	for(i=0;i<vKeyFrame.size();i++)
	{
		if(vKeyFrame[i].BothLabel==1)
		{
			for(j=0;j<vKeyFrame[i].BothNum;j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\%d.jpg",vKeyFrame[i].BothID[j]);
				strcpy(filePathName,bothPath);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,vKeyFrame[i].BothImages[j]);

			}
			bothIdx[bothNum*2]=vKeyFrame[i].BeginFrameID;
			bothIdx[bothNum*2+1]=vKeyFrame[i].EndFrameID;
			bothNum++;
			continue;
		}
		if(vKeyFrame[i].LeftLabel==1)
		{
			for(j=0;j<vKeyFrame[i].LeftNum;j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\%d.jpg",vKeyFrame[i].LeftID[j]);
				strcpy(filePathName,leftPath);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,vKeyFrame[i].LeftImages[j]);

			}
			leftIdx[leftNum*2]=vKeyFrame[i].BeginFrameID;
			leftIdx[leftNum*2+1]=vKeyFrame[i].EndFrameID;
			leftNum++;
		}
		if(vKeyFrame[i].RightLabel==1)
		{
			for(j=0;j<vKeyFrame[i].RightNum;j++)
			{
				char filePathName[100];
				char fileNameNum[100];
				sprintf(fileNameNum,"\\%d.jpg",vKeyFrame[i].RightID[j]);
				strcpy(filePathName,rightPath);
				strcat(filePathName,fileNameNum);
				cvSaveImage(filePathName,vKeyFrame[i].RightImages[j]);

			}
			rightIdx[rightNum*2]=vKeyFrame[i].BeginFrameID;
			rightIdx[rightNum*2+1]=vKeyFrame[i].EndFrameID;
			rightNum++;
		}
	}

	int maxIndex = 0;
	for (i=0;i<rightNum;i++)
	{
		if (rightIdx[i*2]>maxIndex)
		{
			maxIndex = rightIdx[i*2];
		}
	}
	for(i=0;i<leftNum;i++)
	{
		if (leftIdx[i*2]>maxIndex)
		{
			maxIndex = leftIdx[i*2];
		}
	}
	for(i=0;i<bothNum;i++)
	{
		if (bothIdx[i*2]>maxIndex)
		{
			maxIndex = bothIdx[i*2];
		}
	}

	int alll = 0;
	for (i=0; i<maxIndex+1; i++)
	{
		bool overrid = false;
		for (j=0; j<rightNum; j++)
		{
			if (rightIdx[j*2] == i && !overrid)
			{
				alll++;
				overrid = true;
				break;
			}

		}
		for (j=0; j<leftNum;j++)
		{
			if (leftIdx[j*2] == i && !overrid)
			{
				alll++;
				overrid = true;
				break;
			}

		}
		for (j=0; j<bothNum; j++)
		{
			if (bothIdx[j*2] == i && !overrid)
			{
				alll++;
				overrid = true;
				break;
			}

		}
	}



	fstream idxFile_right;
	strcat(rightPath,"\\right.txt");
	idxFile_right.open(rightPath,ios::out);

	fstream idxFile_left;
	strcat(leftPath,"\\left.txt");
	idxFile_left.open(leftPath,ios::out);

	fstream idxFile_both;
	strcat(bothPath,"\\both.txt");
	idxFile_both.open(bothPath,ios::out);

	idxFile_right<<alll<<endl;
	idxFile_left<<alll<<endl;
	idxFile_both<<alll<<endl;


	for (i=0; i<maxIndex+1; i++)
	{
		bool overrid = false;
		for (j=0; j<rightNum; j++)
		{
			if (rightIdx[j*2] == i && !overrid)
			{
				idxFile_right<<rightIdx[j*2]<<"	"<<rightIdx[j*2+1]<<endl;
				idxFile_left<<rightIdx[j*2]<<"	"<<rightIdx[j*2+1]<<endl;
				idxFile_both<<rightIdx[j*2]<<"	"<<rightIdx[j*2+1]<<endl;
				overrid = true;
				break;
			}

		}
		for (j=0; j<leftNum;j++)
		{
			if (leftIdx[j*2] == i && !overrid)
			{
				idxFile_right<<leftIdx[j*2]<<"	"<<leftIdx[j*2+1]<<endl;
				idxFile_left<<leftIdx[j*2]<<"	"<<leftIdx[j*2+1]<<endl;
				idxFile_both<<leftIdx[j*2]<<"	"<<leftIdx[j*2+1]<<endl;
				overrid = true;
				break;
			}

		}
		for (j=0; j<bothNum; j++)
		{
			if (bothIdx[j*2] == i && !overrid)
			{
				idxFile_right<<bothIdx[j*2]<<"	"<<bothIdx[j*2+1]<<endl;
				idxFile_left<<bothIdx[j*2]<<"	"<<bothIdx[j*2+1]<<endl;
				idxFile_both<<bothIdx[j*2]<<"	"<<bothIdx[j*2+1]<<endl;
				overrid = true;
				break;
			}

		}

	}



	idxFile_right.close();
	idxFile_left.close();
	idxFile_both.close();

// 	const char* filePathChar=filePath.c_str();
// 	mkdir(filePathChar);
// 
// 	for(int i=0;i<vKeyFrame.size();i++)
// 	{
// 		if(vKeyFrame[i].BothLabel==1)
// 		{
// 			for(int j=0;j<vKeyFrame[i].BothNum;j++)
// 			{
// 				char filePathName[100];
// 				char fileNameNum[100];
// 				sprintf(fileNameNum,"\\%03d_%03d_Both_%03d.jpg",i,vKeyFrame[i].BothID[j],j);
// 				strcpy(filePathName,filePathChar);
// 				strcat(filePathName,fileNameNum);
// 				cvSaveImage(filePathName,vKeyFrame[i].BothImages[j]);
// 			}
// 			continue;
// 		}
// 		if(vKeyFrame[i].LeftLabel==1)
// 		{
// 			for(int j=0;j<vKeyFrame[i].LeftNum;j++)
// 			{
// 				char filePathName[100];
// 				char fileNameNum[100];
// 				sprintf(fileNameNum,"\\%03d_%03d_Left_%03d.jpg",i,vKeyFrame[i].LeftID[j],j);
// 				strcpy(filePathName,filePathChar);
// 				strcat(filePathName,fileNameNum);
// 				cvSaveImage(filePathName,vKeyFrame[i].LeftImages[j]);
// 			}
// 		}
// 		if(vKeyFrame[i].RightLabel==1)
// 		{
// 			for(int j=0;j<vKeyFrame[i].RightNum;j++)
// 			{
// 				char filePathName[100];
// 				char fileNameNum[100];
// 				sprintf(fileNameNum,"\\%03d_%03d_Right_%03d.jpg",i,vKeyFrame[i].RightID[j],j);
// 				strcpy(filePathName,filePathChar);
// 				strcat(filePathName,fileNameNum);
// 				cvSaveImage(filePathName,vKeyFrame[i].RightImages[j]);
// 			}
// 		}
// 	}
}

bool S_Keyframe::mergeFragment(KeyFrameSegment &Fragment)
{
	//if(Fragment.BothLabel==1)
	//{
	//	if(Fragment.LeftLabel==1 && Fragment.RightLabel==1)
	//	{
	//		//if(coverProp())
	//	}
	//}
	//if(Fragment.LeftLabel==1 && Fragment.RightLabel==1)
	//{

	//}
	//if( (Fragment.LeftLabel==1 && Fragment.RightLabel!=1) || (Fragment.LeftLabel!=1 && Fragment.RightLabel==1) )
	//{

	//}
	return false;
}

double S_Keyframe::coverProp(Rect leftRect,Rect rightRect, Rect bothRect)
{
	double total=leftRect.width*leftRect.height+rightRect.width*rightRect.height;
	double interPix=0;
	int minX,maxX,minY,maxY,height,width;
	
	if(leftRect.x < bothRect.x)
		minX=leftRect.x;
	else
		minX=bothRect.x;
	if( (leftRect.x+leftRect.width-1) < (bothRect.x+bothRect.width-1) )
		maxX=bothRect.x+bothRect.width-1;
	else
		maxX=leftRect.x+leftRect.width-1;
	width=(leftRect.width+bothRect.width)-(maxX-minX+1);

	if(leftRect.y < bothRect.y)
		minY=leftRect.y;
	else
		minY=bothRect.y;
	if( (leftRect.y+leftRect.height-1) < (bothRect.y+bothRect.height-1) )
		maxY=bothRect.y+bothRect.height-1;
	else
		maxY=leftRect.y+leftRect.height-1;
	height=(leftRect.height+bothRect.height)-(maxY-minY+1);
	interPix+=width*height;

	if(rightRect.x < bothRect.x)
		minX=rightRect.x;
	else
		minX=bothRect.x;
	if( (rightRect.x+rightRect.width-1) < (bothRect.x+bothRect.width-1) )
		maxX=bothRect.x+bothRect.width-1;
	else
		maxX=rightRect.x+rightRect.width-1;
	width=(rightRect.width+bothRect.width)-(maxX-minX+1);

	if(rightRect.y < bothRect.y)
		minY=rightRect.y;
	else
		minY=bothRect.y;
	if( (rightRect.y+rightRect.height-1) < (bothRect.y+bothRect.height-1) )
		maxY=bothRect.y+bothRect.height-1;
	else
		maxY=rightRect.y+rightRect.height-1;
	height=(rightRect.height+bothRect.height)-(maxY-minY+1);
	interPix+=width*height;

	return interPix/total;
}

bool S_Keyframe::isSkinColorModel(IplImage* colorImage,int i,int j)
{
	return isSkinColor(colorImage,i,j);
	unsigned char r,g,b;
	double cr,cb;
	r = (colorImage->imageData + colorImage->widthStep*i)[j*3+2];
	g = (colorImage->imageData + colorImage->widthStep*i)[j*3+1];
	b = (colorImage->imageData + colorImage->widthStep*i)[j*3+0];
	cr = 0.5000*r - 0.4187*g - 0.0813*b + 128; // cr
	cb = -0.1687*r - 0.3313*g + 0.5000*b + 128; // cb
	if( fabs(skinColorModel.mean_cr-cr) < 3*skinColorModel.d_cr &&
		fabs(skinColorModel.mean_cb-cb) < 3*skinColorModel.d_cb )
		return true;
	return false;
}

void S_Keyframe::releaseMemory()
{
	for(int i=0;i<bufferSize && i<framesNum;i++)
	{
		cvReleaseImage(&(vColorData[i]));
		vDepthData[i].release();
	}

	bufferBeginPointer=0;
	bufferEndPointer=0;
	framesNum=0;
	processedFrameNum=0;
	//int maxFrameNum=300;
	getDataOver=false;
	segmentOver=false;
	rightLastFrame=-1;
	leftLastFrame=-1;
	bothLastFrame=-1;

	leftHands.clear();
	rightHands.clear();

	for(int i=0;i<bufferSize;i++)
	{
		leftFlag[i]=false;
		rightFlag[i]=false;
		bothFlag[i]=false;
	}

	//for(int i=0;i<v_kfSegment.size();i++)
	//{
	//	releaseFragment(v_kfSegment[i]);
	//}

	//v_kfSegment.clear();
}

void S_Keyframe::releaseFragment(KeyFrameSegment& fs)
{
	for(int j=0;j<fs.BothNum;j++)
	{
		cvReleaseImage(&(fs.BothImages[j]));
	}

	for(int j=0;j<fs.LeftNum;j++)
	{
		cvReleaseImage(&(fs.LeftImages[j]));
	}

	for(int j=0;j<fs.RightNum;j++)
	{
		cvReleaseImage(&(fs.RightImages[j]));
	}

	delete[] fs.BothCoor;
	delete[] fs.BothID;
	delete[] fs.BothImages;

	delete[] fs.LeftCoor;
	delete[] fs.LeftID;
	delete[] fs.LeftImages;

	delete[] fs.RightCoor;
	delete[] fs.RightID;
	delete[] fs.RightImages;
}

bool S_Keyframe::isThereFragment()
{
	EnterCriticalSection(&csFragmentData);
	if(bufferBeginPointer==bufferEndPointer)
	{
		LeaveCriticalSection(&csFragmentData);
		return false;
	}
	LeaveCriticalSection(&csFragmentData);
	return true;
}

bool S_Keyframe::processOver()
{
	EnterCriticalSection(&csFragmentData);
	if(processedFrameNum==framesNum)
	{
		LeaveCriticalSection(&csFragmentData);
		return true;
	}
	LeaveCriticalSection(&csFragmentData);
	return false;
}

void S_Keyframe::putInBuffer(KeyFrameSegment& fs)
{
	if(fs.EndFrameID-fs.BeginFrameID <=0)
	{
		releaseFragment(fs);
		return;
	}
	if(fs.LeftLabel==false && fs.RightLabel==true)
	{
		double min=100;
		for(int i=0;i<fs.RightNum;i++)
		{
			if(rightVel[fs.RightID[i]%bufferSize] < min )
			min =rightVel[fs.RightID[i]%bufferSize];
		}
		if(min>5 && fs.RightNum<5)
		{
//			cout<<"release  "<<fs.RightID[0]<<" "<<fs.RightNum<<endl;
			releaseFragment(fs);
			return;
		}
		if(min>10)
		{
			releaseFragment(fs);
			return;
		}
	}

	if(fs.RightLabel==false && fs.LeftLabel==true)
	{
		double min=100;
		for(int i=0;i<fs.LeftNum;i++)
		{
			if(leftVel[fs.LeftID[i]%bufferSize]<min)
			min=leftVel[fs.LeftID[i]%bufferSize];
		}
		if(min>5 && fs.LeftNum<5)
		{
//			cout<<"release  "<<fs.LeftID[0]<<" "<<fs.LeftNum<<endl;
			releaseFragment(fs);
			return;
		}
		if(min>10)
		{
			releaseFragment(fs);
			return;
		}
	}

	EnterCriticalSection(&csFragmentData);
	bufferEndPointer++;
	bufferEndPointer=bufferEndPointer%bufferSize;
	if( bufferEndPointer == bufferBeginPointer )
	{
		bufferBeginPointer++;
		bufferBeginPointer=bufferBeginPointer%bufferSize;
		//releaseFragment(kfSegmentBuffer[bufferBeginPointer]);
	}
	kfSegmentBuffer[bufferEndPointer]=fs;
	//releaseFragment(fs);
	//cout<<"put in fragment:"<<bufferEndPointer<<":	"<<fs.BeginFrameID<<"	"<<fs.EndFrameID<<endl;
	LeaveCriticalSection(&csFragmentData);
}

KeyFrameSegment S_Keyframe::getFragment()
{
	EnterCriticalSection(&csFragmentData);
	bufferBeginPointer++;
	bufferBeginPointer=bufferBeginPointer%bufferSize;
	LeaveCriticalSection(&csFragmentData);
//	cout<<"get fragment:"<<bufferBeginPointer<<endl;
	return kfSegmentBuffer[bufferBeginPointer];
	
}

bool S_Keyframe::isSameFragment(KeyFrameSegment Fragment1,KeyFrameSegment Fragment2,int index)
{
	IplImage * ori_img=cvCreateImage( cvSize(SIZE,SIZE),8,1);;//原始图像
	IplImage * avg_img=cvCreateImage( cvSize(SIZE,SIZE),8,1);//均值图像
	uchar *pp;
	uchar *qq;
	int Img_sum[SIZE][SIZE];//用于图像求和
	memset( Img_sum,0,sizeof(Img_sum) );//先清零
	int k;
	int m,n;
	if(index==1)
	{
		if(Fragment1.RightNum==0 || Fragment2.RightNum==0)
			return false;
		for(k=0;k<Fragment1.RightNum;k++)
		{
			cvResize(Fragment1.RightImages[k],ori_img);
			for(m=0;m<SIZE;m++)
			{
				pp=(uchar *)(ori_img->imageData+m*ori_img->widthStep);
				for(n=0;n<SIZE;n++)
				{
					Img_sum[m][n]+=pp[n*ori_img->nChannels];
				}
			}
		}
		for(m=0;m<SIZE;m++)
		{
			qq=(uchar *)(avg_img->imageData+m*avg_img->widthStep);
			for(n=0;n<SIZE;n++)
			{
				Img_sum[m][n]=Img_sum[m][n]/Fragment1.RightNum;
				qq[n*avg_img->nChannels]=Img_sum[m][n];
			}
		}
		double mindistance=1.0*0xffffff;
		double imgtemp;
		int index1;
		for(k=0;k<Fragment1.RightNum;k++)
		{
			cvResize(Fragment1.RightImages[k],ori_img);
			imgtemp=Img_distance( ori_img,avg_img );
			if(imgtemp<mindistance)
			{
				mindistance=imgtemp;
				index1=k;
			}
		}
		memset( Img_sum,0,sizeof(Img_sum) );
		for(k=0;k<Fragment2.RightNum;k++)
		{
			cvResize(Fragment2.RightImages[k],ori_img);
			for(m=0;m<SIZE;m++)
			{
				pp=(uchar *)(ori_img->imageData+m*ori_img->widthStep);
				for(n=0;n<SIZE;n++)
				{
					Img_sum[m][n]+=pp[n*ori_img->nChannels];
				}
			}
		}
		for(m=0;m<SIZE;m++)
		{
			qq=(uchar *)(avg_img->imageData+m*avg_img->widthStep);
			for(n=0;n<SIZE;n++)
			{
				Img_sum[m][n]=Img_sum[m][n]/Fragment2.RightNum;
				qq[n*avg_img->nChannels]=Img_sum[m][n];
			}
		}
		mindistance=1.0*0xffffff;
		imgtemp;
		int index2;
		for(k=0;k<Fragment2.RightNum;k++)
		{
			cvResize(Fragment2.RightImages[k],ori_img);
			imgtemp=Img_distance( ori_img,avg_img );
			if(imgtemp<mindistance)
			{
				mindistance=imgtemp;
				index2=k;
			}
		}
		cvResize(Fragment1.RightImages[index1],ori_img);
		cvResize(Fragment2.RightImages[index2],avg_img);

		if(coverProp( ori_img,avg_img )>0.9)
		{
			cvReleaseImage(&avg_img);
			cvReleaseImage(&ori_img);
			return true;
		}
		else
		{
			cvReleaseImage(&avg_img);
			cvReleaseImage(&ori_img);
			return false;
		}
	}

	if(index==2)
	{
		if(Fragment1.LeftNum==0 || Fragment2.LeftNum==0)
			return false;
		for(k=0;k<Fragment1.LeftNum;k++)
		{
			cvResize(Fragment1.LeftImages[k],ori_img);
			for(m=0;m<SIZE;m++)
			{
				pp=(uchar *)(ori_img->imageData+m*ori_img->widthStep);
				for(n=0;n<SIZE;n++)
				{
					Img_sum[m][n]+=pp[n*ori_img->nChannels];
				}
			}
		}
		for(m=0;m<SIZE;m++)
		{
			qq=(uchar *)(avg_img->imageData+m*avg_img->widthStep);
			for(n=0;n<SIZE;n++)
			{
				Img_sum[m][n]=Img_sum[m][n]/Fragment1.LeftNum;
				qq[n*avg_img->nChannels]=Img_sum[m][n];
			}
		}
		double mindistance=1.0*0xffffff;
		double imgtemp;
		int index1;
		for(k=0;k<Fragment1.LeftNum;k++)
		{
			cvResize(Fragment1.LeftImages[k],ori_img);
			imgtemp=Img_distance( ori_img,avg_img );
			if(imgtemp<mindistance)
			{
				mindistance=imgtemp;
				index1=k;
			}
		}
		memset( Img_sum,0,sizeof(Img_sum) );
		for(k=0;k<Fragment2.LeftNum;k++)
		{
			cvResize(Fragment2.LeftImages[k],ori_img);
			for(m=0;m<SIZE;m++)
			{
				pp=(uchar *)(ori_img->imageData+m*ori_img->widthStep);
				for(n=0;n<SIZE;n++)
				{
					Img_sum[m][n]+=pp[n*ori_img->nChannels];
				}
			}
		}
		for(m=0;m<SIZE;m++)
		{
			qq=(uchar *)(avg_img->imageData+m*avg_img->widthStep);
			for(n=0;n<SIZE;n++)
			{
				Img_sum[m][n]=Img_sum[m][n]/Fragment2.LeftNum;
				qq[n*avg_img->nChannels]=Img_sum[m][n];
			}
		}
		mindistance=1.0*0xffffff;
		imgtemp;
		int index2;
		for(k=0;k<Fragment2.LeftNum;k++)
		{
			cvResize(Fragment2.LeftImages[k],ori_img);
			imgtemp=Img_distance( ori_img,avg_img );
			if(imgtemp<mindistance)
			{
				mindistance=imgtemp;
				index2=k;
			}
		}
		cvResize(Fragment1.LeftImages[index1],ori_img);
		cvResize(Fragment2.LeftImages[index2],avg_img);

		if(coverProp( ori_img,avg_img )>0.9)
		{
			cvReleaseImage(&avg_img);
			cvReleaseImage(&ori_img);
			return true;
		}
		else
		{
			cvReleaseImage(&avg_img);
			cvReleaseImage(&ori_img);
			return false;
		}
	}
	
	if(index==3)
	{
		if(Fragment1.BothNum==0 || Fragment2.BothNum==0)
			return false;
		for(k=0;k<Fragment1.BothNum;k++)
		{
			cvResize(Fragment1.BothImages[k],ori_img);
			for(m=0;m<SIZE;m++)
			{
				pp=(uchar *)(ori_img->imageData+m*ori_img->widthStep);
				for(n=0;n<SIZE;n++)
				{
					Img_sum[m][n]+=pp[n*ori_img->nChannels];
				}
			}
		}
		for(m=0;m<SIZE;m++)
		{
			qq=(uchar *)(avg_img->imageData+m*avg_img->widthStep);
			for(n=0;n<SIZE;n++)
			{
				Img_sum[m][n]=Img_sum[m][n]/Fragment1.BothNum;
				qq[n*avg_img->nChannels]=Img_sum[m][n];
			}
		}
		double mindistance=1.0*0xffffff;
		double imgtemp;
		int index1;
		for(k=0;k<Fragment1.BothNum;k++)
		{
			cvResize(Fragment1.BothImages[k],ori_img);
			imgtemp=Img_distance( ori_img,avg_img );
			if(imgtemp<mindistance)
			{
				mindistance=imgtemp;
				index1=k;
			}
		}
		memset( Img_sum,0,sizeof(Img_sum) );
		for(k=0;k<Fragment2.BothNum;k++)
		{
			cvResize(Fragment2.BothImages[k],ori_img);
			for(m=0;m<SIZE;m++)
			{
				pp=(uchar *)(ori_img->imageData+m*ori_img->widthStep);
				for(n=0;n<SIZE;n++)
				{
					Img_sum[m][n]+=pp[n*ori_img->nChannels];
				}
			}
		}
		for(m=0;m<SIZE;m++)
		{
			qq=(uchar *)(avg_img->imageData+m*avg_img->widthStep);
			for(n=0;n<SIZE;n++)
			{
				Img_sum[m][n]=Img_sum[m][n]/Fragment2.BothNum;
				qq[n*avg_img->nChannels]=Img_sum[m][n];
			}
		}
		mindistance=1.0*0xffffff;
		imgtemp;
		int index2;
		for(k=0;k<Fragment2.BothNum;k++)
		{
			cvResize(Fragment2.BothImages[k],ori_img);
			imgtemp=Img_distance( ori_img,avg_img );
			if(imgtemp<mindistance)
			{
				mindistance=imgtemp;
				index2=k;
			}
		}
		cvResize(Fragment1.BothImages[index1],ori_img);
		cvResize(Fragment2.BothImages[index2],avg_img);

		if(coverProp( ori_img,avg_img )>0.8)
		{
			cvReleaseImage(&avg_img);
			cvReleaseImage(&ori_img);
			return true;
		}
		else
		{
			cvReleaseImage(&avg_img);
			cvReleaseImage(&ori_img);
			return false;
		}
	}

	return false;
}

double S_Keyframe::Img_distance(IplImage *dst1,IplImage *dst2)//返回两个图像的欧几里得距离
{
	int i,j;
	uchar *ptr1;
	uchar *ptr2;

	double result=0.0;////////////
	for(i=0;i<dst1->height;i++)
	{
		ptr1=(uchar *)(dst1->imageData+i*dst1->widthStep);
		ptr2=(uchar *)(dst2->imageData+i*dst2->widthStep);

		for(j=0;j<dst1->width;j++)
			result+=(ptr1[j*dst1->nChannels]-ptr2[j*dst2->nChannels])*(ptr1[j*dst1->nChannels]-ptr2[j*dst2->nChannels]);
	}
	result=sqrt(result);
	return result;
}

void S_Keyframe::mergeFragment(KeyFrameSegment& firstFragment,KeyFrameSegment& secondFragment)
{
	if((firstFragment.LeftLabel==0 && firstFragment.RightLabel==1)
		&&(secondFragment.LeftLabel==0 && secondFragment.RightLabel==1)
		&& secondFragment.BeginFrameID-firstFragment.EndFrameID<=2)
	{
		if(isSameFragment(firstFragment,secondFragment,1))
		{
			for(int i=0;i<secondFragment.RightNum;i++)
			{
				firstFragment.RightCoor[firstFragment.RightNum*2]=secondFragment.RightCoor[i*2];
				firstFragment.RightCoor[firstFragment.RightNum*2+1]=secondFragment.RightCoor[i*2+1];
				firstFragment.RightID[firstFragment.RightNum]=secondFragment.RightID[i];
				firstFragment.RightImages[firstFragment.RightNum]=cvCloneImage(secondFragment.RightImages[i]);
				firstFragment.EndFrameID=secondFragment.RightID[i];
				firstFragment.RightNum++;
			}
			releaseFragment(secondFragment);
			return;
		}
	}
	if((firstFragment.LeftLabel==1 && firstFragment.RightLabel==0)
		&&(secondFragment.LeftLabel==1 && secondFragment.RightLabel==0)
		&& secondFragment.BeginFrameID-firstFragment.EndFrameID<=2)
	{
		if(isSameFragment(firstFragment,secondFragment,2))
		{
			for(int i=0;i<secondFragment.LeftNum;i++)
			{
				firstFragment.LeftCoor[firstFragment.LeftNum*2]=secondFragment.LeftCoor[i*2];
				firstFragment.LeftCoor[firstFragment.LeftNum*2+1]=secondFragment.LeftCoor[i*2+1];
				firstFragment.LeftID[firstFragment.LeftNum]=secondFragment.LeftID[i];
				firstFragment.LeftImages[firstFragment.LeftNum]=cvCloneImage(secondFragment.LeftImages[i]);
				firstFragment.EndFrameID=secondFragment.LeftID[i];
				firstFragment.LeftNum++;
			}
			releaseFragment(secondFragment);
			return;
		}
	}
	if((firstFragment.BothLabel==1)
		&&(secondFragment.BothLabel==1)
		&& secondFragment.BeginFrameID-firstFragment.EndFrameID<=2)
	{
		if(isSameFragment(firstFragment,secondFragment,3))
		{
			for(int i=0;i<secondFragment.BothNum;i++)
			{
				firstFragment.BothCoor[firstFragment.BothNum*2]=secondFragment.BothCoor[i*2];
				firstFragment.BothCoor[firstFragment.BothNum*2+1]=secondFragment.BothCoor[i*2+1];
				firstFragment.BothID[firstFragment.BothNum]=secondFragment.BothID[i];
				firstFragment.BothImages[firstFragment.BothNum]=cvCloneImage(secondFragment.BothImages[i]);
				firstFragment.EndFrameID=secondFragment.BothID[i];
				firstFragment.BothNum++;
			}
			releaseFragment(secondFragment);
			return;
		}
	}
	if((firstFragment.LeftLabel==0 && firstFragment.RightLabel==1)||(firstFragment.LeftLabel==1 && firstFragment.RightLabel==0)) 
	{
		if(secondFragment.BothLabel==1 || (secondFragment.LeftLabel==1 && secondFragment.RightLabel==1))
		{
			if(firstFragment.RightNum+firstFragment.LeftNum<=5)
			{
				releaseFragment(firstFragment);
				firstFragment=secondFragment;
				return ;
			}
		}
	}
	if((secondFragment.LeftLabel==0 && secondFragment.RightLabel==1)||(secondFragment.LeftLabel==1 && secondFragment.RightLabel==0)) 
	{
		if(firstFragment.BothLabel==1 || (firstFragment.LeftLabel==1 && firstFragment.RightLabel==1))
		{
			if(secondFragment.RightNum+secondFragment.LeftNum<=5)
			{
				releaseFragment(secondFragment);
				return;
			}
		}
	}
	if(firstFragment.BothLabel==1 && (secondFragment.LeftLabel==1 && secondFragment.RightLabel==1))
	{
		if(firstFragment.BothNum==0 || secondFragment.LeftNum==0 || secondFragment.RightNum==0)
			return;
		if(secondFragment.RightNum<5)
		{
			releaseFragment(secondFragment);
			return;
		}
		//Rect leftRect=cvRect(secondFragment.LeftCoor[(secondFragment.LeftNum-1)*2],secondFragment.LeftCoor[(secondFragment.LeftNum-1)*2+1],secondFragment.LeftImages[secondFragment.LeftNum-1]->width,secondFragment.LeftImages[secondFragment.LeftNum-1]->height);
		//Rect rightRect=cvRect(secondFragment.RightCoor[(secondFragment.RightNum-1)*2],secondFragment.RightCoor[(secondFragment.RightNum-1)*2+1],secondFragment.RightImages[secondFragment.RightNum-1]->width,secondFragment.RightImages[secondFragment.RightNum-1]->height);
		//Rect bothRect=cvRect(firstFragment.BothCoor[(firstFragment.BothNum-1)*2],firstFragment.BothCoor[(firstFragment.BothNum-1)*2+1],firstFragment.BothImages[firstFragment.BothNum-1]->width,firstFragment.BothImages[firstFragment.BothNum-1]->height);
		//if(  coverProp(leftRect,rightRect,bothRect)>0.85 )
		//{
		//	int j=0;
		//	for(int i=0;i<secondFragment.LeftNum;i++)
		//	{
		//		while(j<secondFragment.RightNum)
		//		{
		//			if(secondFragment.RightID[j]>secondFragment.LeftID[i])
		//				break;
		//			if(secondFragment.RightID[j]==secondFragment.LeftID[i])
		//			{
		//				leftRect=cvRect(secondFragment.LeftCoor[i*2],secondFragment.LeftCoor[i*2+1],secondFragment.LeftImages[i]->width,secondFragment.LeftImages[i]->height);
		//				rightRect=cvRect(secondFragment.RightCoor[j*2],secondFragment.RightCoor[j*2+1],secondFragment.RightImages[j]->width,secondFragment.RightImages[j]->height);
		//				if(  coverProp(leftRect,rightRect,bothRect)>0.85 )
		//				{
		//					Point bothPoint;
		//					IplImage* tempImage=mergeHands(secondFragment.LeftImages[i],cvPoint(secondFragment.LeftCoor[i*2],secondFragment.LeftCoor[i*2+1]),
		//						secondFragment.RightImages[j],cvPoint(secondFragment.RightCoor[j*2],secondFragment.RightCoor[j*2+1]),bothPoint);
		//					firstFragment.BothCoor[firstFragment.BothNum*2]=bothPoint.x;
		//					firstFragment.BothCoor[firstFragment.BothNum*2+1]=bothPoint.y;
		//					firstFragment.BothID[firstFragment.BothNum]=secondFragment.LeftID[i];
		//					firstFragment.BothImages[firstFragment.BothNum]=tempImage;
		//					firstFragment.EndFrameID=secondFragment.LeftID[i];
		//					firstFragment.BothNum++;
		//				}
		//			}
		//				j++;
		//		}
		//	}
		//	releaseFragment(secondFragment);
		//	return;
		//}
	}

	if(secondFragment.BothLabel==1 && (firstFragment.LeftLabel==1 && firstFragment.RightLabel==1))
	{
		if(secondFragment.BothNum==0 || firstFragment.LeftNum==0 || firstFragment.RightNum==0)
			return;
		if(firstFragment.RightNum<5)
		{
			releaseFragment(firstFragment);
			firstFragment=secondFragment;
			return;
		}
		//Rect leftRect=cvRect(firstFragment.LeftCoor[(firstFragment.LeftNum-1)*2],firstFragment.LeftCoor[(firstFragment.LeftNum-1)*2+1],firstFragment.LeftImages[firstFragment.LeftNum-1]->width,firstFragment.LeftImages[firstFragment.LeftNum-1]->height);
		//Rect rightRect=cvRect(firstFragment.RightCoor[(firstFragment.RightNum-1)*2],firstFragment.RightCoor[(firstFragment.RightNum-1)*2+1],firstFragment.RightImages[firstFragment.RightNum-1]->width,firstFragment.RightImages[firstFragment.RightNum-1]->height);
		//Rect bothRect=cvRect(secondFragment.BothCoor[(secondFragment.BothNum-1)*2],secondFragment.BothCoor[(secondFragment.BothNum-1)*2+1],secondFragment.BothImages[secondFragment.BothNum-1]->width,secondFragment.BothImages[secondFragment.BothNum-1]->height);
		//if(  coverProp(leftRect,rightRect,bothRect)>0.85 )
		//{
		//	KeyFrameSegment tempFragment;
		//	CreateNewFragment(tempFragment);
		//	tempFragment.BeginFrameID=firstFragment.BeginFrameID;
		//	tempFragment.BothLabel=1;
		//	tempFragment.LeftLabel=0;
		//	tempFragment.RightLabel=0;
		//	tempFragment.BothNum=0;
		//	int j=0;
		//	for(int i=0;i<firstFragment.LeftNum;i++)
		//	{
		//		while(j<firstFragment.RightNum)
		//		{
		//			if(firstFragment.RightID[j]>firstFragment.LeftID[i])
		//				break;
		//			if(firstFragment.RightID[j]==firstFragment.LeftID[i])
		//			{
		//				leftRect=cvRect(firstFragment.LeftCoor[i*2],firstFragment.LeftCoor[i*2+1],firstFragment.LeftImages[i]->width,firstFragment.LeftImages[i]->height);
		//				rightRect=cvRect(firstFragment.RightCoor[j*2],firstFragment.RightCoor[j*2+1],firstFragment.RightImages[j]->width,firstFragment.RightImages[j]->height);
		//				if(  coverProp(leftRect,rightRect,bothRect)>0.85 )
		//				{
		//					Point bothPoint;
		//					IplImage* tempImage=mergeHands(firstFragment.LeftImages[i],cvPoint(firstFragment.LeftCoor[i*2],firstFragment.LeftCoor[i*2+1]),
		//						firstFragment.RightImages[j],cvPoint(firstFragment.RightCoor[j*2],firstFragment.RightCoor[j*2+1]),bothPoint);
		//					tempFragment.BothCoor[tempFragment.BothNum*2]=bothPoint.x;
		//					tempFragment.BothCoor[tempFragment.BothNum*2+1]=bothPoint.y;
		//					tempFragment.BothID[tempFragment.BothNum]=firstFragment.LeftID[i];
		//					tempFragment.BothImages[tempFragment.BothNum]=tempImage;
		//					tempFragment.EndFrameID=firstFragment.LeftID[i];
		//					tempFragment.BothNum++;
		//				}

		//			}
		//			j++;
		//		}
		//	}
		//	for(int i=0;i<secondFragment.BothNum;i++)
		//	{
		//		tempFragment.BothCoor[tempFragment.BothNum*2]=secondFragment.BothCoor[i*2];
		//		tempFragment.BothCoor[tempFragment.BothNum*2+1]=secondFragment.BothCoor[i*2+1];
		//		tempFragment.BothID[tempFragment.BothNum]=secondFragment.BothID[i];
		//		tempFragment.BothImages[tempFragment.BothNum]=cvCloneImage(secondFragment.BothImages[i]);
		//		tempFragment.EndFrameID=secondFragment.BothID[i];
		//		tempFragment.BothNum++;
		//	}
		//	releaseFragment(firstFragment);
		//	releaseFragment(secondFragment);
		//	firstFragment=tempFragment;
		//	return;
		//}

	}

	putInBuffer(firstFragment);
	firstFragment=secondFragment;
}

IplImage* S_Keyframe::mergeHands(IplImage* leftHand,Point leftPoint,IplImage* rightHand,Point rightPoint,Point& bothPoint)
{
	int left,top,right,bottom;
	if(rightPoint.x < leftPoint.x)
		left=rightPoint.x;
	else
		left=leftPoint.x;
	if(rightPoint.y < leftPoint.y)
		top=rightPoint.y;
	else
		top=leftPoint.y;
	if(rightPoint.x+rightHand->width > leftPoint.x+leftHand->width)
		right=rightPoint.x+rightHand->width-1;
	else
		right=leftPoint.x+leftHand->width-1;
	if(rightPoint.y+rightHand->height > leftPoint.y+leftHand->height)
		bottom=rightPoint.y+rightHand->height-1;
	else
		bottom=leftPoint.y+leftHand->height-1;
	IplImage* tempImage=cvCreateImage(cvSize(right-left+1,bottom-top+1),8,1);
	cvSetZero(tempImage);
	cvSetImageROI(tempImage,cvRect(rightPoint.x-left,rightPoint.y-top,rightHand->width,rightHand->height));
	cvCopy(rightHand,tempImage);
	cvResetImageROI(tempImage);
	cvSetImageROI(tempImage,cvRect(leftPoint.x-left,leftPoint.y-top,leftHand->width,leftHand->height));
	cvCopy(leftHand,tempImage);
	cvResetImageROI(tempImage);
	bothPoint.x=left;
	bothPoint.y=top;
	if(rightPoint.x < leftPoint.x)
		left=leftPoint.x;
	else
		left=rightPoint.x;
	if(rightPoint.y < leftPoint.y)
		top=leftPoint.y;
	else
		top=rightPoint.y;
	if(rightPoint.x+rightHand->width > leftPoint.x+leftHand->width)
		right=leftPoint.x+leftHand->width;
	else
		right=rightPoint.x+rightHand->width;
	if(rightPoint.y+rightHand->height > leftPoint.y+leftHand->height)
		bottom=leftPoint.y+leftHand->height;
	else
		bottom=rightPoint.y+rightHand->height;
	if( (right>left) && (bottom>top)  )
	{
		for(int i=top;i<bottom;i++)
		{
			for(int j=left;j<right;j++)
			{
				u8 left= ((char*)(leftHand->imageData + leftHand->widthStep*(i-leftPoint.y)))[j-leftPoint.x] ;
				u8 right= ((char*)(rightHand->imageData + rightHand->widthStep*(i-rightPoint.y)))[j-rightPoint.x] ;
				if(left > right)
					((char*)(tempImage->imageData + tempImage->widthStep*(i-bothPoint.y)))[j-bothPoint.x] = left;
				else
					((char*)(tempImage->imageData + tempImage->widthStep*(i-bothPoint.y)))[j-bothPoint.x] = right;
			}
		}
	}
	return tempImage;
}

void S_Keyframe::setHeightThres(int height)
{
	myHeightThres=height;
}

void S_Keyframe::setGetDataOver(bool dataOver)
{
	EnterCriticalSection(&csMessageData);
	getDataOver=dataOver;
	LeaveCriticalSection(&csMessageData);
}

bool S_Keyframe::getGetDataOver()
{
	bool dataOver;
//	cout<<"enterCriticalSection"<<endl;
	EnterCriticalSection(&csMessageData);
	dataOver = getDataOver;
	LeaveCriticalSection(&csMessageData);
	return dataOver;
}

bool S_Keyframe::getSegmentOver()
{
	bool segOver;
	EnterCriticalSection(&csMessageData);
	segOver = segmentOver;
	LeaveCriticalSection(&csMessageData);
	return segOver;
}

double S_Keyframe::coverProp(IplImage* image1,IplImage* image2)
{
	int unionPix=0;
	int intescPix=0;
	int i,j;
	uchar *ptr1;
	uchar *ptr2;

	double result=0.0;////////////
	for(i=0;i<image1->height;i++)
	{
		ptr1=(uchar *)(image1->imageData+i*image1->widthStep);
		ptr2=(uchar *)(image2->imageData+i*image2->widthStep);

		for(j=0;j<image1->width;j++)
		{
			if(ptr1[j*image1->nChannels] > 10 || ptr2[j*image2->nChannels]>10)
				unionPix++;
			if(ptr1[j*image1->nChannels] > 10 && ptr2[j*image2->nChannels]>10)
				intescPix++;
		}
	}
	return (double)intescPix/(double)unionPix;
}