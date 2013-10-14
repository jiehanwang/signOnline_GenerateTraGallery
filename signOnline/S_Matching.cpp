#include "StdAfx.h"
#include "S_Matching.h"

S_Keyframe      myKeyframe;

S_CMatching::S_CMatching(void)
{
	M_keyFeatureStream = new vector<double> *[MaxKeyFrameNumber];
	for (int m=0; m<MaxKeyFrameNumber; m++)
	{
		M_keyFeatureStream[m] = new vector<double> [LRB];
	}
	featureToBeCom = new vector<double> *[LRB];
	for (int l=0; l<LRB; l++)
	{
		featureToBeCom[l] = new vector<double> [MaxKeyFrameNumber];
	}
	myTopClass = new topClass[topXValue];
	wordClassResultPotential = new vector<int>[topXValue];
	wordClassScorePotential = new vector<double>[topXValue];
	wordClassResultPotentialLengh = new vector<int>[topXValue];
	M_myStreamMatch=new streamMatch[MaxKeyFrameNumber];

	int i,m,n;
	M_firstKeyFrameIndex = 0;
	M_keyFrameNumber = 0;
	for (i=0; i<MaxKeyFrameNumber; i++)
	{
		M_myStreamMatch[i].maxLengh = 0;
		//M_myStreamMatch[i].weight = new double[MaxKeyFrameNumber*Posture_num];
		M_myStreamMatch[i].topX = new int[Posture_num*topXValue];
		for (m=0; m<MaxKeyFrameNumber; m++)
		{
			for (n=0; n<Posture_num; n++)
			{
				M_myStreamMatch[i].traScore[m][n] = 0.0;
				M_myStreamMatch[i].posScore[m][n] = 0.0;
				M_myStreamMatch[i].weight[m][n] = 0.0;
			}
		}
// 		for (m=0; m<Posture_num; m++)
// 		{
// 			for (n=0; n<topXValue; n++)
// 			{
// 				*(M_myStreamMatch[i].topX+m*Posture_num+n) = 0;
// 			}
// 		}
		M_myStreamMatch[i].frameIndex_start = 0;
		M_myStreamMatch[i].frameIndex_end = 0;
	}
	previousEndFrameIndex = 0;
	isDetectSign = false;
	wordClassResult.clear();
	traDetectNo = 0;
	firstProbability = new float[Posture_num];
	priorProbability = new float[Posture_num];
	transProbability = new float *[Posture_num];
	transProbability_sec = new float *[Posture_num];
	for (int i=0; i<Posture_num; i++)
	{
		transProbability[i] = new float [Posture_num];
		transProbability_sec[i] = new float[Posture_num];
	}
	rankFlush = new vector<int>[topXValue];
	rankFlushSentece = new vector<int>[topXValue];
	//////////////////////////////////////////////////////////////////////////
#ifdef saveFiles
	s_filefolder.Format("..\\signOnlineOutput");
	_mkdir(s_filefolder);

	//readInGroundTruth();
#endif
#ifdef onLineProcess
	sentenceIndex = 0;
#endif
}

S_CMatching::~S_CMatching(void)
{
	for (int m=0; m<MaxKeyFrameNumber; m++)
	{
		delete[] M_keyFeatureStream[m];
	}
	delete[] M_keyFeatureStream;
	delete[] M_myStreamMatch;
	delete[] myTopClass;
	delete[] wordClassResultPotential;
	delete[] wordClassScorePotential;
	delete[] wordClassResultPotentialLengh;
	for (int l=0; l<LRB; l++)
	{
		delete[] featureToBeCom[l];
	}
	delete[] featureToBeCom;
	delete[] firstProbability;
	delete[] priorProbability;
	for (int i=0; i<Posture_num; i++)
	{
		delete[] transProbability[i];
		delete[] transProbability_sec[i];
	}
	delete[] transProbability;
	delete[] transProbability_sec;

	delete[] rankFlush;
	delete[] rankFlushSentece;

	for (int i=0; i<sentenceNumber; i++)
	{
		delete[] M_traGallery[i].leftHand;
		delete[] M_traGallery[i].rightHand;
	}
	
}

double S_CMatching::Posture_Distance(vector<double> xx[], int x, vector<double> yy[], int y, int *pairSum)
{
	if(x==0 || y==0)//No key frames in both
	{
		return 0.0;
	}
	if(x>15 || y>15)//Number of key frames>15 will be consider as abnormal.
	{
		return 0.0;
	}
	int i,j;

	int* order =     new int[x];
	double* weight =    new double[x];
	int* maxLength = new int[x];

	for (i=0; i<x; i++)
	{
		*(maxLength+i) = 1;
	}
		//Calculate the maximum weight and its order.
	double tempMatch = 0.0;
	double maxMatch = 0.0;
	for (i=0; i<x; i++)
	{
		maxMatch = 0.0;
		for (j=0; j<y; j++)
		{
			tempMatch = M_myFeature.Histogram(xx[i],yy[j]);//Histogram(xx[i],yy[j]);
			if (tempMatch>maxMatch)
			{
				maxMatch = tempMatch;
				*(order+i)= j;
			}
		}
		*(weight+i) = maxMatch;
	}
		//Obtain the maxLength for each Index
	int maxlengthTemp = 0;
	for (i=1; i<x; i++)
	{
		maxlengthTemp = 0;
		for (j=i-1; j>=0; j--)
		{
			if (*(order+i)>*(order+j))
			{
				if (*(maxLength+j)>maxlengthTemp)
				{
					*(maxLength+i) = *(maxLength+j)+1;
					maxlengthTemp = *(maxLength+j);
				}
			}
		}
	}
		//Get the max length of all
	int maxLengthAll = 0;
	for (i=0; i<x; i++)
	{
		if (*(maxLength+i)>maxLengthAll)
		{
			maxLengthAll = *(maxLength+i);
		}
	}
	int routeNo = 1;
	int* group = new int[maxLengthAll*x];   //At most x element in one group.
	int* groupNo = new int[maxLengthAll];  
	memset(groupNo,0,maxLengthAll*sizeof(int));
	for (i=0; i<maxLengthAll; i++)
	{
		for (j=0; j<x; j++)
		{
			if (*(maxLength+j) == i+1)
			{
				*(group + i*x + (*(groupNo + i)))=j;
				*(groupNo + i) += 1;
			}
		}
		routeNo *= (*(groupNo + i));
	}

	int* routeAll = new int[routeNo*maxLengthAll];
	int step = 1;
	double jChange = 0.0;
	int jFloor = 0;
	int groIndex = 0;
	for (i=0; i<maxLengthAll; i++)
	{
		for (j=0; j<routeNo; j++)
		{
			jChange = j/step;
			jFloor = floor(jChange);
			groIndex = (jFloor)%(*(groupNo + i));
			*(routeAll+j*maxLengthAll+i) = *(group+i*x+groIndex);
		}
		step = *(groupNo + i);
	}

	//To search the routeAll and find the legal and largest one.
	//Indexes are stored in routeAll.
	//*(weight+index)is its weight.
	//*(order+index)is its order. Both of its "order" and "index" should in order.

	double maxWeight = 0.0;
	bool useful = true;
	double weightTempSum = 0.0;
	int former = 0;
	int latter = 0;
	for (i=0; i<routeNo; i++)
	{
		useful = true;
		weightTempSum = 0.0;
		for (j=0; j<maxLengthAll-1; j++)
		{
			former = *(routeAll+i*maxLengthAll+j);
			latter = *(routeAll+i*maxLengthAll+j+1);
			weightTempSum += *(weight + former);
			if (former>latter || *(order+former)>*(order+latter))
			{
				useful = false;
			}
		}
		if (useful)
		{
			weightTempSum += *(weight+maxLengthAll-1);
			if (weightTempSum>maxWeight)
			{
				maxWeight = weightTempSum;
			}
		}
	}

	maxWeight /= maxLengthAll;
	(*pairSum) = maxLengthAll;

	delete[] groupNo;
	delete[] group;
	delete[] routeAll;
	delete[] order;
	delete[] weight;
	delete[] maxLength;

	maxWeight = maxWeight + (maxLengthAll-1)*0.0; //0.03 can been tuned.
	return maxWeight>1?1:maxWeight;
}

int S_CMatching::doMatch()
{
	int i,j,g,m,n,k;
	
	//If it is the last frame. Do the last judgment.
// 	if (isLast)
// 	{
// 		myKeyframe.setGetDataOver(true);
// 		while(!myKeyframe.getSegmentOver());
// 
// 		int stillOne = -3;
// 		#ifdef delay2
// 		stillOne = signWordDetect_forLast_delay2();
// 		#endif
// 
// 		if (stillOne == -2 || stillOne == -3)
// 		{
// 			return signWordDetect_forLast();
// 		}
// 	}

//	keyFrameStart.push_back(M_myStreamMatch[M_keyFrameNumber-1].frameIndex_start);
//	keyFrameEnd.push_back(M_myStreamMatch[M_keyFrameNumber-1].frameIndex_end);

	//for inter and intra trajectory
// 	for (m=0; m<keyFrameEnd.size(); m++)
// 	{ 
// 		if (M_myStreamMatch[0].frameIndex_start == keyFrameStart[m])
// 		{
// 			start_tra_pointer = m;
// 			break;
// 		}
// 	}
// 	for (m=0; m<keyFrameEnd.size();m++)
// 	{
// 		int traStart = keyFrameStart[m];
// 		int traEnd = keyFrameEnd[keyFrameEnd.size()-1];
// 		vector<SLR_ST_Skeleton> skeletonData_temp;
// 		for (k=traStart; k<traEnd; k++)
// 		{
// 			skeletonData_temp.push_back(M_SkeletonData[k]);
// 		}
// 
// 		float length_left = 0.0;
// 		float length_right = 0.0;
// 		float length = 0.0;
// 		for (k=1; k<skeletonData_temp.size(); k++)
// 		{
// 			length_left += sqrt(pow((skeletonData_temp[k]._3dPoint[7].x - skeletonData_temp[k-1]._3dPoint[7].x),2)
// 				+pow((skeletonData_temp[k]._3dPoint[7].y - skeletonData_temp[k-1]._3dPoint[7].y),2)
// 				+pow((skeletonData_temp[k]._3dPoint[7].z - skeletonData_temp[k-1]._3dPoint[7].z),2));
// 
// 			length_right += sqrt(pow((skeletonData_temp[k]._3dPoint[11].x - skeletonData_temp[k-1]._3dPoint[11].x),2)
// 				+pow((skeletonData_temp[k]._3dPoint[11].y - skeletonData_temp[k-1]._3dPoint[11].y),2)
// 				+pow((skeletonData_temp[k]._3dPoint[11].z - skeletonData_temp[k-1]._3dPoint[11].z),2));
// 		}
// 		length = max(length_left, length_right);
// 
// 		if (length > 0.6)
// 		{
// 			doMatch_tra(sentenceID,traStart,traEnd);
// 		}
// 		
// 	}
		//for inter trajectory
// 	for (m=0; m<keyFrameEnd.size()-1; m++)
// 	{
// 		int skeleton_start = keyFrameEnd[m];
// 		int skeleton_end = keyFrameStart[m+1];
// 		//if (skeleton_end - skeleton_start > allowedTraFrameSize)
// 		{
// 			doMatch_tra(sentenceID,skeleton_start,skeleton_end);
// 		}
// 
// 	}
	
	double* lastDistance = new double[Gallery_num*Posture_num];
	int*    pairSumForAll = new int[Gallery_num*Posture_num];

	for (m=0; m<M_keyFrameNumber; m++)
	{
			//steps 1, 2, 3, 4, 5 are severed for completing  "M_myStreamMatch".
		memset(lastDistance,0,Gallery_num*Posture_num*sizeof(double));
		memset(pairSumForAll,0,Gallery_num*Posture_num*sizeof(int));

		for (n=m; n<M_keyFrameNumber; n++)
		{
			featureToBeCom[0][n-m] = M_keyFeatureStream[n][0];
			featureToBeCom[1][n-m] = M_keyFeatureStream[n][1];
			featureToBeCom[2][n-m] = M_keyFeatureStream[n][2];
		}

		int keyFrameProbe = M_keyFrameNumber - m;
		int keyFrameGallery;
		double dou_temp = 0.0;
		int pairSum = 0;  

			//1: Measure the similarities in 5 galleries.
		for (g=0; g<Gallery_num; g++)
		{
			for (i=0; i<Posture_num; i++)
			{
				int LRBNo = 0;
				for (k=0; k<FusedLRB; k++)
				{
					keyFrameGallery = M_myGallery.getKeyFrameNo(g,i,k);
					for (j=0; j<keyFrameGallery; j++)
					{
						featureToCom[j] = M_myGallery.getGalleryValue(i,k,j,g);
					}
					if (keyFrameGallery == 0)
					{
						for (j=0; j<feature_dimension; j++)
						{
							featureToCom[0].push_back(0.0);
						}
						keyFrameGallery = 1;
					}
					pairSum = 0;
					int firstLady;
					int lastLady;
					dou_temp=Posture_Distance_new(featureToBeCom[k], keyFrameProbe, 
						featureToCom, keyFrameGallery, &pairSum, &firstLady, &lastLady);
					(*(lastDistance + g*Posture_num + i)) += dou_temp;
					(*(pairSumForAll + g*Posture_num + i)) = pairSum;
					M_myStreamMatch[m].postureMatchedIndex[M_myStreamMatch[m].maxLengh][g][i][0] = firstLady;
					M_myStreamMatch[m].postureMatchedIndex[M_myStreamMatch[m].maxLengh][g][i][1] = lastLady;
				}
			}
		}

		for (i=0; i<Posture_num; i++)
		{
			for (j=0; j<Gallery_num; j++)
			{
				M_myStreamMatch[m].posScoreDetail[M_myStreamMatch[m].maxLengh][j][i] = *(lastDistance + j*Posture_num + i);
			}
		}
		
			//2: Get the max weight from 5 galleries.
		double maxWeight[2];
		int maxPair[2];
		int maxGNum;
		for (i=0; i<Posture_num; i++)
		{
				//Check if the score is similar with previous one. If so, a penalty will be given. That means 
				//the current posture didn't contribute to the score and it should be punished.
			if (M_myStreamMatch[m].maxLengh>0)
			{
				for (j=0; j<Gallery_num; j++)
				{
					if ((*(lastDistance + j*Posture_num + i)) == M_myStreamMatch[m].posScoreDetail[M_myStreamMatch[m].maxLengh - 1][j][i])
					{
						*(lastDistance + j*Posture_num + i) = M_myStreamMatch[m].posScoreDetail[M_myStreamMatch[m].maxLengh - 1][j][i]
						*(*(pairSumForAll + j*Posture_num + i))/(*(pairSumForAll + j*Posture_num + i)+1);
					}
				}
			}

			for (int rank = 0; rank<2; rank++)
			{
				maxWeight[rank] = 0;
				maxPair[rank] = 0;
				maxGNum = 0;
				for (j=0; j<Gallery_num; j++)
				{
					if ((*(lastDistance + j*Posture_num + i)) > maxWeight[rank])
					{
						maxWeight[rank] = (*(lastDistance + j*Posture_num + i));
						maxGNum = j;
						maxPair[rank] = (*(pairSumForAll + j*Posture_num + i));
					}
				}
				*(lastDistance + maxGNum*Posture_num + i) = 0.0;
			}
			
			M_myStreamMatch[m].posScore[M_myStreamMatch[m].maxLengh][i] = (maxWeight[0]+maxWeight[1])/2;
			M_myStreamMatch[m].pairSum[M_myStreamMatch[m].maxLengh][i] = (maxPair[0] + maxPair[1])/2;
		}

			//3: Get trajectory score. In theory, at least 2 postures lead to a trajectory.
			//I add the score with only one postures.
		double itrajectoryFactor = 0.0;
#ifdef trajectoryKey
		int traStart = M_myStreamMatch[m].frameIndex_start;
		int traEnd = M_myStreamMatch[m + M_myStreamMatch[m].maxLengh].frameIndex_end;
		vector<SLR_ST_Skeleton> skeletonData_temp;
		for (k=traStart; k<traEnd; k++)
		{
			skeletonData_temp.push_back(M_SkeletonData[k]);
		}

		float length_left = 0.0;
		float length_right = 0.0;
		float length = 0.0;
		for (k=1; k<skeletonData_temp.size(); k++)
		{
			length_left += sqrt(pow((skeletonData_temp[k]._3dPoint[7].x - skeletonData_temp[k-1]._3dPoint[7].x),2)
				+pow((skeletonData_temp[k]._3dPoint[7].y - skeletonData_temp[k-1]._3dPoint[7].y),2)
				+pow((skeletonData_temp[k]._3dPoint[7].z - skeletonData_temp[k-1]._3dPoint[7].z),2));

			length_right += sqrt(pow((skeletonData_temp[k]._3dPoint[11].x - skeletonData_temp[k-1]._3dPoint[11].x),2)
				+pow((skeletonData_temp[k]._3dPoint[11].y - skeletonData_temp[k-1]._3dPoint[11].y),2)
				+pow((skeletonData_temp[k]._3dPoint[11].z - skeletonData_temp[k-1]._3dPoint[11].z),2));
		}
		length = max(length_left, length_right);
		
		
		if (length > 0.6)
		{
			itrajectoryFactor = 1 - 1/pow(iexp,lamda*length);
			CurveRecognition(M_HeadPoint3D[traStart], skeletonData_temp, 
				M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh], 
				sentenceIndex,traStart, traEnd, 
				M_myStreamMatch[m].postureMatchedIndex[M_myStreamMatch[m].maxLengh]);
			for (k=0; k<Posture_num; k++)
			{
				double temp = M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][k];
				if (temp<500)  //500 is large enough for temp. 
				{
					temp = 1/pow(iexp,lamda_diff*temp);
					M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][k] = temp;
				}
				else
				{
					M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][k] = 0.0;
				}
			}
		}
		else
		{
			for (k=0; k<Posture_num; k++)
			{
				M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][k] = 0.0;
			}
		}
		double maxScore = 0.0;
		for (k=0; k<Posture_num; k++)
		{
			if (M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][k] > maxScore)
			{
				maxScore = M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][k];
			}
		}
#endif
		
			//4: Get the last score using Posture and trajectory. "trajectoryFactor" is the weight.
		//itrajectoryFactor = 0.0;
		for (i=0; i<Posture_num; i++)
		{
			M_myStreamMatch[m].weight[M_myStreamMatch[m].maxLengh][i] = 
				M_myStreamMatch[m].posScore[M_myStreamMatch[m].maxLengh][i]*(1-itrajectoryFactor)
				+ M_myStreamMatch[m].traScore[M_myStreamMatch[m].maxLengh][i]*itrajectoryFactor;
		}

			//5: The length of the m_th M_myStreamMatch will be added by 1.
		M_myStreamMatch[m].maxLengh += 1;
	
			//Output some information.
		#ifdef saveFiles
		outfile<<"Index "<<M_myStreamMatch[m].frameIndex_start<<" as the start"<<endl;
		int weiNum = M_myStreamMatch[m].maxLengh;
		for (k=0; k<weiNum; k++)
		{
			for (j=0; j<k; j++)
			{
				//outfile<<M_myStreamMatch[j].frameIndex<<" ";
			}
			outfile<<",";
			for (j=0; j<Posture_num; j++)
			{
				outfile<<M_myStreamMatch[m].weight[k][j]<<",";
			}
			outfile<<endl;
		}
		outfile<<endl;
		#endif

			//Empty the memory
		for (i=0; i<FusedLRB; i++)
		{
			for (j=0; j<MaxKeyFrameNumber; j++)
			{
				featureToBeCom[i][j].clear();
			}
		}
		for (i=0; i<25; i++)
		{
			featureToCom[i].clear();
		}
	}	
	#ifdef saveFiles
	outfile<<endl;
	#endif
	delete[] lastDistance;
	delete[] pairSumForAll;
	//////////////////////////////////////////////////////////////////////////
	//The current "M_myStreamMatch" is completed now. 
	//Detection of words will be conducted as follows.
	//////////////////////////////////////////////////////////////////////////
	//return signWordDetect_0614(outfile, outfile_short, outfile_topX);
#ifndef useNewJudge
		//Find the topX class in each matching in "M_myStreamMatch".
	topXclass();

		//To detect if there are words in the "M_myStreamMatch"
	#ifndef delay2
	return signWordDetect();
	#endif
		
	#ifdef delay2
	return signWordDetect_delay2();
	#endif
#endif
#ifdef useNewJudge
	return signWordDetect_0620();
#endif

	//return 1;
}

void S_CMatching::CreateGallery()
{
		//Set mask and read the gallery
	//M_myGallery.setDataMask();
#ifdef CombineGallery
	M_myGallery.galleryReadFromDat_Combine(DatRouteGallery);
#endif

#ifndef CombineGallery
	M_myGallery.galleryReadFromDat();
#endif
	
		//Trajectory initial. read in the trajectory gallery.
	trajectoryInital();
	//readSegmentFile();   //Better read from binary data.

	manuallySegTest(-1);
	creatTraGallery_SG();

}

void S_CMatching::matchUpdate(vector<double> keyFeatureStream, int iframeIndex_start, int iframeIndex_end)
{
	int i,j,k,m,n,g;

		//If a sign is detected in the last loop.
	if (isDetectSign)
	{
		releaseAfterDetect();
	}

		//Check if the number of current key frames achieve 5. 
	if (M_keyFrameNumber == MaxKeyFrameNumber && M_firstKeyFrameIndex == 0)
	{
		for (i=1; i<MaxKeyFrameNumber; i++)
		{
			M_myStreamMatch[i-1].maxLengh = M_myStreamMatch[i].maxLengh;
			M_myStreamMatch[i-1].frameIndex_start = M_myStreamMatch[i].frameIndex_start;
			M_myStreamMatch[i-1].frameIndex_end = M_myStreamMatch[i].frameIndex_end;
			//memcpy(M_myStreamMatch[i-1].weight,M_myStreamMatch[i].weight,MaxKeyFrameNumber*Posture_num*sizeof(double));
			memcpy(M_myStreamMatch[i-1].topX,M_myStreamMatch[i].topX,Posture_num*topXValue*sizeof(int));

			for (j=0; j<FusedLRB; j++)
			{
				M_keyFeatureStream[i-1][j] = M_keyFeatureStream[i][j];
			}
			for (m=0; m<MaxKeyFrameNumber; m++)
			{
				for (n=0; n<Posture_num; n++)
				{
					M_myStreamMatch[i-1].weight[m][n] = M_myStreamMatch[i].weight[m][n];
					M_myStreamMatch[i-1].weightSort[m][n] = M_myStreamMatch[i].weightSort[m][n];
					M_myStreamMatch[i-1].traScore[m][n] = M_myStreamMatch[i].traScore[m][n];
					M_myStreamMatch[i-1].posScore[m][n] = M_myStreamMatch[i].posScore[m][n];
					M_myStreamMatch[i-1].pairSum[m][n] = M_myStreamMatch[i].pairSum[m][n];
				}
				for (g=0; g<Gallery_num; g++)
				{
					M_myStreamMatch[i-1].postureMatchedIndex[m][g][n][0] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-1].postureMatchedIndex[m][g][n][1] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][1];
					M_myStreamMatch[i-1].trajectoryMatchedIndex[m][g][n][0] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-1].trajectoryMatchedIndex[m][g][n][1] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][1];
					for (k=0; k<Posture_num; k++)
					{
						M_myStreamMatch[i-M_firstKeyFrameIndex-1].posScoreDetail[m][g][k] = M_myStreamMatch[i].posScoreDetail[m][g][k];
					}
				}
			}
		}

		M_keyFrameNumber = MaxKeyFrameNumber-1;
	}

		//Add new frame.

	M_keyFeatureStream[M_keyFrameNumber][0] = keyFeatureStream;

	M_myStreamMatch[M_keyFrameNumber].frameIndex_start = iframeIndex_start;
	M_myStreamMatch[M_keyFrameNumber].frameIndex_end = iframeIndex_end;
	M_myStreamMatch[M_keyFrameNumber].maxLengh = 0;
	M_keyFrameNumber++;
}


double S_CMatching::Posture_Distance_new(vector<double> probe[], int probeNo,
	vector<double> gallery[], int galleryNo, int* pairSum, int *firstLady, int *lastLady)
{
	int m, n, k;
	int pairNum = probeNo*galleryNo;

	vector<Pair> myPair;
	for (m=0; m<probeNo; m++)
	{
		for (n=0; n<galleryNo; n++)
		{
			Pair tempPair;
			tempPair.man = m;
			tempPair.woman = n;
			tempPair.love = M_myFeature.Histogram(probe[m],gallery[n])/*-abs(m-n)*0.03*/;  //0.03 is a threshold to restrict the distance.
			tempPair.love = tempPair.love>0?tempPair.love:0;
			tempPair.married = 2;
			myPair.push_back(tempPair);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//Find the hog_final from myPair.
	int Maybe_num = 0;
	for (k=0; k<pairNum; k++)
	{
		if (myPair[k].married == 2)
		{
			Maybe_num++;
		}
	}
	//Label the married.
	//int count = 0;
	while (Maybe_num > 0 /*&& count<pairNum*/)
	{
		//count++;
		//Find the largest love and marry them.
		double max = 0.0;
		int maxIndex = 0;
		for (k=0; k<pairNum; k++)
		{
			if (myPair[k].married==2 && myPair[k].love >= max)
			{
				max = myPair[k].love;
				maxIndex = k;
			}
		}
		if (myPair[maxIndex].love > MarryThre)
		{
			myPair[maxIndex].married = 1;

			//Unmarried the related others. 
			for (k=0; k<pairNum; k++)
			{
				if (k!=maxIndex)
				{
					bool sad = false;   //If sad is true, they will be unmarried (0).
					if (myPair[k].man == myPair[maxIndex].man)
					{
						sad = true;
					}
					if (myPair[k].woman == myPair[maxIndex].woman)
					{
						sad = true;
					}
					if (myPair[k].man > myPair[maxIndex].man && myPair[k].woman < myPair[maxIndex].woman)
					{
						sad = true;
					}
					if (myPair[k].man < myPair[maxIndex].man && myPair[k].woman > myPair[maxIndex].woman)
					{
						sad = true;
					}
					if (sad)
					{
						myPair[k].married = 0;  //They can not be married.
					}
				}
			}

			Maybe_num = 0;
			for (k=0; k<pairNum; k++)
			{
				if (myPair[k].married == 2)
				{
					Maybe_num++;
				}
			}
		}
		else
		{
			for (k=0; k<pairNum; k++)
			{
				if (myPair[k].married == 2)
				{
					myPair[k].married = 0;
				}
			}

			break;
		}
	}

	double weight = 0.0;
	int marryNo = 0;
	int firstWoman = pairNum-1;
	int lastWoman = 0;
	for (k=0; k<pairNum; k++)
	{
		if (myPair[k].married == 1)
		{
			weight += myPair[k].love;
			marryNo++;
			if (myPair[k].woman > lastWoman)
			{
				lastWoman = myPair[k].woman;
			}
			if (myPair[k].woman < firstWoman)
			{
				firstWoman = myPair[k].woman;
			}
		}
	}
	(*pairSum) = marryNo;
	(*firstLady) = firstWoman;
	(*lastLady) = lastWoman;
	double PairPer = marryNo/galleryNo;
	return weight/(marryNo+0.000001) /*+ PairPer*/;
	//return weight/(probeNo+0.000001);
}

double S_CMatching::Posture_Distance_new_lastProbe(vector<double> probe[], int probeNo,
	vector<double> gallery[], int galleryNo, int* pairSum, int *firstLady, int *lastLady)
{
	int m, n, k;
	int pairNum = probeNo*galleryNo;

	vector<Pair> myPair;
	for (m=0; m<probeNo; m++)
	{
		for (n=0; n<galleryNo; n++)
		{
			Pair tempPair;
			tempPair.man = m;
			tempPair.woman = n;
			tempPair.love = M_myFeature.Histogram(probe[m],gallery[n])/*-abs(m-n)*0.03*/;  //0.03 is a threshold to restrict the distance.
			tempPair.love = tempPair.love>0?tempPair.love:0;
			tempPair.married = 2;
			myPair.push_back(tempPair);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//Find the hog_final from myPair.
	int Maybe_num = 0;
	for (k=0; k<pairNum; k++)
	{
		if (myPair[k].married == 2)
		{
			Maybe_num++;
		}
	}

	//Label the married.
	//int count = 0;
	while (Maybe_num > 0 /*&& count<pairNum*/)
	{
		//count++;
		//Find the largest love and marry them.
		double max = 0.0;
		int maxIndex = 0;
		for (k=0; k<pairNum; k++)
		{
			if (myPair[k].married==2 && myPair[k].love >= max)
			{
				max = myPair[k].love;
				maxIndex = k;
			}
		}
		if (myPair[maxIndex].love > MarryThre)
		{
			myPair[maxIndex].married = 1;

			//Unmarried the related others. 
			for (k=0; k<pairNum; k++)
			{
				if (k!=maxIndex)
				{
					bool sad = false;   //If sad is true, they will be unmarried (0).
					if (myPair[k].man == myPair[maxIndex].man)
					{
						sad = true;
					}
					if (myPair[k].woman == myPair[maxIndex].woman)
					{
						sad = true;
					}
					if (myPair[k].man > myPair[maxIndex].man && myPair[k].woman < myPair[maxIndex].woman)
					{
						sad = true;
					}
					if (myPair[k].man < myPair[maxIndex].man && myPair[k].woman > myPair[maxIndex].woman)
					{
						sad = true;
					}
					if (sad)
					{
						myPair[k].married = 0;  //They can not be married.
					}
				}
			}

			Maybe_num = 0;
			for (k=0; k<pairNum; k++)
			{
				if (myPair[k].married == 2)
				{
					Maybe_num++;
				}
			}
		}
		else
		{
			for (k=0; k<pairNum; k++)
			{
				if (myPair[k].married == 2)
				{
					myPair[k].married = 0;
				}
			}

			break;
		}
	}

	double weight = 0.0;
	int marryNo = 0;
	int firstWoman = pairNum-1;
	int lastWoman = 0;
	for (k=0; k<pairNum; k++)
	{
		if (myPair[k].married == 1)
		{
			weight += myPair[k].love;
			marryNo++;
			if (myPair[k].woman > lastWoman)
			{
				lastWoman = myPair[k].woman;
			}
			if (myPair[k].woman < firstWoman)
			{
				firstWoman = myPair[k].woman;
			}
		}
	}
	(*pairSum) = marryNo;
	(*firstLady) = firstWoman;
	(*lastLady) = lastWoman;
	double PairPer = marryNo/galleryNo;
	return weight/(marryNo+0.000001) /*+ PairPer*/;
	//return weight/(probeNo+0.000001);
}
void S_CMatching::release()
{
	myKeyframe.setGetDataOver(true);
	while(!myKeyframe.getSegmentOver());
}
int S_CMatching::release(int rank[][topXValue], double score[][topXValue], int mask[])
{
	myKeyframe.setGetDataOver(true);
	while(!myKeyframe.getSegmentOver());

	int detectSignClass = -1;
	int detectNum = 0;


#ifdef useNewJudge
//  	detectSignClass = signWordDetect_0620_forLast();
// 	if (detectSignClass > -1)
// 	{
// 		detectNum++;
// 	}
//  	detectSignClass = signWordDetect_0620_forLast();  //if detectSignClass == -2, that means the last key is empty.
// 	if (detectSignClass > -1)
// 	{
// 		detectNum++;
// 	}
#endif

#ifndef generateGalleryKey
	#ifndef useNewJudge
	detectSignClass = signWordDetect_forLast_delay2();
	if (detectSignClass > -1)
	{
		detectNum++;
	}
	detectSignClass = signWordDetect_forLast();
	if (detectSignClass > -1)
	{
		detectNum++;
	}
	#endif
#endif

		//To return the last two classes.
// 	for (int j=0; j<detectNum; j++)
// 	{
// 		
// 		vector<scoreAndIndex> sequence;
// 		for (int r=0; r<topXValue; r++)
// 		{
// 			scoreAndIndex temp;
// 			int index = 0;
// 			if (detectNum == 1)
// 			{
// 				index = 1;
// 			}
// 			if (detectNum == 2)
// 			{
// 				index = 2-j;
// 			}
// 			temp.index = wordClassResultPotential[r][wordClassResultPotential[0].size()-index];
// 			temp.score = wordClassScorePotential[r][wordClassScorePotential[0].size()-index];
// 			sequence.push_back(temp);
// 		}
// 		sort(sequence.begin(),sequence.end(),comp);
// 
// 		for (int r=0; r<topXValue; r++)
// 		{
// 			rank[j][r] = sequence[r].index;
// 			score[j][r] = sequence[r].score;
// 		}
// 	}
// 	if (detectNum == 1)
// 	{
// 		for (int r=0; r<topXValue; r++)
// 		{
// 			rank[1][r] = -1;
// 			score[1][r] = 0.0;
// 		}
// 	}

		//Output the ground truth
	cout<<"Ground truth sentenceID: "<<'\t'<<sentenceIndex<<endl;
	for (int i=0; i<groundTruth[sentenceIndex].size(); i++)
	{
		cout<<groundTruth[sentenceIndex][i]<<'\t';
	}
	cout<<endl;

	doMatch_SG_last();
	int sentenceID;
	int occurTime = 0;
	for (int i=0; i<competeSen.size(); i++)
	{
		if (competeSen[i].score > occurTime)
		{
			occurTime = competeSen[i].score;
			sentenceID = competeSen[i].index;
		}
		cout<<competeSen[i].index<<" "<<competeSen[i].score<<endl;
	}
	detectNum = groundTruth[sentenceID].size();
	cout<<"Recognize sentenceID: "<<'\t'<<sentenceID<<endl;
	for (int i=0; i<detectNum; i++)
	{
		cout<<groundTruth[sentenceID][i]<<'\t';
	}
	cout<<endl;

	int finalDetectNum = 0;
	int *occupys;
	occupys = new int[detectNum];
	int *occupySe;
	occupySe = new int[detectNum];

	for (int i=0; i<detectNum; i++)
	{
		occupys[i] = 0;
		occupySe[i] = 0;
	}

	for (int i=0; i<rankFlush[0].size(); i++)
	{
		if (rankFlushMask[i] > 0)
		{
			mask[finalDetectNum] = 0;
			for (int j=0; j<topXValue; j++)
			{
				rank[finalDetectNum][j] = rankFlush[j][i];
				score[finalDetectNum][j] = 0.0;
			}
			
			for (int k=0; k<topXValue; k++)
			{
				bool det = false;
					//mask = 0: null. mask = 1,2,3,4,5: in rank X. 
				for (int j=0; j<detectNum; j++)   
				{
					//cout<<k<<": "<<groundTruth[sentenceID][j]<<" "<<rankFlush[k][i]<<endl;
					bool occu = false;
					if (occupys[j] == 1 && occupySe[j] <= k)
					{
						occu = true;
					}
					
					if (groundTruth[sentenceID][j] == rankFlush[k][i] && !occu)
					{
						mask[finalDetectNum] = k + 1;
						det = true;
						occupys[j] = 1;
						occupySe[j] = k;
						break;
					}
				}
				if (det)
				{
					break;
				}

			}
			//cout<<rankFlush[0][i]<<" mask: "<<mask[finalDetectNum]<<endl;
			finalDetectNum++;
		}
		
	}

	delete[] occupys;
	delete[] occupySe;

#ifdef generateGalleryKey
	generateGallery();
#endif

#ifdef svmTrainKey
	forSVMTraining();
#endif

#ifdef saveFiles
	//outputForLanguageModel();
	//The correctness.
// 	int matchedWord = 0;
// 	for (int i=0; i<groundTruth[sentenceIndex].size(); i++)
// 	{
// 		int result = groundTruth[sentenceIndex][i];
// 		for (int j=0; j<getWordclassSize(); j++)
// 		{
// 			if (result == getWordclassResult(j))
// 			{
// 				matchedWord++;
// 				break;
// 			}
// 		}
// 	}
// 
// 	int potentialMatcheWord = 0;
// 	int *occupy = new int[wordClassResultPotential[0].size()];
// 	for (int i=0; i<wordClassResultPotential[0].size(); i++)
// 	{
// 		*(occupy + i) = 0;
// 	}
// 	for (int i=0; i<groundTruth[sentenceIndex].size(); i++)
// 	{
// 		int result = groundTruth[sentenceIndex][i];
// 		for (int k=0;k<topXValue;k++)
// 		{
// 			//cout<<"----------"<<myMatching.wordClassResultPotential[k].size()<<endl;
// 			bool detec = false;
// 			for (int j=0; j<wordClassResultPotential[k].size(); j++)
// 			{
// 				if (result == wordClassResultPotential[k][j] && *(occupy + j) == 0)
// 				{
// 					potentialMatcheWord++;
// 					detec = true;
// 					*(occupy + j) = 1;
// 					//cout<<result<<" is correct"<<endl;
// 					if (k == 0)
// 					{
// 						//matchedWord++;
// 					}
// 					break;
// 				}
// 			}
// 			if (detec)
// 			{
// 				break;
// 			}
// 		}
// 	}
// 	delete[] occupy;
// 
// 	cout<<endl<<"The result: ";
// 	outfile_short<<endl<<"The result: ";
// 	for (int i=0; i<getWordclassSize(); i++)
// 	{
// 		cout<<getWordclassResult(i)<<'\t';
// 		outfile_short<<getWordclassResult(i)<<'\t';
// 	}
// 	cout<<endl<<"The ground truth: ";
// 	outfile_short<<endl<<"The ground truth: ";
// 	for (int i=0; i<groundTruth[sentenceIndex].size(); i++)
// 	{
// 		cout<<groundTruth[sentenceIndex][i]<<'\t';
// 		outfile_short<<groundTruth[sentenceIndex][i]<<'\t';
// 	}
// 	cout<<endl;
// 	outfile_short<<endl;
// 	cout<<"correctness: "<<matchedWord<<"/"<<groundTruth[sentenceIndex].size()<<endl;
// 	cout<<"Precise Rate: "<<matchedWord<<"/"<<getWordclassSize()<<endl;
// 	cout<<"correctness rank 5: "<<potentialMatcheWord<<"/"<<groundTruth[sentenceIndex].size()<<endl;
// 	outfile_short<<"Recall Rate: "<<matchedWord<<"/"<<groundTruth[sentenceIndex].size()<<endl;
// 	outfile_short<<"Precise Rate: "<<matchedWord<<"/"<<getWordclassSize()<<endl;
// 
// 	outfile_all.open("..\\signOnlineOutput\\Result_all.txt",ios::out | ios::app);
// 	outfile_all<<"Sentence: "<<sentenceIndex<<" correctness: "<<"\t"<<matchedWord<<"\t"<<potentialMatcheWord
// 		<<"\t"<<groundTruth[sentenceIndex].size()<<"\t"<<getWordclassSize()<<endl;
// 	outfile_all.close();
#endif
	//////////////////////////////////////////////////////////////////////////
	int i, j,m,n;
	M_firstKeyFrameIndex = 0;
	M_keyFrameNumber = 0;
	for (i=0; i<MaxKeyFrameNumber; i++)
	{
		M_myStreamMatch[i].maxLengh = 0;
		for (j=0; j<LRB; j++)
		{
			M_keyFeatureStream[i][j].clear();
		}
		for (m=0; m<MaxKeyFrameNumber; m++)
		{
			for (n=0; n<Posture_num; n++)
			{
				M_myStreamMatch[i].weight[m][n] = 0.0;
				M_myStreamMatch[i].traScore[m][n] = 0.0;
				M_myStreamMatch[i].posScore[m][n] = 0.0;
			}
		}
		M_myStreamMatch[i].frameIndex_start = 0;
		M_myStreamMatch[i].frameIndex_end = 0;
	}
	previousEndFrameIndex = 0;
	isDetectSign = false;
	wordClassResult.clear();
	traDetectNo = 0;
	for(int i=0;i<topXValue;i++)
	{
		wordClassResultPotential[i].clear();//[topXValue];
		wordClassScorePotential[i].clear();
	}
	M_SkeletonData.clear();
	M_HeadPoint3D.clear();
	//keyFrameStart.clear();
	//keyFrameEnd.clear();
	M_traMatchResult.clear();

	myKeyframe.releaseMemory();
	//////////////////////////////////////////////////////////////////////////
#ifdef saveFiles
	outfile.close();
	outfile_topX.close();
	outfile_short.close();
#endif
	competeSen.clear();
	rankFlushMask.clear();
	for (int i=0; i<topXValue; i++)
	{
		rankFlush[i].clear();
	}
	//return detectNum;
	return finalDetectNum;
}

int S_CMatching::getWordclassSize(void)
{
	return wordClassResult.size();
}

int S_CMatching::getWordclassResult(int index)
{
	return wordClassResult[index];
}

int S_CMatching::trajectoryInital(void)
{
	InitData();
	return 0;
}

void S_CMatching::pushSkeletonData(SLR_ST_Skeleton skeletonData, CvPoint3D32f headPoint3D)
{
	M_SkeletonData.push_back(skeletonData);
	M_HeadPoint3D.push_back(headPoint3D);
	if (M_SkeletonData.size()==1)  //Read the hand position at the first time.
	{
		traHandPositionInitial(skeletonData);
	}
}

int S_CMatching::signWordDetect_forTraOnly(ofstream &outfile, ofstream &outfile_short)
{ 
	int k,i,j;
	if (M_keyFrameNumber == 1)
	{
		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].hit = 1;
		}
	}
	if (M_keyFrameNumber == 2)
	{
		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[0].topX + 1*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[0].weight[1][myTopClass[k].classIndex];//k
			myTopClass[k].scorePre = 0.0;
		}
	}
	if (M_keyFrameNumber > 2)
	{
		for (k=0; k<topXValue; k++)
		{
			bool thisHit = false;
			int po = 0;
			int heightPo = 0;
			for (i=0; i<topXValue; i++)
			{
				if (*(M_myStreamMatch[M_keyFrameNumber-2].topX + 1*topXValue + i) == myTopClass[k].classIndex
					/*&& abs(i-k)<topXValue/2*/)
				{
					thisHit = true;
					po = i;
					heightPo = 0;
					break;
				}
				if (*(M_myStreamMatch[M_keyFrameNumber-2].topX + 1*topXValue + i) != myTopClass[k].classIndex)
				{
					//for (j=1; j<M_keyFrameNumber; j++)
					for (j=(M_keyFrameNumber-1); j>1; j--)  //j=M_keyFrameNumber-1;
					{
						double formerScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j-1][myTopClass[k].classIndex];
						double latterScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j][myTopClass[k].classIndex];
						if (*(M_myStreamMatch[M_keyFrameNumber-1-j].topX + j*topXValue + i) == myTopClass[k].classIndex
							&& formerScoreOf0 != latterScoreOf0)
						{
							thisHit = true;
							po = i;
							heightPo = j;
							break;

						}
					}

				}
			}
			if (thisHit && myTopClass[k].hit == 1)
			{
				myTopClass[k].hit = 1;
				myTopClass[k].scorePre = myTopClass[k].score;
				if (heightPo == 0)
				{
					myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-2].weight[1][myTopClass[k].classIndex]; 
				}
				else
				{
					myTopClass[k].score += M_myStreamMatch[0].weight[j][myTopClass[k].classIndex]; 
				}

			}
			else
			{
				myTopClass[k].hit = 0;
				myTopClass[k].scorePre = myTopClass[k].score;
				myTopClass[k].score = 0.0;//MaxKeyFrameNumber*topXValue;
			}
		}
	}

	bool noHit = true;
	cout<<"The potential class Index:";
	for (k=0; k<topXValue; k++)
	{
		if (myTopClass[k].hit == 1)
		{
			noHit = false;
			cout<<myTopClass[k].classIndex<<" ";
		}
	}
	cout<<endl;

	if (noHit /*&& (M_myStreamMatch[0].frameIndex_start - previousEndFrameIndex) > keyFrameGap*/)
	{
		int maxLength = M_keyFrameNumber -1;

		//Get the final score
		int maxClass;
		double maxScore = 0;
		//int minScore = MaxKeyFrameNumber*topXValue - 1;
		int maxK = 0;
		for (k=0; k<topXValue; k++)
		{
			double tempScore = 0.0;
			tempScore = myTopClass[k].scorePre;
			if (tempScore > maxScore)
			{
				maxScore = tempScore;
				maxClass = myTopClass[k].classIndex;
				maxK = k;
			}
		}

		double maxValue = myTopClass[maxK].scorePre / maxLength;
		//Change the pointer to the first place.
		M_firstKeyFrameIndex = M_keyFrameNumber-2;
		//skeletonIndex_start = M_myStreamMatch[M_firstKeyFrameIndex].frameIndex_start;

		//Output information
		cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
			<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
		outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
		outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
		wordClassResult.push_back(maxClass);

		//lhandx[0][0] = 2;
		//Change the detect flag.
		isDetectSign = true;
		//previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

// 		for (k=0; k<topXValue; k++)
// 		{
// 			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber -2].topX + 1*topXValue + k);  //??????why 1
// 			myTopClass[k].hit = 1;
// 			myTopClass[k].score = *(M_myStreamMatch[M_keyFrameNumber -2].weight + 1*Posture_num + myTopClass[k].classIndex);
// 			myTopClass[k].scorePre = 0.0;
// 		}
		return maxClass;
	}
	else
	{
		return -1;
	}
}


int S_CMatching::signWordDetect()
{
	int i, k, j;
	if (M_keyFrameNumber == 1)
	{
		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[0].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[0].weight[0][myTopClass[k].classIndex];//k
			myTopClass[k].scorePre = 0.0;
		}
	}
	else
	{
		for (k=0; k<topXValue; k++)
		{
			bool thisHit = false;
			int po = 0;
			int heightPo = 0;
			for (i=0; i<topXValue; i++)
			{
				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) == myTopClass[k].classIndex)
				{
					bool recheck = false;
					for (j=0; j<topXValue; j++)
					{
						if (*(M_myStreamMatch[0].topX + (M_keyFrameNumber-1)*topXValue + j) == myTopClass[k].classIndex)
						{
							recheck = true;
						}
					}
					if (recheck)
					{
						thisHit = true;
						po = i;
						heightPo = 0;
						break;
					}
				}
				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) != myTopClass[k].classIndex)
				{
					bool endLoop = false;
					//j=M_keyFrameNumber-1;
					for (j=M_keyFrameNumber-1; j>0; j--)
					{
						
						double formerScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j-1][myTopClass[k].classIndex];
						double latterScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j][myTopClass[k].classIndex];
						if (*(M_myStreamMatch[M_keyFrameNumber-1-j].topX + j*topXValue + i) == myTopClass[k].classIndex
							&& formerScoreOf0 != latterScoreOf0)
						{
							thisHit = true;
							po = i;
							heightPo = j;
							endLoop = true;
							break;

						}
					}
					if (endLoop)
					{
						break;
					}
					
				}
			}
			if (thisHit && myTopClass[k].hit == 1)
			{
				myTopClass[k].hit = 1;
				myTopClass[k].scorePre = myTopClass[k].score;
				if (heightPo == 0)
				{
					myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex]; 
				}
				else
				{
					myTopClass[k].score += M_myStreamMatch[0].weight[heightPo][myTopClass[k].classIndex]; 
				}

			}
			else
			{
				myTopClass[k].hit = 0;
				myTopClass[k].scorePre = myTopClass[k].score;
				myTopClass[k].score = 0.0;//MaxKeyFrameNumber*topXValue;
			}
		}
	}

	bool noHit = true;
	for (k=0; k<topXValue; k++)
	{
		#ifdef saveFiles
		outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
			<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		#endif
		if (myTopClass[k].hit == 1)
		{
			noHit = false;
		}
	}
	#ifdef saveFiles
	outfile_short<<endl;
	#endif

	if (noHit /*&& (M_myStreamMatch[0].frameIndex_start - previousEndFrameIndex) > keyFrameGap*/)
	{
		int maxLength = M_keyFrameNumber - 1;  

		//Get the final score
		int maxClass;
		double maxScore = 0;
		int maxK = 0;
		for (k=0; k<topXValue; k++)
		{
			double tempScore = 0.0;

			double keyFrameNoTemp = M_myGallery.getKeyFrameNo(0,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(1,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(2,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(3,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(4,myTopClass[k].classIndex,0);
			keyFrameNoTemp /= Gallery_num;
			if (keyFrameNoTemp - maxLength > 1)
			{
				tempScore = myTopClass[k].scorePre - pairSumPenalty*(keyFrameNoTemp - maxLength);
			}
			else
			{
				tempScore = myTopClass[k].scorePre;
			}
			if (tempScore > maxScore)
			{
				maxScore = tempScore;
				maxClass = myTopClass[k].classIndex;
				maxK = k;
			}
		}

		double maxValue = myTopClass[maxK].scorePre / maxLength;

			//Change the pointer to the first place.
		M_firstKeyFrameIndex = M_keyFrameNumber - 2;

			//Output information
		cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
			<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
		#ifdef saveFiles
		outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
		outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
		#endif
		wordClassResult.push_back(maxClass);
		for (k=0; k<topXValue; k++)
		{
			int potentialClass = *(M_myStreamMatch[0].topX + (M_keyFrameNumber-3)*topXValue + k);
			wordClassResultPotential[k].push_back(myTopClass[k].classIndex);
			wordClassResultPotentialLengh[k].push_back(maxLength);
			//wordClassResultPotential[k].push_back(potentialClass);
		}

		//Change the detect flag.
		isDetectSign = true;
		previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber - 1].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[M_keyFrameNumber - 1].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = 0.0;
		}
		return maxClass;
	}
	else
	{
		return -1;
	}
}

int S_CMatching::signWordDetect_forLast()
{
	int i,j,g,m,n,k;
	if (isDetectSign)
	{
		isDetectSign = false;
		for (i=M_firstKeyFrameIndex+1; i<MaxKeyFrameNumber; i++)
		{
			M_myStreamMatch[i-M_firstKeyFrameIndex-1].maxLengh = M_myStreamMatch[i].maxLengh;
			M_myStreamMatch[i-M_firstKeyFrameIndex-1].frameIndex_start = M_myStreamMatch[i].frameIndex_start;
			M_myStreamMatch[i-M_firstKeyFrameIndex-1].frameIndex_end = M_myStreamMatch[i].frameIndex_end;
			//memcpy(M_myStreamMatch[i-M_firstKeyFrameIndex-1].weight,M_myStreamMatch[i].weight,MaxKeyFrameNumber*Posture_num*sizeof(double));
			memcpy(M_myStreamMatch[i-M_firstKeyFrameIndex-1].topX,M_myStreamMatch[i].topX,Posture_num*topXValue*sizeof(int));

			for (j=0; j<FusedLRB; j++)
			{
				M_keyFeatureStream[i-M_firstKeyFrameIndex-1][j] = M_keyFeatureStream[i][j];
			}
			for (m=0; m<MaxKeyFrameNumber; m++)
			{
				for (n=0; n<Posture_num; n++)
				{
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].weight[m][n] = M_myStreamMatch[i].weight[m][n];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].traScore[m][n] = M_myStreamMatch[i].traScore[m][n];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].posScore[m][n] = M_myStreamMatch[i].posScore[m][n];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].pairSum[m][n] = M_myStreamMatch[i].pairSum[m][n];
				}
				for (g=0; g<Gallery_num; g++)
				{

					M_myStreamMatch[i-M_firstKeyFrameIndex-1].postureMatchedIndex[m][g][n][0] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].postureMatchedIndex[m][g][n][1] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][1];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].trajectoryMatchedIndex[m][g][n][0] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].trajectoryMatchedIndex[m][g][n][1] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][1];
				}
			}
		}
		M_keyFrameNumber = M_keyFrameNumber-M_firstKeyFrameIndex-1;
		M_firstKeyFrameIndex = 0;
	}
	
	int maxClass;
	double maxScore = 0;
	int maxK = 0;
	int maxLength = M_keyFrameNumber;
	for (int i=0; i<topXValue; i++)
	{
		myTopClass[i].classIndex = *(M_myStreamMatch[0].topX + (maxLength-1)*topXValue + i);
		myTopClass[i].score = M_myStreamMatch[0].weight[maxLength-1][myTopClass[i].classIndex];
	}
	for (k=0; k<topXValue; k++)
	{
		double tempScore = 0.0;
		double keyFrameNoTemp = M_myGallery.getKeyFrameNo(0,myTopClass[k].classIndex,0)
			+ M_myGallery.getKeyFrameNo(1,myTopClass[k].classIndex,0)
			+ M_myGallery.getKeyFrameNo(2,myTopClass[k].classIndex,0)
			+ M_myGallery.getKeyFrameNo(3,myTopClass[k].classIndex,0)
			+ M_myGallery.getKeyFrameNo(4,myTopClass[k].classIndex,0);
		keyFrameNoTemp /= Gallery_num;

		if (keyFrameNoTemp - maxLength > 1)
		{
			tempScore = myTopClass[k].score - 0.02*(keyFrameNoTemp - maxLength);
		}
		else
		{
			tempScore = myTopClass[k].score;
		}
		//cout<<tempScore<<endl;
		if (tempScore > maxScore)
		{
			maxScore = tempScore;//myTopClass[k].score;
			maxClass = myTopClass[k].classIndex;
			maxK = k;
		}
	}
	
	double maxValue = myTopClass[maxK].score / maxLength;
	//////////////////////////////////////////////////////////////////////////
	//Test the dynamic model
	float speed = 0.0;
// 	if (maxLength == 1)
// 	{
// 		int traStart = M_myStreamMatch[0].frameIndex_start;
// 		int traEnd = M_myStreamMatch[0].frameIndex_end;
// 		int oriSize = traEnd - traStart +1;
// 		traStart = traStart + (traEnd-traStart)/2;
// 		vector<SLR_ST_Skeleton> skeletonData_temp;
// 		for (k=traStart; k<traEnd; k++)
// 		{
// 			skeletonData_temp.push_back(M_SkeletonData[k]);
// 		}
// 
// 		speed = calSpeed(skeletonData_temp);
// 		//cout<<"speed: "<<speed<<" start: "<<traStart<<" end: "<<traEnd<<" frame size: "<<oriSize<<endl;
// 	}

	//////////////////////////////////////////////////////////////////////////
	bool isTransFlame = false;
#ifdef laguageModelByWang
	isTransFlame = probabilityCheck_oldFramwork(myTopClass, maxLength, 1);
#endif
#ifndef laguageModelByWang
	isFirstAvilable = true;
#endif
	if (maxValue > finalResultThre && speed < transitionFrameThre && !isTransFlame && isFirstAvilable)
	{
		//Output information
		cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
			<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
		#ifdef saveFiles
		outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
		outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
		#endif
		wordClassResult.push_back(maxClass);
		for (k=0; k<topXValue; k++)
		{
			int potentialClass = *(M_myStreamMatch[0].topX + (M_keyFrameNumber-1)*topXValue + k);
			wordClassResultPotential[k].push_back(myTopClass[k].classIndex);
			wordClassScorePotential[k].push_back(myTopClass[k].score);
			wordClassResultPotentialLengh[k].push_back(maxLength);
			//wordClassResultPotential[k].push_back(potentialClass);
		}
	}
	else
	{
		maxClass = -1;
	}
	
	//Return the max class.
	return maxClass;
}

int S_CMatching::signWordDetect_forLast_delay2()
{
	int i,j,g,m,n,k;
	if (isDetectSign)
	{
		isDetectSign = false;
		for (i=M_firstKeyFrameIndex+1; i<MaxKeyFrameNumber; i++)
		{
			M_myStreamMatch[i-M_firstKeyFrameIndex-1].maxLengh = M_myStreamMatch[i].maxLengh;
			M_myStreamMatch[i-M_firstKeyFrameIndex-1].frameIndex_start = M_myStreamMatch[i].frameIndex_start;
			M_myStreamMatch[i-M_firstKeyFrameIndex-1].frameIndex_end = M_myStreamMatch[i].frameIndex_end;
			//memcpy(M_myStreamMatch[i-M_firstKeyFrameIndex-1].weight,M_myStreamMatch[i].weight,MaxKeyFrameNumber*Posture_num*sizeof(double));
			memcpy(M_myStreamMatch[i-M_firstKeyFrameIndex-1].topX,M_myStreamMatch[i].topX,Posture_num*topXValue*sizeof(int));

			for (j=0; j<FusedLRB; j++)
			{
				M_keyFeatureStream[i-M_firstKeyFrameIndex-1][j] = M_keyFeatureStream[i][j];
			}
			for (m=0; m<MaxKeyFrameNumber; m++)
			{
				for (n=0; n<Posture_num; n++)
				{
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].weight[m][n] = M_myStreamMatch[i].weight[m][n];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].traScore[m][n] = M_myStreamMatch[i].traScore[m][n];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].posScore[m][n] = M_myStreamMatch[i].posScore[m][n];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].pairSum[m][n] = M_myStreamMatch[i].pairSum[m][n];
				}
				for (g=0; g<Gallery_num; g++)
				{

					M_myStreamMatch[i-M_firstKeyFrameIndex-1].postureMatchedIndex[m][g][n][0] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].postureMatchedIndex[m][g][n][1] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][1];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].trajectoryMatchedIndex[m][g][n][0] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].trajectoryMatchedIndex[m][g][n][1] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][1];
				}
			}
		}
		M_keyFrameNumber = M_keyFrameNumber-M_firstKeyFrameIndex-1;
		M_firstKeyFrameIndex = 0;
	}

	int maxClass;
	double maxScore = 0;
	int maxK = 0;
	int maxLength;
	if (M_keyFrameNumber < 2)
	{
		return -2;
	}
	bool isLastAll0 = true;
	for (k=0; k<topXValue; k++)
	{
		if (myTopClass[k].score != 0)
		{
			isLastAll0 = false;
		}
	}

	if (isLastAll0)
	{
		maxLength = M_keyFrameNumber - 1; 

		//Get the final score
		int maxClass;
		double maxScore = 0;
		int maxK = 0;
		for (int i=0; i<topXValue; i++)
		{
			myTopClass[i].classIndex = *(M_myStreamMatch[0].topX + (maxLength-1)*topXValue + i);
			myTopClass[i].scorePre = M_myStreamMatch[0].weight[maxLength-1][myTopClass[i].classIndex];
		}
		for (k=0; k<topXValue; k++)
		{
			double tempScore = 0.0;

			double keyFrameNoTemp = M_myGallery.getKeyFrameNo(0,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(1,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(2,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(3,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(4,myTopClass[k].classIndex,0);
			keyFrameNoTemp /= Gallery_num;
			if (keyFrameNoTemp - maxLength > 1)
			{
				tempScore = myTopClass[k].scorePre - pairSumPenalty*(keyFrameNoTemp - maxLength);
			}
			else
			{
				tempScore = myTopClass[k].scorePre;
			}
			if (tempScore > maxScore)
			{
				maxScore = tempScore;
				maxClass = myTopClass[k].classIndex;
				maxK = k;
			}
		}

		double maxValue = myTopClass[maxK].scorePre / maxLength;

		//Change the pointer to the first place.
		M_firstKeyFrameIndex = M_keyFrameNumber - 2;
		//////////////////////////////////////////////////////////////////////////
		//Test the dynamic model
		float speed = 0.0;
// 		if (maxLength == 1)
// 		{
// 			int traStart = M_myStreamMatch[0].frameIndex_start;
// 			int traEnd = M_myStreamMatch[maxLength-1].frameIndex_end;
// 			int oriSize = traEnd - traStart +1;
// 			traStart = traStart + (traEnd-traStart)/2;
// 			vector<SLR_ST_Skeleton> skeletonData_temp;
// 			for (k=traStart; k<traEnd; k++)
// 			{
// 				skeletonData_temp.push_back(M_SkeletonData[k]);
// 			}
// 
// 			speed = calSpeed(skeletonData_temp);
// 			//cout<<"speed: "<<speed<<" start: "<<traStart<<" end: "<<traEnd<<" frame size: "<<oriSize<<endl;
// 		}

		//////////////////////////////////////////////////////////////////////////
		bool isTransFlame = false;
#ifdef laguageModelByWang
		isTransFlame = probabilityCheck_oldFramwork(myTopClass, maxLength, 2);
#endif
#ifndef laguageModelByWang
		isFirstAvilable = true;
#endif
		if (maxValue > finalResultThre && speed < transitionFrameThre && !isTransFlame && isFirstAvilable)
		{
			//Output information
			cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
				<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
			#ifdef saveFiles
			outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
				<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
				<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			#endif
			wordClassResult.push_back(maxClass);
			for (k=0; k<topXValue; k++)
			{
				int potentialClass = *(M_myStreamMatch[0].topX + (M_keyFrameNumber-2)*topXValue + k);
				wordClassResultPotential[k].push_back(myTopClass[k].classIndex);
				wordClassScorePotential[k].push_back(myTopClass[k].scorePre);
				wordClassResultPotentialLengh[k].push_back(maxLength);
				//wordClassResultPotential[k].push_back(potentialClass);
			}

			
		}
		else
		{
			maxClass = -1;
		}
		//Change the detect flag.
		isDetectSign = true;
		previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber - 1].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[M_keyFrameNumber - 1].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = 0.0;
		}
		return -2;
	}
	else
	{
		return -2;
	}
	//////////////////////////////////////////////////////////////////////////
// 	if (M_keyFrameNumber > 3)
// 	{
// 		maxLength = M_keyFrameNumber - 2;
// 	}
// 	else if (M_keyFrameNumber > 2)
// 	{
// 
// 	}
// 	else if (M_keyFrameNumber > 0)
// 	{
// 	}
// 	if (M_keyFrameNumber > 2)
// 	{
// 		for (k=0; k<topXValue; k++)
// 		{
// 			cout<<myTopClass[k].classIndex<<" "<<myTopClass[k].scorePre<<endl;
// 			double tempScore = 0.0;
// 			double keyFrameNoTemp = M_myGallery.getKeyFrameNo(0,myTopClass[k].classIndex,0)
// 				+ M_myGallery.getKeyFrameNo(1,myTopClass[k].classIndex,0)
// 				+ M_myGallery.getKeyFrameNo(2,myTopClass[k].classIndex,0)
// 				+ M_myGallery.getKeyFrameNo(3,myTopClass[k].classIndex,0)
// 				+ M_myGallery.getKeyFrameNo(4,myTopClass[k].classIndex,0);
// 			keyFrameNoTemp /= Gallery_num;
// 
// 			if (keyFrameNoTemp - maxLength > 1)
// 			{
// 				tempScore = myTopClass[k].scorePre - 0.02*(keyFrameNoTemp - maxLength);
// 			}
// 			else
// 			{
// 				tempScore = myTopClass[k].scorePre;
// 			}
// 			//cout<<tempScore<<endl;
// 			if (tempScore > maxScore)
// 			{
// 				maxScore = tempScore;//myTopClass[k].score;
// 				maxClass = myTopClass[k].classIndex;
// 				maxK = k;
// 			}
// 		}
// 
// 		double maxValue = myTopClass[maxK].scorePre / maxLength;
// 
// 		//Output information
// 		cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
// 			<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
// 		outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
// 			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
// 		outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<" maxValue "
// 			<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
// 		wordClassResult.push_back(maxClass);
// 
// 		M_firstKeyFrameIndex = M_keyFrameNumber - 3;
// 		isDetectSign = true;
// 
// 		for (k=0; k<topXValue; k++)
// 		{
// 			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber - 2].topX + 0*topXValue + k);
// 			myTopClass[k].hit = 1;
// 			myTopClass[k].score = M_myStreamMatch[M_keyFrameNumber - 2].weight[0][myTopClass[k].classIndex];
// 			myTopClass[k].scorePre = 0.0;
// 			//myTopClass[k].scorePrePre = 0.0;
// 		}
// 	}
// 	
// 	if (M_keyFrameNumber >= 2)
// 	{
// 		//For the last one. 
// 		for (k=0; k<topXValue; k++)
// 		{
// 			bool thisHit = false;
// 			int po = 0;
// 			int heightPo = 0;
// 			for (i=0; i<topXValue; i++)
// 			{
// 				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) == myTopClass[k].classIndex)
// 				{
// 					bool recheck = false;
// 					for (j=0; j<topXValue; j++)
// 					{
// 						if (*(M_myStreamMatch[0].topX + (M_keyFrameNumber-1)*topXValue + j) == myTopClass[k].classIndex)
// 						{
// 							recheck = true;
// 						}
// 					}
// 					if (recheck)
// 					{
// 						thisHit = true;
// 						po = i;
// 						heightPo = 0;
// 						break;
// 					}
// 				}
// 				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) != myTopClass[k].classIndex)
// 				{
// 
// 					//j=M_keyFrameNumber-1;
// 					for (j=M_keyFrameNumber-1; j>0; j--)
// 					{
// 						double formerScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j-1][myTopClass[k].classIndex];
// 						double latterScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j][myTopClass[k].classIndex];
// 						if (*(M_myStreamMatch[M_keyFrameNumber-1-j].topX + j*topXValue + i) == myTopClass[k].classIndex
// 							&& formerScoreOf0 != latterScoreOf0)
// 						{
// 							thisHit = true;
// 							po = i;
// 							heightPo = j;
// 							break;
// 
// 						}
// 					}
// 
// 				}
// 			}
// 			int rescue = 0;
// 			if (myTopClass[k].hit == 1)
// 			{
// 				rescue = 1;
// 			}
// 			else if (myTopClass[k].hit != 1 && heightPo == 0)
// 			{
// 				rescue = 2;  //It is a rescued one.
// 			}
// 			if (thisHit && rescue>0)
// 			{
// 				myTopClass[k].hit = 1;
// 				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
// 				myTopClass[k].scorePre = myTopClass[k].score;
// 				if (heightPo == 0)
// 				{
// 					if (rescue == 1)
// 					{
// 						myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex]; 
// 					}
// 					else if (rescue == 2)
// 					{
// 						myTopClass[k].score = 0.0;
// 						for (int p=0; p<M_keyFrameNumber; p++)
// 						{
// 							myTopClass[k].score += M_myStreamMatch[p].weight[0][myTopClass[k].classIndex]; 
// 						}
// 					}
// 				}
// 				else
// 				{
// 					myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber -2].weight[heightPo][myTopClass[k].classIndex];
// 				}
// 
// 			}
// 			else
// 			{
// 				myTopClass[k].hit = 0;
// 				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
// 				myTopClass[k].scorePre = myTopClass[k].score;
// 				myTopClass[k].score = 0.0;//MaxKeyFrameNumber*topXValue;
// 			}
// 		}
// 	}
	

	//Return the max class.
//	return maxClass;
}

void S_CMatching::topXclass()
{
	///////////////////////////////////////////////////////////////////////////
	//Find all the top 5 classes for the key posture combinations.
	int i, j, m, k;
	for (i=0; i<M_keyFrameNumber; i++)
	{
		for (j=0; j<M_myStreamMatch[i].maxLengh; j++)
		{
				//For outputting information.
			#ifdef saveFiles
			for (k=0; k<=j; k++)
			{
				outfile_topX<<M_myStreamMatch[k+i].frameIndex_start<<" ";
			}
			outfile_topX<<","<<",";
			#endif

				//Copy it.
			double* stream = new double[Posture_num];
			for (m=0; m<Posture_num; m++)
			{
				*(stream + m) = M_myStreamMatch[i].weight[j][m];
			}
				//Time cost: topXValue*Posture_num
				//Fast than std::sort
			for (k=0; k<topXValue; k++)
			{
				double maxvalue = 0.0;
				int maxClass = 0;
				for (m=0; m<Posture_num; m++)
				{
					if (*(stream + m) > maxvalue)
					{
						maxvalue = *(stream + m);
						maxClass = m;
					} 
				}
				*(stream+maxClass) = 0.0;
				*(M_myStreamMatch[i].topX + j*topXValue + k) = maxClass;
				#ifdef saveFiles
				outfile_topX<<maxClass<<","<<M_myStreamMatch[i].weight[j][maxClass]<<",";
				#endif
			}
			delete[] stream;
			#ifdef saveFiles
			outfile_topX<<endl;
			#endif
		}
		#ifdef saveFiles
		outfile_topX<<endl;
		#endif
	}
}

int S_CMatching::signWordDetect_delay2()
{
	int i, k, j;
	if (M_keyFrameNumber == 1)
	{
		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[0].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[0].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = 0.0;
			myTopClass[k].scorePrePre = 0.0;
		}
	}
	else
	{
		for (k=0; k<topXValue; k++)
		{
			bool thisHit = false;
			int po = 0;
			int heightPo = 0;
			for (i=0; i<topXValue; i++)
			{
				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) == myTopClass[k].classIndex)
				{
					
					bool recheck = false;
					for (j=0; j<topXValue; j++)
					{
						if (*(M_myStreamMatch[0].topX + (M_keyFrameNumber-1)*topXValue + j) 
							== myTopClass[k].classIndex)
						{
							recheck = true;
						}
					}
					if (recheck)
					{
						thisHit = true;
						po = i;
						heightPo = 0;
						break;
					}
				}
			}
			if (!thisHit)
			{
				for (i=0; i<topXValue; i++)
				{
					bool endLoop = false;
					//j=M_keyFrameNumber-1;
					for (j=M_keyFrameNumber-1; j>0; j--)
					{
						double formerScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j-1][myTopClass[k].classIndex];
						double latterScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j][myTopClass[k].classIndex];
						if (*(M_myStreamMatch[M_keyFrameNumber-1-j].topX + j*topXValue + i) == myTopClass[k].classIndex
							&& formerScoreOf0 != latterScoreOf0 &&
							latterScoreOf0 > 0.6 && (latterScoreOf0*(j+1)-formerScoreOf0*j)>0.6)
						{
							thisHit = true;
							po = i;
							heightPo = j;
							endLoop = true;
							break;

						}
					}
					if (endLoop)
					{
						break;
					}

				
				}
			}
			
			int rescue = 0;
			if (myTopClass[k].hit == 1)
			{
				rescue = 1;
			}
			else if (myTopClass[k].hit != 1 && heightPo == 0 && thisHit)
			{
				rescue = 2;  //It is a rescued one.
				myTopClass[k].hit = 1;
			}
			if (thisHit && rescue>0)
			{
				myTopClass[k].hit = 1;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				if (heightPo == 0)
				{
					if (rescue == 1)
					{
						myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex]; 
					}
					else if (rescue == 2)
					{
						myTopClass[k].score = 0.0;
						for (int p=0; p<M_keyFrameNumber; p++)
						{
							myTopClass[k].score += M_myStreamMatch[p].weight[0][myTopClass[k].classIndex]; 
						}
					}
					
				}
				else
				{
					myTopClass[k].score += M_myStreamMatch[0].weight[heightPo][myTopClass[k].classIndex]; 
				}

			}
			else
			{
				myTopClass[k].hit = 0;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				myTopClass[k].score = 0.0;//MaxKeyFrameNumber*topXValue;
			}
		}
	}

	
	bool noHit = true;
	for (k=0; k<topXValue; k++)
	{
		#ifdef saveFiles
		outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
			<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		#endif
		if (myTopClass[k].hit == 1)
		{
			noHit = false;
		}
		if (myTopClass[k].scorePre != 0)
		{
			noHit = false;
		}
	}
	#ifdef saveFiles
	outfile_short<<endl;
	#endif

	if (noHit)
	{
		int maxLength = M_keyFrameNumber - 2;  //2 is delay

			//Get the final score
		int maxClass;
		double maxScore = 0;
		int maxK = 0;
		double keyFrameNoTemp;
		for (int i=0; i<topXValue; i++)
		{
			myTopClass[i].classIndex = *(M_myStreamMatch[0].topX + (maxLength-1)*topXValue + i);
			myTopClass[i].scorePrePre = M_myStreamMatch[0].weight[maxLength-1][myTopClass[i].classIndex];
		}
		for (k=0; k<topXValue; k++)
		{
			double tempScore = 0.0;
			keyFrameNoTemp = M_myGallery.getKeyFrameNo(0,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(1,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(2,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(3,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(4,myTopClass[k].classIndex,0);
			keyFrameNoTemp /= Gallery_num;
			if (keyFrameNoTemp - maxLength > 1)
			{
				tempScore = myTopClass[k].scorePrePre - pairSumPenalty*(keyFrameNoTemp - maxLength);
			}
			else
			{
				tempScore = myTopClass[k].scorePrePre;
			}
			if (tempScore > maxScore)
			{
				maxScore = tempScore;
				maxClass = myTopClass[k].classIndex;
				maxK = k;
			}
		}

		double maxValue = myTopClass[maxK].scorePrePre / maxLength;
			//Change the pointer to the first place.
		M_firstKeyFrameIndex = M_keyFrameNumber - 3;

		//////////////////////////////////////////////////////////////////////////
		//Test the dynamic model
		float speed = 0.0;
// 		if (maxLength == 1)
// 		{
// 			int traStart = M_myStreamMatch[0].frameIndex_start;//M_myStreamMatch[0].frameIndex_end;//
// 			int traEnd = M_myStreamMatch[0].frameIndex_end;//M_myStreamMatch[1].frameIndex_start;//
// 			int oriSize = traEnd - traStart +1;
// 			traStart = traStart + (traEnd-traStart)/2;
// 			vector<SLR_ST_Skeleton> skeletonData_temp;
// 			for (k=traStart; k<traEnd; k++)
// 			{
// 				skeletonData_temp.push_back(M_SkeletonData[k]);
// 			}
// 			//
// // 			for (k=0; k<skeletonData_temp.size();k++)
// // 			{
// // 				cout<<skeletonData_temp[k]._3dPoint[7].x<<" "
// // 					<<skeletonData_temp[k]._3dPoint[7].y<<" "
// // 					<<skeletonData_temp[k]._3dPoint[7].z<<" "
// // 					<<skeletonData_temp[k]._3dPoint[11].x<<" "
// // 					<<skeletonData_temp[k]._3dPoint[11].y<<" "
// // 					<<skeletonData_temp[k]._3dPoint[11].z<<" "
// // 					<<endl;
// // 			}
// 			speed = calSpeed(skeletonData_temp);
// 			//cout<<"speed: "<<speed<<" start: "<<traStart<<" end: "<<traEnd<<" frame size: "<<oriSize<<endl;
// 		}
		
		//////////////////////////////////////////////////////////////////////////
		bool isTransFlame = false;
#ifdef laguageModelByWang
		isTransFlame = probabilityCheck_oldFramwork(myTopClass, maxLength, 3);
#endif
#ifndef laguageModelByWang
		isFirstAvilable = true;
#endif
		if (maxValue>finalResultThre && speed<transitionFrameThre && !isTransFlame && isFirstAvilable )
		{
			//Output information
			cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
				<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
			#ifdef saveFiles
			outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxValue "
				<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxValue "
				<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			#endif
			wordClassResult.push_back(maxClass);

			for (k=0; k<topXValue; k++)
			{
				int potentialClass = *(M_myStreamMatch[0].topX + (M_keyFrameNumber-3)*topXValue + k);
				wordClassResultPotential[k].push_back(myTopClass[k].classIndex);
				wordClassScorePotential[k].push_back(myTopClass[k].scorePrePre);
				wordClassResultPotentialLengh[k].push_back(maxLength);
				//wordClassResultPotential[k].push_back(potentialClass);
			}

			
		}
		else
		{
			maxClass = -1;
		}
		//Change the detect flag.
		isDetectSign = true;
		previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber - 2].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[M_keyFrameNumber - 2].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = 0.0;
			myTopClass[k].scorePrePre = 0.0;
		}
		#ifdef saveFiles
		for (k=0; k<topXValue; k++)
		{
			outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
				<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		}
		outfile_short<<endl;
		#endif


			//For the last one. 
		for (k=0; k<topXValue; k++)
		{
			bool thisHit = false;
			int po = 0;
			int heightPo = 0;
			for (i=0; i<topXValue; i++)
			{
				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) == myTopClass[k].classIndex)
				{
					bool recheck = false;
					for (j=0; j<topXValue; j++)
					{
						if (*(M_myStreamMatch[M_keyFrameNumber-2].topX + (1)*topXValue + j) == myTopClass[k].classIndex)
						{
							recheck = true;
						}
					}
					if (recheck)
					{
						thisHit = true;
						po = i;
						heightPo = 0;
						break;
					}
				}

			}
			if (!thisHit)
			{
				for (i=0; i<topXValue; i++)
				{
					j=M_keyFrameNumber-1;
					bool endLoop = false;
					//for (j=M_keyFrameNumber-1; j>0; j--)
					{
						double formerScoreOf0 = M_myStreamMatch[j-1].weight[1][myTopClass[k].classIndex];
						double latterScoreOf0 = M_myStreamMatch[j].weight[0][myTopClass[k].classIndex];
						if (*(M_myStreamMatch[j-1].topX + j*topXValue + i) == myTopClass[k].classIndex
							&& formerScoreOf0 != latterScoreOf0&&
							latterScoreOf0 > 0.6 && (latterScoreOf0*(j+1)-formerScoreOf0*j)>0.6)
						{
							thisHit = true;
							po = i;
							heightPo = j;
							endLoop = true;
							break;
						}
					}
					if (endLoop)
					{
						break;
					}
				}
			}
			
			int rescue = 0;
			if (myTopClass[k].hit == 1)
			{
				rescue = 1;
			}
			else if (myTopClass[k].hit != 1 && heightPo == 0)
			{
				rescue = 2;  //It is a rescued one.
			}
			if (thisHit && rescue>0)
			{
				myTopClass[k].hit = 1;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				if (heightPo == 0)
				{
					if (rescue == 1)
					{
						myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex]; 
					}
					else if (rescue == 2)
					{
						myTopClass[k].score = 0.0;
						for (int p=0; p<M_keyFrameNumber; p++)
						{
							myTopClass[k].score += M_myStreamMatch[p].weight[0][myTopClass[k].classIndex]; 
						}
						myTopClass[k].hit = 1;
					}
				}
				else
				{
					myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber -2].weight[heightPo][myTopClass[k].classIndex];
				}

			}
			else
			{
				myTopClass[k].hit = 0;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				myTopClass[k].score = 0.0;
			}
		}
		#ifdef saveFiles
		for (k=0; k<topXValue; k++)
		{
			outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
				<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		}
		outfile_short<<endl;
		#endif
		return maxClass;
	}
	else
	{
		return -1;
	}
}

int S_CMatching::signWordDetect_delay2_0616(ofstream &outfile, ofstream &outfile_short)
{
	int i, k, j;
	if (M_keyFrameNumber == 1)
	{
		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[0].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[0].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = 0.0;
			myTopClass[k].scorePrePre = 0.0;
		}
	}
	else
	{
		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber-2].topX + 0*topXValue + k);
			myTopClass[k].score = M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex];
			if (M_keyFrameNumber>2)
			{
				myTopClass[k].scorePrePre = M_myStreamMatch[M_keyFrameNumber-2].weight[0][myTopClass[k].classIndex];
			}
			else
			{
				myTopClass[k].scorePrePre = 0.0;
			}
		}

		for (k=0; k<topXValue; k++)
		{
			bool thisHit = false;
			int po = 0;
			int heightPo = 0;
			for (i=0; i<topXValue; i++)
			{
				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) == myTopClass[k].classIndex)
				{

					bool recheck = false;
					for (j=0; j<topXValue; j++)
					{
						if (*(M_myStreamMatch[0].topX + (M_keyFrameNumber-1)*topXValue + j) 
							== myTopClass[k].classIndex)
						{
							recheck = true;
						}
					}
					if (recheck)
					{
						thisHit = true;
						po = i;
						heightPo = 0;
						break;
					}
				}
			}
			if (!thisHit)
			{
				for (i=0; i<topXValue; i++)
				{
					bool endLoop = false;
					//j=M_keyFrameNumber-1;
					for (j=M_keyFrameNumber-1; j>0; j--)
					{
						double formerScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j-1][myTopClass[k].classIndex];
						double latterScoreOf0 = M_myStreamMatch[M_keyFrameNumber-1-j].weight[j][myTopClass[k].classIndex];
						if (*(M_myStreamMatch[M_keyFrameNumber-1-j].topX + j*topXValue + i) == myTopClass[k].classIndex
							&& formerScoreOf0 != latterScoreOf0 &&
							latterScoreOf0 > 0.6 && (latterScoreOf0*(j+1)-formerScoreOf0*j)>0.6)
						{
							thisHit = true;
							po = i;
							heightPo = j;
							endLoop = true;
							break;

						}
					}
					if (endLoop)
					{
						break;
					}


				}
			}

			int rescue = 0;
			if (myTopClass[k].hit == 1)
			{
				rescue = 1;
			}
			else if (myTopClass[k].hit != 1 && heightPo == 0 && thisHit)
			{
				rescue = 2;  //It is a rescued one.
				myTopClass[k].hit = 1;
			}
			if (thisHit && rescue>0)
			{
				myTopClass[k].hit = 1;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				if (heightPo == 0)
				{
					if (rescue == 1)
					{
						myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex]; 
					}
					else if (rescue == 2)
					{
						myTopClass[k].score = 0.0;
						for (int p=0; p<M_keyFrameNumber; p++)
						{
							myTopClass[k].score += M_myStreamMatch[p].weight[0][myTopClass[k].classIndex]; 
						}
					}

				}
				else
				{
					myTopClass[k].score += M_myStreamMatch[0].weight[heightPo][myTopClass[k].classIndex]; 
				}

			}
			else
			{
				myTopClass[k].hit = 0;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				myTopClass[k].score = 0.0;//MaxKeyFrameNumber*topXValue;
			}
		}
	}


	bool noHit = true;
	for (k=0; k<topXValue; k++)
	{
		outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
			<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		if (myTopClass[k].hit == 1)
		{
			noHit = false;
		}
		if (myTopClass[k].scorePre != 0)
		{
			noHit = false;
		}
	}
	outfile_short<<endl;

	if (noHit)
	{
		int maxLength = M_keyFrameNumber - 2;  //2 is delay

		//Get the final score
		int maxClass;
		double maxScore = 0;
		int maxK = 0;
		double keyFrameNoTemp;
		for (k=0; k<topXValue; k++)
		{
			double tempScore = 0.0;
			keyFrameNoTemp = M_myGallery.getKeyFrameNo(0,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(1,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(2,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(3,myTopClass[k].classIndex,0)
				+ M_myGallery.getKeyFrameNo(4,myTopClass[k].classIndex,0);
			keyFrameNoTemp /= Gallery_num;
			if (keyFrameNoTemp - maxLength > 1)
			{
				tempScore = myTopClass[k].scorePrePre - pairSumPenalty*(keyFrameNoTemp - maxLength);
			}
			else
			{
				tempScore = myTopClass[k].scorePrePre;
			}
			if (tempScore > maxScore)
			{
				maxScore = tempScore;
				maxClass = myTopClass[k].classIndex;
				maxK = k;
			}
		}

		double maxValue = myTopClass[maxK].scorePrePre / maxLength;
		//Change the pointer to the first place.
		M_firstKeyFrameIndex = M_keyFrameNumber - 3;

		//////////////////////////////////////////////////////////////////////////
		//Test the dynamic model
		float speed = 0.0;
		if (maxLength == 1)
		{
			int traStart = M_myStreamMatch[0].frameIndex_start;//M_myStreamMatch[0].frameIndex_end;//
			int traEnd = M_myStreamMatch[0].frameIndex_end;//M_myStreamMatch[1].frameIndex_start;//
			int oriSize = traEnd - traStart +1;
			traStart = traStart + (traEnd-traStart)/2;
			vector<SLR_ST_Skeleton> skeletonData_temp;
			for (k=traStart; k<traEnd; k++)
			{
				skeletonData_temp.push_back(M_SkeletonData[k]);
			}
			//
			// 			for (k=0; k<skeletonData_temp.size();k++)
			// 			{
			// 				cout<<skeletonData_temp[k]._3dPoint[7].x<<" "
			// 					<<skeletonData_temp[k]._3dPoint[7].y<<" "
			// 					<<skeletonData_temp[k]._3dPoint[7].z<<" "
			// 					<<skeletonData_temp[k]._3dPoint[11].x<<" "
			// 					<<skeletonData_temp[k]._3dPoint[11].y<<" "
			// 					<<skeletonData_temp[k]._3dPoint[11].z<<" "
			// 					<<endl;
			// 			}
			speed = calSpeed(skeletonData_temp);
			//cout<<"speed: "<<speed<<" start: "<<traStart<<" end: "<<traEnd<<" frame size: "<<oriSize<<endl;
		}

		//////////////////////////////////////////////////////////////////////////
		if (maxValue>finalResultThre && speed<transitionFrameThre )
		{
			//Output information
			cout<<"Word--------------------Class "<<maxClass<<"; Start frameIndex "
				<<M_myStreamMatch[0].frameIndex_start<<"; Length "<<maxLength<<endl;
			outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxValue "
				<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxValue "
				<<maxValue<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			wordClassResult.push_back(maxClass);

			for (k=0; k<topXValue; k++)
			{
				int potentialClass = *(M_myStreamMatch[0].topX + (M_keyFrameNumber-3)*topXValue + k);
				wordClassResultPotential[k].push_back(myTopClass[k].classIndex);
				wordClassResultPotentialLengh[k].push_back(maxLength);
				//wordClassResultPotential[k].push_back(potentialClass);
			}


		}
		else
		{
			maxClass = -1;
		}
		//Change the detect flag.
		isDetectSign = true;
		previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

		for (k=0; k<topXValue; k++)
		{
			myTopClass[k].classIndex = *(M_myStreamMatch[M_keyFrameNumber - 2].topX + 0*topXValue + k);
			myTopClass[k].hit = 1;
			myTopClass[k].score = M_myStreamMatch[M_keyFrameNumber - 2].weight[0][myTopClass[k].classIndex];
			myTopClass[k].scorePre = 0.0;
			myTopClass[k].scorePrePre = 0.0;
		}
		for (k=0; k<topXValue; k++)
		{
			outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
				<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		}
		outfile_short<<endl;


		//For the last one. 
		for (k=0; k<topXValue; k++)
		{
			bool thisHit = false;
			int po = 0;
			int heightPo = 0;
			for (i=0; i<topXValue; i++)
			{
				if (*(M_myStreamMatch[M_keyFrameNumber-1].topX + 0*topXValue + i) == myTopClass[k].classIndex)
				{
					bool recheck = false;
					for (j=0; j<topXValue; j++)
					{
						if (*(M_myStreamMatch[M_keyFrameNumber-2].topX + (1)*topXValue + j) == myTopClass[k].classIndex)
						{
							recheck = true;
						}
					}
					if (recheck)
					{
						thisHit = true;
						po = i;
						heightPo = 0;
						break;
					}
				}

			}
			if (!thisHit)
			{
				for (i=0; i<topXValue; i++)
				{
					j=M_keyFrameNumber-1;
					bool endLoop = false;
					//for (j=M_keyFrameNumber-1; j>0; j--)
					{
						double formerScoreOf0 = M_myStreamMatch[j-1].weight[1][myTopClass[k].classIndex];
						double latterScoreOf0 = M_myStreamMatch[j].weight[0][myTopClass[k].classIndex];
						if (*(M_myStreamMatch[j-1].topX + j*topXValue + i) == myTopClass[k].classIndex
							&& formerScoreOf0 != latterScoreOf0&&
							latterScoreOf0 > 0.6 && (latterScoreOf0*(j+1)-formerScoreOf0*j)>0.6)
						{
							thisHit = true;
							po = i;
							heightPo = j;
							endLoop = true;
							break;
						}
					}
					if (endLoop)
					{
						break;
					}
				}
			}

			int rescue = 0;
			if (myTopClass[k].hit == 1)
			{
				rescue = 1;
			}
			else if (myTopClass[k].hit != 1 && heightPo == 0)
			{
				rescue = 2;  //It is a rescued one.
			}
			if (thisHit && rescue>0)
			{
				myTopClass[k].hit = 1;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				if (heightPo == 0)
				{
					if (rescue == 1)
					{
						myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber-1].weight[0][myTopClass[k].classIndex]; 
					}
					else if (rescue == 2)
					{
						myTopClass[k].score = 0.0;
						for (int p=0; p<M_keyFrameNumber; p++)
						{
							myTopClass[k].score += M_myStreamMatch[p].weight[0][myTopClass[k].classIndex]; 
						}
						myTopClass[k].hit = 1;
					}
				}
				else
				{
					myTopClass[k].score += M_myStreamMatch[M_keyFrameNumber -2].weight[heightPo][myTopClass[k].classIndex];
				}

			}
			else
			{
				myTopClass[k].hit = 0;
				myTopClass[k].scorePrePre = myTopClass[k].scorePre;
				myTopClass[k].scorePre = myTopClass[k].score;
				myTopClass[k].score = 0.0;
			}
		}
		for (k=0; k<topXValue; k++)
		{
			outfile_short<<myTopClass[k].classIndex<<"\t h: "<<myTopClass[k].hit<<"\t sPP: "<<myTopClass[k].scorePrePre
				<<"\t sP: "<<myTopClass[k].scorePre<<"\t s: "<<myTopClass[k].score<<endl;
		}
		outfile_short<<endl;
		return maxClass;
	}
	else
	{
		return -1;
	}
}

void S_CMatching::doMatch_tra(int sentenceID, int skeleton_start, int skeleton_end)
{
	int k, j, p;
	int traStart = skeleton_start;
	int traEnd = skeleton_end;
	vector<SLR_ST_Skeleton> skeletonData_temp;
	for (k=traStart; k<traEnd; k++)
	{
		skeletonData_temp.push_back(M_SkeletonData[k]);
	}
	traMatch traMatch_temp;
		//Initial the struct.
	traMatch_temp.startFrameID = 0;
	traMatch_temp.endFrameID = 0;
	traMatch_temp.startFrameID_match = 0;
	traMatch_temp.endFrameID_match = 0;
	traMatch_temp.bestScore = 1000;
	traMatch_temp.isMatch =false;
	traMatch_temp.matchedClass = -1;

	int justForPara[Gallery_num][Posture_num][2];
	CurveRecognition(M_HeadPoint3D[traStart], skeletonData_temp, 
		traMatch_temp.score, sentenceID,traStart,traEnd,justForPara);

	vector<double> distanceVector;
	for (j=0; j<WordsNum; j++)
	{
		distanceVector.push_back(traMatch_temp.score[j]);
	}
	sort(distanceVector.begin(),distanceVector.end());

// 	vector<scoreAndIndex> dis;
// 	for (j=0; j<WordsNum; j++)
// 	{
// 		scoreAndIndex dis_temp;
// 		dis_temp.score = traMatch_temp.score[j];
// 		dis_temp.index = j;
// 	}
// 	sort(dis.begin(),dis.end(),cmp);
// 	for (j=0; j<WordsNum; j++)
// 	{
// 		cout<<dis[j].score<<" "<<dis[j].index<<endl;
// 	}

	traMatch_temp.bestScore = distanceVector[0];

	for (p=0; p<5; p++)
	{
		for (j=0; j<WordsNum; j++)
		{
			if (traMatch_temp.score[j] == distanceVector[p])
			{
				traMatch_temp.rankIndex[p] = j;
				traMatch_temp.rankScore[p] = traMatch_temp.score[j];
			}
		}
	}

	if (traMatch_temp.bestScore < 0.5)
	{
		traMatch_temp.isMatch = true;
	}

	M_traMatchResult.push_back(traMatch_temp);
}

bool S_CMatching::comp(scoreAndIndex dis_1, scoreAndIndex dis_2)
{
	return dis_1.score > dis_2.score;
}

bool S_CMatching::comp2(scoreAndIndex dis_1, scoreAndIndex dis_2)
{
	return dis_1.score < dis_2.score;
}

void S_CMatching::manuallySegTest(int sentenceID)
{
	//Read in the ground truth.
	int sentenceNumber;
	int sentenceLength[250];
	FILE *filein;
	char oneline[255];
#ifndef onLineProcess
	filein = fopen("..\\segManually_P08_02.txt", "rt");    // File To Load World Data From
#endif
#ifdef onLineProcess
	filein = fopen(".\\resource\\segManually_P08_02.txt", "rt");    // File To Load World Data From
#endif
	readstr(filein,oneline);
	sscanf(oneline, "NUMBER %d\n", &sentenceNumber);
	for (int loop = 0; loop < sentenceNumber; loop++)
	{
		sentenceLength[loop] = 0;
		readstr(filein,oneline);
		char* sp = oneline; 
		int num; 
		int read; 
		int wordNum = 0;
		while( sscanf(sp, "%d %n", &num, &read)!=EOF )
		{ 
			//printf("%d\t", num); 
			//sentenceLength[loop]++;
			//groundTruth[loop].push_back(num);
			if (sentenceLength[loop] == 0)
			{
				myManuallySeg[loop].sentenceID = num;
			}
			else if (sentenceLength[loop]%2 == 1)
			{
				myManuallySeg[loop].beginFrame[wordNum] = num;
			}
			else if (sentenceLength[loop]%2 == 0)
			{
				myManuallySeg[loop].endFrame[wordNum++] = num;
			}
			sentenceLength[loop]++;
			sp += read-1; 
		} 
		myManuallySeg[loop].wordNum = wordNum;
	}
	fclose(filein);

// 	bool calThisSen = false;
// 	int thisSenID = -1;
// 		//for inter trajectory
// 	for (int i=0; i<50; i++)
// 	{
// 		if (myManuallySeg[i].sentenceID == sentenceID)
// 		{
// 			calThisSen = true;
// 			thisSenID = i;
// 		}
// 	}
// 	if (calThisSen)
// 	{
// 			//Intra
// 		for (int i=0; i<myManuallySeg[thisSenID].wordNum; i++)
// 		{
// 			cout<<"tra: "<<i<<endl;
// 			int skeleton_start = myManuallySeg[thisSenID].beginFrame[i];
// 			int skeleton_end = myManuallySeg[thisSenID].endFrame[i];
// 			doMatch_tra(sentenceID,skeleton_start,skeleton_end);
// 		}
// 			//Inter
// // 		for (int i=0; i<myManuallySeg[thisSenID].wordNum-1; i++)
// // 		{
// // 			cout<<"tra: "<<i<<endl;
// // 			int skeleton_start = myManuallySeg[thisSenID].endFrame[i];
// // 			int skeleton_end = myManuallySeg[thisSenID].beginFrame[i+1];
// // 			doMatch_tra(sentenceID,skeleton_start,skeleton_end);
// // 		}
// 
// // 		int skeleton_start_ = 0;
// // 		int skeleton_end_ = M_SkeletonData.size()-1;
// // 		doMatch_tra(sentenceID,skeleton_start_,skeleton_end_);
// 	}

}

void S_CMatching::readstr(FILE *f,char *string)
{
	do
	{
		fgets(string, 255, f);
	} while ((string[0] == '/') || (string[0] == '\n'));
	return;
}

void S_CMatching::readstrLong(FILE *f,char *string)
{
	do
	{
		fgets(string, 6400, f);
	} while ((string[0] == '/') || (string[0] == '\n'));
	return;
}

float S_CMatching::calSpeed(vector<SLR_ST_Skeleton> skeletonData_temp)
{
	float length_left = 0.0;
	float length_right = 0.0;
	float speed_left = 0.0;
	float speed_right = 0.0;
	float speed;
	int frameSize = skeletonData_temp.size();
	for (int k=1; k<frameSize; k++)
	{
		length_left += sqrt(pow((skeletonData_temp[k]._3dPoint[7].x - skeletonData_temp[k-1]._3dPoint[7].x),2)
			+pow((skeletonData_temp[k]._3dPoint[7].y - skeletonData_temp[k-1]._3dPoint[7].y),2)
			+pow((skeletonData_temp[k]._3dPoint[7].z - skeletonData_temp[k-1]._3dPoint[7].z),2));

		length_right += sqrt(pow((skeletonData_temp[k]._3dPoint[11].x - skeletonData_temp[k-1]._3dPoint[11].x),2)
			+pow((skeletonData_temp[k]._3dPoint[11].y - skeletonData_temp[k-1]._3dPoint[11].y),2)
			+pow((skeletonData_temp[k]._3dPoint[11].z - skeletonData_temp[k-1]._3dPoint[11].z),2));
	}
	speed_left = length_left/(frameSize + Theda);
	speed_right = length_right/(frameSize + Theda);
	speed = max(speed_left, speed_right);
	return speed;
}

int S_CMatching::signWordDetect_0614()
{
	int m, i, j, k;
		//Sort all the score
	for (m=0; m<M_keyFrameNumber; m++)
	{
		int n = M_keyFrameNumber -m - 1;
		for (i=0; i<Posture_num; i++)
		{
			M_myStreamMatch[m].weightSort[n][i].score = M_myStreamMatch[m].weight[n][i];
			M_myStreamMatch[m].weightSort[n][i].index = i;

		}
		std::sort(M_myStreamMatch[m].weightSort[n], M_myStreamMatch[m].weightSort[n]+Posture_num, comp);
	}

	m = 0;
	int length = M_keyFrameNumber - m;
	bool isCut = false;

		//To judge the "isCut"
	if (M_myStreamMatch[m].weightSort[length-1][0].score < 0.6 && length<=1)
	{
		isCut = true; 
	}
	if (/*M_myStreamMatch[m].weightSort[length-1][0].score < 0.6 &&*/ length>1)
	{
		isCut = true;
		for (k=1; k<length; k++)
		{
			bool stopIt1 = false;
			for (i=0; i<topXValue; i++)
			{
				bool stopIt2 = false;
				for (j=0; j<topXValue; j++)
				{
					if (M_myStreamMatch[m].weightSort[length-1][i].index == 
						M_myStreamMatch[m].weightSort[length-1-k][j].index)
					{
						stopIt2 = true;
						isCut = false;
						break;
					}
				}
				if (stopIt2)
				{
					break;
				}
			}
			if (stopIt1)
			{
				break;
			}
		}
	}
// 	if (M_myStreamMatch[m].weightSort[length-1][0].score > 0.6 && length>1)
// 	{
// 		bool there = false;
// 		for (i=0; i<topXValue; i++)
// 		{
// 			bool stopIt = false;
// 			for (j=0; j<topXValue; j++)
// 			{
// 				if (M_myStreamMatch[m].weightSort[length-1][i].index == 
// 					M_myStreamMatch[m].weightSort[length-2][j].index)
// 				{
// 					there = true;
// 					stopIt = true;
// 					outfile_topX<<M_myStreamMatch[m].weightSort[length-1][i].index<<endl;
// 					break;
// 				}
// 			}
// 			if (stopIt)
// 			{
// 				break;
// 			}
// 		}
// 		//outfile_topX<<"there: "<<there<<" ";
// 		for (i=0; i<topXValue; i++)
// 		{
// 			//cur[i] = M_myStreamMatch[m].weightSort[length-1][i].score;
// 			double pre = M_myStreamMatch[m].weight[length-2][M_myStreamMatch[m].weightSort[length-1][i].index];
// 			if (pre > 0.64)   //0.64 is similar enough.
// 			{
// 				there = true;
// 				outfile_topX<<M_myStreamMatch[m].weightSort[length-1][i].index<<" "<<pre<<endl;
// 				break;
// 			}
// 		}
// 		//outfile_topX<<"there: "<<there<<endl;
// 		if (!there)
// 		{
// 			isCut = true;
// 		}
// 	}
	//outfile_topX<<"isCut: "<<isCut<<endl;
	int curClassID[topXValue];
	if (isCut)
	{
		if (length == 1)
		{
			M_firstKeyFrameIndex = 0;
			isDetectSign = true;
			return -1;
		}
		for (i=0; i<topXValue; i++)
		{
			curClassID[i] = M_myStreamMatch[m].weightSort[length-2][i].index;
			cout<<curClassID[i]<<" ";
		}
		cout<<endl;
		if (curClassID[0] > 0.6)
		{
			wordClassResult.push_back(curClassID[0]);
			for (i=0; i<topXValue; i++)
			{
				wordClassResultPotential[i].push_back(curClassID[i]);
			}
				//Output some information.
			#ifdef saveFiles
			for (j=0; j<length; j++)
			{
				for (i=0; i<=j; i++)
				{
					outfile_topX<<M_myStreamMatch[i].frameIndex_start<<" ";
				}
				outfile_topX<<","<<",";

				for (i=0; i<topXValue; i++)
				{
					outfile_topX<<M_myStreamMatch[m].weightSort[j][i].index<<","<<M_myStreamMatch[m].weightSort[j][i].score<<",";
				}
				outfile_topX<<endl;
			}
			outfile_topX<<endl;
			#endif
			
			M_firstKeyFrameIndex = M_keyFrameNumber - 2;
			isDetectSign = true;

			cout<<"------"<<curClassID[0]<<" "<<curClassID[1]<<" "<<curClassID[2]<<" "<<curClassID[3]<<" "<<curClassID[4]<<endl;
			return curClassID[0];
		}
		else
		{
			M_firstKeyFrameIndex = M_keyFrameNumber - 2;
			isDetectSign = true;
			return -1;
		}
	}
	else
	{
		return -1;
	}

	return 0;
}

int S_CMatching::signWordDetect_0620()
{
	//Sort all the score
	for (int m=0; m<M_keyFrameNumber; m++)
	{
		int n = M_keyFrameNumber -m - 1;
		for (int i=0; i<Posture_num; i++)
		{
			M_myStreamMatch[m].weightSort[n][i].score = M_myStreamMatch[m].weight[n][i];
			M_myStreamMatch[m].weightSort[n][i].index = i;

		}
		std::sort(M_myStreamMatch[m].weightSort[n], M_myStreamMatch[m].weightSort[n]+Posture_num, comp);
	}
	
#ifdef useSegmentation
	double thre = 0.6;
// 	double thre2 = 0.65;
// 	double threPa = 0.92;
	double great = 0.699;
#endif

#ifndef useSegmentation
	double thre = 0.65;
	double great = 0.759;
#endif

		//Situation where key frame number is 1
	if (M_keyFrameNumber == 1)
	{
		if (M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][0].score < thre)
		{
			M_firstKeyFrameIndex = 0;
			isDetectSign = true;
			
		}
		else
		{
			for (int i=0; i<topXValue; i++)
			{
				myCurrentRank[0].index[i] = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].index;
				myCurrentRank[0].score[i] = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].score;
				myCurrentRank[0].flag[i] = 1;
			}
		}
		return -1;
	}	//Situation where key frame number is 2
	else if (M_keyFrameNumber == 2)
	{
		for (int i=0; i<topXValue; i++)
		{
			myCurrentRank[1].index[i] = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].index;
			myCurrentRank[1].score[i] = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].score;
		}
		if (M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][0].score < thre)
		{
			for (int i=0; i<topXValue; i++)
			{
				myCurrentRank[1].flag[i] = -1;
			}
		}
		else
		{
			bool isIn = false;
			for (int i=0; i<topXValue; i++)
			{
				bool stopIt2 = false;
				for (int j=0; j<topXValue; j++)
				{
					bool con1 = false;
					if (myCurrentRank[1].index[i] == myCurrentRank[0].index[j])
					{
						con1 = true;
					}
// 					if (myCurrentRank[1].index[i] == M_myStreamMatch[1].weightSort[0][j].index
// 						&& M_myStreamMatch[1].weightSort[0][j].score > thre)
// 					{
// 						con1 = true;
// 					}
					if (con1
						&& myCurrentRank[0].flag[j] == 1
						&& myCurrentRank[1].score[i] > thre
						)
					{
						stopIt2 = true;
						break;
					}
				}
				if (stopIt2)
				{
// 					int fIndex = myCurrentRank[1].index[i];
// 					if (M_myStreamMatch[M_keyFrameNumber-1].weight[0][fIndex] > thre)
					{
						myCurrentRank[1].flag[i] = 1;
					}
// 					else
// 					{
// 						myCurrentRank[1].flag[i] = -1;
// 					}
					
					isIn = true;
				}
				else
				{
					//int fIndex = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].index;
// 					int fIndex = myCurrentRank[1].index[i];
// 					double maxS1 = M_myStreamMatch[0].weightSort[M_keyFrameNumber-2][0].score;
// 					double maxS2 = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][0].score;
// 					if (M_myStreamMatch[0].weight[M_keyFrameNumber-2][fIndex] > maxS1*threPa//thre2
// 						&& M_myStreamMatch[M_keyFrameNumber-1].weight[0][fIndex] > maxS2*threPa//thre2
// 						)
// 					{
// 						myCurrentRank[1].flag[i] = 1;
// 					}
// 					else
					{
						myCurrentRank[1].flag[i] = -1;
					}
				}
			}

		}
	}	//Situation where key frame number larger than 3
	else if (M_keyFrameNumber >= 3)
	{
		if (M_keyFrameNumber > 3)
		{
			for (int j=0; j<2; j++)
			{
				for (int i=0; i<topXValue; i++)
				{
					myCurrentRank[j].flag[i] = myCurrentRank[j+1].flag[i];
					myCurrentRank[j].score[i] = myCurrentRank[j+1].score[i];
					myCurrentRank[j].index[i] = myCurrentRank[j+1].index[i];
				}
			}
		}
		for (int i=0; i<topXValue; i++)
		{
			myCurrentRank[2].index[i] = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].index;
			myCurrentRank[2].score[i] = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].score;
		}
		
			//For a situation where the rank 1 occurs twice and have high score. 
		if (myCurrentRank[0].index[0] == myCurrentRank[1].index[0] 
			&& myCurrentRank[1].score[0] > great 
			&& myCurrentRank[2].index[0] != myCurrentRank[0].index[0]
			&& alarm == 0)
		{
			alarm++;
		}
		else if (alarm == 1)
		{
			alarm++;
		}

		if (M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][0].score < thre)
		{
			for (int i=0; i<topXValue; i++)
			{
				myCurrentRank[2].flag[i] = -1;
			}
		}
		else
		{
			bool isIn = false;
			int lastTocom = 1;//M_keyFrameNumber-2;
			int sum = 0;
			for (int i=0; i<topXValue; i++)
			{
				sum += myCurrentRank[1].flag[i];
			}
			if (sum == -topXValue)
			{
				lastTocom = 0;//M_keyFrameNumber-3;
			}
			for (int i=0; i<topXValue; i++)   //current
			{
				bool stopIt2 = false;
				for (int j=0; j<topXValue; j++)   //previous
				{
					bool con1 = false;
					if (myCurrentRank[2].index[i] == myCurrentRank[lastTocom].index[j])
					{
						con1 = true;
					}
// 					if (myCurrentRank[2].index[i] == M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][j].index
// 						&& M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][j].score > thre)
// 					{
// 						con1 = true;
// 					}
					if (con1
						&& myCurrentRank[lastTocom].flag[j] == 1
						&& myCurrentRank[2].score[i] > thre)    //Add on 0627.
					{
						stopIt2 = true;
						break;
					}
				}
				if (stopIt2)
				{
					//double maxS2 = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][0].score;
// 					int fIndex = myCurrentRank[2].index[i];
// 					if (M_myStreamMatch[M_keyFrameNumber-1].weight[0][fIndex] > thre)
					{
						myCurrentRank[2].flag[i] = 1;
					}
// 					else
// 					{
// 						myCurrentRank[2].flag[i] = -1;
// 					}
					
					isIn = true;
				}
				else
				{
					//int fIndex = M_myStreamMatch[0].weightSort[M_keyFrameNumber-1][i].index;
// 					int fIndex = myCurrentRank[2].index[i];
// 					double maxS1 = M_myStreamMatch[0].weightSort[lastTocom][0].score;
// 					double maxS2 = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][0].score;
// 					if (M_myStreamMatch[0].weight[lastTocom][fIndex] > maxS1*threPa//thre2
// 						&& M_myStreamMatch[M_keyFrameNumber-1].weight[0][fIndex] > maxS2*threPa//thre2
// 						)
// 					{
// 						myCurrentRank[2].flag[i] = 1;
// 					}
// 					else
					{
						myCurrentRank[2].flag[i] = -1;
					}
				}
			}
		}
			//To judge it
		bool isCut = false;
		if (isAllFail(myCurrentRank[1].flag)
			&& isAllFail(myCurrentRank[2].flag))
		{
			isCut = true;
		}

		if (alarm == 2)
		{
			isCut = true;
			alarm = 0;
		}
		bool forceCut = false;
		if (M_keyFrameNumber == MaxKeyFrameNumber && !isCut)
		{
			cout<<"Force cut!"<<endl;
			forceCut = true;
			isCut = true;
		}

		if (isCut)
		{
			int maxClass;
			double maxScore;
			int maxLength;
			maxLength = M_keyFrameNumber - 2;
			bool isAvailable = false;
			
 			//currentRank myCurrentRank_final;
 			//myCurrentRank_final = rankReRank(M_myStreamMatch[0], 3);

			//if (myCurrentRank[0].score[0] > thre-0.02)
			bool isTransFlame = false;
			#ifdef laguageModelByWang
			isTransFlame = probabilityCheck(myCurrentRank[0], maxLength);
			#endif
			#ifndef laguageModelByWang
			isFirstAvilable = true;
			#endif
			
			if (!forceCut && !isTransFlame && isFirstAvilable)
			{
				#ifdef saveFiles
				for (int j=0; j<M_myStreamMatch[0].maxLengh; j++)
				{
					
					for (int k=0; k<=j; k++)
					{
						outfile_topX<<M_myStreamMatch[k].frameIndex_start<<" ";
					}
					outfile_topX<<","<<",";
					

					for (int i=0; i<topXValue; i++)
					{
						//outfile_topX<<myCurrentRank[0].index[i]<<","<<myCurrentRank[0].score[i]<<",";
						outfile_topX<<M_myStreamMatch[0].weightSort[j][i].index<<","<<M_myStreamMatch[0].weightSort[j][i].score<<",";
					}

					outfile_topX<<endl;
				}
				outfile_topX<<endl;
				#endif

				cout<<"Find one--------";
// 				for (int i=0; i<topXValue; i++)
// 				{
// 					cout<<myCurrentRank[0].index[i]<<" ";
// 				}
// 				cout<<endl;
				for (int i=0; i<topXValue; i++)
				{
					cout<<myCurrentRank[0].index[i]<<" ";
				}
				cout<<endl;

				wordClassResult.push_back(myCurrentRank[0].index[0]);
				
				maxClass = myCurrentRank[0].index[0];
				maxScore = myCurrentRank[0].score[0];
				#ifdef saveFiles
				outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
					<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
				outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
					<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
				#endif
				for (int k=0; k<topXValue; k++)
				{
					wordClassResultPotential[k].push_back(myCurrentRank[0].index[k]);
					wordClassScorePotential[k].push_back(myCurrentRank[0].score[k]);
					wordClassResultPotentialLengh[k].push_back(maxLength);
				}
				isAvailable = true;
			}
			
				//Change the detect flag.
			M_firstKeyFrameIndex = M_keyFrameNumber - 3;
			isDetectSign = true;
			previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

			//////////////////////////////////////////////////////////////////////////
			//Initial for the last two.
			for (int i=0; i<topXValue; i++)
			{
				myCurrentRank[0].flag[i] = 1;
				myCurrentRank[0].score[i] = M_myStreamMatch[M_keyFrameNumber-2].weightSort[0][i].score;
				myCurrentRank[0].index[i] = M_myStreamMatch[M_keyFrameNumber-2].weightSort[0][i].index;
			}

			for (int i=0; i<topXValue; i++)
			{
				myCurrentRank[1].index[i] = M_myStreamMatch[M_keyFrameNumber-2].weightSort[1][i].index;
				myCurrentRank[1].score[i] = M_myStreamMatch[M_keyFrameNumber-2].weightSort[1][i].score;
			}

			if (myCurrentRank[1].score[0] < thre)
			{
				for (int i=0; i<topXValue; i++)
				{
					myCurrentRank[1].flag[i] = -1;
				}
			}
			else
			{
				bool isIn = false;
				for (int i=0; i<topXValue; i++)  //current
				{
					bool stopIt2 = false;
					for (int j=0; j<topXValue; j++)  //previous
					{
						bool con1 = false;
						if (myCurrentRank[1].index[i] == myCurrentRank[0].index[j])
						{
							con1 = true;
						}
// 						if (myCurrentRank[1].index[i] == M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][j].index
// 							&& M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][j].score > thre)
// 						{
// 							con1 = true;
// 						}
						if (con1
							&& myCurrentRank[0].flag[j] == 1
							&& myCurrentRank[1].score[i] > thre
							)
						{
							stopIt2 = true;
							break;
						}
					}
					if (stopIt2)
					{
// 						int fIndex = myCurrentRank[1].index[i];
// 						if (M_myStreamMatch[M_keyFrameNumber-1].weight[0][fIndex] > thre)
						{
							myCurrentRank[1].flag[i] = 1;
						}
// 						else
// 						{
// 							myCurrentRank[1].flag[i] = -1;
// 						}
						
						isIn = true;
					}
					else
					{
						//int fIndex = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][i].index;
						
// 						int fIndex = myCurrentRank[1].index[i];
// 						double maxS1 = M_myStreamMatch[M_keyFrameNumber-2].weightSort[0][0].score;
// 						double maxS2 = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][0].score;
// 						if (M_myStreamMatch[M_keyFrameNumber-2].weight[0][fIndex] > maxS1*threPa//thre2
// 							&& M_myStreamMatch[M_keyFrameNumber-1].weight[0][fIndex] > maxS2*threPa//thre2
// 							)
// 						{
// 							myCurrentRank[1].flag[i] = 1;
// 						}
// 						else
						{
							myCurrentRank[1].flag[i] = -1;
						}
					}
				}

			}
			//////////////////////////////////////////////////////////////////////////

			if (isAvailable)
			{
				return maxClass;
			}
			else
			{
				return -1;
			}
			
		}
	}

	return -1;
}

	//If thre is 5 "-1", return true.
bool S_CMatching::isAllFail(int rank[])
{
	bool isCut = true;

	for (int j=0; j<topXValue; j++)
	{
		if (rank[j] == 1)
		{
			isCut = false;
			break;
		}
	}

	return isCut;
}

int S_CMatching::signWordDetect_0620_forLast()
{
	if (isDetectSign)
	{
		releaseAfterDetect();
	}

	if (M_keyFrameNumber < 1)
	{
		return -2; //Return -2 means it is an end of recognition. 
	}
	if (M_keyFrameNumber > 1)
	{
		int po = 0;
		if (M_keyFrameNumber == 2)
		{
			po = 1;
		}
		else
		{
			po = 2;
		}

		if (isAllFail(myCurrentRank[po].flag))
		{
			int maxClass = -1;
			double maxScore = 0.0;
			int maxLength;
			maxLength = M_keyFrameNumber - 1;
			bool isAvailable = false;

			//currentRank myCurrentRank_final;
			//myCurrentRank_final = rankReRank(M_myStreamMatch[0], 2);

			bool isTransFlame = false;
			#ifdef laguageModelByWang
			isTransFlame = probabilityCheck(myCurrentRank[po-1], maxLength);
			#endif
			#ifndef laguageModelByWang
			isFirstAvilable = true;
			#endif
			if (!isTransFlame && isFirstAvilable)
			{
				cout<<"Find one--------";
// 				for (int i=0; i<topXValue; i++)
// 				{
// 					cout<<myCurrentRank[po-1].index[i]<<" ";//myCurrentRank[po-1]
// 				}
// 				cout<<endl;
				for (int i=0; i<topXValue; i++)
				{
					cout<<myCurrentRank[po-1].index[i]<<" ";
				}
				cout<<endl;
				wordClassResult.push_back(myCurrentRank[po-1].index[0]);//myCurrentRank[po-1]

				maxClass = myCurrentRank[po-1].index[0];
				maxScore = myCurrentRank[po-1].score[0];

				#ifdef saveFiles
				for (int j=0; j<M_myStreamMatch[0].maxLengh; j++)
				{

					for (int k=0; k<=j; k++)
					{
						outfile_topX<<M_myStreamMatch[k].frameIndex_start<<" ";
					}
					outfile_topX<<","<<",";


					for (int i=0; i<topXValue; i++)
					{
						//outfile_topX<<myCurrentRank[0].index[i]<<","<<myCurrentRank[0].score[i]<<",";
						outfile_topX<<M_myStreamMatch[0].weightSort[j][i].index<<","<<M_myStreamMatch[0].weightSort[j][i].score<<",";
					}

					outfile_topX<<endl;
				}
				outfile_topX<<endl;

				outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
					<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
				outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
					<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
				#endif
				for (int k=0; k<topXValue; k++)
				{

					wordClassResultPotential[k].push_back(myCurrentRank[po-1].index[k]);
					wordClassScorePotential[k].push_back(myCurrentRank[po-1].score[k]);
					wordClassResultPotentialLengh[k].push_back(maxLength);
				}
			}
			

			//Change the detect flag.
			M_firstKeyFrameIndex = M_keyFrameNumber - 2;
			isDetectSign = true;
			previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;

			//Initial for the last one.
			
			for (int i=0; i<topXValue; i++)
			{
				myCurrentRank[0].flag[i] = 1;
				myCurrentRank[0].score[i] = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][i].score;
				myCurrentRank[0].index[i] = M_myStreamMatch[M_keyFrameNumber-1].weightSort[0][i].index;
				//cout<<"test:"<<myCurrentRank[0].index[i]<<endl;
			}
			
			return maxClass; 
		}
		else
		{
			int maxClass = -1;
			double maxScore = 0.0;
			int maxLength;
			maxLength = M_keyFrameNumber;
			bool isAvailable = false;

			//currentRank myCurrentRank_final;
			//myCurrentRank_final = rankReRank(M_myStreamMatch[0],1);

			bool isTransFlame = false;
			#ifdef laguageModelByWang
			isTransFlame = probabilityCheck(myCurrentRank[po], maxLength);
			#endif
			#ifndef laguageModelByWang
			isFirstAvilable = true;
			#endif
			if (!isTransFlame && isFirstAvilable)
			{
				cout<<"Find one--------";
// 				for (int i=0; i<topXValue; i++)
// 				{
// 					cout<<myCurrentRank[po].index[i]<<" ";
// 				}
// 				cout<<endl;
				for (int i=0; i<topXValue; i++)
				{
					cout<<myCurrentRank[po].index[i]<<" ";
				}
				cout<<endl;
				wordClassResult.push_back(myCurrentRank[po].index[0]);//myCurrentRank[po]
				maxClass = myCurrentRank[po].index[0];
				maxScore = myCurrentRank[po].score[0];

				#ifdef saveFiles
				for (int j=0; j<M_myStreamMatch[0].maxLengh; j++)
				{

					for (int k=0; k<=j; k++)
					{
						outfile_topX<<M_myStreamMatch[k].frameIndex_start<<" ";
					}
					outfile_topX<<","<<",";


					for (int i=0; i<topXValue; i++)
					{
						//outfile_topX<<myCurrentRank[0].index[i]<<","<<myCurrentRank[0].score[i]<<",";
						outfile_topX<<M_myStreamMatch[0].weightSort[j][i].index<<","<<M_myStreamMatch[0].weightSort[j][i].score<<",";
					}

					outfile_topX<<endl;
				}
				outfile_topX<<endl;

				outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
					<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
				outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
					<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
				#endif
				for (int k=0; k<topXValue; k++)
				{
					wordClassResultPotential[k].push_back(myCurrentRank[po].index[k]);
					wordClassScorePotential[k].push_back(myCurrentRank[po].score[k]);
					wordClassResultPotentialLengh[k].push_back(maxLength);
				}
			}
			

			//Change the detect flag.
			M_firstKeyFrameIndex = M_keyFrameNumber - 1;
			isDetectSign = true;
			previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;
			return maxClass;    
		}
		
		
	}
	else if (M_keyFrameNumber == 1)
	{
		int po = 0;
		int maxClass = -1;
		double maxScore = 0.0;
		int maxLength;
		maxLength = M_keyFrameNumber;
		bool isAvailable = false;

		//currentRank myCurrentRank_final;
		//myCurrentRank_final = rankReRank(M_myStreamMatch[0],1);

		bool isTransFlame = false;
		#ifdef laguageModelByWang
		isTransFlame = probabilityCheck(myCurrentRank[po], maxLength);
		#endif
		#ifndef laguageModelByWang
		isFirstAvilable = true;
		#endif
		if (!isTransFlame && isFirstAvilable)
		{
			cout<<"Find one--------";
// 			for (int i=0; i<topXValue; i++)
// 			{
// 				cout<<myCurrentRank[po].index[i]<<" ";
// 			}
// 			cout<<endl;
			for (int i=0; i<topXValue; i++)
			{
				cout<<myCurrentRank[po].index[i]<<" ";
			}
			cout<<endl;
			wordClassResult.push_back(myCurrentRank[po].index[0]);//myCurrentRank[po]
			maxClass = myCurrentRank[po].index[0];
			maxScore = myCurrentRank[po].score[0];

			#ifdef saveFiles
			for (int j=0; j<M_myStreamMatch[0].maxLengh; j++)
			{

				for (int k=0; k<=j; k++)
				{
					outfile_topX<<M_myStreamMatch[k].frameIndex_start<<" ";
				}
				outfile_topX<<","<<",";


				for (int i=0; i<topXValue; i++)
				{
					//outfile_topX<<myCurrentRank[0].index[i]<<","<<myCurrentRank[0].score[i]<<",";
					outfile_topX<<M_myStreamMatch[0].weightSort[j][i].index<<","<<M_myStreamMatch[0].weightSort[j][i].score<<",";
				}

				outfile_topX<<endl;
			}
			outfile_topX<<endl;

			outfile<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
				<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			outfile_short<<"FrameIndex "<<M_myStreamMatch[0].frameIndex_start<<"-"<<M_myStreamMatch[0].frameIndex_end<<" maxScore "
				<<maxScore<<" maxLength "<<maxLength<<" maxClass "<<maxClass<<endl;
			#endif
			for (int k=0; k<topXValue; k++)
			{
				wordClassResultPotential[k].push_back(myCurrentRank[po].index[k]);
				wordClassScorePotential[k].push_back(myCurrentRank[po].score[k]);
				wordClassResultPotentialLengh[k].push_back(maxLength);
			}
		}
		

		//Change the detect flag.
		M_firstKeyFrameIndex = M_keyFrameNumber - 1;
		isDetectSign = true;
		previousEndFrameIndex = M_myStreamMatch[maxLength].frameIndex_end;
		return maxClass;    
	}
}

int S_CMatching::signWordDetect_0614_forLast(ofstream &outfile, ofstream &outfile_short, ofstream &outfile_topX)
{
	int i, j, m;
	m = 0;
	int length = M_keyFrameNumber - m;
	int curClassID[topXValue];
	for (i=0; i<topXValue; i++)
	{
		curClassID[i] = M_myStreamMatch[m].weightSort[length-1][i].index;
	}
	if (curClassID[0] > 0.6)
	{
		wordClassResult.push_back(curClassID[0]);
		for (i=0; i<topXValue; i++)
		{
			wordClassResultPotential[i].push_back(curClassID[i]);
		}
		//Output some information.
		for (j=0; j<length; j++)
		{
			for (i=0; i<=j; i++)
			{
				outfile_topX<<M_myStreamMatch[i].frameIndex_start<<" ";
			}
			outfile_topX<<","<<",";

			for (i=0; i<topXValue; i++)
			{
				outfile_topX<<M_myStreamMatch[m].weightSort[j][i].index<<","<<M_myStreamMatch[m].weightSort[j][i].score<<",";
			}
			outfile_topX<<endl;
		}
		outfile_topX<<endl;

		M_firstKeyFrameIndex = M_keyFrameNumber - 2;
		isDetectSign = true;

		cout<<"------"<<curClassID[0]<<" "<<curClassID[1]<<" "<<curClassID[2]<<" "<<curClassID[3]<<" "<<curClassID[4]<<endl;
		return curClassID[0];
	}
	else
	{
		M_firstKeyFrameIndex = M_keyFrameNumber - 2;
		isDetectSign = true;
		return -1;
	}
}


void S_CMatching::initial(int maxY)
{
	myKeyframe.setHeightThres(maxY-20);
	//Dual-thread begin
	pRecvParam = new RECVPARAM;
	hThread = CreateThread(NULL, 0, RecvProc, (LPVOID)pRecvParam, 0, &thredID);
	CloseHandle(hThread);
	if(hThread == NULL)
	{
		DWORD dwError = GetLastError();
		cout<<"Error in Creating thread"<<dwError<<endl ;
	}
	alarm = 0;

	#ifdef laguageModelByWang
	readInProbability();
	readInProbability_sec();
	#endif
	//readinSentenceMask();
	isFisrtWord = true;
	isFirstAvilable = false;
	gapFrame = 0;
	isFirstReadySG = false;
	//////////////////////////////////////////////////////////////////////////
#ifdef onLineProcess
	sentenceIndex++;
#endif

#ifdef saveFiles
	

	s_filefolder.Format("..\\signOnlineOutput\\%d",sentenceIndex);
	_mkdir(s_filefolder);

	s_FileName.Format("..\\signOnlineOutput\\%d\\testMulti%d.csv",sentenceIndex,sentenceIndex);
	outfile.open(s_FileName,ios::out);
	outfile<<"PoIndex:"<<",";
	for (int i=0; i<Posture_num; i++)
	{
		outfile<<i<<",";
	}
	outfile<<endl;

	s_FileName.Format("..\\signOnlineOutput\\%d\\topX%d.csv",sentenceIndex,sentenceIndex);
	outfile_topX.open(s_FileName,ios::out);

	s_FileName.Format("..\\signOnlineOutput\\%d\\zclassAndFrame%d.txt",sentenceIndex,sentenceIndex);
	outfile_short.open(s_FileName,ios::out);
#endif
	
}

DWORD WINAPI S_CMatching::RecvProc(LPVOID lpParameter)
{
	myKeyframe.KeyframeExtractionOnline();
	return 0;
}

int S_CMatching::onlineDetect(SLR_ST_Skeleton skeletonCurrent, Mat depthCurrent, IplImage* frameCurrent,  CvPoint3D32f headPoint,
	int rank[][topXValue], double score[][topXValue])
{
	int detectSignClass = -1;  //-1 means no sign is detected. 0,1,2...are the class indexes.
	//Push the new frame to the keyFrame class.
	myKeyframe.pushImageData(skeletonCurrent,depthCurrent,frameCurrent);                                                     
	pushSkeletonData(skeletonCurrent, headPoint);  //headPoint3D: which one is the best since there are many frames.


	while(!myKeyframe.processOver());
	int detectNum = 0;
	if (myKeyframe.segmentOver == false || myKeyframe.isThereFragment())													 
	{																													     
		while(myKeyframe.isThereFragment())																					 
		{	
			//Get key frame candidates.																					 
			KeyFrameSegment tempSegment = myKeyframe.getFragment();		
			v_kfSegment.push_back(tempSegment);
			cout<<endl<<"KeyFrame: Begin ID-"<<tempSegment.BeginFrameID														 
				<<" End ID-"<<tempSegment.EndFrameID<<endl;	
			SegEnd.push_back(tempSegment.EndFrameID);
		}																													 
	}	
	return detectNum;
}


void S_CMatching::getSentenceIndex(int isentenceIndex)
{
	sentenceIndex = isentenceIndex;
}

#ifdef saveFiles
void S_CMatching::readInGroundTruth(void)
{

	//Read in the ground truth.
	FILE *filein;
	char oneline[255];
#ifndef onLineProcess
	filein = fopen("..\\sentence_wang.txt", "rt");    // File To Load World Data From
#endif
#ifdef onLineProcess
	filein = fopen(".\\resource\\sentence_wang.txt", "rt");    // File To Load World Data From
#endif
	readstr(filein,oneline);
	sscanf(oneline, "NUMBER %d\n", &sentenceNumber);
	for (int loop = 0; loop < sentenceNumber; loop++)
	{
		sentenceLength[loop] = 0;
		readstr(filein,oneline);
		char* sp = oneline; 
		int num; 
		int read; 
		while( sscanf(sp, "%d %n", &num, &read)!=EOF )
		{ 
			//printf("%d\t", num); 
			sentenceLength[loop]++;
			groundTruth[loop].push_back(num);
			sp += read-1; 
		} 
	}
	fclose(filein);

}
#endif

#ifdef saveFiles
void S_CMatching::outputForLanguageModel(void)
{

	outfile_languageModel.open("..\\signOnlineOutput\\languageModel.txt",ios::out | ios::app);
	outfile_languageModel<<1<<'\t'<<sentenceIndex<<'\t';
	for (int i=0; i<groundTruth[sentenceIndex].size(); i++)
	{
		outfile_languageModel<<groundTruth[sentenceIndex][i]<<'\t';
	}
	outfile_languageModel<<endl;

	for (int deN = 0; deN<wordClassResultPotential[0].size(); deN++)
	{
		vector<scoreAndIndex> sequence;
		outfile_languageModel<<1<<'\t'<<0<<'\t';
		for (int r=0; r<topXValue; r++)
		{
			scoreAndIndex temp;
			temp.index = wordClassResultPotential[r][deN];
			temp.score = wordClassScorePotential[r][deN];
			sequence.push_back(temp);
		}
		sort(sequence.begin(),sequence.end(),comp);
		for (int r=0; r<5; r++)
		{
			outfile_languageModel<<sequence[r].index<<'\t';
		}
		outfile_languageModel<<endl;
	}
	outfile_languageModel.close();

}
#endif

#ifdef generateGalleryKey
int S_CMatching::generateGallery(void)
{
	int i,j;
	manuallySegTest(sentenceIndex);  //Only used for readin the manually segment file.
	int wordStart[20];
	int wordEnd[20];
	int wordNumber = 0;
	ofstream replaceLog;
	for (i=0; i<244; i++)
	{
		if (myManuallySeg[i].sentenceID == sentenceIndex)
		{
			wordNumber = myManuallySeg[i].wordNum;
			for (j=0; j<wordNumber; j++)
			{
				wordStart[j] = myManuallySeg[i].beginFrame[j];
				wordEnd[j] = myManuallySeg[i].endFrame[j];
			}
		}
	}
	int seg[100];
	int segNum = vKeyFrameAll.size()*2;
	for (i=0; i<vKeyFrameAll.size(); i++)
	{
		seg[i*2 + 0] = vKeyFrameAll[i].BeginFrameID;
		seg[i*2 + 1] = vKeyFrameAll[i].EndFrameID;
	}

	for (i=0; i<wordNumber; i++)
	{
		bool last = false;
		for (j=0; j<segNum-1; j++)
		{
			if (wordStart[i]>=seg[j] && wordStart[i]<=seg[j+1])
			{
				if (j%2 == 0)
				{
					wordStart[i] = seg[j];
					break;
				}
				else
				{
					wordStart[i] = seg[j+1];
					break;
				}
			}
		}
		for (j=0; j<segNum-1; j++)
		{
			if (wordEnd[i]>=seg[j] && wordEnd[i]<=seg[j+1])
			{
				last = true;
				if (j%2 == 0)
				{
					wordEnd[i] = seg[j+1];
					break;
				}
				else
				{
					wordEnd[i] = seg[j];
					break;
				}
			}
		}
		if (!last && i == wordNumber-1)
		{
			wordEnd[i] = seg[segNum-1];
		}
		//cout<<groundTruth[sentenceIndex][i]<<" "<<wordStart[i]<<" "<<wordEnd[i]<<endl;
	}

	SYSTEMTIME sys; 

	int negCount = 0;
	for (i=0; i<wordNumber; i++)
	{
		GetLocalTime(&sys);
		vector<KeyFrameSegment> vKeyFrame;
		int currentTime = sys.wDay*24*60 + sys.wHour*60 + sys.wMinute;

		CString s_filefolder;
		WIN32_FILE_ATTRIBUTE_DATA ct;
		_SYSTEMTIME t;

		int folderTime = 0;
		bool insert = false;
		int finalPX = 0;

		for (int px=0; px<5; px++)
		{
			if (groundTruth[sentenceIndex][i]<10)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_000%d_1_0_20121002.oni\\KeyPosture",
					px, px, groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i]<100)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_00%d_1_0_20121002.oni\\KeyPosture",
					px, px, groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i]<239)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_0%d_1_0_20121002.oni\\KeyPosture",
					px, px, groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i]<1000)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_0%d_1_0_20121208.oni\\KeyPosture",
					px, px, groundTruth[sentenceIndex][i]);

			}
			//const char *lpctStr=(LPCTSTR)s_filefolder;
			char tmp[200];
			sprintf(tmp, "%s",(LPCSTR)s_filefolder); 
			GetFileAttributesEx(tmp,GetFileExInfoStandard,&ct);
			FileTimeToSystemTime(&ct.ftLastWriteTime, &t);

			folderTime = t.wDay*24*60 + (t.wHour+8)*60 + t.wMinute;
// 			if ((currentTime - folderTime) > 1*60)
// 			{
// 				char *p = (LPSTR)(LPCTSTR)s_filefolder;
// 				MyDeleteFile(p); 
// 				insert = true;
// 				finalPX = px;
// 				break;
// 			}

		}
		// 		if (finalPX > 5)
		// 		{
		// 			wordStart[i] == vKeyFrameAll[j].BeginFrameID;
		// 			cout<<finalPX<<endl;
		// 		}

		insert = true;
		if (insert)
		{
			if (groundTruth[sentenceIndex][i]<10)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_000%d_1_0_20121002.oni",
					finalPX, finalPX, groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i]<100)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_00%d_1_0_20121002.oni",
					finalPX, finalPX, groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i]<239)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_0%d_1_0_20121002.oni",
					finalPX, finalPX, groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i]<1000)
			{
				s_filefolder.Format("D:\\iData\\Kinect sign data\\Test\\20130628\\P5%d\\P5%d_0%d_1_0_20121208.oni",
					finalPX, finalPX, groundTruth[sentenceIndex][i]);

			}

			int IDstart = 0;
			int IDend = 0;
			for (j=0; j<vKeyFrameAll.size(); j++)
			{
				if (wordStart[i] == vKeyFrameAll[j].BeginFrameID)
				{
					IDstart = j;
				}
				if (wordEnd[i] == vKeyFrameAll[j].EndFrameID)
				{
					IDend = j;
				}
			}

			for (j=IDstart; j<=IDend; j++)
			{
				KeyFrameSegment tempSegment = vKeyFrameAll[j];
				//cout<<<<endl;
				vKeyFrame.push_back(tempSegment);
			}
			string route = s_filefolder.GetBuffer(0);
			//cout<<route<<" "<<vKeyFrame.size()<<endl;
			//myKeyframe.saveKeyFrameSegment(route,vKeyFrame);

			replaceLog.open("..\\posAndTraGallery\\replaceLog.txt",ios::out | ios::app);
			replaceLog<<"p5"<<finalPX<<"\t class: \t"<<groundTruth[sentenceIndex][i]<<"\t sentence: \t"<<sentenceIndex<<endl;
			replaceLog.close();

			//Trajectory
			s_filefolder.Format("..\\posAndTraGallery\\K5%d",finalPX+1);
			_mkdir(s_filefolder);

			CString s_fileName;
			if (groundTruth[sentenceIndex][i] < 10)
			{
				s_fileName.Format("..\\posAndTraGallery\\K5%d\\P5%d_000%d_1_0_20121002.oni.txt",
					finalPX+1,finalPX+1,groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i] < 100)
			{
				s_fileName.Format("..\\posAndTraGallery\\K5%d\\P5%d_00%d_1_0_20121002.oni.txt",
					finalPX+1,finalPX+1,groundTruth[sentenceIndex][i]);
			}
			else if (groundTruth[sentenceIndex][i] < 1000)
			{
				s_fileName.Format("..\\posAndTraGallery\\K5%d\\P5%d_0%d_1_0_20121002.oni.txt",
					finalPX+1,finalPX+1,groundTruth[sentenceIndex][i]);
			}

			ofstream outfile_g;
			outfile_g.open(s_fileName,ios::out);
			outfile_g<<1000*M_HeadPoint3D[wordStart[i]].x<<"\t"
				<<1000*M_HeadPoint3D[wordStart[i]].y<<"\t"
				<<1000*M_HeadPoint3D[wordStart[i]].z<<endl;
			outfile_g<<1000*M_SkeletonData[0]._3dPoint[7].x<<"\t"
				<<1000*M_SkeletonData[0]._3dPoint[7].y<<"\t"
				<<1000*M_SkeletonData[0]._3dPoint[7].z<<"\t"
				<<1000*M_SkeletonData[0]._3dPoint[11].x<<"\t"
				<<1000*M_SkeletonData[0]._3dPoint[11].y<<"\t"
				<<1000*M_SkeletonData[0]._3dPoint[11].z<<endl;
			for (j=wordStart[i]; j<wordEnd[i]; j++)
			{
				outfile_g<<1000*M_SkeletonData[j]._3dPoint[7].x<<"\t"
					<<1000*M_SkeletonData[j]._3dPoint[7].y<<"\t"
					<<1000*M_SkeletonData[j]._3dPoint[7].z<<"\t"
					<<1000*M_SkeletonData[j]._3dPoint[11].x<<"\t"
					<<1000*M_SkeletonData[j]._3dPoint[11].y<<"\t"
					<<1000*M_SkeletonData[j]._3dPoint[11].z<<endl;
			}
			outfile_g.close();
		}
	}

	return 0;
}
#endif

bool S_CMatching::MyDeleteFile(char * lpszPath) 
{ 
	SHFILEOPSTRUCT FileOp={0}; 
	FileOp.fFlags = //FOF_ALLOWUNDO |  //To recycle
		FOF_NOCONFIRMATION; //No delete dialog
	FileOp.pFrom = lpszPath; 
	FileOp.pTo = NULL;     //Must be NULL
	FileOp.wFunc = FO_DELETE;   //Delete it.
	return SHFileOperation(&FileOp) == 0; 
}

#ifdef svmTrainKey
void S_CMatching::forSVMTraining(void)
{
	//Sample of read in float probability.
	;
// 	FILE *filein;
// 	char oneline[6400];
// 	filein = fopen("..\\probability\\transProbability.txt", "rt");    // File To Load World Data From
// 	// 	readstr(filein,oneline);
// 	// 	sscanf(oneline, "NUMBER %d\n", &sentenceNumber);
// 	for (int loop = 0; loop < 370; loop++)
// 	{
// 		//sentenceLength[loop] = 0;
// 		int y = 0;
// 		readstrLong(filein,oneline);
// 		char* sp = oneline; 
// 		float num; 
// 		int read; 
// 		while( sscanf(sp, "%8f %n", &num, &read)!=EOF )
// 		{ 
// 			//printf("%d\t", num); 
// 			//sentenceLength[loop]++;
// 			//groundTruth[loop].push_back(num);
// 			transProbability[loop][y++] = num;
// 			sp += read-1; 
// 		} 
// 	}
// 	fclose(filein);

	SYSTEMTIME sys; 
	GetLocalTime( &sys );
	CString s_filefolder;

	s_filefolder.Format("..\\posAndTraGallery");
	_mkdir(s_filefolder);

	s_filefolder.Format("..\\posAndTraGallery\\positive");
	_mkdir(s_filefolder);

	s_filefolder.Format("..\\posAndTraGallery\\negative");
	_mkdir(s_filefolder);

	ofstream outfile_svm;
	outfile_svm.open("..\\posAndTraGallery\\forSvm.txt",ios::out | ios::app);

// 	ofstream outfile_transProNum;
// 	outfile_transProNum.open("..\\probability\\transProNum.txt",ios::out | ios::app);

	int transCountPositive = 0;
	int transCountNegative = 0;
	for (int i=0; i<wordClassResultPotential[0].size(); i++)
	{
		bool isNegative = false;
		bool isPositive = false;
		if (wordClassResultPotentialLengh[0][i] == 1)
		{
			for (int j=0; j<topXValue; j++)
			{
				for (int g=0; g<groundTruth[sentenceIndex].size(); g++)
				{
					if (wordClassResultPotential[j][i] == groundTruth[sentenceIndex][g])
					{
						isNegative = true;
					}
				}
			}
			if (!isNegative)
			{
				isPositive = true;
				int lastClass;

			}
		}
		
		if (isNegative)
		{
			
			s_filefolder.Format("..\\posAndTraGallery\\negative\\%d-%d-%d-%d", sys.wHour, sys.wMinute, sys.wSecond, transCountNegative);
			_mkdir(s_filefolder);

			string route = s_filefolder.GetBuffer(0);
			vector<KeyFrameSegment> vKeyFrame;
			vKeyFrame.clear();
			vKeyFrame.push_back(vKeyFrameAllSVM[i]);
			myKeyframe.saveKeyFrameSegment(route,vKeyFrame);
			transCountNegative++;

			M_myPostures.keyFrameSelect(vKeyFrame[0].RightImages,vKeyFrame[0].LeftImages,vKeyFrame[0].BothImages,				 
				vKeyFrame[0].RightNum, vKeyFrame[0].LeftNum, vKeyFrame[0].BothNum,												 
				vKeyFrame[0].BeginFrameID, vKeyFrame[0].EndFrameID,sentenceIndex);											 

			//Compute its feature.																						 
			M_myPostures.featureCalPosture();

			outfile_svm<<-1;
			for (int svmIndex = 0; svmIndex<feature_dimension; svmIndex++)
			{
				outfile_svm<<" "<<svmIndex+1<<":"<<M_myPostures.keyFeatureStream[svmIndex];
			}
			outfile_svm<<'\n';
			

			
		}
		if (isPositive)
		{
			s_filefolder.Format("..\\posAndTraGallery\\positive\\%d-%d-%d-%d", sys.wHour, sys.wMinute, sys.wSecond, transCountPositive);
			_mkdir(s_filefolder);

			string route = s_filefolder.GetBuffer(0);
			vector<KeyFrameSegment> vKeyFrame;
			vKeyFrame.clear();
			vKeyFrame.push_back(vKeyFrameAllSVM[i]);
			myKeyframe.saveKeyFrameSegment(route,vKeyFrame);
			transCountPositive++;

			M_myPostures.keyFrameSelect(vKeyFrame[0].RightImages,vKeyFrame[0].LeftImages,vKeyFrame[0].BothImages,				 
				vKeyFrame[0].RightNum, vKeyFrame[0].LeftNum, vKeyFrame[0].BothNum,												 
				vKeyFrame[0].BeginFrameID, vKeyFrame[0].EndFrameID,sentenceIndex);											 
																																	 
				//Compute its feature.																						 
			M_myPostures.featureCalPosture();

			outfile_svm<<"+"<<1;
			for (int svmIndex = 0; svmIndex<feature_dimension; svmIndex++)
			{
				outfile_svm<<" "<<svmIndex+1<<":"<<M_myPostures.keyFeatureStream[svmIndex];
			}
			outfile_svm<<'\n';
		}
	}

	outfile_svm.close();
}
#endif


void S_CMatching::releaseAfterDetect(void)
{
	isDetectSign = false;
	for (int i=M_firstKeyFrameIndex+1; i<MaxKeyFrameNumber; i++)
	{
		M_myStreamMatch[i-M_firstKeyFrameIndex-1].maxLengh = M_myStreamMatch[i].maxLengh;
		M_myStreamMatch[i-M_firstKeyFrameIndex-1].frameIndex_start = M_myStreamMatch[i].frameIndex_start;
		M_myStreamMatch[i-M_firstKeyFrameIndex-1].frameIndex_end = M_myStreamMatch[i].frameIndex_end;
		//memcpy(M_myStreamMatch[i-M_firstKeyFrameIndex-1].weight,M_myStreamMatch[i].weight,MaxKeyFrameNumber*Posture_num*sizeof(double));
		memcpy(M_myStreamMatch[i-M_firstKeyFrameIndex-1].topX,M_myStreamMatch[i].topX,Posture_num*topXValue*sizeof(int));

		for (int j=0; j<FusedLRB; j++)
		{
			M_keyFeatureStream[i-M_firstKeyFrameIndex-1][j] = M_keyFeatureStream[i][j];
		}
		for (int m=0; m<MaxKeyFrameNumber; m++)
		{
			for (int n=0; n<Posture_num; n++)
			{
				M_myStreamMatch[i-M_firstKeyFrameIndex-1].weight[m][n] = M_myStreamMatch[i].weight[m][n];
				M_myStreamMatch[i-M_firstKeyFrameIndex-1].weightSort[m][n] = M_myStreamMatch[i].weightSort[m][n];
				M_myStreamMatch[i-M_firstKeyFrameIndex-1].traScore[m][n] = M_myStreamMatch[i].traScore[m][n];
				M_myStreamMatch[i-M_firstKeyFrameIndex-1].posScore[m][n] = M_myStreamMatch[i].posScore[m][n];
				M_myStreamMatch[i-M_firstKeyFrameIndex-1].pairSum[m][n] = M_myStreamMatch[i].pairSum[m][n];
				for (int g=0; g<Gallery_num; g++)
				{

					M_myStreamMatch[i-M_firstKeyFrameIndex-1].postureMatchedIndex[m][g][n][0] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].postureMatchedIndex[m][g][n][1] = M_myStreamMatch[i].postureMatchedIndex[m][g][n][1];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].trajectoryMatchedIndex[m][g][n][0] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][0];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].trajectoryMatchedIndex[m][g][n][1] = M_myStreamMatch[i].trajectoryMatchedIndex[m][g][n][1];
					M_myStreamMatch[i-M_firstKeyFrameIndex-1].posScoreDetail[m][g][n] = M_myStreamMatch[i].posScoreDetail[m][g][n];
				}
			}

		}
	}
	M_keyFrameNumber = M_keyFrameNumber-M_firstKeyFrameIndex-1;
	M_firstKeyFrameIndex = 0;
}


void S_CMatching::readInProbability(void)
{
	FILE *filein;
	char oneline[6400];
#ifdef onLineProcess
	filein = fopen(".\\resource\\probability\\priorProbability.txt", "rt");
#endif

#ifndef onLineProcess
	filein = fopen("..\\probability\\priorProbability.txt", "rt");    // File To Load World Data From
#endif
	
	for (int loop = 0; loop < Posture_num; loop++)
	{
		int y = 0;
		int first = 0;
		readstrLong(filein,oneline);
		char* sp = oneline; 
		float num; 
		int read; 
		while( sscanf(sp, "%f %n", &num, &read)!=EOF )
		{ 
			if (first == 0)
			{
				firstProbability[loop] = num;
			}
			else if (first == 1)
			{
				priorProbability[loop] = num;
			}
			else
			{
				transProbability[loop][y++] = num;
			}
			first++;
			sp += read-1; 
		} 
	}
	fclose(filein);
}

void S_CMatching::readInProbability_sec(void)
{
	FILE *filein;
	char oneline[6400];
#ifdef onLineProcess
	filein = fopen(".\\resource\\probability\\priorProbabilitySec.txt", "rt");
#endif
#ifndef onLineProcess
	filein = fopen("..\\probability\\priorProbabilitySec.txt", "rt");    // File To Load World Data From
#endif
	
	for (int loop = 0; loop < 370; loop++)
	{
		int y = 0;
		int first = 0;
		readstrLong(filein,oneline);
		char* sp = oneline; 
		float num; 
		int read; 
		while( sscanf(sp, "%f %n", &num, &read)!=EOF )
		{ 
			transProbability_sec[loop][y++] = num;
			sp += read-1; 
		} 
	}
	fclose(filein);
}

#ifdef laguageModelByWang
bool S_CMatching::probabilityCheck(currentRank myCurrentRank, int length)
{
	bool isTransFlame = false;

	if (isFisrtWord)
	{
		cout<<"Until the first one to be trusted..."<<endl;
		for (int m=0; m<topXValue; m++)
		{
			if (firstProbability[myCurrentRank.index[m]] != 0)
			{
				isFisrtWord = false;
				isFirstAvilable = true;
				break;
			}
		}
	}
	if (wordClassResult.size()>1 && length==1)
	{
		bool isDe = false;
		for (int m=0; m<topXValue; m++)
		{
			bool stop = false;
			for (int n=0; n<topXValue; n++)
			{
				//
// 				cout<<"2 "<<wordClassResultPotential[m][wordClassResult.size()-1]<<" "
// 					<<rank[n]<<" "
// 					<<transProbability[wordClassResultPotential[m][wordClassResult.size()-1]][rank[n]]
// 				<<endl;
				if (wordClassScorePotential[m][wordClassResult.size()-1] > 0.6
					&& myCurrentRank.score[n]>0.6)
				{
					bool isCon = false;
					if (wordClassResult.size()>2)
					{
						if (transProbability[wordClassResultPotential[m][wordClassResult.size()-1]][myCurrentRank.index[n]] > 0
							|| transProbability[wordClassResultPotential[m][wordClassResult.size()-2]][myCurrentRank.index[n]] > 0)
						{
							isCon = true;
						}
					}
					else
					{
						if (transProbability[wordClassResultPotential[m][wordClassResult.size()-1]][myCurrentRank.index[n]] > 0)
						{
							isCon = true;
						}
					}

					if (isCon
						&& priorProbability[myCurrentRank.index[n]] > 0
						&& isFirstAvilable)
					{
						stop = true;
						break;
					}
				}
			}
			if (stop)
			{
				isDe = true;
				break;
			}
		}
		if (!isDe)
		{
			isTransFlame = true;
		}
	}

	return isTransFlame;
}

bool S_CMatching::probabilityCheck_oldFramwork(topClass myTopClass[], int length, int style)
{
	bool isTransFlame = false;

	if (isFisrtWord)
	{
		cout<<"Until the first one to be trusted..."<<endl;
		for (int m=0; m<topXValue; m++)
		{
			if (firstProbability[myTopClass[m].classIndex] != 0)
			{
				isFisrtWord = false;
				isFirstAvilable = true;
				break;
			}
		}
	}
	if (wordClassResult.size()>1 && length==1)
	{
		bool isDe = false;
		for (int m=0; m<topXValue; m++)
		{
			bool stop = false;
			for (int n=0; n<topXValue; n++)
			{
//
// 				cout<<"2 "<<wordClassResultPotential[m][wordClassResult.size()-1]<<" "
// 					<<rank[n]<<" "
// 					<<transProbability[wordClassResultPotential[m][wordClassResult.size()-1]][rank[n]]
// 				<<endl;
				double score = 0;
				if (style == 1)
				{
					score = myTopClass[n].score;
				}
				else if (style == 2)
				{
					score = myTopClass[n].scorePre;
				}
				else if (style == 3)
				{
					score = myTopClass[n].scorePrePre;
				}
				if (wordClassScorePotential[m][wordClassResult.size()-1] > 0.6
					&& score>0.6)
				{
					bool isCon = false;
					if (wordClassResult.size()>2)
					{
						if (transProbability[wordClassResultPotential[m][wordClassResult.size()-1]][myTopClass[n].classIndex] > 0
							|| transProbability[wordClassResultPotential[m][wordClassResult.size()-2]][myTopClass[n].classIndex] > 0)
						{
							isCon = true;
						}
					}
					else
					{
						if (transProbability[wordClassResultPotential[m][wordClassResult.size()-1]][myTopClass[n].classIndex] > 0)
						{
							isCon = true;
						}
					}

					if (isCon
						&& priorProbability[myTopClass[n].classIndex] > 0
						&& isFirstAvilable)
					{
						stop = true;
						break;
					}
				}
			}
			if (stop)
			{
				isDe = true;
				break;
			}
		}
		if (!isDe)
		{
			isTransFlame = true;
		}
	}

	return isTransFlame;
}
#endif

currentRank S_CMatching::rankReRank(streamMatch src, int style)
{
	currentRank dst;
	double thre = 0.6;
	int num;
	if (style == 3)
	{
		num = src.maxLengh - 3;
	}
	else if (style == 2)
	{
		num = src.maxLengh - 2;
	}
	else if (style == 1)
	{
		num = src.maxLengh - 1;
	}
	int dstNum = 0;
	for (int i=num; i>=0; i--)
	{
		for (int j=0; j<topXValue; j++)
		{
			bool isthere = false;
			for (int k=0; k<dstNum; k++)
			{
				if (src.weightSort[i][j].index == dst.index[k])
				{
					isthere = true;
				}
			}
			if (src.weightSort[i][j].score > thre && !isthere && dstNum<=topXValue)
			{
				dst.index[dstNum] = src.weightSort[i][j].index;
				dst.score[dstNum] = src.weightSort[i][j].score;
				dstNum++;
			}
		}
	}
	if (dstNum<topXValue)
	{
		for (int i=num; i>=0; i--)
		{
			for (int j=0; j<topXValue; j++)
			{
				bool isthere = false;
				for (int k=0; k<dstNum; k++)
				{
					if (src.weightSort[i][j].index == dst.index[k])
					{
						isthere = true;
					}
				}
				if (!isthere && dstNum<=topXValue)
				{
					dst.index[dstNum] = src.weightSort[i][j].index;
					dst.score[dstNum] = src.weightSort[i][j].score;
					dstNum++;
				}
			}
		}
	}
	return dst;
}


void S_CMatching::outPutTrajectoryGallery(void)
{
		//
	CString fileName;
	if (sentenceIndex<10)
	{
		fileName.Format("..\\posGallery\\000%d",sentenceIndex);
	}
	else if (sentenceIndex<100)
	{
		fileName.Format("..\\posGallery\\00%d",sentenceIndex);
	}
	else if (sentenceIndex<1000)
	{
		fileName.Format("..\\posGallery\\0%d",sentenceIndex);
	}

	mkdir(fileName);
	string strStl; 
	strStl=fileName.GetBuffer(0);
	myKeyframe.saveKeyFrameSegment(strStl,v_kfSegment);

		//Record the head positions. 
	ofstream outfile_head;
	outfile_head.open("..\\traGallery\\HeadPosition.dat", ios::binary | ios::app | ios::out);
	int headX = (int)(M_HeadPoint3D[0].x*1000);
	int headY = (int)(M_HeadPoint3D[0].y*1000);
	int headZ = (int)(M_HeadPoint3D[0].z*1000);
	outfile_head.write((char*)&headX, sizeof(headX));
	outfile_head.write((char*)&headY, sizeof(headY));
	outfile_head.write((char*)&headZ, sizeof(headZ));
	outfile_head.close();

		//Record the hand positions. 
	ofstream outfile_tra;
	outfile_tra.open("..\\traGallery\\Trajectory.dat", ios::binary | ios::app | ios::out);
	for (int j=0; j<M_SkeletonData.size(); j++)
	{
		int leftX = (int)(M_SkeletonData[j]._3dPoint[7].x*1000);
		int leftY = (int)(M_SkeletonData[j]._3dPoint[7].y*1000);
		int leftZ = (int)(M_SkeletonData[j]._3dPoint[7].z*1000);
		int rightX = (int)(M_SkeletonData[j]._3dPoint[11].x*1000);
		int rightY = (int)(M_SkeletonData[j]._3dPoint[11].y*1000);
		int rightZ = (int)(M_SkeletonData[j]._3dPoint[11].z*1000);
		outfile_tra.write((char*)&leftX,sizeof(leftX));
		outfile_tra.write((char*)&leftY,sizeof(leftY));
		outfile_tra.write((char*)&leftZ,sizeof(leftZ));
		outfile_tra.write((char*)&rightX,sizeof(rightX));
		outfile_tra.write((char*)&rightY,sizeof(rightY));
		outfile_tra.write((char*)&rightZ,sizeof(rightZ));
	}
	outfile_tra.close();

		//Record the label. 
	ofstream outfile_label;
	outfile_label.open("..\\traGallery\\LabelForTra.dat", ios::binary | ios::app | ios::out);
	outfile_label.write((char*)&sentenceIndex, sizeof(sentenceIndex));
	int frameNum = M_SkeletonData.size();
	outfile_label.write((char*)&frameNum, sizeof(frameNum));
	int segNum = SegEnd.size()+1; //"+1" for the whole trajectory, the last one.
	outfile_label.write((char*)&segNum, sizeof(segNum));

	for (int i=0; i<SegEnd.size(); i++)
	{
		int segFrames = SegEnd[i]-1;  //The frame index of the end of the segmentation.
		outfile_label.write((char*)&segFrames, sizeof(segFrames));
	}
	frameNum -= 1;
	outfile_label.write((char*)&frameNum, sizeof(frameNum));
	outfile_label.close();
	



		//CString  FileName_tra;
// 		//For the whole trajectory
// 	if (sentenceIndex<10)
// 	{
// 		FileName_tra.Format("..\\traGallery\\S_000%d.txt",sentenceIndex);
// 	}
// 	else if (sentenceIndex<100)
// 	{
// 		FileName_tra.Format("..\\traGallery\\S_00%d.txt",sentenceIndex);
// 	}
// 	else if (sentenceIndex<1000)
// 	{
// 		FileName_tra.Format("..\\traGallery\\S_0%d.txt",sentenceIndex);
// 	}
// 	//FileName_tra.Format("..\\traGallery\\G_%d.txt",sentenceIndex);
// 	outfile_tra.open(FileName_tra,ios::out);
// 	outfile_tra<<(int)(M_HeadPoint3D[sentenceIndex].x*1000)<<'\t'
// 		<<(int)(M_HeadPoint3D[sentenceIndex].y*1000)<<'\t'
// 		<<(int)(M_HeadPoint3D[sentenceIndex].z*1000)<<endl;
// 	for (int j=0; j<M_SkeletonData.size(); j++)
// 	{
// 		outfile_tra<<(int)(M_SkeletonData[j]._3dPoint[7].x*1000)<<'\t'
// 			<<(int)(M_SkeletonData[j]._3dPoint[7].y*1000)<<'\t'
// 			<<(int)(M_SkeletonData[j]._3dPoint[7].z*1000)<<'\t'
// 			<<(int)(M_SkeletonData[j]._3dPoint[11].x*1000)<<'\t'
// 			<<(int)(M_SkeletonData[j]._3dPoint[11].y*1000)<<'\t'
// 			<<(int)(M_SkeletonData[j]._3dPoint[11].z*1000)<<endl;
// 	}
// 	outfile_tra.close();
// 
// 		//For each state. 
// 	for (int i=0; i<SegEnd.size(); i++)
// 	{
// 		//manuallySegTest(sentenceIndex);
// 
// 		
// 		if (sentenceIndex<10)
// 		{
// 			FileName_tra.Format("..\\traGallery\\S_000%d_%d.txt",sentenceIndex,i);
// 		}
// 		else if (sentenceIndex<100)
// 		{
// 			FileName_tra.Format("..\\traGallery\\S_00%d_%d.txt",sentenceIndex,i);
// 		}
// 		else if (sentenceIndex<1000)
// 		{
// 			FileName_tra.Format("..\\traGallery\\S_0%d_%d.txt",sentenceIndex,i);
// 		}
// 		//FileName_tra.Format("..\\traGallery\\G_%d.txt",sentenceIndex);
// 		outfile_tra.open(FileName_tra,ios::out);
// 		outfile_tra<<(int)(M_HeadPoint3D[sentenceIndex].x*1000)<<'\t'
// 			<<(int)(M_HeadPoint3D[sentenceIndex].y*1000)<<'\t'
// 			<<(int)(M_HeadPoint3D[sentenceIndex].z*1000)<<endl;
// 		//for (int j=0; j<M_SkeletonData.size(); j++)
// 		for (int j=0; j<SegEnd[i]; j++)
// 		{
// 			outfile_tra<<(int)(M_SkeletonData[j]._3dPoint[7].x*1000)<<'\t'
// 				<<(int)(M_SkeletonData[j]._3dPoint[7].y*1000)<<'\t'
// 				<<(int)(M_SkeletonData[j]._3dPoint[7].z*1000)<<'\t'
// 				<<(int)(M_SkeletonData[j]._3dPoint[11].x*1000)<<'\t'
// 				<<(int)(M_SkeletonData[j]._3dPoint[11].y*1000)<<'\t'
// 				<<(int)(M_SkeletonData[j]._3dPoint[11].z*1000)<<endl;
// 		}
// 		
// 		outfile_tra.close();
// 	}

	
}


int S_CMatching::doMatch_SG(int rank[][topXValue], double score[][topXValue], int Nindex)
{
	//vector<SLR_ST_Skeleton> skeletonData_temp;
	vector<CvPoint3D32f> leftHand_probe;
	vector<CvPoint3D32f> rightHand_probe;
	CvPoint3D32f         head_probe;

	vector<CvPoint3D32f> leftHand_gallery;
	vector<CvPoint3D32f> rightHand_gallery;
	CvPoint3D32f         head_gallery;

	double thre = 200;  //for the temp
	

	int traSart = 0;
	int traEnd = M_myStreamMatch[M_keyFrameNumber-1].frameIndex_end;
	//int traEnd = M_HeadPoint3D.size();
	//gapFrame += 65;
	//int traEnd = gapFrame;
	
	head_probe.x = M_HeadPoint3D[traSart].x*1000;
	head_probe.y = M_HeadPoint3D[traSart].y*1000;
	head_probe.z = M_HeadPoint3D[traSart].z*1000;



	for (int i=traSart; i<traEnd; i++)
	{
		SLR_ST_Skeleton skeletonData_station = M_SkeletonData[i];
		//skeletonData_temp.push_back(skeletonData_station);
		CvPoint3D32f probe_temp_left;
		CvPoint3D32f probe_temp_right;
		probe_temp_left.x = skeletonData_station._3dPoint[7].x*1000;
		probe_temp_left.y = skeletonData_station._3dPoint[7].y*1000;
		probe_temp_left.z = skeletonData_station._3dPoint[7].z*1000;

		probe_temp_right.x = skeletonData_station._3dPoint[11].x*1000;
		probe_temp_right.y = skeletonData_station._3dPoint[11].y*1000;
		probe_temp_right.z = skeletonData_station._3dPoint[11].z*1000;

		leftHand_probe.push_back(probe_temp_left);
		rightHand_probe.push_back(probe_temp_right);
	}

	float probeLength = traLength(leftHand_probe, rightHand_probe);
	float galleryLength = 0.0;


	vector<float> dis[sentenceNum];
	scoreAndIndex disFinal[sentenceNum];
	for (int s=0; s<sentenceNum; s++)
	{
		int wordNum = M_traGallery[s].wordNum;

		int k = wordNum-1;
		for (int k=0; k<wordNum; k++)
		{
			leftHand_gallery = M_traGallery[s].leftHand[k];
			rightHand_gallery = M_traGallery[s].rightHand[k];

			//leftHand_gallery = get_leftHand_SG(s);
			//rightHand_gallery = get_rightHand_SG(s);


			head_gallery = M_traGallery[s].head;
			galleryLength = traLength(leftHand_gallery, rightHand_gallery);

			float disRe = 100000;// big enough
			//if (abs(probeLength - galleryLength) < thre)
			//if (sentenceMask[s] == 1)
			{
				disRe = Sentence_match(head_probe, leftHand_probe, rightHand_probe,
					head_gallery, leftHand_gallery,rightHand_gallery);
			}
// 			if (abs(probeLength - galleryLength) < thre)
// 			{
// 				cout<<probeLength<<" "<<galleryLength<<endl;
// 			}
			
			dis[s].push_back(disRe);
		}
	}

		//Get the minDis of all the sentences.
	for (int i=0; i<sentenceNum; i++)
	{
		disFinal[i].score = *min_element(dis[i].begin(),dis[i].end());
		disFinal[i].index = i;
	}

	sort(disFinal,disFinal+sentenceNum,comp2);

		//Get the posture score
	scoreAndIndex posFinal[Posture_num];
	for (int i=0; i<Posture_num; i++)
	{
		posFinal[i].score = M_myStreamMatch[0].weight[M_myStreamMatch[0].maxLengh-1][i];
		posFinal[i].index = i;
	}
	//sort(posFinal, posFinal+Posture_num, comp);

		//Get the rank 5 from the candidate sentences.
	vector<int> rank5;
	int point = 0;
	vector<int> rank5Sentence;
	while (rank5.size() < topXValue)
	{
		int classID = disFinal[point].index;
		int wordID = 0;
		for (int j=0; j<dis[classID].size(); j++)
		{
			if (dis[classID][j] == disFinal[wordID].score)
			{
				wordID = j;
			}
		}

		bool exist = false;
		for (int i=0; i<rank5.size(); i++)
		{
			if (rank5[i] == groundTruth[classID][wordID])
			{
				exist = true;
				break;
			}
		}
		if (!exist)
		{
			rank5.push_back(groundTruth[classID][wordID]);
			rank5Sentence.push_back(classID);
		}

		point++;
	}

	for (int i=0; i<topXValue; i++)
	{
		rankFlush[i].push_back(rank5[i]);
		rankFlushSentece[i].push_back(rank5Sentence[i]);
	}

		//Get the "rankFlushMask" values by previous rankFlush or posture scores. 
	if (rankFlush[0].size() < 2 /*|| !isFirstReadySG*/)
	{
		rankFlushMask.push_back(1);
	}
	else
	{
		for (int i=rankFlush[0].size()-2; i>=0; i--)
		{
			if (rankFlushMask[i] == 1)
			{
// 				int repeat = 0;
// 				for (int r=0; r<topXValue; r++)
// 				{
// 					for (int rr=0; rr<topXValue; rr++)
// 					{
// 						if (rankFlush[r][i] == rankFlush[rr][rankFlush[0].size()-1])
// 						{
// 							repeat++;
// 						}
// 					}
// 				}
// 				bool repeatFinal = false;
// 				if (repeat == 5)
// 				{
// 					repeatFinal = true;
// 				}
				bool stop = false;
				for (int j=0; j<1/*topXValue-2*/; j++)
				{
					if (rankFlush[0][i] == rankFlush[j][rankFlush[0].size()-1]
					/*&& rankFlushSentece[0][i] == rankFlushSentece[j][rankFlush[0].size()-1])*/
					/*|| repeatFinal*/)
					{
						rankFlushMask.push_back(0);
						stop = true;
						break;
					}
				}
				if (!stop)
				{
					rankFlushMask.push_back(1);
				}

// 				if (rankFlush[0][i] == rankFlush[0][rankFlush[0].size()-1])
// 				{
// 					rankFlushMask.push_back(0);
// 				}
// 				else
// 				{
// 					rankFlushMask.push_back(1);
// 				}

				break;
			}
		}
	}

		//For testing. 
// 	int last = rankFlushMask.size() - 1;
// 	if (rankFlushMask[last] == 1)
// 	{
// 		int classIID = rankFlush[0][last];
// 		cout<<"the first class score in pos: "<<posFinal[classIID].score<<endl;
// 		if (posFinal[classIID].score < 0.6)
// 		{
// 			rankFlushMask[last] = 0;
// 		}
// 		else if (posFinal[classIID].score >= 0.6 && !isFirstReadySG)
// 		{
// 			isFirstReadySG = true;
// 		}
// 	}
		//Return the ranks and scores.
	int returnNum = 0;
	for (int i=0; i<rankFlush[0].size(); i++)
	{
		if (rankFlushMask[i] == 1)
		{
			for (int j=0; j<topXValue; j++)
			{
				rank[returnNum][j] = rankFlush[j][i];
				score[returnNum][j] = 0.0;
			}
			
			returnNum += 1;
		}
	}

// 	for (int i=0; i<topXValue; i++)
// 	{
// 		rank[Nindex][i] = rank5[i];
// 		score[Nindex][i] = 0.0;
// 	}

		//To record the sentence in order to rank them.
	for (int i=0; i<topXValue; i++)
	{
		scoreAndIndex competeSen_temp;
		competeSen_temp.index = disFinal[i].index;
		bool exist = false;
		for (int j=0; j<competeSen.size(); j++)
		{
			if (competeSen_temp.index == competeSen[j].index)
			{
				competeSen[j].score += 1;
				exist = true;
			}
		}

		if (!exist)
		{
			competeSen_temp.score = 1;
			competeSen.push_back(competeSen_temp);
		}
		
	}
	
	
	return returnNum;
	//return disFinal[0].index;


}


void S_CMatching::creatTraGallery_SG(void)
{
	for (int i=0; i<sentenceNum; i++)
	{
		vector<CvPoint3D32f> lHand;
		vector<CvPoint3D32f> rHand;
		lHand = get_leftHand_SG(i);
		rHand = get_rightHand_SG(i);

		M_traGallery[i].wordNum = myManuallySeg[i].wordNum;
		M_traGallery[i].leftHand = new vector<CvPoint3D32f>[M_traGallery[i].wordNum];
		M_traGallery[i].rightHand = new vector<CvPoint3D32f>[M_traGallery[i].wordNum];


		M_traGallery[i].head.x = get_head_SG(i).x;
		M_traGallery[i].head.y = get_head_SG(i).y;
		M_traGallery[i].head.z = get_head_SG(i).z;

		for (int j=0; j<M_traGallery[i].wordNum; j++)
		{
			int start =0; 
			int end = myManuallySeg[i].endFrame[j];

			for (int k=start; k<end; k++)
			{
				M_traGallery[i].leftHand[j].push_back(lHand[k]);
				M_traGallery[i].rightHand[j].push_back(rHand[k]);
			}
			
		}

	}
}


float S_CMatching::traLength(vector<CvPoint3D32f> leftHand, vector<CvPoint3D32f> rightHand)
{
	float length_left = 0.0;
	float length_right = 0.0;
	float length = 0.0;
	for (int k=1; k<leftHand.size(); k++)
	{
		length_left += sqrt(pow((leftHand[k].x - leftHand[k-1].x),2)
			+pow((leftHand[k].y - leftHand[k-1].y),2)
			+pow((leftHand[k].z - leftHand[k-1].z),2));

		length_right += sqrt(pow((rightHand[k].x - rightHand[k-1].x),2)
			+pow((rightHand[k].y - rightHand[k-1].y),2)
			+pow((rightHand[k].z - rightHand[k-1].z),2));
	}
	length = max(length_left, length_right);
	return length;
}


int S_CMatching::doMatch_record(void)
{
	int i,j,g,m,n,k;
	double* lastDistance = new double[Gallery_num*Posture_num];
	int*    pairSumForAll = new int[Gallery_num*Posture_num];

	m=0;
	//steps 1, 2, 3, 4, 5 are severed for completing  "M_myStreamMatch".
	memset(lastDistance,0,Gallery_num*Posture_num*sizeof(double));
	memset(pairSumForAll,0,Gallery_num*Posture_num*sizeof(int));

	for (n=m; n<M_keyFrameNumber; n++)
	{
		featureToBeCom[0][n-m] = M_keyFeatureStream[n][0];
		featureToBeCom[1][n-m] = M_keyFeatureStream[n][1];
		featureToBeCom[2][n-m] = M_keyFeatureStream[n][2];
	}

	int keyFrameProbe = M_keyFrameNumber - m;
	int keyFrameGallery;
	double dou_temp = 0.0;
	int pairSum = 0;  

	//1: Measure the similarities in 5 galleries.
	for (g=0; g<Gallery_num; g++)
	{
		for (i=0; i<Posture_num; i++)
		{
			int LRBNo = 0;
			for (k=0; k<FusedLRB; k++)
			{
				keyFrameGallery = M_myGallery.getKeyFrameNo(g,i,k);
				for (j=0; j<keyFrameGallery; j++)
				{
					featureToCom[j] = M_myGallery.getGalleryValue(i,k,j,g);
				}
				if (keyFrameGallery == 0)
				{
					for (j=0; j<feature_dimension; j++)
					{
						featureToCom[0].push_back(0.0);
					}
					keyFrameGallery = 1;
				}
				pairSum = 0;
				int firstLady;
				int lastLady;
				dou_temp=Posture_Distance_new(featureToBeCom[k], keyFrameProbe, 
					featureToCom, keyFrameGallery, &pairSum, &firstLady, &lastLady);
				(*(lastDistance + g*Posture_num + i)) += dou_temp;
				(*(pairSumForAll + g*Posture_num + i)) = pairSum;
				M_myStreamMatch[m].postureMatchedIndex[M_myStreamMatch[m].maxLengh][g][i][0] = firstLady;
				M_myStreamMatch[m].postureMatchedIndex[M_myStreamMatch[m].maxLengh][g][i][1] = lastLady;
			}
		}
	}

	for (i=0; i<Posture_num; i++)
	{
		for (j=0; j<Gallery_num; j++)
		{
			M_myStreamMatch[m].posScoreDetail[M_myStreamMatch[m].maxLengh][j][i] = *(lastDistance + j*Posture_num + i);
		}
	}

	//2: Get the max weight from 5 galleries.
	double maxWeight[2];
	int maxPair[2];
	int maxGNum;
	for (i=0; i<Posture_num; i++)
	{
		//Check if the score is similar with previous one. If so, a penalty will be given. That means 
		//the current posture didn't contribute to the score and it should be punished.
		if (M_myStreamMatch[m].maxLengh>0)
		{
			for (j=0; j<Gallery_num; j++)
			{
				if ((*(lastDistance + j*Posture_num + i)) == M_myStreamMatch[m].posScoreDetail[M_myStreamMatch[m].maxLengh - 1][j][i])
				{
					*(lastDistance + j*Posture_num + i) = M_myStreamMatch[m].posScoreDetail[M_myStreamMatch[m].maxLengh - 1][j][i]
					*(*(pairSumForAll + j*Posture_num + i))/(*(pairSumForAll + j*Posture_num + i)+1);
				}
			}
		}

		for (int rank = 0; rank<2; rank++)
		{
			maxWeight[rank] = 0;
			maxPair[rank] = 0;
			maxGNum = 0;
			for (j=0; j<Gallery_num; j++)
			{
				if ((*(lastDistance + j*Posture_num + i)) > maxWeight[rank])
				{
					maxWeight[rank] = (*(lastDistance + j*Posture_num + i));
					maxGNum = j;
					maxPair[rank] = (*(pairSumForAll + j*Posture_num + i));
				}
			}
			*(lastDistance + maxGNum*Posture_num + i) = 0.0;
		}

		M_myStreamMatch[m].posScore[M_myStreamMatch[m].maxLengh][i] = (maxWeight[0]+maxWeight[1])/2;
		M_myStreamMatch[m].pairSum[M_myStreamMatch[m].maxLengh][i] = (maxPair[0] + maxPair[1])/2;
	}

	for (i=0; i<Posture_num; i++)
	{
		M_myStreamMatch[m].weight[M_myStreamMatch[m].maxLengh][i] = 
			M_myStreamMatch[m].posScore[M_myStreamMatch[m].maxLengh][i];
	}

	//5: The length of the m_th M_myStreamMatch will be added by 1.
	M_myStreamMatch[m].maxLengh += 1;


	delete[] lastDistance;
	delete[] pairSumForAll;

	return 0;
}


int S_CMatching::doMatch_SG_last(void)
{
	//vector<SLR_ST_Skeleton> skeletonData_temp;
	vector<CvPoint3D32f> leftHand_probe;
	vector<CvPoint3D32f> rightHand_probe;
	CvPoint3D32f         head_probe;

	vector<CvPoint3D32f> leftHand_gallery;
	vector<CvPoint3D32f> rightHand_gallery;
	CvPoint3D32f         head_gallery;

	double thre = 200;  //for the temp


	int traSart = 0;
	//int traEnd = M_myStreamMatch[M_keyFrameNumber-1].frameIndex_end;
	int traEnd = M_HeadPoint3D.size();
	//gapFrame += 65;
	//int traEnd = gapFrame;

	head_probe.x = M_HeadPoint3D[traSart].x*1000;
	head_probe.y = M_HeadPoint3D[traSart].y*1000;
	head_probe.z = M_HeadPoint3D[traSart].z*1000;



	for (int i=traSart; i<traEnd; i++)
	{
		SLR_ST_Skeleton skeletonData_station = M_SkeletonData[i];
		//skeletonData_temp.push_back(skeletonData_station);
		CvPoint3D32f probe_temp_left;
		CvPoint3D32f probe_temp_right;
		probe_temp_left.x = skeletonData_station._3dPoint[7].x*1000;
		probe_temp_left.y = skeletonData_station._3dPoint[7].y*1000;
		probe_temp_left.z = skeletonData_station._3dPoint[7].z*1000;

		probe_temp_right.x = skeletonData_station._3dPoint[11].x*1000;
		probe_temp_right.y = skeletonData_station._3dPoint[11].y*1000;
		probe_temp_right.z = skeletonData_station._3dPoint[11].z*1000;

		leftHand_probe.push_back(probe_temp_left);
		rightHand_probe.push_back(probe_temp_right);
	}

	float probeLength = traLength(leftHand_probe, rightHand_probe);
	float galleryLength = 0.0;


	vector<float> dis[sentenceNum];
	scoreAndIndex disFinal[sentenceNum];
	for (int s=0; s<sentenceNum; s++)
	{
		int wordNum = M_traGallery[s].wordNum;

		int k = wordNum-1;
		for (int k=0; k<wordNum; k++)
		{
			//leftHand_gallery = M_traGallery[s].leftHand[k];
			//rightHand_gallery = M_traGallery[s].rightHand[k];

			leftHand_gallery = get_leftHand_SG(s);
			rightHand_gallery = get_rightHand_SG(s);


			head_gallery = M_traGallery[s].head;
			galleryLength = traLength(leftHand_gallery, rightHand_gallery);

			float disRe = 100000;// big enough
			//if (abs(probeLength - galleryLength) < thre)
			//if (sentenceMask[s] == 1)
			{
				disRe = Sentence_match(head_probe, leftHand_probe, rightHand_probe,
					head_gallery, leftHand_gallery,rightHand_gallery);
			}
// 			if (abs(probeLength - galleryLength) < thre)
// 			{
// 				cout<<probeLength<<" "<<galleryLength<<endl;
// 			}

			dis[s].push_back(disRe);
		}
	}

	//Get the minDis of all the sentences.
	for (int i=0; i<sentenceNum; i++)
	{
		disFinal[i].score = *min_element(dis[i].begin(),dis[i].end());
		disFinal[i].index = i;
	}

	sort(disFinal,disFinal+sentenceNum,comp2);

	for (int i=0; i<topXValue; i++)
	{
		scoreAndIndex competeSen_temp;
		competeSen_temp.index = disFinal[i].index;
		bool exist = false;
		for (int j=0; j<competeSen.size(); j++)
		{
			if (competeSen_temp.index == competeSen[j].index)
			{
				competeSen[j].score += 3;
				exist = true;
			}
		}

		if (!exist)
		{
			competeSen_temp.score = 1;
			competeSen.push_back(competeSen_temp);
		}

	}

// 	scoreAndIndex posFinal[Posture_num];
// 
// 	for (int i=0; i<Posture_num; i++)
// 	{
// 		posFinal[i].score = M_myStreamMatch[0].weight[M_myStreamMatch[0].maxLengh-1][i];
// 		posFinal[i].index = i;
// 	}
// 
// 	sort(posFinal, posFinal+Posture_num, comp);

	return disFinal[0].index;


}

void S_CMatching::readinSentenceMask(void)
{
	//Read in the ground truth.
	FILE *filein;
	char oneline[255];
#ifndef onLineProcess
	filein = fopen("..\\sentenceMask.txt", "rt");    // File To Load World Data From
#endif
#ifdef onLineProcess
	filein = fopen(".\\resource\\sentenceMask.txt", "rt");    // File To Load World Data From
#endif
	readstr(filein,oneline);
	sscanf(oneline, "NUMBER %d\n", &sentenceNumber);
	for (int loop = 0; loop < sentenceNumber; loop++)
	{
		readstr(filein,oneline);
		char* sp = oneline; 
		int num; 
		int index;
		int read; 
		int jap = 0;
		while( sscanf(sp, "%d %d %n", &index, &num, &read)!=EOF )
		{ 
			sentenceMask[loop] = num;
			sp += read-1; 
		} 
	}
	fclose(filein);
}
