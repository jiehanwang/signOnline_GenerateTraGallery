#include "StdAfx.h"
#include "S_GalleryCreate.h"


S_CGalleryCreate::S_CGalleryCreate(void)
{

	feature = new vector<double> ***[Gallery_num];
	for (int g=0; g<Gallery_num; g++)
	{
		feature[g] = new vector<double> **[Posture_num];
		for (int p=0; p<Posture_num; p++)
		{
			feature[g][p] = new vector<double> *[LRB];
			for (int l=0; l<LRB; l++)
			{
				feature[g][p][l] = new vector<double> [25];
			}
		}
	}

	ikeyFrameNo=new int**[Gallery_num];
	for(int i=0;i<Gallery_num;i++)
	{
		ikeyFrameNo[i]=new int*[Posture_num];
		for(int j=0;j<Posture_num;j++)
			ikeyFrameNo[i][j]=new int[LRB];
	}

		
	int  i=0;
	for (i=0; i<Posture_num; i++)
	{
		testFlag[i]=true;
	}
}


S_CGalleryCreate::~S_CGalleryCreate(void)
{
	for (int g=0; g<Gallery_num; g++)
	{
		for(int i=0;i<Posture_num;i++)
		{
			for(int j=0;j<LRB;j++)
			{
				delete[] feature[g][i][j];
			}
			delete[] feature[g][i];
		}
		delete[] feature[g];
	}
	delete[] feature;
	
	for(int i=0;i<Gallery_num;i++)
	{

		for(int j=0;j<Posture_num;j++)
			delete[] ikeyFrameNo[i][j];
		delete[] ikeyFrameNo[i];
	}
	delete[] ikeyFrameNo;
}


void S_CGalleryCreate::galleryReadFromDat()
{
#ifndef onLineProcess
	CString route = "D:\\iData\\Kinect sign data\\NewGallery_LRToOne_20130630";
#endif
#ifdef onLineProcess
	CString route = ".\\resource\\S_PostureGallery";
#endif
	
	int i, j, k, galleryIndex, m;
	int* Label_sequence;     //Original label sequence.
	int* Label_sequence_locate;
	double* p_gallery;           //Gallery HOG

	int labelSize = Gallery_num*Posture_num*FusedLRB;  //Label_size=5*370*3
	Label_sequence = new int[labelSize];
	Label_sequence_locate = new int[labelSize];

	ifstream infile1;
	infile1.open(route+"\\Gallery_Label.dat",ios::binary);
	infile1.read( (char *)Label_sequence, labelSize*sizeof(int) );//将Gallery_Label中的数据读到数组Label_sequence1中
	infile1.close();

	int keyFrameIntotal = 0;
	*(Label_sequence_locate+0) = *(Label_sequence + 0);
	for(i=0;i<labelSize;i++)
	{
		keyFrameIntotal += *(Label_sequence + i);
		if (i>0)
		{
			*(Label_sequence_locate+i) = *(Label_sequence_locate+i-1) + *(Label_sequence + i);
		}
	}
	cout<<"Label has been read into memory"<<endl;
	int HOGsize=keyFrameIntotal * feature_dimension;//HOG_size
	p_gallery=new double[HOGsize];                         //p_gallery

	ifstream infile2;
	infile2.open(route+"\\Gallery_Data.dat",ios::binary);
	infile2.read((char*)p_gallery,HOGsize * sizeof(double));
	infile2.close();
	cout<<"Gallery has been read into the memory"<<endl;

	//////////////////////////////////////////////////////////////////////////
	// 	int testFlag[375];
	// 	for (i=0; i<Posture_num; i++)
	// 	{
	// 		infileMask>>testFlag[i];
	// 	}
	int count;
	for (galleryIndex = 0; galleryIndex<Gallery_num; galleryIndex++)
	{
		count = 0;
		for(i=0; i<Posture_num; i++)                             //Posture
		{
			//if (testFlag[i] == 1)
			{
				for(j=0;j<FusedLRB;j++)                                         //Left, right, both
				{
					ikeyFrameNo[galleryIndex][count][j] = *(Label_sequence + galleryIndex*FusedLRB*Posture_num + i*FusedLRB + j);
					int frameLocation;
					if (galleryIndex == 0 && i == 0 && j == 0)
					{
						frameLocation = 0;
					}
					else
					{
						frameLocation = *(Label_sequence_locate + galleryIndex*FusedLRB*Posture_num + i*FusedLRB + j-1);
					}

					for(k=0; k<ikeyFrameNo[galleryIndex][count][j]; k++)            //Key frame
					{
						for (m=0; m<feature_dimension; m++)
						{
							feature[galleryIndex][count][j][k].push_back(*(p_gallery + feature_dimension*(frameLocation+k) + m));
						}
					}
				}
				count +=1;
			}
		}
		cout<<"Gallery "<<galleryIndex<<" has been read into array"<<endl;
	}

	delete[] p_gallery;
	delete[] Label_sequence;
	delete[] Label_sequence_locate;
}


void S_CGalleryCreate::setDataMask()
{
	CString route = "D:\\iData\\Kinect sign data\\galleryCombine_weight";
	ifstream  infileMask; //Define input file object: infile
	infileMask.open(route,ios::in); 
	for (int i=0; i<Posture_num; i++)
	{
		infileMask>>testFlag[i];
	}
}


vector<double> S_CGalleryCreate::getGalleryValue(int wordIndex, int lrb, int keyFrameIndex, int GalleryIndex)
{
	return feature[GalleryIndex][wordIndex][lrb][keyFrameIndex];
}

void S_CGalleryCreate::getGalleryValue2(int wordIndex, int lrb, int GalleryIndex, vector<double> ifeature[])
{
	ifeature = feature[GalleryIndex][wordIndex][lrb];

}


int S_CGalleryCreate::getKeyFrameNo(int galleryIndex, int postureIndex, int lrb)
{
	return ikeyFrameNo[galleryIndex][postureIndex][lrb];
}


void S_CGalleryCreate::galleryReadFromDat_Combine(CString route)
{
	int i, j, k, galleryIndex, m;
	int* Label_sequence;     //Original label sequence.
	int* Label_sequence_locate;
	double* p_gallery;           //Gallery feature

	int labelSize = 1*Posture_num*LRB;  //Label_size=5*370*3
	Label_sequence = new int[labelSize];
	Label_sequence_locate = new int[labelSize];

	ifstream infile1;
	infile1.open(route+"\\Gallery_Label_Combine_all.dat",ios::binary);
	infile1.read( (char *)Label_sequence, labelSize*sizeof(int) );//将Gallery_Label中的数据读到数组Label_sequence1中
	infile1.close();

	int keyFrameIntotal = 0;
	*(Label_sequence_locate+0) = *(Label_sequence + 0);
	for(i=0;i<labelSize;i++)
	{
		keyFrameIntotal += *(Label_sequence + i);
		if (i>0)
		{
			*(Label_sequence_locate+i) = *(Label_sequence_locate+i-1) + *(Label_sequence + i);
		}
	}
	cout<<"Combined label has been read into memory"<<endl;

	int featuresize=keyFrameIntotal * feature_dimension;//feature_size
	p_gallery=new double[featuresize];                         //p_gallery

	ifstream infile2;
	infile2.open(route+"\\Gallery_Data_Combine_all.dat",ios::binary);
	infile2.read((char*)p_gallery,featuresize * sizeof(double));
	infile2.close();
	cout<<"Combined Gallery has been read into the memory"<<endl;

	//////////////////////////////////////////////////////////////////////////
	int count;
	galleryIndex = 0;
	count = 0;

	for(i=0; i<Posture_num; i++)                             //Posture
	{
		if (testFlag[i])
		{
			for(j=0;j<LRB;j++)                                         //Left, right, both
			{
				ikeyFrameNo[galleryIndex][count][j] = *(Label_sequence + galleryIndex*LRB*Posture_num + i*LRB + j);
				int frameLocation;
				if (galleryIndex == 0 && i == 0 && j == 0)
				{
					frameLocation = 0;
				}
				else
				{
					frameLocation = *(Label_sequence_locate + galleryIndex*LRB*Posture_num + i*LRB + j-1);
				}

				for(k=0; k<ikeyFrameNo[galleryIndex][count][j]; k++)            //Key frame
				{
					for (m=0; m<feature_dimension; m++)
					{
						feature[0][count][j][k].push_back(*(p_gallery + feature_dimension*(frameLocation+k) + m));
					}
				}
			}
			count +=1;
		}
	}

	cout<<"Combined gallery has been read into array"<<endl;
	posture_num_used = count;

	delete[] p_gallery;
	delete[] Label_sequence;
	delete[] Label_sequence_locate;
}


bool S_CGalleryCreate::getTestFlag(int index)
{
	return testFlag[index];
}
