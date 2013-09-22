#include "StdAfx.h"
#include <iostream>
#include <atlstr.h>
#include <opencv2\opencv.hpp>
#include "S_TrajectoryRecognition.h"
using namespace std;

const int sg_num=2;
const int SentencesNums=226;

CvPoint3D32f head_SG[sg_num][SentencesNums];
vector<CvPoint3D32f> lhand_SG[sg_num][SentencesNums];
vector<CvPoint3D32f> rhand_SG[sg_num][SentencesNums];

vector<int> tra_result[5];
vector<double> tra_result_score[5];
bool once;
CString P00[WordsNum];
CString P01[WordsNum];
CString P02[WordsNum];
//CString P03[WordsNum];
CString P03[WordsNum];
CString P04[WordsNum];

CString P_SG[sg_num][SentencesNums];

CString eP00[WordsNum];
CString eP01[WordsNum];
CString eP02[WordsNum];
CString eP03[WordsNum];
CString eP04[WordsNum];

//segment path
CString SP00[WordsNum];
CString SP01[WordsNum];
CString SP02[WordsNum];
CString SP03[WordsNum];
CString SP04[WordsNum];

// -----�ؼ�֡Ƭ����
// int KeyPostureNum[Gallery_num][WordsNum];
// -----�ؼ�֡Ƭ��ͷβ
// int KeyPostureBeginEnd[Gallery_num][WordsNum][MaxKeyPostureNum][2];
// int PostureMatchedIndex[Gallery_num][Posture_num][2];

float headx[num];float heady[num];float headz[num];
float lhandx[num][F];float lhandy[num][F];float lhandz[num][F];float rhandx[num][F];float rhandy[num][F];float rhandz[num][F];
int framenum[num];

//���Ա���P00 probe�����ݣ��������޸ĺ���Ҫ��ζ���
float plhandx[F];float plhandy[F];float plhandz[F];float prhandx[F];float prhandy[F];float prhandz[F];

float lE0[dim][dim];
float rE0[dim][dim];
//��¼gallery
float gheadx[WordsNum][num];float gheady[WordsNum][num];float gheadz[WordsNum][num];
float glhandx[WordsNum][num][F];
float glhandy[WordsNum][num][F];
float glhandz[WordsNum][num][F];
float grhandx[WordsNum][num][F];
float grhandy[WordsNum][num][F];
float grhandz[WordsNum][num][F];
int gframenum[WordsNum][num];
//��¼������������
float lE[WordsNum][num][dim][dim];
float rE[WordsNum][num][dim][dim];
//�켣�÷ֽ��
double score_trajectory[WordsNum];
//���ε÷ֽ��
double score_posture[WordsNum];
//�ںϹ켣�����κ�÷ֽ��
double score[WordsNum];
//�ںϺ�ĵ÷ִӴ�С���±�
int ranking_index[WordsNum];
//first hand position of every sentence
float fir_setence_lhandx;
float fir_setence_lhandy;
float fir_setence_lhandz;
float fir_setence_rhandx;
float fir_setence_rhandy;
float fir_setence_rhandz;


CvPoint3D32f get_head_SG(int sentenceID)
{
	return head_SG[0][sentenceID];
}

vector<CvPoint3D32f> get_leftHand_SG(int sentenceID)
{
	return lhand_SG[0][sentenceID];
}

vector<CvPoint3D32f> get_rightHand_SG(int sentenceID)
{
	return rhand_SG[0][sentenceID];
}

int getTra_resultSize()
{
	return tra_result[0].size();
}

int getTra_result(int index, int rank)
{
	return tra_result[rank][index];
}

double getTra_result_score(int index, int rank)
{
	return tra_result_score[rank][index];
}

void setProbeCurve(CvPoint3D32f headPoint3D, vector<SLR_ST_Skeleton> vSkeletonData)
{
	int length = vSkeletonData.size();
	float *lhandx0 = new float[length];
	float *lhandy0 = new float[length];
	float *lhandz0 = new float[length];
	float *rhandx0 = new float[length];
	float *rhandy0 = new float[length];
	float *rhandz0 = new float[length];
	for(int i=0; i<length; i++)
	{
		lhandx0[i] = 1000 * vSkeletonData[i]._3dPoint[7].x;
		lhandy0[i] = 1000 * vSkeletonData[i]._3dPoint[7].y;
		lhandz0[i] = 1000 * vSkeletonData[i]._3dPoint[7].z;
		rhandx0[i] = 1000 * vSkeletonData[i]._3dPoint[11].x;
		rhandy0[i] = 1000 * vSkeletonData[i]._3dPoint[11].y;
		rhandz0[i] = 1000 * vSkeletonData[i]._3dPoint[11].z;
	}
	float headx0 = float((int)(headPoint3D.x*1000));
	float heady0 = float((int)(headPoint3D.y*1000));
	float headz0 = float((int)(headPoint3D.z*1000));


	framenum[0]=0;
	
	//����ԭʼ���ݣ�ת��Ϊ���λ��)
	RawdataProcess(headx0,heady0,headz0,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,length);
	const int loops=3;
	MedianFliter(lhandx[0],lhandy[0],lhandz[0],framenum[0]);
	MedianFliter(rhandx[0],rhandy[0],rhandz[0],framenum[0]);
	ReSample(lhandx[0],lhandy[0],lhandz[0],framenum[0],rsnum);
	ReSample(rhandx[0],rhandy[0],rhandz[0],framenum[0],rsnum);
	for (int t=1;t<loops;t++)
	{
		ReSample(lhandx[0],lhandy[0],lhandz[0],rsnum,rsnum);
		ReSample(rhandx[0],rhandy[0],rhandz[0],rsnum,rsnum);
	}

	CopyHandData(lhandx[0],lhandy[0],lhandz[0],rhandx[0],rhandy[0],rhandz[0],plhandx,plhandy,plhandz,prhandx,prhandy,prhandz);


	delete [] lhandx0;
	delete [] lhandy0;
	delete [] lhandz0;
	delete [] rhandx0;
	delete [] rhandy0;
	delete [] rhandz0;
}

int getCurrentFrameNo(int galleryIndex)  //0 is probe
{
	return framenum[galleryIndex];
}

void getProbeCurve(CvPoint3D32f leftHand[], CvPoint3D32f rightHand[])
{
	for (int i=0; i<rsnum; i++)
	{

		leftHand[i].x = plhandx[i];
		leftHand[i].y = plhandy[i];
		leftHand[i].z = plhandz[i];


		rightHand[i].x = prhandx[i];
		rightHand[i].y = prhandy[i];
		rightHand[i].z = prhandz[i];
	}
}

void getGalleryCurve(CvPoint3D32f leftHand[], CvPoint3D32f rightHand[], int galleryIndex, int classIndex)
{
	int k = galleryIndex;
	int j = classIndex;
	int loops = 3;
	CopyHandData(glhandx[j][k],glhandy[j][k],glhandz[j][k],grhandx[j][k],grhandy[j][k],grhandz[j][k],lhandx[k],lhandy[k],lhandz[k],rhandx[k],rhandy[k],rhandz[k]);
	framenum[k]=gframenum[j][k];
	MedianFliter(lhandx[k],lhandy[k],lhandz[k],framenum[k]);
	MedianFliter(rhandx[k],rhandy[k],rhandz[k],framenum[k]);
	ReSample(lhandx[k],lhandy[k],lhandz[k],framenum[k],rsnum);
	ReSample(rhandx[k],rhandy[k],rhandz[k],framenum[k],rsnum);
	//�ڶ��ε�����ʼ����������rsnum��
	for (int t=1;t<loops;t++)
	{
		ReSample(lhandx[k],lhandy[k],lhandz[k],rsnum,rsnum);
		ReSample(rhandx[k],rhandy[k],rhandz[k],rsnum,rsnum);
	}

	for (int i=0; i<rsnum; i++)
	{

		leftHand[i].x = lhandx[galleryIndex][i];
		leftHand[i].y = lhandy[galleryIndex][i];
		leftHand[i].z = lhandz[galleryIndex][i];


		rightHand[i].x = rhandx[galleryIndex][i];
		rightHand[i].y = rhandy[galleryIndex][i];
		rightHand[i].z = rhandz[galleryIndex][i];
	}
	

}

void GetSegmentPath(CString P00[WordsNum],CString SP00[WordsNum])
{
	CString cs_person;
	CString cs_filename;

	int begin_pos=-1;
	int end_pos=-1;

	for (int i=0;i<WordsNum;i++)
	{
		begin_pos=P00[i].ReverseFind('\\');
		end_pos=P00[i].ReverseFind('.');

		if (begin_pos==-1||end_pos==-1)
		{
			cout<<P00[i]<<" file name error!"<<endl;
			return;
		}
		cs_person=P00[i].Mid(begin_pos+1,3);
		cs_filename=P00[i].Mid(begin_pos+1,end_pos-begin_pos-1);

		SP00[i]="D:\\iData\\Kinect sign data\\Test\\20130529\\"+cs_person+"\\"+cs_filename+"\\KeyPosture\\Both\\both.txt";

	}
}

bool ReadSegmentFile(CString positionfile,int &KeyPostureNum,int BeginEnd[MaxKeyPostureNum][2])
{
	FILE* fp;

	fp = fopen(positionfile, "r");
	if (fp == NULL) 
	{
		fprintf(stderr, "Error: File %s not found \n", positionfile);
		getchar();
		return false;
	}
	//get KeyPosttureNum
	fscanf(fp, "%d",&KeyPostureNum);
	if (KeyPostureNum>MaxKeyPostureNum)
	{
		fclose(fp);
		cout<<positionfile<<" error : KeyPostureNum>MaxKeyPostureNum "<<endl;
		return false;
	}
	//get the begins and ends of KeyPostture
	for (int i=0;i<KeyPostureNum;i++)
	{
		fscanf(fp, "%d",&BeginEnd[i][0]);//begin
		fscanf(fp, "%d",&BeginEnd[i][1]);//end
	}
	fclose(fp);
	return true;
}
//definition-------------------------------
int JustReadDataFile(CString positionfile,int j,int n)//j:������� n:�˵����
{
	FILE* fp;

	float flhandx,flhandy,flhandz,frhandx,frhandy,frhandz;//��һ֡�������ֵ�λ��
	int  gridheight;//����߶�
	int gridwidth;//������
	int griddepth;//�������
	//���������ͷ��λ�õ�ƫ����
	int cx;
	int cy;
	int cz;

	/* read the positionfile */
	fp = fopen(positionfile, "r");
	if (fp == NULL) 
	{
		fprintf(stderr, "Error: File %s not found \n", positionfile);
		getchar();
		return false;
	}
	fscanf(fp, "%f",&gheadx[j][n]);
	fscanf(fp, "%f",&gheady[j][n]);
	fscanf(fp, "%f",&gheadz[j][n]);




	for (int i=0;i<F;i++)
	{
		int k=0;
		k=fscanf(fp, "%f",&glhandx[j][n][i]);
		if (k==EOF)//��ס�����ļ���ĩβ
		{
			gframenum[j][n]=i;
			break;
		}

		//fscanf(fp, "%f",&lhandx[n][i]);
		fscanf(fp, "%f",&glhandy[j][n][i]);
		fscanf(fp, "%f",&glhandz[j][n][i]);
		fscanf(fp, "%f",&grhandx[j][n][i]);
		fscanf(fp, "%f",&grhandy[j][n][i]);
		fscanf(fp, "%f",&grhandz[j][n][i]);
	}

	return 1;
}
void CopyHandData(float scrlhandx[F],float scrlhandy[F],float scrlhandz[F],float scrrhandx[F],float scrrhandy[F],float scrrhandz[F],
	float dstlhandx[F],float dstlhandy[F],float dstlhandz[F],float dstrhandx[F],float dstrhandy[F],float dstrhandz[F])
{
	for (int i=0;i<F;i++)
	{
		dstlhandx[i]=scrlhandx[i];
		dstlhandy[i]=scrlhandy[i];
		dstlhandz[i]=scrlhandz[i];
		dstrhandx[i]=scrrhandx[i];
		dstrhandy[i]=scrrhandy[i];
		dstrhandz[i]=scrrhandz[i];
	}
	return;
}

void CopyHandData_seg(int begin, int end, float scrlhandx[F],float scrlhandy[F],float scrlhandz[F],float scrrhandx[F],float scrrhandy[F],float scrrhandz[F],float dstlhandx[F],float dstlhandy[F],float dstlhandz[F],float dstrhandx[F],float dstrhandy[F],float dstrhandz[F])
{
	for (int i=begin;i<end;i++)
	{
		dstlhandx[i-begin]=scrlhandx[i];
		dstlhandy[i-begin]=scrlhandy[i];
		dstlhandz[i-begin]=scrlhandz[i];
		dstrhandx[i-begin]=scrrhandx[i];
		dstrhandy[i-begin]=scrrhandy[i];
		dstrhandz[i-begin]=scrrhandz[i];
	}
	return;
}
int ReadEigDataFile(CString positionfile,int j,int k)
{
	FILE* fp;
	/* read the positionfile */
	fp = fopen(positionfile, "r");
	if (fp == NULL) 
	{
		fprintf(stderr, "Error: File %s not found \n", positionfile);
		cout<<j<<endl;
		cout<<k<<endl;
		getchar();
		return false;
	}
	fscanf(fp,"%f  ",&lE[j][k][0][0]);
	fscanf(fp,"%f  ",&lE[j][k][0][1]);
	fscanf(fp,"%f  ",&lE[j][k][0][2]);

	fscanf(fp,"%f  ",&lE[j][k][1][0]);
	fscanf(fp,"%f  ",&lE[j][k][1][1]);
	fscanf(fp,"%f  ",&lE[j][k][1][2]);

	fscanf(fp,"%f  ",&lE[j][k][2][0]);
	fscanf(fp,"%f  ",&lE[j][k][2][1]);
	fscanf(fp,"%f  ",&lE[j][k][2][2]);

	fscanf(fp,"%f  ",&rE[j][k][0][0]);
	fscanf(fp,"%f  ",&rE[j][k][0][1]);
	fscanf(fp,"%f  ",&rE[j][k][0][2]);

	fscanf(fp,"%f  ",&rE[j][k][1][0]);
	fscanf(fp,"%f  ",&rE[j][k][1][1]);
	fscanf(fp,"%f  ",&rE[j][k][1][2]);

	fscanf(fp,"%f  ",&rE[j][k][2][0]);
	fscanf(fp,"%f  ",&rE[j][k][2][1]);
	fscanf(fp,"%f  ",&rE[j][k][2][2]);

	fclose(fp);
	return 1;
}
void RawdataProcess(float headx0, float heady0, float headz0, float lhandx0[],float lhandy0[],float lhandz0[],float rhandx0[],float rhandy0[],float rhandz0[],int n)
{
	float flhandx,flhandy,flhandz,frhandx,frhandy,frhandz;//��һ֡�������ֵ�λ��
	int  gridheight;//����߶�
	int gridwidth;//������
	int griddepth;//�������
	//���������ͷ��λ�õ�ƫ����
	int cx;
	int cy;
	int cz;

	flhandx=fir_setence_lhandx;
	flhandy=fir_setence_lhandy;
	flhandz=fir_setence_lhandz;
	frhandx=fir_setence_rhandx;
	frhandy=fir_setence_rhandy;
	frhandz=fir_setence_rhandz;

	//��������������߶ȺͿ��
	gridheight=(int)abs((((flhandy+frhandy)/2-heady0)/4.5));
	gridwidth=(int)abs(((flhandx-frhandx)/2.2));
	griddepth=(int)abs(((flhandx-frhandx)/2.2));

	cx=5*gridwidth;
	cy=2*gridheight;
	cz=griddepth;

	/* scale,ȥ�����š�С�����Լ���һ */
	//ȥ����
	for (int i=0; i < n; i++) 
	{
		lhandx[0][i]=-(lhandx0[i]-headx0)+cx;
		lhandy[0][i]=-(lhandy0[i]-heady0)+cy;
		lhandz[0][i]=-(lhandz0[i]-headz0)+cz;
		rhandx[0][i]=-(rhandx0[i]-headx0)+cx;
		rhandy[0][i]=-(rhandy0[i]-heady0)+cy;
		rhandz[0][i]=-(rhandz0[i]-headz0)+cz;

		//�ǲ��Ǹ���-��
		/*lhandx[n][i]=-(lhandx[n][i]-headx[n])+cx;
		lhandy[n][i]=-(lhandy[n][i]-heady[n])+cy;
		lhandz[n][i]=-(lhandz[n][i]-headz[n])+cz;
		rhandx[n][i]=-(rhandx[n][i]-headx[n])+cx;
		rhandy[n][i]=-(rhandy[n][i]-heady[n])+cy;
		rhandz[n][i]=-(rhandz[n][i]-headz[n])+cz;*/

	}
	//��һ
	for (int i=0; i < n; i++) 
	{
		lhandx[0][i]=(lhandx[0][i])/gridwidth+1;
		lhandy[0][i]=(lhandy[0][i])/gridheight+1;
		lhandz[0][i]=(lhandz[0][i])/griddepth+1;
		rhandx[0][i]=(rhandx[0][i])/gridwidth+1;
		rhandy[0][i]=(rhandy[0][i])/gridheight+1;
		rhandz[0][i]=(rhandz[0][i])/griddepth+1;
	}

	framenum[0]=n;
}			  
int ReadDataFile(CString positionfile,int j,int n)//j:������� n:�˵����
{
	FILE* fp;

	float flhandx,flhandy,flhandz,frhandx,frhandy,frhandz;//��һ֡�������ֵ�λ��
	int  gridheight;//����߶�
	int gridwidth;//������
	int griddepth;//�������
	//���������ͷ��λ�õ�ƫ����
	int cx;
	int cy;
	int cz;

	/* read the positionfile */
	fp = fopen(positionfile, "r");
	if (fp == NULL) 
	{
		fprintf(stderr, "Error: File %s not found \n", positionfile);
		getchar();
		return false;
	}
	fscanf(fp, "%f",&gheadx[j][n]);
	fscanf(fp, "%f",&gheady[j][n]);
	fscanf(fp, "%f",&gheadz[j][n]);



	fscanf(fp, "%f",&flhandx);
	fscanf(fp, "%f",&flhandy);
	fscanf(fp, "%f",&flhandz);
	fscanf(fp, "%f",&frhandx);
	fscanf(fp, "%f",&frhandy);
	fscanf(fp, "%f",&frhandz);

	for (int i=0;i<F;i++)
	{
		int k=0;
		k=fscanf(fp, "%f",&glhandx[j][n][i]);
		if (k==EOF)//��ס�����ļ���ĩβ
		{
			gframenum[j][n]=i;
			break;
		}

		//fscanf(fp, "%f",&lhandx[n][i]);
		fscanf(fp, "%f",&glhandy[j][n][i]);
		fscanf(fp, "%f",&glhandz[j][n][i]);
		fscanf(fp, "%f",&grhandx[j][n][i]);
		fscanf(fp, "%f",&grhandy[j][n][i]);
		fscanf(fp, "%f",&grhandz[j][n][i]);
	}


	//��������������߶ȺͿ��
	gridheight=(int)abs((((flhandy+frhandy)/2.0-gheady[j][n])/4.5));
	gridwidth=(int)abs(((flhandx-frhandx)/2.2));
	griddepth=(int)abs(((flhandx-frhandx)/2.2));

	cx=5*gridwidth;
	cy=2*gridheight;
	cz=griddepth;
	fclose(fp);
	/* scale,ȥ�����š�С�����Լ���һ */
	//ȥ����
	for (int i=0; i < gframenum[j][n]; i++) 
	{
		glhandx[j][n][i]=-(glhandx[j][n][i]-gheadx[j][n])+cx;
		glhandy[j][n][i]=-(glhandy[j][n][i]-gheady[j][n])+cy;
		glhandz[j][n][i]=-(glhandz[j][n][i]-gheadz[j][n])+cz;
		grhandx[j][n][i]=-(grhandx[j][n][i]-gheadx[j][n])+cx;
		grhandy[j][n][i]=-(grhandy[j][n][i]-gheady[j][n])+cy;
		grhandz[j][n][i]=-(grhandz[j][n][i]-gheadz[j][n])+cz;

	}
	//��һ
	for (int i=0; i < gframenum[j][n]; i++) 
	{
		glhandx[j][n][i]=(glhandx[j][n][i])/gridwidth+1;
		glhandy[j][n][i]=(glhandy[j][n][i])/gridheight+1;
		glhandz[j][n][i]=(glhandz[j][n][i])/griddepth+1;
		grhandx[j][n][i]=(grhandx[j][n][i])/gridwidth+1;
		grhandy[j][n][i]=(grhandy[j][n][i])/gridheight+1;
		grhandz[j][n][i]=(grhandz[j][n][i])/griddepth+1;
	}
	return 1;
}
//��ȡ����ֵ��Ӧ��������������E��������ȣ�
//opencv ��
//int GetE(float x[F],float y[F],float z[F],float E[dim][dim],int n)//n:�ڼ�����
//{
//	//1����ʼ������
//	//ÿһ�б�ʾһ������
//	CvMat* pData = cvCreateMat(rsnum,dim,CV_32FC1);
//	CvMat* pMean = cvCreateMat(1,dim,CV_32FC1);
//
//	for (int j=0;j<rsnum;j++)
//	{
//		cvmSet(pData,j,0,x[j]);
//		cvmSet(pData,j,1,y[j]);
//		cvmSet(pData,j,2,z[j]);
//	}
//
//	//pEigVals�е�ÿ������ʾһ������ֵ
//	CvMat* pEigVals = cvCreateMat(1,dim, CV_32FC1);
//	//ÿһ�б�ʾһ����������
//	CvMat* pEigVecs = cvCreateMat(dim,dim, CV_32FC1);
//	//2��PCA����,�����ƽ������pMean,����ֵpEigVals����������pEigVecs
//	cvCalcPCA( pData, pMean, pEigVals, pEigVecs, CV_PCA_DATA_AS_ROW );
//
//	for(int i=0;i<dim;i++)
//	{
//		for (int j=0;j<dim;j++)
//		{
//			E[i][j]=cvmGet(pEigVecs,i,j);
//		}
//	}
//
//	cvReleaseMat(&pData);
//	cvReleaseMat(&pMean);
//	cvReleaseMat(&pEigVals);
//	cvReleaseMat(&pEigVecs);
//	return 1;
//}
int IsPrincipleDirMatch(float E0[dim][dim],float Ek[dim][dim])
{
	double angle=30.0;
	float cosin=(E0[0][0]*Ek[0][0]+E0[0][1]*Ek[0][1]+E0[0][2]*Ek[0][2])/(sqrt(E0[0][0]*E0[0][0]+E0[0][1]*E0[0][1]+E0[0][2]*E0[0][2])*(Ek[0][0]*Ek[0][0]+Ek[0][1]*Ek[0][1]+Ek[0][2]*Ek[0][2]));
	if (abs(cosin)>cos(angle*PI/180.0))
	{
		return 1;
	}
	else
	{
		return 0;
	}

}
//�ж�Ek��������E0�ļнǣ�������90�ȵķ���
void ChangeEkAccordingToE0(float E0[dim][dim],float Ek[dim][dim])
{
	float cosin[dim];
	cosin[0]=(E0[0][0]*Ek[0][0]+E0[0][1]*Ek[0][1]+E0[0][2]*Ek[0][2])/(sqrt(E0[0][0]*E0[0][0]+E0[0][1]*E0[0][1]+E0[0][2]*E0[0][2])*(Ek[0][0]*Ek[0][0]+Ek[0][1]*Ek[0][1]+Ek[0][2]*Ek[0][2]));
	cosin[1]=(E0[1][0]*Ek[1][0]+E0[1][1]*Ek[1][1]+E0[1][2]*Ek[1][2])/(sqrt(E0[1][0]*E0[1][0]+E0[1][1]*E0[1][1]+E0[1][2]*E0[1][2])*(Ek[1][0]*Ek[1][0]+Ek[1][1]*Ek[1][1]+Ek[1][2]*Ek[1][2]));
	cosin[2]=(E0[2][0]*Ek[2][0]+E0[2][1]*Ek[2][1]+E0[2][2]*Ek[2][2])/(sqrt(E0[2][0]*E0[2][0]+E0[2][1]*E0[2][1]+E0[2][2]*E0[2][2])*(Ek[2][0]*Ek[2][0]+Ek[2][1]*Ek[2][1]+Ek[2][2]*Ek[2][2]));
	if (cosin[0]<0)
	{
		Ek[0][0]=-1*Ek[0][0];
		Ek[0][1]=-1*Ek[0][1];
		Ek[0][2]=-1*Ek[0][2];
	}
	if (cosin[1]<0)
	{
		Ek[1][0]=-1*Ek[1][0];
		Ek[1][1]=-1*Ek[1][1];
		Ek[1][2]=-1*Ek[1][2];
	}
	if (cosin[2]<0)
	{
		Ek[2][0]=-1*Ek[2][0];
		Ek[2][1]=-1*Ek[2][1];
		Ek[2][2]=-1*Ek[2][2];
	}
	return;
}
//��ȡ���ߵ�����
//n:set num
int GetCentroid(int n,float x[F],float y[F],float z[F],float& zx,float& zy,float& zz)
{
	zx=0.0;
	zy=0.0;
	zz=0.0;
	for (int i=0;i<rsnum;i++)
	{
		zx=zx+x[i];
		zy=zy+y[i];
		zz=zz+z[i];
	}
	zx=zx/rsnum;
	zy=zy/rsnum;
	zz=zz/rsnum;
	return 0;
}
void BasisTransform(float Ek[dim][dim],float xk[],float yk[],float zk[])
{
	//�任���µĻ���
	//��ʼ������Ek Ek' (Ek')^-1
	//CvMat* ME=cvCreateMat(dim,dim,CV_32FC1);
	CvMat ME=cvMat(dim,dim,CV_32FC1,Ek);
	CvMat* MET= cvCreateMat(dim,dim,CV_32FC1);
	CvMat* InvMET= cvCreateMat(dim,dim,CV_32FC1);

	//��ME��ת��MET
	cvTranspose(&ME,MET);
	//��MET����InvMET
	cvInvert(MET,InvMET);

	//��ʼ������k�����ݾ���Dk
	float* temp=new float[dim*rsnum];
	for(int j=0;j<rsnum;j++)
	{
		*(temp+0*rsnum+j)=xk[j];
		*(temp+1*rsnum+j)=yk[j];
		*(temp+2*rsnum+j)=zk[j];
	}

	//CvMat* MData=cvCreateMat(dim,rsnum,CV_32FC1);
	CvMat MData=cvMat(dim,rsnum,CV_32FC1,temp);
	CvMat* MNewData=cvCreateMat(dim,rsnum,CV_32FC1);
	cvMatMul(InvMET, &MData, MNewData);

	for (int j=0;j<rsnum;j++)
	{
		xk[j]=cvmGet(MNewData,0,j);
		yk[j]=cvmGet(MNewData,1,j);
		zk[j]=cvmGet(MNewData,2,j);
	}

	

	//cvReleaseMat(&ME);
	cvReleaseMat(&MET);
	cvReleaseMat(&InvMET);
	//cvReleaseMat(&MData);
	cvReleaseMat(&MNewData);
	

	delete []temp;
	/*temp=NULL;
	ME=NULL;
	MET=NULL;
	InvMET=NULL;
	MData=NULL;
	MNewData=NULL;
	if (temp==NULL)
	{
	getchar();
	}*/

	return ;
}
float AdjustDirection(float x0[],float y0[],float z0[],float xk[],float yk[],float zk[],float &d)
{
	float dp=0.0;float dm=0.0;
	float Ep[dim][dim]={{0.999,0.050},{-0.050,0.999}};
	float Em[dim][dim]={{0.999,-0.050},{0.050,0.999}};
	float px0[rsnum];float py0[rsnum];float pz0[rsnum];//������˳ʱ�룩���ӽǶ�
	float mx0[rsnum];float my0[rsnum];float mz0[rsnum];//��������ʱ�룩���ӽǶ�
	float tx0[rsnum];float ty0[rsnum];float tz0[rsnum];												   //��ʱ����
	for (int i=0;i<rsnum;i++)						  
	{
		px0[i]=x0[i];
		py0[i]=y0[i];
		pz0[i]=z0[i];
		mx0[i]=x0[i];
		my0[i]=y0[i];
		mz0[i]=z0[i];
	}
	BasisTransform(Ep,px0,py0,pz0);
	BasisTransform(Em,mx0,my0,mz0);
	dp=CalDistance(px0,py0,pz0,xk,yk,zk);
	dm=CalDistance(mx0,my0,mz0,xk,yk,zk);
	if (dp<d&&dp<=dm)
	{
		while(dp<d)
		{
			for (int i=0;i<rsnum;i++)						  
			{
				tx0[i]=px0[i];
				ty0[i]=py0[i];
				tz0[i]=pz0[i];
			}
			d=dp;
			BasisTransform(Ep,px0,py0,pz0);
			dp=CalDistance(px0,py0,pz0,xk,yk,zk);
		}
		//ò�������ת��һ�Σ���������ʱ������ֵ
		for (int i=0;i<rsnum;i++)
		{
			x0[i]=tx0[i];
			y0[i]=ty0[i];
			z0[i]=tz0[i];
		}
	} 
	else
	{
		if (dm<d&&dm<dp)
		{
			while(dm<d)
			{
				for (int i=0;i<rsnum;i++)						  
				{
					tx0[i]=mx0[i];
					ty0[i]=my0[i];
					tz0[i]=mz0[i];
				}
				d=dm;
				BasisTransform(Em,mx0,my0,mz0);
				dm=CalDistance(mx0,my0,mz0,xk,yk,zk);
			}
			for (int i=0;i<rsnum;i++)
			{
				x0[i]=tx0[i];
				y0[i]=ty0[i];
				z0[i]=tz0[i];
			}
		}
	}

	return d;
}
void GetFileNames(CString firstP00,CString P00[WordsNum],HANDLE hFind2,WIN32_FIND_DATA &FindFileData2)
{
	HANDLE hFind = INVALID_HANDLE_VALUE;
	WIN32_FIND_DATA FindFileData;
	//����P00���ļ���
	hFind =FindFirstFile(firstP00,&FindFileData);
	if (hFind == INVALID_HANDLE_VALUE)
		return;
	else
	{
		P00[0]=firstP00.Left(firstP00.GetLength()-5)+FindFileData.cFileName;
		//for(int i=1;FindNextFile(hFind,&FindFileData);i++)//����������txt�ļ�
		for(int i=1;i<WordsNum;i++)
		{ 
			FindNextFile(hFind,&FindFileData);
			P00[i]=firstP00.Left(firstP00.GetLength()-5)+FindFileData.cFileName;
		}
		hFind = INVALID_HANDLE_VALUE;
	}
}
//double CalDistance(float x1[],float y1[],float z1[],float x2[],float y2[],float z2[])
//{
//	double d=0.0;
//	for(int i=0;i<rsnum;i++)
//	{
//		d=d+sqrt((x1[i]-x2[i])*(x1[i]-x2[i])+(y1[i]-y2[i])*(y1[i]-y2[i])+(z1[i]-z2[i])*(z1[i]-z2[i]));
//	}
//	return d;
//}
double CalDistance(float x1[],float y1[],float z1[],float x2[],float y2[],float z2[])
{
	double d=0.0;
	for(int i=0;i<rsnum;i++)
	{
		d=d+(x1[i]-x2[i])*(x1[i]-x2[i])+(y1[i]-y2[i])*(y1[i]-y2[i])+(z1[i]-z2[i])*(z1[i]-z2[i]);
	}
	//�����ںϣ����������濪
	//d=sqrt(d);
	return d;
}
void ReSample(float x[F],float y[F],float z[F],int n,int m)//n��ԭ���� m:Ŀ�����
{
	double len=0.0;
	double D=0.0;
	double d=0.0;
	int k=1;
	float pointx[F];
	float pointy[F];
	float pointz[F];
	for (int i=0;i<n;i++)
	{
		pointx[i]=x[i];
		pointy[i]=y[i];
		pointz[i]=z[i];
	}

	for (int j=1;j<n;j++)
	{
		len+=sqrt((pointx[j]-pointx[j-1])*(pointx[j]-pointx[j-1])+(pointy[j]-pointy[j-1])*(pointy[j]-pointy[j-1])+(pointz[j]-pointz[j-1])*(pointz[j]-pointz[j-1]));	
	}
	double I=len/(m-1);

	if (/*I==0*/I<eps)
	{
		for (int k=0;k<m;k++)
		{
			x[k]=pointx[0];
			y[k]=pointy[0];
			z[k]=pointz[0];
		}
	}
	else
	{
		for (int i=1;i<n;i++)
		{
			d=sqrt((pointx[i]-pointx[i-1])*(pointx[i]-pointx[i-1])+(pointy[i]-pointy[i-1])*(pointy[i]-pointy[i-1])+(pointz[i]-pointz[i-1])*(pointz[i]-pointz[i-1]));
			if(D+d>=I)
			{
				if (d!=0)//��ʵ�ϣ�����I!=0,d�����ﲻ�����0
				{
					x[k]=pointx[i-1]+((I-D)/d)*(pointx[i]-pointx[i-1]);
					y[k]=pointy[i-1]+((I-D)/d)*(pointy[i]-pointy[i-1]);
					z[k]=pointz[i-1]+((I-D)/d)*(pointz[i]-pointz[i-1]);
				}
				else//��ʱI==0��������㣨֮ǰ��I==0������Ѿ�������ˣ�
				{
					break;
				}


				pointx[i-1]=x[k];
				pointy[i-1]=y[k];
				pointz[i-1]=z[k];

				k++;
				i=i-1;
				D=0.0;
			}
			else
			{
				D=D+d;
			}
		}
	}

	//���������ԭ��D+d>=I�ĵȺŲ���ȷ�����һ���㱻��¼�����Ҫ�صؼ�¼����
	x[m-1]=pointx[n-1];
	y[m-1]=pointy[n-1];
	z[m-1]=pointz[n-1];
	return;
}
double DistanceOfTwoCurves(int k,float x0[],float y0[],float z0[],float xk[],float yk[],float zk[],float E0[dim][dim],float Ek[dim][dim])
{
	//start: PCA--transform to new basis
	//----------------------------------------------------------
	//clock_t starttime2 =clock();
	//�ж��������Ƿ�һ��
// 	int flag0=0;
// 	int flagk=0;
// 	for (int i=0;i<rsnum;i++)
// 	{
// 		if (abs(y0[i]-y0[0])>0.5)//Ӧ���þ���ֵ
// 		{
// 			flag0=1;
// 			break;
// 		}
// 	}
// 	for (int i=0;i<rsnum;i++)
// 	{
// 		if (abs(yk[i]-yk[0])>0.5)//Ӧ���þ���ֵ
// 		{
// 			flagk=1;
// 			break;
// 		}
// 	}
// 	//һ��һ��-----------(��Ҫ���⽫�˶����߷�������ֹ����ʱ�����Դ�������)
// 	if (flag0+flagk==1)
// 	{
// 		return 10000.0;
// 	}
// 	//����-----------������������㣩
// 	if (flag0+flagk==0)
// 	{
// 		return 0.0;
// 	}
	//����-------------
	//if (flag0+flagk==2)
	//{
	//	//�ж��������Ƿ�һ��
	//	if (!IsPrincipleDirMatch(E0,Ek))
	//	{
	//		//cout<<"curves' Principle direction  match."<<endl;
	//		//������ͬ���������켣�������ڹ̶��������ܼ����������򷵻�һ���ϴ�ľ��롣
	//		return 10000.0;
	//	}
	//}
	//�ж�Ek��������E0�ļнǣ�������90�ȵķ���
	/*ChangeEkAccordingToE0(E0,Ek);
	
	BasisTransform(E0,x0,y0,z0);
	BasisTransform(Ek,xk,yk,zk);*/

	////clock_t endtime3 =clock(); 
	////cout<<"MatMul: "<<static_cast<double>(endtime3-starttime3)/CLOCKS_PER_SEC*1000<<"ms"<<endl;
	////----------------------------------------------------------
	////end: PCA--transform to new basis

	////clock_t starttime4 =clock();


	//���Ķ���
	float zx[2]={0.0};float zy[2]={0.0};float zz[2]={0.0};


	GetCentroid(0,x0,y0,z0,zx[0],zy[0],zz[0]);
	GetCentroid(k,xk,yk,zk,zx[1],zy[1],zz[1]);


	for (int i=0;i<rsnum;i++)
	{
		x0[i]-=zx[0];
		y0[i]-=zy[0];
		z0[i]-=zz[0];
	}
	for (int i=0;i<rsnum;i++)
	{
		xk[i]-=zx[1];
		yk[i]-=zy[1];
		zk[i]-=zz[1];
	}

	////���ձ任��Ļ�����
	/*double Plen=0.0;
	double len=0.0;
	double rate=0.0;


	for (int j=1;j<rsnum;j++)
	{
		Plen+=sqrt((x0[j]-x0[j-1])*(x0[j]-x0[j-1])+(y0[j]-y0[j-1])*(y0[j]-y0[j-1])+(z0[j]-z0[j-1])*(z0[j]-z0[j-1]));

	}
	for (int j=1;j<rsnum;j++)
	{
		len+=sqrt((xk[j]-xk[j-1])*(xk[j]-xk[j-1])+(yk[j]-yk[j-1])*(yk[j]-yk[j-1])+(zk[j]-zk[j-1])*(zk[j]-zk[j-1]));
	}

	if (abs(Plen)<=eps)
	{
		rate=1.0;
	}
	else
	{
		rate=len/Plen;
	}


	for (int j=0;j<rsnum;j++)
	{
		xk[j]/=rate;
		yk[j]/=rate;
		zk[j]/=rate;
	}*/
	
	//clock_t endtime4 =clock(); 
	//cout<<"�������Ķ���: "<<static_cast<double>(endtime4-starttime4)/CLOCKS_PER_SEC*1000<<"ms"<<endl;
	//----------------------------------------------------------
	//clock_t starttime5 =clock(); 

	//�����������ߵ�DTW����
	/*DTW = new float *[framenum[k]+1]; 
	for(int i=0;i<framenum[k]+1;++i) 
		DTW[i] = new float[framenum[0]+1];
	*/

	/*for(int i=0;i<F;i++)
	{
		for(int j=0;j<F;j++)
		{
			DTW[i][j]=0.0;
		}
	}
	point handpoint[2][F];
	GetPointSeq(handpoint[0],framenum[0],x0,y0);
	GetPointSeq(handpoint[1],framenum[k],xk,yk);*/
	
	
	//d=DTWDistance(handpoint[1],framenum[k],handpoint[0],framenum[0],DTW);
	//cout<<d<<endl;

	
	/*for(int i=0;i<framenum[k]+1;++i) 
		delete[] DTW[i]; 
	delete[] DTW; */
	/*for(int i=0;i<dim;++i) 
	delete[] NewData0[i]; 
	delete[] NewData0; 
	for(int i=0;i<dim;++i) 
	delete[] NewDatak[i]; 
	delete[] NewDatak; */

	//clock_t endtime5 =clock(); 
	//cout<<"DTW "<<static_cast<double>(endtime5-starttime5)/CLOCKS_PER_SEC*1000<<"ms"<<endl;
	//getchar(); 
	float d=0.0;
	d=CalDistance(x0,y0,z0,xk,yk,zk);

	//΢������(����Ϊ2.86�ȣ�20��1�Ŀ�߱�)
	//AdjustDirection(x0,y0,z0,xk,yk,zk,d);

	return d;
}
void InitData()
{
	//������ȡ�ļ���
#ifndef onLineProcess
	CString firstP00="D:\\iData\\Kinect sign data\\trajectory\\D3_clean_20130616\\K50\\*.txt";
	CString firstP01="D:\\iData\\Kinect sign data\\trajectory\\D3_clean_20130616\\K51\\*.txt";
	CString firstP02="D:\\iData\\Kinect sign data\\trajectory\\D3_clean_20130616\\K52\\*.txt";
	CString firstP03="D:\\iData\\Kinect sign data\\trajectory\\D3_clean_20130616\\K53\\*.txt";
	CString firstP04="D:\\iData\\Kinect sign data\\trajectory\\D3_clean_20130616\\K54\\*.txt";

	CString firtst_SG = "..\\traGallery\\*.txt";
#endif
	
#ifdef onLineProcess
	CString firstP00=".\\resource\\D3_clean_20130616\\K50\\*.txt";
	CString firstP01=".\\resource\\D3_clean_20130616\\K51\\*.txt";
	CString firstP02=".\\resource\\D3_clean_20130616\\K52\\*.txt";
	CString firstP03=".\\resource\\D3_clean_20130616\\K53\\*.txt";
	CString firstP04=".\\resource\\D3_clean_20130616\\K54\\*.txt";
	CString firtst_SG = ".\\resource\\traGallery\\*.txt";
#endif
	WIN32_FIND_DATA FindFileData;
	HANDLE hFind=INVALID_HANDLE_VALUE;

	//����P00���ļ���
	//GetFileNames(firstP00,P00,hFind,FindFileData);
	//����P01���ļ���
	GetFileNames(firstP01,P01,hFind,FindFileData);
	//����P02���ļ���
	GetFileNames(firstP02,P02,hFind,FindFileData);
	//����P03���ļ���
	GetFileNames(firstP03,P03,hFind,FindFileData);
	//����P04���ļ���
	GetFileNames(firstP04,P04,hFind,FindFileData);

	GetFileNames(firtst_SG,P_SG[0],hFind,FindFileData);

	//-----����gallery data------
	for (int j=0;j<WordsNum;j++)
	{
		//cout<<j<<endl;
		//ReadDataFile(P00[j],j,0);
		ReadDataFile(P01[j],j,1);
		ReadDataFile(P02[j],j,2);
		ReadDataFile(P03[j],j,3);
		ReadDataFile(P04[j],j,4);
	}

	for (int j=0; j<SentencesNums; j++)
	{
		Read_Sentece_File(P_SG[0][j],j,0);
	}



// 	//GetSegmentPath(P00,SP00);
// 	GetSegmentPath(P01,SP01);
// 	GetSegmentPath(P02,SP02);
// 	GetSegmentPath(P03,SP03);
// 	GetSegmentPath(P04,SP04);
// 
// 	for (int i=0;i<WordsNum;i++)
// 	{  
// 		//cout<<"WordNum: "<<i<<endl;
// 		//ReadSegmentFile(SP00[i], KeyPostureNum[0][i], KeyPostureBeginEnd[0][i]);
// 		ReadSegmentFile(SP01[i], KeyPostureNum[1][i], KeyPostureBeginEnd[1][i]);
// 		ReadSegmentFile(SP02[i], KeyPostureNum[2][i], KeyPostureBeginEnd[2][i]);
// 		ReadSegmentFile(SP03[i], KeyPostureNum[3][i], KeyPostureBeginEnd[3][i]);
// 		ReadSegmentFile(SP04[i], KeyPostureNum[4][i], KeyPostureBeginEnd[4][i]);
// 	}
	once = true;
	//������ȡeig�ļ���

	//CString firsteP00=".\\NewDirData\\re3_3D_K\\K50\\*.txt";
	//CString firsteP01=".\\NewDirData\\re3_3D_K\\K51\\*.txt";
	//CString firsteP02=".\\NewDirData\\re3_3D_K\\K52\\*.txt";
	//CString firsteP06=".\\NewDirData\\re3_3D_K\\K53\\*.txt";
	//CString firsteP08=".\\NewDirData\\re3_3D_K\\K54\\*.txt";

	////CString firsteP00=".\\NewDirData\\re3\\P00\\*.txt";
	///*CString firsteP01=".\\NewDirData\\re3\\P01\\*.txt";
	//CString firsteP02=".\\NewDirData\\re3\\P02\\*.txt";
	//CString firsteP06=".\\NewDirData\\re3\\P06\\*.txt";
	//CString firsteP08=".\\NewDirData\\re3\\P08\\*.txt";*/
 //   
	//hFind=INVALID_HANDLE_VALUE;
	////����eP00���ļ���
	////GetFileNames(firsteP00,eP00,hFind,FindFileData);
	////����eP01���ļ���	 
	//GetFileNames(firsteP01,eP01,hFind,FindFileData);
	////����eP02���ļ���	 
	//GetFileNames(firsteP02,eP02,hFind,FindFileData);
	////����eP06���ļ���	 
	//GetFileNames(firsteP06,eP06,hFind,FindFileData);
	////����eP08���ļ���	 
	//GetFileNames(firsteP08,eP08,hFind,FindFileData);

	////-----����Eig����------
	//for (int j=0;j<WordsNum;j++)
	//{

	//	//ReadEigDataFile(eP00[j],j,0);
	//	ReadEigDataFile(eP01[j],j,1);
	//	ReadEigDataFile(eP02[j],j,2);
	//	ReadEigDataFile(eP06[j],j,3);
	//	ReadEigDataFile(eP08[j],j,4);

	//}
	return;
}
float GetMedian(float x[],int winlen)//n�Ǵ��ڵĳ��ȣ�Ӧ��������
{
	float median=0.0;
	float temp=0.0;
	int med;
	med=winlen/2;
	for (int i=winlen-1;i>0;i--)
	{
		for (int j=0;j<i;j++)
		{
			if (x[j]>x[j+1])
			{
				temp=x[j+1];
				x[j+1]=x[j];
				x[j]=temp;
			}
		}
	}
	median=x[med];
	return median;
}
void MedianFliter(float x[],float y[],float z[],int n)//n������x,y,z�ĳ���
{
	const int winlen=5;//winlen������
	if (winlen%2==0)
	{
		cout<<"winlen should be an odd num."<<endl;
		return;
	}
	float winx[winlen];
	float winy[winlen];
	float winz[winlen];
	int med;
	med=winlen/2;
	float ox[F];
	float oy[F];
	float oz[F];
	for (int i=0;i<n;i++)
	{
		ox[i]=x[i];
		oy[i]=y[i];
		oz[i]=z[i];
	}
	for(int i=0;i<n;i++)
	{
		for (int j=0;j<winlen;j++)
		{
			if (i-med+j<0)
			{
				winx[j]=ox[0];
				winy[j]=oy[0];
				winz[j]=oz[0];
			}
			if (i-med+j>n-1)
			{
				winx[j]=ox[n-1];
				winy[j]=oy[n-1];
				winz[j]=oz[n-1];
			}
			if(i-med+j>=0&&i-med+j<n)
			{
				winx[j]=ox[i-med+j];
				winy[j]=oy[i-med+j];
				winz[j]=oz[i-med+j];
			}
		}
		x[i]=GetMedian(winx,winlen);
		y[i]=GetMedian(winy,winlen);
		z[i]=GetMedian(winz,winlen);
	}
	return;
}
int CurveRecognision(float headx0, float heady0, float headz0
	, float lhandx0[],float lhandy0[], float lhandz0[]
	, float rhandx0[],float rhandy0[], float rhandz0[]
	,int n,double score[WordsNum]
	,int sentenceID,int startFrameID, int endFrameID, 
	int trajectoryMatchedIndex[][Posture_num][2])
{
	for(int i=0;i<num;i++)
	{
		framenum[i]=0;
	}
	//Record the probe before "MedianFliter" and "ReSample".
	CvPoint3D32f oriLeftHand[512];
	CvPoint3D32f oriRightHand[512];
	for (int i=0;i<n;i++)
	{
		oriLeftHand[i].x = lhandx0[i]/1000;
		oriLeftHand[i].y = lhandy0[i]/1000;
		oriLeftHand[i].z = lhandz0[i]/1000;
		oriRightHand[i].x = rhandx0[i]/1000;
		oriRightHand[i].y = rhandy0[i]/1000;
		oriRightHand[i].z = rhandz0[i]/1000;
	}
		//Process original data 
		//(change them to the relative position according to human head and initial position of hand)
	RawdataProcess(headx0,heady0,headz0
		,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,n);


	//The first frame is the original of hand, which is served for "RawdataProcess". 
	//So, it is removed in that function.
	int probeSize =(endFrameID-startFrameID)>0?(endFrameID-startFrameID):0;

		//
	const int loops=3;
	MedianFliter(lhandx[0],lhandy[0],lhandz[0],framenum[0]);
	MedianFliter(rhandx[0],rhandy[0],rhandz[0],framenum[0]);
	//ReSample(lhandx[0],lhandy[0],lhandz[0],framenum[0],rsnum);
	//ReSample(rhandx[0],rhandy[0],rhandz[0],framenum[0],rsnum);
	//for (int t=1;t<loops;t++)
	//{
	//	ReSample(lhandx[0],lhandy[0],lhandz[0],rsnum,rsnum);
	//	ReSample(rhandx[0],rhandy[0],rhandz[0],rsnum,rsnum);
	//}

	CopyHandData(lhandx[0],lhandy[0],lhandz[0],rhandx[0],rhandy[0],rhandz[0]
	,plhandx,plhandy,plhandz,prhandx,prhandy,prhandz);
	//����probe������------------------------------------------------------

	//GetE(lhandx[0],lhandy[0],lhandz[0],lE0,0);
	//GetE(rhandx[0],rhandy[0],rhandz[0],rE0,0);

	//����������------------------------------------------------------

	double distance[WordsNum];
	double mindis=100000.0;
	int minind=-1;
	for (int i=0;i<WordsNum;i++)
	{
			distance[i]=0.0;
			score_trajectory[i]=0.0;
	}

	CopyHandData(plhandx,plhandy,plhandz,prhandx,prhandy,prhandz
		,lhandx[0],lhandy[0],lhandz[0],rhandx[0],rhandy[0],rhandz[0]);

	for (int j=0;j<WordsNum;j++)
	{	
		vector<double> temp;
		double d_seg = 0.0;
		for (int k=1;k<num;k++)
		{
			d_seg = 1000000;
			framenum[k] = gframenum[j][k];

			CopyHandData(glhandx[j][k],glhandy[j][k],glhandz[j][k]
				,grhandx[j][k],grhandy[j][k],grhandz[j][k]
				,lhandx[k],lhandy[k],lhandz[k]
				,rhandx[k],rhandy[k],rhandz[k]);
			MedianFliter(lhandx[k],lhandy[k],lhandz[k],gframenum[j][k]);
			MedianFliter(rhandx[k],rhandy[k],rhandz[k],gframenum[j][k]);
			ReSample(lhandx[k],lhandy[k],lhandz[k],gframenum[j][k],rsnum);
			ReSample(rhandx[k],rhandy[k],rhandz[k],gframenum[j][k],rsnum);

// 			int cut = KeyPostureBeginEnd[k][j][PostureMatchedIndex[k][j][0]][0];
// 			int begin = KeyPostureBeginEnd[k][j][trajectoryMatchedIndex[k][j][0]][0] - cut;
// 			int end   = KeyPostureBeginEnd[k][j][trajectoryMatchedIndex[k][j][1]][1] - cut;
// 			int segSize = end - begin + 1;
// 			CopyHandData_seg(begin, end,
// 				glhandx[j][k],glhandy[j][k],glhandz[j][k]
// 				,grhandx[j][k],grhandy[j][k],grhandz[j][k]
// 				,lhandx[k],lhandy[k],lhandz[k]
// 				,rhandx[k],rhandy[k],rhandz[k]);
// 			MedianFliter(lhandx[k],lhandy[k],lhandz[k],segSize);
// 			MedianFliter(rhandx[k],rhandy[k],rhandz[k],segSize);
// 			ReSample(lhandx[k],lhandy[k],lhandz[k],segSize,rsnum);
// 			ReSample(rhandx[k],rhandy[k],rhandz[k],segSize,rsnum);

				//Iterator again, with point number: rsnum
			for (int t=1;t<loops;t++)
			{
				ReSample(lhandx[k],lhandy[k],lhandz[k],rsnum,rsnum);
				ReSample(rhandx[k],rhandy[k],rhandz[k],rsnum,rsnum);
			}

			double galleryLength_left = 0;
			double probeLength_left = 0;
			double galleryLength_right = 0;
			double probeLength_right = 0;
			for (int i=1; i<rsnum; i++)
			{
				galleryLength_left += sqrt(pow((lhandx[k][i] - lhandx[k][i-1]),2) 
					+ pow((lhandy[k][i] - lhandy[k][i-1]),2)
					+ pow((lhandz[k][i] - lhandz[k][i-1]),2));
				galleryLength_right += sqrt(pow((rhandx[k][i] - rhandx[k][i-1]),2)
					+ pow((rhandy[k][i] - rhandy[k][i-1]),2)
					+ pow((rhandz[k][i] - rhandz[k][i-1]),2));
			}
			for (int i=1; i<framenum[0]; i++)
			{
				probeLength_left += sqrt(pow((lhandx[0][i] - lhandx[0][i-1]),2) 
					+ pow((lhandy[0][i] - lhandy[0][i-1]),2)
					+ pow((lhandz[0][i] - lhandz[0][i-1]),2));
				probeLength_right += sqrt(pow((rhandx[0][i] - rhandx[0][i-1]),2)
					+ pow((rhandy[0][i] - rhandy[0][i-1]),2)
					+ pow((rhandz[0][i] - rhandz[0][i-1]),2));
			}

			float diff = (abs(galleryLength_left-probeLength_left)
				+abs(galleryLength_right-probeLength_right))
				/(galleryLength_left+galleryLength_right+Theda);

			if (/*KeyPostureNum[k][j]!=0 &&*/ diff<0.5)
			{
				d_seg = DistanceOfGallery_Slide_Probe(k,lhandx[0],lhandy[0],lhandz[0],
					lhandx[k],lhandy[k],lhandz[k],
					rhandx[0],rhandy[0],rhandz[0],
					rhandx[k],rhandy[k],rhandz[k],framenum[0]);
			}
			
			d_seg = sqrt(d_seg);
			temp.push_back(d_seg);
		}
			//sort and choose the smallest two.
		sort(temp.begin(),temp.end());
		distance[j]=(temp[0]+temp[1])/2.0;
		temp.clear();

		if (distance[j]<mindis)
		{
			mindis=distance[j];
			minind=j;
		}
	}
#ifdef saveFiles
	//if (mindis<10) //A threshold to exclude matches with large distances.
	{
		CString s_FileName;
		float probe_im[6][rsnum];
		int maxClass = 0;
		int galleryMaxStart;
		int galleryMaxEnd;

			//Folder to save some information.
		CString s_filefolder;
		s_filefolder.Format("..\\signOnlineOutput\\%d\\trajectory",sentenceID);
		_mkdir(s_filefolder);
		s_filefolder.Format("..\\signOnlineOutput\\%d\\trajectory\\startFrame_%d",sentenceID,startFrameID);
		_mkdir(s_filefolder);
		//Save the galleries. Once done, not forever. 
		;
		//s_filefolder.Format("..\\testKeyImage\\%d\\trajectory\\allTheGallery4",sentenceID);
		//_mkdir(s_filefolder);

			//Save all the score of this comparing.
		s_FileName.Format("..\\signOnlineOutput\\%d\\trajectory\\detailDistance.csv",sentenceID);
		ofstream outfile_Dis;
		outfile_Dis.open(s_FileName,ios::out | ios::app);
	
		if (once)
		{
			outfile_Dis<<"NO."<<",";
			for (int i=0;i<WordsNum;i++)
			{
				outfile_Dis<<i<<",";
			}
			once = false;
			outfile_Dis<<endl;

			//Save the galleries. Once done, not forever. 
			;
			//
// 		for (int j=0;j<WordsNum;j++)
// 		{
// 			float gallery_im_all[6][rsnum];
// 			int k = 4;
// 			int igalleryStartFrame = KeyPostureBeginEnd[k][j][0][0];
// 			int igalleryEndFrame = KeyPostureBeginEnd[k][j][KeyPostureNum[k][j]-1][1];
// 			Get_Gallery_IM(gallery_im_all,igalleryStartFrame,igalleryEndFrame,j,1);
// 			//if (galleryStartFrame[k][j] >0 && galleryEndFrame[k][j]>0)
// 			{
// 				CString s_FileName_gallery;
// 
// 				s_FileName_gallery.Format("..\\testKeyImage\\%d\\trajectory\\allTheGallery4\\gallery4_%d.txt",
// 					sentenceID,j/*,KeyPostureBeginEnd[k][j][galleryStartFrame[k][j]][0]*/
// 					/*,KeyPostureBeginEnd[k][j][galleryEndFrame[k][j]][1]*/);
// 				ofstream outfile_gallery_all;
// 				outfile_gallery_all.open(s_FileName_gallery,ios::out);
// 				for (int j=0; j<rsnum; j++)
// 				{
// 					for (int p=0; p<6; p++)
// 					{
// 						outfile_gallery_all<<gallery_im_all[p][j]<<'\t';
// 					}
// 					outfile_gallery_all<<endl;
// 				}
// 				outfile_gallery_all.close();
// 			}
// 		}
		}
		outfile_Dis<<startFrameID<<"-"<<endFrameID<<",";
		for (int i=0;i<WordsNum;i++)
		{
			outfile_Dis<<distance[i]<<",";
			score[i] = distance[i];
		}
		outfile_Dis<<endl;
		outfile_Dis.close();
	

			//Output the normalized probe for the convenient of comparing with galleries, 
			//which is stored in folder "D:\iData\Kinect sign data\trajectory\allTheGallery_20130529"
		Get_Probe_IM(probe_im,framenum[0]);
		CString s_FileName_probe;
		s_FileName_probe.Format("..\\signOnlineOutput\\%d\\trajectory\\startFrame_%d\\probe_%d_%d_%d.txt",
			sentenceID,startFrameID,startFrameID,endFrameID,maxClass);
		ofstream outfile_probe;
		outfile_probe.open(s_FileName_probe,ios::out);
		for (int j=0; j<rsnum; j++)
		{
			for (int k=0; k<6; k++)
			{
				outfile_probe<<probe_im[k][j]<<'\t';
			}
			outfile_probe<<endl;
		}
			//Output the inter and intra dynamic testing result. 
// 		CString s_FileName_variousall;
// 		s_FileName_variousall.Format("..\\signOnlineOutput\\Variousall.csv");
// 		ofstream outfile_variousall;
// 		outfile_variousall.open(s_FileName_variousall,ios::out | ios::app);
// 
// 		float length_left = 0.0;
// 		for (int k=2*probeSize/3; k<probeSize; k++)
// 		{
// 			length_left += sqrt(pow((oriLeftHand[k].x - oriLeftHand[k-1].x),2)
// 				+pow((oriLeftHand[k].y - oriLeftHand[k-1].y),2)
// 				+pow((oriLeftHand[k].z - oriLeftHand[k-1].z),2));
// 		}
// 
// 		float length_right = 0.0;
// 		for (int k=2*probeSize/3; k<probeSize; k++)
// 		{
// 			length_right += sqrt(pow((oriRightHand[k].x - oriRightHand[k-1].x),2)
// 				+pow((oriRightHand[k].y - oriRightHand[k-1].y),2)
// 				+pow((oriRightHand[k].z - oriRightHand[k-1].z),2));
// 		}
// 		float maxSpeed = max((length_left/(probeSize/3)), (length_right/(probeSize/3)));
// 		outfile_variousall<<sentenceID<<","<<startFrameID<<","<<endFrameID<<","<<"right"<<","<<maxSpeed<<endl;

		outfile_probe.close();
		//																																																																																																	// 	//
// 	ofstream galleryMatchIDOutput;
// 	CString galleryMatchID;
// 	galleryMatchID.Format("..\\testKeyImage\\%d\\trajectory\\startFrame_%d\\gallery_1_MatchID.txt",sentenceID,startFrameID);
// 	galleryMatchIDOutput.open(galleryMatchID,ios::out);
// 	for (int j=0;j<WordsNum;j++)
// 	{
// 		if (galleryStartFrame[1][j] <0 && galleryEndFrame[1][j]<0)
// 		{
// 			galleryMatchIDOutput<<"Gallery ID: "<<j<<'\t'<<"has no key frames."<<endl;
// 		}
// 		else
// 		{
// 			galleryMatchIDOutput<<"Gallery ID: "<<j<<'\t'<<KeyPostureBeginEnd[1][j][galleryStartFrame[1][j]][0]<<'\t'
// 				<<KeyPostureBeginEnd[1][j][galleryEndFrame[1][j]][1]<<endl;
// 		}
// 		//////////////////////////////////////////////////////////////////////////
// 		float gallery_im_all[6][rsnum];
// 		int k = 1;
// 		int igalleryStartFrame = KeyPostureBeginEnd[k][j][galleryStartFrame[k][j]][0];
// 		int igalleryEndFrame = KeyPostureBeginEnd[k][j][galleryEndFrame[k][j]][1];
// 		Get_Gallery_IM(gallery_im_all,igalleryStartFrame,igalleryEndFrame,j,1);
// 		if (galleryStartFrame[k][j] >0 && galleryEndFrame[k][j]>0)
// 		{
// 			CString s_FileName_gallery;
// 
// 			s_FileName_gallery.Format("..\\testKeyImage\\%d\\trajectory\\startFrame_%d\\allTheGallery1\\gallery1_%d_%d_%d.txt",
// 				sentenceID,startFrameID,j,KeyPostureBeginEnd[k][j][galleryStartFrame[k][j]][0]
// 			,KeyPostureBeginEnd[k][j][galleryEndFrame[k][j]][1]);
// 			ofstream outfile_gallery_all;
// 			outfile_gallery_all.open(s_FileName_gallery,ios::out);
// 			for (int j=0; j<rsnum; j++)
// 			{
// 				for (int p=0; p<6; p++)
// 				{
// 					outfile_gallery_all<<gallery_im_all[p][j]<<'\t';
// 				}
// 				outfile_gallery_all<<endl;
// 			}
// 			outfile_gallery_all.close();
// 		}
// 		
// 		//////////////////////////////////////////////////////////////////////////
// 
// 		if (distance[j]==0)
// 		{
// 			score_trajectory[j]=1;
// 			score[j] = score_trajectory[j];
// 		}
// 		else
// 		{
// 			score_trajectory[j]=mindis/(distance[j]);
// 			score[j] = score_trajectory[j];
// 		}
// 
// 		if (score[j]==1)
// 		{
// 			maxClass = j;
// 			int k=1;
// 
// 			//int galleryStartPo =trajectoryMatchedIndex[k][j][0];
// 
// 			//int galleryEndPo = trajectoryMatchedIndex[k][j][1];
// 
// 			int igalleryStartFrame = KeyPostureBeginEnd[k][j][galleryStartFrame[k][j]][0];
// 
// 			int igalleryEndFrame = KeyPostureBeginEnd[k][j][galleryEndFrame[k][j]][1];
// 
// 			Get_Gallery_IM(gallery_im,igalleryStartFrame,igalleryEndFrame,j,k);
// 			galleryMaxStart = igalleryStartFrame;
// 			galleryMaxEnd = igalleryEndFrame;
// 
// 		}
// 	}
// 	galleryMatchIDOutput.close();
// 	Get_Probe_IM(probe_im);
// 
// 	CString s_ImgFileName;
// 	CString s_FileName_gallery;
// 	CString s_FileName_probe;
// 	s_FileName_gallery.Format("..\\testKeyImage\\%d\\trajectory\\startFrame_%d\\gallery_%d_%d_%d.txt",sentenceID,startFrameID,galleryMaxStart,galleryMaxEnd,maxClass);
// 	s_FileName_probe.Format("..\\testKeyImage\\%d\\trajectory\\startFrame_%d\\probe_%d_%d_%d.txt",sentenceID,startFrameID,startFrameID,endFrameID,maxClass);
// 	ofstream outfile_gallery;
// 	ofstream outfile_probe;
// 	outfile_gallery.open(s_FileName_gallery,ios::out);
// 	outfile_probe.open(s_FileName_probe,ios::out);
// 	for (int j=0; j<rsnum; j++)
// 	{
// 		for (int k=0; k<6; k++)
// 		{
// 			outfile_gallery<<gallery_im[k][j]<<'\t';
// 			outfile_probe<<probe_im[k][j]<<'\t';
// 		}
// 		outfile_gallery<<endl;
// 		outfile_probe<<endl;
// 	}
// 	outfile_gallery.close();
// 	outfile_probe.close();
		;
			//Find the first 5 classes.	I use a stupid greedy search method.
		vector<double> distanceVector;
		for (int j=0; j<WordsNum; j++)
		{
			distanceVector.push_back(distance[j]);
		}
		sort(distanceVector.begin(),distanceVector.end());
		int rank[5];
		for (int p=0; p<5; p++)
		{
			for (int j=0; j<WordsNum; j++)
			{
				if (distance[j] == distanceVector[p])
				{
					bool repeat = false;
					for (int i=0; i<p; i++)
					{
						if (rank[i] == j)
						{
							repeat = true;
							break;
						}
					}
					if (!repeat)
					{
						rank[p] = j;
					}
				}
			}
		}

		s_FileName.Format("..\\signOnlineOutput\\%d\\trajectory\\rank5detail.txt",sentenceID);
		ofstream outfile_short;
		outfile_short.open(s_FileName,ios::out | ios::app);
		outfile_short<<"start: "<<startFrameID<<" end: "<<endFrameID<<" minDis: "<<mindis<<" BestID: "
			<<rank[0]<<" "<<rank[1]<<" "<<rank[2]<<" "<<rank[3]<<" "<<rank[4]<<" "<<endl;
// 		cout<<"start: "<<startFrameID<<" end: "<<endFrameID<<" minDis: "<<mindis<<" BestID: "
// 			<<rank[0]<<" "<<rank[1]<<" "<<rank[2]<<" "<<rank[3]<<" "<<rank[4]<<" "<<endl;
		outfile_short.close();

		for (int i=0; i<5; i++)
		{
			tra_result[i].push_back(rank[i]);
			tra_result_score[i].push_back(distanceVector[i]);
		}
	}
#endif

	return minind;
}
int MergeTrajectoryAndPostureResult(double score_trajectory[WordsNum],double score_posture[WordsNum],double score[WordsNum])
{
	//p�ǹ켣ϵ����Ŀǰȡ0.3
	float p=0.3;
	for (int i=0;i<WordsNum;i++)
	{
		score[i]=p*score_trajectory[i]+(1-p)*score_posture[i];
	}
	
	return 1;
}
void Arrayrankingindex2(double a[],int length,int index[])//��������
{
	float temp=0.0;
	int int_temp=0;
	//�±��ʼ��
	for (int i=0;i<WordsNum;i++)
	{
		index[i]=i;
	}

	for (int i=0;i<length-1;i++)
	{
		for (int j=0;j<length-i-1;j++)
		{
			if (a[j]<a[j+1])
			{
				temp=a[j];
				a[j]=a[j+1];
				a[j+1]=temp;

				int_temp=index[j];
				index[j]=index[j+1];
				index[j+1]=int_temp;
			}
		}
	}
	return;
}
// void curve()
// {
// 	float headx0;float heady0;float headz0;
// 	float lhandx0[F];float lhandy0[F];float lhandz0[F];
// 	float rhandx0[F];float rhandy0[F];float rhandz0[F];
// 	int n;
// 	CString positionfile=".\\resource\\D3\\K50\\K50_0000_1_0_20121002.oni.txt";
// 	JustReadDataFile(positionfile,1,0);
// 	headx0=gheadx[1][0];
// 	heady0=gheady[1][0];
// 	headz0=gheadz[1][0];
// 	CopyHandData(glhandx[1][0],glhandy[1][0],glhandz[1][0],grhandx[1][0],grhandy[1][0],grhandz[1][0],lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0);
// 	n=gframenum[1][0];
// 	InitData();
// 	double score[WordsNum];
// 	CurveRecognision(headx0,heady0,headz0,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,n,score);
// }
void CurveRecognition(CvPoint3D32f headPoint3D, vector<SLR_ST_Skeleton> vSkeletonData, 
	double score[WordsNum], int sentenceID,int startFrameID, int endFrameID, int trajectoryMatchedIndex[][Posture_num][2])
{
	int length = vSkeletonData.size();
	float *lhandx0 = new float[length];
	float *lhandy0 = new float[length];
	float *lhandz0 = new float[length];
	float *rhandx0 = new float[length];
	float *rhandy0 = new float[length];
	float *rhandz0 = new float[length];
	for(int i=0; i<length; i++)
	{
		lhandx0[i] = 1000 * vSkeletonData[i]._3dPoint[7].x;
		lhandy0[i] = 1000 * vSkeletonData[i]._3dPoint[7].y;
		lhandz0[i] = 1000 * vSkeletonData[i]._3dPoint[7].z;
		rhandx0[i] = 1000 * vSkeletonData[i]._3dPoint[11].x;
		rhandy0[i] = 1000 * vSkeletonData[i]._3dPoint[11].y;
		rhandz0[i] = 1000 * vSkeletonData[i]._3dPoint[11].z;
	}
	float headx0 = float((int)(headPoint3D.x*1000));
	float heady0 = float((int)(headPoint3D.y*1000));
	float headz0 = float((int)(headPoint3D.z*1000));

	CurveRecognision(headx0,heady0,headz0,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,length,score, 
		sentenceID,startFrameID,endFrameID, 
		trajectoryMatchedIndex);

	delete [] lhandx0;
	delete [] lhandy0;
	delete [] lhandz0;
	delete [] rhandx0;
	delete [] rhandy0;
	delete [] rhandz0;
}
//------------------------
void Get_Probe_IM(float probe_im[6][rsnum],int probe_num)
{
	CopyHandData(plhandx,plhandy,plhandz,prhandx,prhandy,prhandz,lhandx[0],lhandy[0],lhandz[0],rhandx[0],rhandy[0],rhandz[0]);
// 	const int loops=3;
// 	MedianFliter(lhandx[0],lhandy[0],lhandz[0],framenum[0]);
// 	MedianFliter(rhandx[0],rhandy[0],rhandz[0],framenum[0]);
// 	ReSample(lhandx[0],lhandy[0],lhandz[0],framenum[0],rsnum);
// 	ReSample(rhandx[0],rhandy[0],rhandz[0],framenum[0],rsnum);
// 	for (int t=1;t<loops;t++)
// 	{
// 		ReSample(lhandx[0],lhandy[0],lhandz[0],rsnum,rsnum);
// 		ReSample(rhandx[0],rhandy[0],rhandz[0],rsnum,rsnum);
// 	}
// 	CString s_FileName;
// 	s_FileName.Format("..\\testKeyImage\\181\\trajectory\\48point.csv");
// 	ofstream outfile;
// 	outfile.open(s_FileName,ios::out | ios::app);
// 	outfile<<"out"<<",";
// 	for (int i=0; i<probe_num; i++)
// 	{
// 		outfile<<lhandz[0][i]<<",";
// 	}
// 	outfile<<endl;
// 	outfile.close();

	ReSample(lhandx[0],lhandy[0],lhandz[0],probe_num,rsnum);
	ReSample(rhandx[0],rhandy[0],rhandz[0],probe_num,rsnum);


	Alignment_translate(rsnum,lhandx[0],lhandy[0],lhandz[0]);
	Alignment_translate(rsnum,rhandx[0],rhandy[0],rhandz[0]);
	
	for (int i=0;i<rsnum;i++)
	{
		probe_im[0][i]=lhandx[0][i];
		probe_im[1][i]=lhandy[0][i];
		probe_im[2][i]=lhandz[0][i];
		probe_im[3][i]=rhandx[0][i];
		probe_im[4][i]=rhandy[0][i];
		probe_im[5][i]=rhandz[0][i];
	}

	return;
}
void Get_Gallery_IM(float gallery_im[6][rsnum],int begin,int end,int wordindex,int k)
{
// 	CopyHandData_seg(begin,end,glhandx[wordindex][k],glhandy[wordindex][k],glhandz[wordindex][k],
// 		grhandx[wordindex][k],grhandy[wordindex][k],grhandz[wordindex][k],
// 		lhandx[k],lhandy[k],lhandz[k],rhandx[k],rhandy[k],rhandz[k]);
	CopyHandData(glhandx[wordindex][k],glhandy[wordindex][k],glhandz[wordindex][k],
		grhandx[wordindex][k],grhandy[wordindex][k],grhandz[wordindex][k],
		lhandx[k],lhandy[k],lhandz[k],rhandx[k],rhandy[k],rhandz[k]);

//	int framenum_seg = end - begin;
	int framenum_seg = framenum[k];
	MedianFliter(lhandx[k],lhandy[k],lhandz[k],framenum_seg);
	MedianFliter(rhandx[k],rhandy[k],rhandz[k],framenum_seg);
	ReSample(lhandx[k],lhandy[k],lhandz[k],framenum_seg,rsnum);
	ReSample(rhandx[k],rhandy[k],rhandz[k],framenum_seg,rsnum);
	//�ڶ��ε�����ʼ����������rsnum��
	const int loops=3;
	for (int t=1;t<loops;t++)
	{
		ReSample(lhandx[k],lhandy[k],lhandz[k],rsnum,rsnum);
		ReSample(rhandx[k],rhandy[k],rhandz[k],rsnum,rsnum);
	}

	Alignment_translate(rsnum,lhandx[k],lhandy[k],lhandz[k]);
	Alignment_translate(rsnum,rhandx[k],rhandy[k],rhandz[k]);

	for (int i=0;i<rsnum;i++)
	{
		gallery_im[0][i]=lhandx[k][i];
		gallery_im[1][i]=lhandy[k][i];
		gallery_im[2][i]=lhandz[k][i];
		gallery_im[3][i]=rhandx[k][i];
		gallery_im[4][i]=rhandy[k][i];
		gallery_im[5][i]=rhandz[k][i];
	}
	return;
}
//---------------------------
void arraycopy_1D(float old_array[],float new_arrey[],int length)
{
	for (int i=0;i<length;i++ )
	{
		new_arrey[i]=old_array[i];
	}
	return;
}
double GetLenRate(int k,float x0[],float y0[],float z0[],float xk[],float yk[],float zk[])
{
	//���㳤�ȱ�
	double len_0=0.0;
	double len_k=0.0;
	double rate=0.0;
	for (int j=1;j<rsnum;j++)
	{
		len_0+=sqrt((x0[j]-x0[j-1])*(x0[j]-x0[j-1])+(y0[j]-y0[j-1])*(y0[j]-y0[j-1])+(z0[j]-z0[j-1])*(z0[j]-z0[j-1]));

	}
	for (int j=1;j<rsnum;j++)
	{
		len_k+=sqrt((xk[j]-xk[j-1])*(xk[j]-xk[j-1])+(yk[j]-yk[j-1])*(yk[j]-yk[j-1])+(zk[j]-zk[j-1])*(zk[j]-zk[j-1]));
	}

	if (abs(len_0)<=eps||abs(len_k)<=eps)
	{
		if (abs(len_0)<=eps)
		{
			//cout<<"len_0 =0"<<endl;
		}
		else
		{
			//cout<<"len_"<<k<<" =0"<<endl;
		}
		rate=1.0;
	}
	else
	{
		rate=len_0/len_k;
	}
	return rate;
}

double RandomPartialMatchDistance(int k,float x0[],float y0[],float z0[],float xk[],float yk[],float zk[],int &beg ,int &end)
{
	float xt[F],yt[F],zt[F];
	int len_seg=0;
	double d=0,min_d=1000000;
	double rate=1.0;
	rate=GetLenRate(k,x0,y0,z0,xk,yk,zk);
	if (rate>1)
	{
		len_seg=(int)(rsnum/rate);

		for (int begin=0;begin<=rsnum-len_seg;begin++)
		{
			arraycopy_1D(x0,xt,rsnum);
			arraycopy_1D(y0,yt,rsnum);
			arraycopy_1D(z0,zt,rsnum);

			CurveSegment(xt,yk,zt,begin,len_seg);
			ReSample(xt,yt,zt,len_seg,rsnum);

			Alignment_translate(k,xt,yt,zt);
			Alignment_translate(k,xk,yk,zk);

			d=CalDistance(xt,yt,zt,xk,yk,zk);
			if (d<min_d)
			{
				beg=begin;
				end=begin+len_seg-1;
				min_d=d;
			}
			//cout<<begin<<endl;
		}
	}
	if (rate<1)
	{
		len_seg=(int)(rsnum*rate);

		for (int begin=0;begin<=rsnum-len_seg;begin++)
		{
			arraycopy_1D(xk,xt,rsnum);
			arraycopy_1D(yk,yt,rsnum);
			arraycopy_1D(zk,zt,rsnum);

			CurveSegment(xt,yt,zt,begin,len_seg);
			ReSample(xt,yt,zt,len_seg,rsnum);

			Alignment_translate(k,x0,y0,z0);
			Alignment_translate(k,xt,yt,zt);

			d=CalDistance(x0,y0,z0,xt,yt,zt);
			if (d<min_d)
			{
				beg=begin;
				end=begin+len_seg-1;
				min_d=d;
			}
		}
	}
	return min_d;
}
void Alignment_translate(int k,float xk[],float yk[],float zk[])
{
	//���Ķ���
	float zx=0.0;float zy=0.0;float zz=0.0;


	GetCentroid(k,xk,yk,zk,zx,zy,zz);



	for (int i=0;i<rsnum;i++)
	{
		xk[i]-=zx;
		yk[i]-=zy;
		zk[i]-=zz;
	}
	return;
}
//----------------------------------
double GetLenRate_6d(int k,float lx0[],float ly0[],float lz0[],float lxk[],float lyk[],float lzk[],float rx0[],float ry0[],float rz0[],float rxk[],float ryk[],float rzk[])
{
	//���㳤�ȱ�
	double len_0=0.0;
	double len_k=0.0;
	double rate=0.0;
	for (int j=1;j<rsnum;j++)
	{
		len_0+=sqrt((lx0[j]-lx0[j-1])*(lx0[j]-lx0[j-1])+(ly0[j]-ly0[j-1])*(ly0[j]-ly0[j-1])+(lz0[j]-lz0[j-1])*(lz0[j]-lz0[j-1])
			+(rx0[j]-rx0[j-1])*(rx0[j]-rx0[j-1])+(ry0[j]-ry0[j-1])*(ry0[j]-ry0[j-1])+(rz0[j]-rz0[j-1])*(rz0[j]-rz0[j-1]));

	}
	for (int j=1;j<rsnum;j++)
	{
		len_k+=sqrt((lxk[j]-lxk[j-1])*(lxk[j]-lxk[j-1])+(lyk[j]-lyk[j-1])*(lyk[j]-lyk[j-1])+(lzk[j]-lzk[j-1])*(lzk[j]-lzk[j-1])
			+(rxk[j]-rxk[j-1])*(rxk[j]-rxk[j-1])+(ryk[j]-ryk[j-1])*(ryk[j]-ryk[j-1])+(rzk[j]-rzk[j-1])*(rzk[j]-rzk[j-1]));
	}

	if (abs(len_0)<=eps||abs(len_k)<=eps)
	{
		if (abs(len_0)<=eps)
		{
			cout<<"len_0 =0"<<endl;
		}
		else
		{
			cout<<"len_"<<k<<" =0"<<endl;
		}
		rate=1.0;
	}
	else
	{
		rate=len_0/len_k;
	}
	return rate;
}
void CurveSegment(float x0[],float y0[],float z0[],int begin,int len_seg)
{
	for(int i=0;i<len_seg;i++)
	{
		x0[i]=x0[begin+i];
		y0[i]=y0[begin+i];
		z0[i]=z0[begin+i];
	}
	return ;
}

//fix curve1 x0[],y0[],z0[],go through curve2 xk[],yk[],zk[],notice:curve1 will not be longer than curve2(no more than delta*length of curve1)
double Distance0fFixed_slidedCurves(int k,float lx0[],float ly0[],float lz0[],float lxk[],float lyk[],float lzk[],float rx0[],float ry0[],float rz0[],float rxk[],float ryk[],float rzk[])
{
	float lxt_0[F],lyt_0[F],lzt_0[F];
	float lxt_k[F],lyt_k[F],lzt_k[F];
	float rxt_0[F],ryt_0[F],rzt_0[F];
	float rxt_k[F],ryt_k[F],rzt_k[F];

	int len_seg=0;
	double d=0,min_d=1000000.0;
	double rate=1.0;
	double rate_flu=1.0;
	double delta=0.2;//probe�����ķ�ֵ
	double m=0.1;//probe�����Ĳ���
	double xita=1.5;//rate_flu>xitaʱ�����Ƚ�,��Ϊ�����ơ�

	rate=GetLenRate_6d(k,lx0,ly0,lz0,lxk,lyk,lzk,rx0,ry0,rz0,rxk,ryk,rzk);
	for (double i=-delta;i<=delta;i=i+m)
	{
		rate_flu=rate*(1+i);
		if (rate_flu<1.0)//then slide
		{
			len_seg=(int)(rsnum*rate_flu);

			for (int begin=0;begin<=rsnum-len_seg;begin++)
			{
				arraycopy_1D(lx0,lxt_0,rsnum);
				arraycopy_1D(ly0,lyt_0,rsnum);
				arraycopy_1D(lz0,lzt_0,rsnum);

				arraycopy_1D(lxk,lxt_k,rsnum);
				arraycopy_1D(lyk,lyt_k,rsnum);
				arraycopy_1D(lzk,lzt_k,rsnum);

				arraycopy_1D(rx0,rxt_0,rsnum);
				arraycopy_1D(ry0,ryt_0,rsnum);
				arraycopy_1D(rz0,rzt_0,rsnum);

				arraycopy_1D(rxk,rxt_k,rsnum);
				arraycopy_1D(ryk,ryt_k,rsnum);
				arraycopy_1D(rzk,rzt_k,rsnum);

				CurveSegment(lxt_k,lyt_k,lzt_k,begin,len_seg);
				CurveSegment(rxt_k,ryt_k,rzt_k,begin,len_seg);
				ReSample(lxt_k,lyt_k,lzt_k,len_seg,rsnum);
				ReSample(rxt_k,ryt_k,rzt_k,len_seg,rsnum);


				Alignment_translate(rsnum,lxt_0,lyt_0,lzt_0);
				Alignment_translate(rsnum,lxt_k,lyt_k,lzt_k);
				Alignment_translate(rsnum,rxt_0,ryt_0,rzt_0);
				Alignment_translate(rsnum,rxt_k,ryt_k,rzt_k);

				d=CalDistance(lxt_0,lyt_0,lzt_0,lxt_k,lyt_k,lzt_k)+CalDistance(rxt_0,ryt_0,rzt_0,rxt_k,ryt_k,rzt_k);
				d=d/rate_flu;
				if (d<min_d)
				{
					min_d=d;
				}
			}
		}
		else
		{
			if (rate_flu>=1.0&&rate_flu<xita)//rate_flu>=1&&rate_flu<1+delta,than compare then directly
			{
				Alignment_translate(rsnum,lxt_0,lyt_0,lzt_0);
				Alignment_translate(rsnum,lxt_k,lyt_k,lzt_k);
				Alignment_translate(rsnum,rxt_0,ryt_0,rzt_0);
				Alignment_translate(rsnum,rxt_k,ryt_k,rzt_k);

				d=CalDistance(lxt_0,lyt_0,lzt_0,lxt_k,lyt_k,lzt_k)+CalDistance(rxt_0,ryt_0,rzt_0,rxt_k,ryt_k,rzt_k);
				if (d<min_d)
				{
					min_d=d;
				}
			}
			else//rate_flu>xitaʱ�����Ƚϡ�
			{
				break;
			}

		}
	}


	return min_d;	

}
//go through curve1 x0[] y0[] z0[]
double DistanceOfSlidedCurves(int k,float lx0[],float ly0[],float lz0[],float lxk[],float lyk[],float lzk[],float rx0[],float ry0[],float rz0[],float rxk[],float ryk[],float rzk[])
{
	float lxt_0[F],lyt_0[F],lzt_0[F];
	float lxt_k[F],lyt_k[F],lzt_k[F];
	float rxt_0[F],ryt_0[F],rzt_0[F];
	float rxt_k[F],ryt_k[F],rzt_k[F];
	double d=0,min_d=1000000.0;
	int slide_len;
	for (int i=1;i<=10;i++)
	{
		//cout<<i<<endl;
		slide_len=rsnum*0.1*i;
		for(int begin=0;begin<=rsnum-slide_len;begin++ )
		{
			//cout<<"begin: "<<begin<<endl;
			arraycopy_1D(lx0,lxt_0,rsnum);
			arraycopy_1D(ly0,lyt_0,rsnum);
			arraycopy_1D(lz0,lzt_0,rsnum);

			arraycopy_1D(lxk,lxt_k,rsnum);
			arraycopy_1D(lyk,lyt_k,rsnum);
			arraycopy_1D(lzk,lzt_k,rsnum);

			arraycopy_1D(rx0,rxt_0,rsnum);
			arraycopy_1D(ry0,ryt_0,rsnum);
			arraycopy_1D(rz0,rzt_0,rsnum);

			arraycopy_1D(rxk,rxt_k,rsnum);
			arraycopy_1D(ryk,ryt_k,rsnum);
			arraycopy_1D(rzk,rzt_k,rsnum);

			CurveSegment(lxt_0,lyt_0,lzt_0,begin,slide_len);
			CurveSegment(rxt_0,ryt_0,rzt_0,begin,slide_len);
			ReSample(lxt_0,lyt_0,lzt_0,slide_len,rsnum);
			ReSample(rxt_0,ryt_0,rzt_0,slide_len,rsnum);

			d=Distance0fFixed_slidedCurves(k,lxt_0,lyt_0,lzt_0,lxt_k,lyt_k,lzt_k,rxt_0,ryt_0,rzt_0,rxt_k,ryt_k,rzt_k);
		}

	}

	return min_d;

}

double DistanceOfGallerySlideProbe(int k,float lx0[],float ly0[],float lz0[],
	float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],
	float rxk[],float ryk[],float rzk[])
{
	float lxt_0[F],lyt_0[F],lzt_0[F];
	float lxt_k[F],lyt_k[F],lzt_k[F];
	float rxt_0[F],ryt_0[F],rzt_0[F];
	float rxt_k[F],ryt_k[F],rzt_k[F];
	double d=0,min_d=1000000.0;
	int len_seg;

	double rate=1.0;
	double rate_flu=1.0;
	double gallery_len=0.0;

	const double delta=0.2;//probe�����ķ�ֵ
	const double m=0.1;//probe�����Ĳ���
	const double xita=0.9;//rate_flu>xitaʱ�����Ƚ�,��Ϊ�����ơ�
	const double alpha=1.0;//��λ���Ⱦ����һ��ϵ��
	const double MinGalleryLen=1.0;//���м������СGallery����

	rate=GetLenRate_6d(k,lx0,ly0,lz0,lxk,lyk,lzk,rx0,ry0,rz0,rxk,ryk,rzk);

	len_seg=(int)(rsnum/rate_flu);
	for (double i=-delta;i<=delta;i=i+m)
	{
		rate_flu=rate*(1+i);
		if (rate_flu>1.0)
		{
			for (int begin=0;begin<=rsnum-len_seg;begin++)
			{
				arraycopy_1D(lx0,lxt_0,rsnum);
				arraycopy_1D(ly0,lyt_0,rsnum);
				arraycopy_1D(lz0,lzt_0,rsnum);

				arraycopy_1D(lxk,lxt_k,rsnum);
				arraycopy_1D(lyk,lyt_k,rsnum);
				arraycopy_1D(lzk,lzt_k,rsnum);

				arraycopy_1D(rx0,rxt_0,rsnum);
				arraycopy_1D(ry0,ryt_0,rsnum);
				arraycopy_1D(rz0,rzt_0,rsnum);

				arraycopy_1D(rxk,rxt_k,rsnum);
				arraycopy_1D(ryk,ryt_k,rsnum);
				arraycopy_1D(rzk,rzt_k,rsnum);

				CurveSegment(lxt_0,lyt_0,lzt_0,begin,len_seg);
				CurveSegment(rxt_0,ryt_0,rzt_0,begin,len_seg);
				ReSample(lxt_0,lyt_0,lzt_0,len_seg,rsnum);
				ReSample(rxt_0,ryt_0,rzt_0,len_seg,rsnum);


				Alignment_translate(rsnum,lxt_0,lyt_0,lzt_0);
				Alignment_translate(rsnum,lxt_k,lyt_k,lzt_k);
				Alignment_translate(rsnum,rxt_0,ryt_0,rzt_0);
				Alignment_translate(rsnum,rxt_k,ryt_k,rzt_k);

				d=CalDistance(lxt_0,lyt_0,lzt_0,lxt_k,lyt_k,lzt_k)+CalDistance(rxt_0,ryt_0,rzt_0,rxt_k,ryt_k,rzt_k);
				d=d/rate_flu;
				if (d<min_d)
				{
					min_d=d;
				}
			}
		}
		if (rate_flu<1.0&&rate_flu>xita)
		{
			Alignment_translate(rsnum,lxt_0,lyt_0,lzt_0);
			Alignment_translate(rsnum,lxt_k,lyt_k,lzt_k);
			Alignment_translate(rsnum,rxt_0,ryt_0,rzt_0);
			Alignment_translate(rsnum,rxt_k,ryt_k,rzt_k);

			d=CalDistance(lxt_0,lyt_0,lzt_0,lxt_k,lyt_k,lzt_k)+CalDistance(rxt_0,ryt_0,rzt_0,rxt_k,ryt_k,rzt_k);
			if (d<min_d)
			{
				min_d=d;
			}
		}
		if (rate_flu<=xita)
		{
			break;
		}
	}
	for (int j=1;j<rsnum;j++)
	{
		gallery_len+=sqrt((lxk[j]-lxk[j-1])*(lxk[j]-lxk[j-1])+(lyk[j]-lyk[j-1])*(lyk[j]-lyk[j-1])+(lzk[j]-lzk[j-1])*(lzk[j]-lzk[j-1])
			+(rxk[j]-rxk[j-1])*(rxk[j]-rxk[j-1])+(ryk[j]-ryk[j-1])*(ryk[j]-ryk[j-1])+(rzk[j]-rzk[j-1])*(rzk[j]-rzk[j-1]));

	}
	if (gallery_len<=MinGalleryLen)
	{
		//cout<<"gallery_len<=1"<<endl;
		min_d=1000000.0;
	}
	else
	{
		min_d=min_d/(alpha*gallery_len);
	}
	return min_d;
}
	//���PROBE�������ֹ켣Ҫͬ����ȡ�����⣺��ʱ�䣨ԭʼ���ݣ�����������gallery�����Ƚ�ȡ
	//��������PROBEû�о���resample�����Ǿ���Ԥ������ֵ�˲���������֡����probe_num
double DistanceOfGallery_Slide_Probe(int k,float lx0[],float ly0[],float lz0[],
	float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],
	float rxk[],float ryk[],float rzk[],int probe_num)
{

	float lxt_0[F],lyt_0[F],lzt_0[F];
	float lxt_k[F],lyt_k[F],lzt_k[F];
	float rxt_0[F],ryt_0[F],rzt_0[F];
	float rxt_k[F],ryt_k[F],rzt_k[F];
	double d=0,min_d=1000000.0;
	int len_seg;

	double rate=1.0;
	double rate_flu=1.0;
	double gallery_left_len=0.0;
	double gallery_right_len=0.0;
	double probe_left_len=0.0;
	double probe_right_len=0.0;
	double cut_left_len=0.0;
	double cut_right_len=0.0;

	const double delta=0.2;//probe�����ķ�ֵ
	const double m=0.1;//probe�����Ĳ���
	const double xita=0.9;//rate_flu>xitaʱ�����Ƚ�,��Ϊ�����ơ�
	const double alpha=1.0;//��λ���Ⱦ����һ��ϵ��
	const double MinGalleryLen=1.0;//���м������СGallery����

	for (int j=1;j<rsnum;j++)
	{
		gallery_left_len+=sqrt((lxk[j]-lxk[j-1])*(lxk[j]-lxk[j-1])+(lyk[j]-lyk[j-1])*(lyk[j]-lyk[j-1])+(lzk[j]-lzk[j-1])*(lzk[j]-lzk[j-1]));
		gallery_right_len+=sqrt((rxk[j]-rxk[j-1])*(rxk[j]-rxk[j-1])+(ryk[j]-ryk[j-1])*(ryk[j]-ryk[j-1])+(rzk[j]-rzk[j-1])*(rzk[j]-rzk[j-1]));
	}
// 	for (int j=1;j<probe_num;j++)
// 	{
// 		probe_left_len+=sqrt((lx0[j]-lx0[j-1])*(lx0[j]-lx0[j-1])+(ly0[j]-ly0[j-1])*(ly0[j]-ly0[j-1])+(lz0[j]-lz0[j-1])*(lz0[j]-lz0[j-1]));
// 		probe_right_len+=sqrt((rx0[j]-rx0[j-1])*(rx0[j]-rx0[j-1])+(ry0[j]-ry0[j-1])*(ry0[j]-ry0[j-1])+(rz0[j]-rz0[j-1])*(rz0[j]-rz0[j-1]));
// 	}


// 	for(int begin=0;begin<probe_num-1;begin++)
 	{
// 		cut_left_len=0.0;
// 		cut_right_len=0.0;
// 		for (int j=begin+1;j<probe_num;j++)//slide
 		{
// 			len_seg=j-begin+1;
// 
//  			cut_left_len+=sqrt((lx0[j]-lx0[j-1])*(lx0[j]-lx0[j-1])+(ly0[j]-ly0[j-1])*(ly0[j]-ly0[j-1])+(lz0[j]-lz0[j-1])*(lz0[j]-lz0[j-1]));
// 			cut_right_len+=sqrt((rx0[j]-rx0[j-1])*(rx0[j]-rx0[j-1])+(ry0[j]-ry0[j-1])*(ry0[j]-ry0[j-1])+(rz0[j]-rz0[j-1])*(rz0[j]-rz0[j-1]));
//  			if (cut_left_len>(1.0+delta)*gallery_left_len||cut_right_len>(1.0+delta)*gallery_right_len)
//  			{
//  				break;
//  			}
// 			if (cut_left_len>(1.0-delta)*gallery_left_len&&cut_left_len<(1.0+delta)*gallery_left_len&&
// 				cut_right_len>(1.0-delta)*gallery_right_len&&cut_right_len<(1.0+delta)*gallery_right_len)
			{
				arraycopy_1D(lx0,lxt_0,probe_num);
				arraycopy_1D(ly0,lyt_0,probe_num);
				arraycopy_1D(lz0,lzt_0,probe_num);

				arraycopy_1D(lxk,lxt_k,rsnum);
				arraycopy_1D(lyk,lyt_k,rsnum);
				arraycopy_1D(lzk,lzt_k,rsnum);

				arraycopy_1D(rx0,rxt_0,probe_num);
				arraycopy_1D(ry0,ryt_0,probe_num);
				arraycopy_1D(rz0,rzt_0,probe_num);

				arraycopy_1D(rxk,rxt_k,rsnum);
				arraycopy_1D(ryk,ryt_k,rsnum);
				arraycopy_1D(rzk,rzt_k,rsnum);

				//CurveSegment(lxt_0,lyt_0,lzt_0,begin,len_seg);
				//CurveSegment(rxt_0,ryt_0,rzt_0,begin,len_seg);
				//ReSample(lxt_0,lyt_0,lzt_0,len_seg,rsnum);
				//ReSample(rxt_0,ryt_0,rzt_0,len_seg,rsnum);
				
				
// 				CString s_FileName;
// 				s_FileName.Format("..\\testKeyImage\\181\\trajectory\\48point.csv");
// 				ofstream outfile;
// 				outfile.open(s_FileName,ios::out | ios::app);
// 				outfile<<"in"<<",";
// 				for (int i=0; i<probe_num; i++)
// 				{
// 					outfile<<lzt_0[i]<<",";
// 				}
// 				outfile<<endl;
// 				outfile.close();

				ReSample(lxt_0,lyt_0,lzt_0,probe_num,rsnum);
				ReSample(rxt_0,ryt_0,rzt_0,probe_num,rsnum);

				Alignment_translate(rsnum,lxt_0,lyt_0,lzt_0);
				Alignment_translate(rsnum,lxt_k,lyt_k,lzt_k);
				Alignment_translate(rsnum,rxt_0,ryt_0,rzt_0);
				Alignment_translate(rsnum,rxt_k,ryt_k,rzt_k);

				d=CalDistance(lxt_0,lyt_0,lzt_0,lxt_k,lyt_k,lzt_k)
					+CalDistance(rxt_0,ryt_0,rzt_0,rxt_k,ryt_k,rzt_k);

				if (d<min_d)
				{
					min_d=d;
				}
			}

		}
	}

	return min_d;
}
void traHandPositionInitial(SLR_ST_Skeleton vSkeletonData)
{
	fir_setence_lhandx = 1000 * vSkeletonData._3dPoint[7].x;
	fir_setence_lhandy = 1000 * vSkeletonData._3dPoint[7].y;
	fir_setence_lhandz = 1000 * vSkeletonData._3dPoint[7].z;
	fir_setence_rhandx = 1000 * vSkeletonData._3dPoint[11].x;
	fir_setence_rhandy = 1000 * vSkeletonData._3dPoint[11].y;
	fir_setence_rhandz = 1000 * vSkeletonData._3dPoint[11].z;
}

int Read_Sentece_File(CString positionfile,int j,int n)//j:sentence num  n:sentence gallery 
{
	FILE* fp;

	CvPoint3D32f point;

	float flhandx,flhandy,flhandz,frhandx,frhandy,frhandz;//��һ֡�������ֵ�λ��
	int  gridheight;//����߶�
	int gridwidth;//������
	int griddepth;//�������
	//���������ͷ��λ�õ�ƫ����
	int cx;
	int cy;
	int cz;

	int framenum;
	/* read the positionfile */
	fp = fopen(positionfile, "r");
	if (fp == NULL) 
	{
		fprintf(stderr, "Error: File %s not found \n", positionfile);
		getchar();
		return false;
	}
	fscanf(fp, "%f",&head_SG[n][j].x);
	fscanf(fp, "%f",&head_SG[n][j].y);
	fscanf(fp, "%f",&head_SG[n][j].z);


	while (1)
	{
		int k=0;
		k=fscanf(fp, "%f",&point.x);
		if (k==EOF)//��ס�����ļ���ĩβ
		{
			break;
		}


		fscanf(fp, "%f",&point.y);
		fscanf(fp, "%f",&point.z);
		lhand_SG[n][j].push_back(point);
		fscanf(fp, "%f",&point.x);
		fscanf(fp, "%f",&point.y);
		fscanf(fp, "%f",&point.z);
		rhand_SG[n][j].push_back(point);
	}
	fclose(fp);

	// 	flhandx=lhand_SG[n][j][0].x;
	// 	flhandy=lhand_SG[n][j][0].y;
	// 	flhandz=lhand_SG[n][j][0].z;
	// 	frhandx=rhand_SG[n][j][0].x;
	// 	frhandy=rhand_SG[n][j][0].y;
	// 	frhandz=rhand_SG[n][j][0].z;
	// 
	// 	//��������������߶ȺͿ��
	// 	gridheight=(int)abs(((flhandy+frhandy)/2.0-head_SG[n][j].y/4.5));
	// 	gridwidth=(int)abs(((flhandx-frhandx)/2.2));
	// 	griddepth=(int)abs(((flhandx-frhandx)/2.2));
	// 
	// 	cx=5*gridwidth;
	// 	cy=2*gridheight;
	// 	cz=griddepth;


	// 	framenum=lhand_SG[n][j].size();
	// 	/* scale,ȥ�����š�С�����Լ���һ */
	// 	//ȥ����
	// 	for (int i=0; i < framenum; i++) 
	// 	{
	// 		lhand_SG[n][j][i].x=-(lhand_SG[n][j][i].x-head_SG[n][j].x)+cx;
	// 		lhand_SG[n][j][i].y=-(lhand_SG[n][j][i].y-head_SG[n][j].y)+cy;
	// 		lhand_SG[n][j][i].z=-(lhand_SG[n][j][i].z-head_SG[n][j].z)+cz;
	// 		rhand_SG[n][j][i].x=-(rhand_SG[n][j][i].x-head_SG[n][j].x)+cx;
	// 		rhand_SG[n][j][i].y=-(rhand_SG[n][j][i].y-head_SG[n][j].y)+cy;
	// 		rhand_SG[n][j][i].z=-(rhand_SG[n][j][i].z-head_SG[n][j].z)+cz;
	// 
	// 	}
	// 	//��һ
	// 	for (int i=0; i < framenum; i++) 
	// 	{
	// 		lhand_SG[n][j][i].x=(lhand_SG[n][j][i].x)/gridwidth+1;
	// 		lhand_SG[n][j][i].y=(lhand_SG[n][j][i].y)/gridheight+1;
	// 		lhand_SG[n][j][i].z=(lhand_SG[n][j][i].z)/griddepth+1;
	// 		rhand_SG[n][j][i].x=(rhand_SG[n][j][i].x)/gridwidth+1;
	// 		rhand_SG[n][j][i].y=(rhand_SG[n][j][i].y)/gridheight+1;
	// 		rhand_SG[n][j][i].z=(rhand_SG[n][j][i].z)/griddepth+1;
	// 	}
	return 1;
}

void CurveRecognition_SG(CvPoint3D32f headPoint3D_probe, vector<SLR_ST_Skeleton> vSkeletonData_probe, 
	CvPoint3D32f headPoint3D_gallery, vector<SLR_ST_Skeleton> vSkeletonData_gallery)
{

}

double Sentence_match(CvPoint3D32f probe_head_SG,vector<CvPoint3D32f> probe_lhand_SG,vector<CvPoint3D32f>probe_rhand_SG,CvPoint3D32f gallery_head_SG,vector<CvPoint3D32f> gallery_lhand_SG,vector<CvPoint3D32f>gallery_rhand_SG)
{
	double d=0;

	//-------data preprocessing
	int len_probe = probe_lhand_SG.size();
	int len_gallery = gallery_lhand_SG.size();

	float lhandx0[F];
	float lhandy0[F];
	float lhandz0[F];
	float rhandx0[F];
	float rhandy0[F];
	float rhandz0[F];

	float lhandxk[F];
	float lhandyk[F];
	float lhandzk[F];
	float rhandxk[F];
	float rhandyk[F];
	float rhandzk[F];

	float headx0 = (float)(probe_head_SG.x);
	float heady0 = (float)(probe_head_SG.y);
	float headz0 = (float)(probe_head_SG.z);

	for(int i=0; i<len_probe; i++)
	{
		lhandx0[i] = probe_lhand_SG[i].x;
		lhandy0[i] = probe_lhand_SG[i].y;
		lhandz0[i] = probe_lhand_SG[i].z;
		rhandx0[i] = probe_rhand_SG[i].x;
		rhandy0[i] = probe_rhand_SG[i].y;
		rhandz0[i] = probe_rhand_SG[i].z;
	}

	float headxk = (float)(gallery_head_SG.x);
	float headyk = (float)(gallery_head_SG.y);
	float headzk = (float)(gallery_head_SG.z);

	for(int i=0; i<len_gallery; i++)
	{
		lhandxk[i] = gallery_lhand_SG[i].x;
		lhandyk[i] = gallery_lhand_SG[i].y;
		lhandzk[i] = gallery_lhand_SG[i].z;
		rhandxk[i] = gallery_rhand_SG[i].x;
		rhandyk[i] = gallery_rhand_SG[i].y;
		rhandzk[i] = gallery_rhand_SG[i].z;
	}

	RawsentenceProcess(headx0,heady0,headz0,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,len_probe);
	RawsentenceProcess(headxk,headyk,headzk,lhandxk,lhandyk,lhandzk,rhandxk,rhandyk,rhandzk,len_gallery);

	MedianFliter(lhandx0,lhandy0,lhandz0,len_probe);
	MedianFliter(rhandx0,rhandy0,rhandz0,len_probe);
	MedianFliter(lhandxk,lhandyk,lhandzk,len_gallery);
	MedianFliter(rhandxk,rhandyk,rhandzk,len_gallery);

	const int loops=3;
	ReSample(lhandx0,lhandy0,lhandz0,len_probe,rsnum);
	ReSample(rhandx0,rhandy0,rhandz0,len_probe,rsnum);
	for (int t=1;t<loops;t++)
	{
		ReSample(lhandx0,lhandy0,lhandz0,rsnum,rsnum);
		ReSample(rhandx0,rhandy0,rhandz0,rsnum,rsnum);
	}

	ReSample(lhandxk,lhandyk,lhandzk,len_gallery,rsnum);
	ReSample(rhandxk,rhandyk,rhandzk,len_gallery,rsnum);
	for (int t=1;t<loops;t++)
	{
		ReSample(lhandxk,lhandyk,lhandzk,rsnum,rsnum);
		ReSample(rhandxk,rhandyk,rhandzk,rsnum,rsnum);
	}

	//---------------------------------------------------------
	Alignment_translate(rsnum,lhandx0,lhandy0,lhandz0);
	Alignment_translate(rsnum,lhandxk,lhandyk,lhandzk);
	Alignment_translate(rsnum,rhandx0,rhandy0,rhandz0);
	Alignment_translate(rsnum,rhandxk,rhandyk,rhandzk);

	d=CalDistance(lhandx0,lhandy0,lhandz0,lhandxk,lhandyk,lhandzk)
		+CalDistance(rhandx0,rhandy0,rhandz0,rhandxk,rhandyk,rhandzk);

	return d;
}
void RawsentenceProcess(float headx0, float heady0, float headz0, float lhandx0[],float lhandy0[],float lhandz0[],float rhandx0[],float rhandy0[],float rhandz0[],int n)//n: sentence length
{
	float flhandx,flhandy,flhandz,frhandx,frhandy,frhandz;//��һ֡�������ֵ�λ��
	int  gridheight;//����߶�
	int gridwidth;//������
	int griddepth;//�������
	//���������ͷ��λ�õ�ƫ����
	int cx;
	int cy;
	int cz;

	//flhandx=fir_setence_lhandx;
	//flhandy=fir_setence_lhandy;
	//flhandz=fir_setence_lhandz;
	//frhandx=fir_setence_rhandx;
	//frhandy=fir_setence_rhandy;
	//frhandz=fir_setence_rhandz;

	flhandx=lhandx0[0];
	flhandy=lhandy0[0];
	flhandz=lhandz0[0];
	frhandx=rhandx0[0];
	frhandy=rhandy0[0];
	frhandz=rhandz0[0];

	//��������������߶ȺͿ��
	gridheight=(int)abs((((flhandy+frhandy)/2-heady0)/4.5));
	gridwidth=(int)abs(((flhandx-frhandx)/2.2));
	griddepth=(int)abs(((flhandx-frhandx)/2.2));

	cx=5*gridwidth;
	cy=2*gridheight;
	cz=griddepth;

	/* scale,ȥ�����š�С�����Լ���һ */
	//ȥ����
	for (int i=0; i < n; i++) 
	{
		lhandx0[i]=-(lhandx0[i]-headx0)+cx;
		lhandy0[i]=-(lhandy0[i]-heady0)+cy;
		lhandz0[i]=-(lhandz0[i]-headz0)+cz;
		rhandx0[i]=-(rhandx0[i]-headx0)+cx;
		rhandy0[i]=-(rhandy0[i]-heady0)+cy;
		rhandz0[i]=-(rhandz0[i]-headz0)+cz;

	}
	//��һ
	for (int i=0; i < n; i++) 
	{
		lhandx0[i]=(lhandx0[i])/gridwidth+1;
		lhandy0[i]=(lhandy0[i])/gridheight+1;
		lhandz0[i]=(lhandz0[i])/griddepth+1;
		rhandx0[i]=(rhandx0[i])/gridwidth+1;
		rhandy0[i]=(rhandy0[i])/gridheight+1;
		rhandz0[i]=(rhandz0[i])/griddepth+1;
	}

	return;
}	

// void curveTest(CString filePath,CString fileName)
// {
// 	float headx0;float heady0;float headz0;
// 	float lhandx0[F];float lhandy0[F];float lhandz0[F];
// 	float rhandx0[F];float rhandy0[F];float rhandz0[F];
// 	int n;
// 	JustReadDataFile(filePath,1,0);
// 	headx0=gheadx[1][0];
// 	heady0=gheady[1][0];
// 	headz0=gheadz[1][0];
// 	CopyHandData(glhandx[1][0],glhandy[1][0],glhandz[1][0],grhandx[1][0],grhandy[1][0],grhandz[1][0],lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0);
// 	n=gframenum[1][0];
// 	//InitData();
// 	double resultCurve[WordsNum];
// 	CurveRecognision(headx0,heady0,headz0,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,n,resultCurve);
// 
// 	//fstream fresult(".\\videoResult.txt",ios::app);
// 	//fresult << fileName << " ";
// 	//for(int i=0; i<5; i++)
// 	//{
// 	//	int index = resultCurve[i].index;
// 	//	fresult << index << " ";
// 	//}
// 	//fresult << endl;
// 	//fresult.close();
// }
// void calculate()
// {
// 	for(int i=0; i<WordsNum; i++)
// 	{
// 		CString filePath = P00[i];
// 		CString fileName = filePath;
// 		curveTest(filePath,fileName);
// 	}
// }

//int main()
//{
//	float headx0;float heady0;float headz0;float lhandx0[F];float lhandy0[F];float lhandz0[F];float rhandx0[F];float rhandy0[F];float rhandz0[F];int n;
//	//CString positionfile="C:\\Users\\Lshun21\\Documents\\HCI\\����ʶ��\\����\\��ѡ�ʻ��ֹ���ע\\2ά\\P00\\P00_0011_1_0_20120221.oni.txt";
//	CString positionfile="D:\\data\\���´ʻ�켣����\\D3\\K50\\K50_0238_1_0_20121002.oni.txt";
//	JustReadDataFile(positionfile,1,0);
//	headx0=gheadx[1][0];
//	heady0=gheady[1][0];
//	headz0=gheadz[1][0];
//	CopyHandData(glhandx[1][0],glhandy[1][0],glhandz[1][0],grhandx[1][0],grhandy[1][0],grhandz[1][0],lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0);
//	n=gframenum[1][0];
//	InitData();
//	CurveRecognision(headx0,heady0,headz0,lhandx0,lhandy0,lhandz0,rhandx0,rhandy0,rhandz0,n);
//	MergeTrajectoryAndPostureResult(score_trajectory,score_posture,score);
//	Arrayrankingindex(score,WordsNum,ranking_index);
//	getchar();
//	return 0;
//}