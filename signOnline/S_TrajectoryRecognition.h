#pragma once
#include <windows.h>
#include <iostream>
#include <atlstr.h>
#include <opencv2\opencv.hpp>
#include "globalDefine.h"
#include <fstream>
#include <direct.h>

using namespace std;
using namespace cv;

CvPoint3D32f get_head_SG(int sentenceID);
vector<CvPoint3D32f> get_leftHand_SG(int sentenceID);
vector<CvPoint3D32f> get_rightHand_SG(int sentenceID);

int getCurrentFrameNo(int galleryIndex);
void getProbeCurve(CvPoint3D32f leftHand[], CvPoint3D32f rightHand[]);
void getGalleryCurve(CvPoint3D32f leftHand[], CvPoint3D32f rightHand[], int galleryIndex, int classIndex);
void setProbeCurve(CvPoint3D32f headPoint3D, vector<SLR_ST_Skeleton> vSkeletonData);
int getTra_resultSize();
int getTra_result(int index, int rank);
double getTra_result_score(int index, int rank);

int JustReadDataFile(CString positionfile,int j,int n);
void CopyHandData(float scrlhandx[F],float scrlhandy[F],float scrlhandz[F],float scrrhandx[F],float scrrhandy[F],float scrrhandz[F],float dstlhandx[F],float dstlhandy[F],float dstlhandz[F],float dstrhandx[F],float dstrhandy[F],float dstrhandz[F]);
int ReadEigDataFile(CString positionfile,int j,int k);
void RawdataProcess(float headx0, float heady0, float headz0, float lhandx0[],float lhandy0[],float lhandz0[],float rhandx0[],float rhandy0[],float rhandz0[],int n);
int ReadDataFile(CString positionfile,int j,int n);
//int GetE(float x[F],float y[F],float z[F],float E[dim][dim],int n);
int IsPrincipleDirMatch(float E0[dim][dim],float Ek[dim][dim]);
void ChangeEkAccordingToE0(float E0[dim][dim],float Ek[dim][dim]);
int GetCentroid(int n,float x[F],float y[F],float z[F],float& zx,float& zy,float& zz);
void BasisTransform(float Ek[dim][dim],float xk[],float yk[],float zk[]);
float AdjustDirection(float x0[],float y0[],float z0[],float xk[],float yk[],float zk[],float &d);
void GetFileNames(CString firstP00,CString P00[WordsNum],HANDLE hFind2,WIN32_FIND_DATA &FindFileData2);
double CalDistance(float x1[],float y1[],float z1[],float x2[],float y2[],float z2[]);
void ReSample(float x[F],float y[F],float z[F],int n,int m);
double DistanceOfTwoCurves(int k,float x0[],float y0[],float z0[],float xk[],float yk[],float zk[],float E0[dim][dim],float Ek[dim][dim]);
void InitData();
float GetMedian(float x[],int winlen);
void MedianFliter(float x[],float y[],float z[],int n);
int CurveRecognision(float headx0, float heady0, float headz0, 
	float lhandx0[],float lhandy0[], float lhandz0[],
	float rhandx0[],float rhandy0[], float rhandz0[],
	int n,double score[WordsNum],int sentenceID,int startFrameID, int endFrameID, 
	int trajectoryMatchedIndex[][Posture_num][2]);
void Arrayrankingindex2(double a[],int length,int ind[]);
int MergeTrajectoryAndPostureResult(double score_trajectory[WordsNum],double score_posture[WordsNum],double score[WordsNum]);
//void curve();
void CurveRecognition(CvPoint3D32f headPoint3D, vector<SLR_ST_Skeleton> vSkeletonData, 
	double score[WordsNum],int sentenceID,int startFrameID, int endFrameID, 
	int trajectoryMatchedIndex[][Posture_num][2]);
//void curveTest(CString filePath,CString fileName);
//void calculate();
void Get_Probe_IM(float probe_im[6][rsnum],int probe_num);
void Get_Gallery_IM(float gallery_im[6][rsnum],int begin,int end,int wordindex,int k);
void arraycopy_1D(float old_array[],float new_arrey[],int length);
double GetLenRate(int k,float x0[],float y0[],float z0[],float xk[],float yk[],float zk[]);
void CurveSegment(float x0[],float y0[],float z0[],int begin,int len_seg);
double RandomPartialMatchDistance(int k,float x0[],float y0[],float z0[],float xk[],float yk[],float zk[],int &beg ,int &end);
void Alignment_translate(int k,float xk[],float yk[],float zk[]);
double GetLenRate_6d(int k,float lx0[],float ly0[],float lz0[],float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],float rxk[],float ryk[],float rzk[]);
double Distance0fFixed_slidedCurves(int k,float lx0[],float ly0[],float lz0[],float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],float rxk[],float ryk[],float rzk[]);
double DistanceOfSlidedCurves(int k,float lx0[],float ly0[],float lz0[],float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],float rxk[],float ryk[],float rzk[]);
double DistanceOfGallerySlideProbe(int k,float lx0[],float ly0[],float lz0[],
	float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],
	float rxk[],float ryk[],float rzk[]);
double DistanceOfGallery_Slide_Probe(int k,float lx0[],float ly0[],float lz0[],
	float lxk[],float lyk[],float lzk[],
	float rx0[],float ry0[],float rz0[],
	float rxk[],float ryk[],float rzk[],int probe_num);
void traHandPositionInitial(SLR_ST_Skeleton vSkeletonData);
int Read_Sentece_File(CString positionfile,int j,int n);
double Sentence_match(CvPoint3D32f probe_head_SG,vector<CvPoint3D32f> probe_lhand_SG,vector<CvPoint3D32f>probe_rhand_SG,CvPoint3D32f gallery_head_SG,vector<CvPoint3D32f> gallery_lhand_SG,vector<CvPoint3D32f>gallery_rhand_SG);
void RawsentenceProcess(float headx0, float heady0, float headz0, float lhandx0[],float lhandy0[],float lhandz0[],float rhandx0[],float rhandy0[],float rhandz0[],int n);
