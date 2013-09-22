#include "StdAfx.h"
#include "PostureRecognition.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Label_sequence1[Gallery_num*WordsNum*LRB];//原始的Label序列
int Label_sequence2[Gallery_num*WordsNum*LRB];//经过处理的Label序列
double *p_gallery;//Gallery HOG

HandSegment hand[LRB];
int Akeyframe;//左右双手所有关键帧的个数
IplImage* *p_probeimg;//存储所有关键帧的代表图像
double*    *p_probehog;//存储所有关键帧的代表图像对应的HOG特征，是一个二维数组

double similarity_matrix[Gallery_num][WordsNum];//
double result_matrix[WordsNum];       //合并后的距离矩阵（以最大值的方式），注意待议的归一化问题
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadData()
{
	string route1="Data";
	string route2="Data";
	ifstream infile1;
	infile1.open(route1+"\\Gallery_LabelNew.dat",ios::binary);
	infile1.read( (char *)&Label_sequence1[0],sizeof(Label_sequence1) );//将Gallery_Label中的数据读到数组Label_sequence1中
	infile1.close();

	int Label_size=Gallery_num*WordsNum*LRB;//Label_size=4*370*3
	int i;
	Label_sequence2[0]=Label_sequence1[0];
	for(i=1;i<Label_size;i++)
	{
		Label_sequence2[i]=Label_sequence2[i-1]+Label_sequence1[i];
	}
	/*
	Label_sequence2[ Label_size-1 ]表示所有关键帧的个数，下面开辟的空间即为文件Gallery_Data中存储的double数据的个数
	*/
	int HOG_size=Label_sequence2[ Label_size-1 ]*feature_dimension;//HOG_size
	p_gallery=new double[ HOG_size ];//p_gallery

	ifstream infile2;
	infile2.open(route2+"\\Gallery_DataNew.dat",ios::binary);
	infile2.read( (char *)p_gallery,HOG_size*sizeof(double) );
	infile2.close();
}
void DealProbeData()//将Probe中的数据从关键帧（图片）转换成HOG特征存起来
{
	Akeyframe=hand[0].keyframe_num + hand[1].keyframe_num + hand[2].keyframe_num;//左右双手，所有关键帧的个数
	p_probeimg=new IplImage* [ Akeyframe ];//存储所有关键帧的代表图像

	int i;
	p_probehog=new double*[ Akeyframe ];//是一个二维数组，存储所有关键帧的代表图像的HOG特征
	for(i=0;i<Akeyframe;i++)
		p_probehog[i]=new double[ feature_dimension ];
} 
double Img_distance(IplImage *dst1,IplImage *dst2)//返回两个图像的欧几里得距离
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
IplImage* Extract_image(int index1,int index2)//从关键帧序列中返回代表图像
{
	IplImage * ori_img;//原始图像
	IplImage * avg_img=cvCreateImage( cvSize(SIZE,SIZE),8,1);//均值图像
	uchar *pp;
	uchar *qq;
	int Img_sum[SIZE][SIZE];//用于图像求和
	memset( Img_sum,0,sizeof(Img_sum) );//先清零
	int k;
	int m,n;
	for(k=0;k<hand[index1].keyframe_no[index2];k++)
	{
		ori_img=hand[index1].keyframe_pic[index2][k];
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
			Img_sum[m][n]=Img_sum[m][n]/hand[index1].keyframe_no[index2];
			qq[n*avg_img->nChannels]=Img_sum[m][n];
		}
	}
	double mindistance=1.0*0xffffff;
	double imgtemp;
	int index;
	for(k=0;k<hand[index1].keyframe_no[index2];k++)
	{
		imgtemp=Img_distance( hand[index1].keyframe_pic[index2][k],avg_img );
		if(imgtemp<mindistance)
		{
			mindistance=imgtemp;
			index=k;
		}
	}
	cvReleaseImage(&avg_img);
	return hand[index1].keyframe_pic[index2][index];
}
bool GetHOGHistogram_Patch(IplImage *img,double hog_hist[])//取得图像img的HOG特征向量
{
	//HOGDescriptor *hog=new HOGDescriptor(cvSize(SIZE,SIZE),cvSize(8,8),cvSize(4,4),cvSize(4,4),9);
	HOGDescriptor *hog=new HOGDescriptor(cvSize(SIZE,SIZE),cvSize(16,16),cvSize(8,8),cvSize(8,8),9);
	/////////////////////window大小为64*64，block大小为8*8，block步长为4*4，cell大小为4*4
	Mat handMat(img);
	vector<float> *descriptors = new std::vector<float>();
	hog->compute(handMat, *descriptors,Size(0,0), Size(0,0));
	////////////////////window步长为0*0
	double total=0.0;
	int i;
	for(i=0;i<descriptors->size();i++)
		total+=abs((*descriptors)[i]);
	//	total=sqrt(total);
	for(i=0;i<descriptors->size();i++)
		hog_hist[i] = (*descriptors)[i]/total ;
	return true; 
}
void Get_Image_HOG()//建立Probe图像的HOG数组
{
	IplImage * Rep_img;//关键帧序列中的代表图像
	int Temp=0;
	int i,j;
	for(i=0;i<LRB;i++)//左右双手
	{
		for(j=0;j<hand[i].keyframe_num;j++)
		{
			Rep_img=Extract_image(i,j);
			p_probeimg[Temp]=Rep_img;
			GetHOGHistogram_Patch(Rep_img,p_probehog[Temp]);
			Temp++;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
double Hisogram(double *vec1,double *vec2)//返回两个向量直方图交的结果
{
	double mat_score=0.0;
	int i;
	for(i=0;i<feature_dimension;i++)
		mat_score+=vec1[i]<vec2[i] ? vec1[i] : vec2[i];
	return mat_score;
}
int combination[6440][15];//存放组合的结果，默认关键帧的个数<=15. 此时有C(15,7)=6435
int c_temp[15];//临时数组
int c_count;//组合的个数
int c_choice;//关键帧个数较小的那个
void Combination(int pos,int m,int n)//输出组合C(n,m)的所有情况
{
	int i;
	if(m==0)
	{
		for(i=1;i<=c_choice;i++)
			combination[c_count][i]=c_temp[i];
		c_count++;
		return ;
	}
	if(m>n)
		return ;
	if(n==0)
		return ;																								
	c_temp[pos]=n-1;///////////////////////非常非常重要！！！！！！！！！
	Combination(pos-1,m-1,n-1);
	Combination(pos,m,n-1);
}
double hand_distance(double* *xx,int x,double *yy,int y)//Probe和Gallery的左(右或双)手之间的相似度
{
	double Max=0.0;
	double result;
	if( x==0 || y==0 )//两者至少有一个没有关键帧
		return 0.0;
	if( x>15 || y>15 )//两者如有一个关键帧数>15则返回0, 作为异常处理
		return 0.0;
	c_count=0;//
	int i,j;
	if(x<=y)
	{
		c_choice=x;
		Combination(x,x,y);
		for(i=0;i<c_count;i++)
		{
			result=0;
			for(j=0;j<x;j++)
				result+=Hisogram( xx[j],yy+combination[i][j+1]*feature_dimension );
			if(result>Max)
				Max=result;
		}
		Max=Max/x;
	}
	else
	{
		c_choice=y;
		Combination(y,y,x);
		for(i=0;i<c_count;i++)
		{
			result=0;
			for(j=0;j<y;j++)
				result+=Hisogram( xx[ combination[i][j+1] ],yy+j*feature_dimension );
			if(result>Max)
				Max=result;
		}
		Max=Max/y;
	}
	return Max;
}
double Posture_distance(int gallery_pos,int posture_pos)
{
	int x1,y1,z1;//分别代表Gallery的左右双手的关键帧个数
	int x2,y2,z2;//分别代表Probe的左右双手的关键帧个数

	int pos=gallery_pos*WordsNum*LRB+posture_pos*LRB;
	double *pos1;//左手关键帧序列HOG特征在p_gallery数组中的起始地址
	double *pos2;//右手关键帧序列HOG特征在p_gallery数组中的起始地址
	double *pos3;//双手关键帧序列HOG特征在p_gallery数组中的起始地址

	if(pos==0)
		pos1=p_gallery;
	else
		pos1=p_gallery+Label_sequence2[pos-1]*feature_dimension;
	pos2=p_gallery+Label_sequence2[pos]*feature_dimension;
	pos3=p_gallery+Label_sequence2[pos+1]*feature_dimension;

	x1=*(Label_sequence1+pos);    //Gallery的左手关键帧个数
	y1=*(Label_sequence1+pos+1);//Gallery的右手关键帧个数
	z1=*(Label_sequence1+pos+2);//Gallery的双手关键帧个数
	x2=hand[0].keyframe_num;      //Probe的左手关键帧个数
	y2=hand[1].keyframe_num;      //Probe的右手关键帧个数
	z2=hand[2].keyframe_num;      //Probe的双手关键帧个数

	int weight=LRB;//
	double temp;
	double result=0.0;
	int i,j;
	double* *addresslist;///////////////////////////////////////////////////////////////The list of addresses
	for(i=0;i<LRB;i++)
	{
		if(i==0)//左手
		{
			addresslist=new double*[x2];
			for(j=0;j<x2;j++)
				addresslist[j]=p_probehog[j];
			temp=hand_distance(addresslist,x2,pos1,x1);   //计算probe和gallery左手的相似度
		}
		else if(i==1)//右手
		{
			addresslist=new double*[y2];
			for(j=0;j<y2;j++)
				addresslist[j]=p_probehog[x2+j];
			temp=hand_distance(addresslist,y2,pos2,y1);        //计算probe和gallery右手的相似度
		}
		else//双手
		{
			addresslist=new double*[z2];
			for(j=0;j<z2;j++)
				addresslist[j]=p_probehog[x2+y2+j];
			temp=hand_distance(addresslist,z2,pos3,z1);//计算probe和gallery双手的相似度
		}
		delete addresslist;//////////////////////////////////////////////////////////
		if(temp==0)
			weight--;
		result+=temp;
	}
	if(weight==0)
		result=0.0;
	else 
		result=result/weight;

	return result;
}
void Get_matrix()
{
	int i,j;
	double _max;
	double last_max=0.0;//
	for(i=0;i<WordsNum;i++)//手势个数
	{
		_max=0.0;
		for(j=0;j<Gallery_num;j++)//gallery个数
		{
			similarity_matrix[j][i]=Posture_distance(j,i);//similarity_matrix[i][j]表示Probe和第i个Gallery中的第j个手势的相似度
			if(similarity_matrix[j][i]>_max)
				_max=similarity_matrix[j][i];
		}
		result_matrix[i]=_max;//找出同一个手势对应的所有gallery中最大值（相似度）
		if( _max>last_max )
			last_max=_max;
	}
	if( last_max==0.0 )
		;
	else//归一化!!!
	{
		/*for(i=0;i<WordsNum;i++)
			result_matrix[i]=result_matrix[i]/last_max;*/
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////
void Del()
{
//	delete p_gallery;//////////////////////
	delete p_probeimg;///////////////////
	int i;
	for(i=0;i<Akeyframe;i++)
	{
		delete p_probehog[i];
	}

	delete p_probehog;///////////////////
}
void GetPostureScore(HandSegment leftHands,HandSegment rightHands,HandSegment bothHands,double* score)
{
	hand[0]=leftHands;
	hand[1]=rightHands;
	hand[2]=bothHands;
	DealProbeData();   //构造好Probe数据需要的数据结构
	Get_Image_HOG();//将Probe中的数据写入建好的数据结构中
	Get_matrix();//获得最终的归一化向量
	for(int i=0;i<WordsNum;i++)
		score[i]=result_matrix[i];
	Del();
}

void Arrayrankingindex(double a[],int length,int index[])//降序排列
{
	float temp=0.0;
	int int_temp=0;
	//下标初始化
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

//int main()
//{
//	//需要尹芳师姐提供路径
//	CString route1="";//route1是指Gallery_Label的路径
//	CString route2="";//route2是指Gallery_Data的路径
//	ReadData(route1,route2);//经过以上步骤，我们将Gallery_Label和Gallery_Data中的数据分别读到了int型数组Label_sequence1(Label_sequence2)和double型空间p中
//	/*
//	下面要从采集的视频片段中解析数据，然后写到结构体数组hand中
//	*/
//	DealProbeData();   //构造好Probe数据需要的数据结构
//	Get_Image_HOG();//将Probe中的数据写入建好的数据结构中
//	Get_matrix();//获得最终的归一化向量
//	return 0;
//}