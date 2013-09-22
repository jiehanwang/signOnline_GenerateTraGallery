#include "StdAfx.h"
#include "PostureRecognition.h"
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int Label_sequence1[Gallery_num*WordsNum*LRB];//ԭʼ��Label����
int Label_sequence2[Gallery_num*WordsNum*LRB];//���������Label����
double *p_gallery;//Gallery HOG

HandSegment hand[LRB];
int Akeyframe;//����˫�����йؼ�֡�ĸ���
IplImage* *p_probeimg;//�洢���йؼ�֡�Ĵ���ͼ��
double*    *p_probehog;//�洢���йؼ�֡�Ĵ���ͼ���Ӧ��HOG��������һ����ά����

double similarity_matrix[Gallery_num][WordsNum];//
double result_matrix[WordsNum];       //�ϲ���ľ�����������ֵ�ķ�ʽ����ע�����Ĺ�һ������
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ReadData()
{
	string route1="Data";
	string route2="Data";
	ifstream infile1;
	infile1.open(route1+"\\Gallery_LabelNew.dat",ios::binary);
	infile1.read( (char *)&Label_sequence1[0],sizeof(Label_sequence1) );//��Gallery_Label�е����ݶ�������Label_sequence1��
	infile1.close();

	int Label_size=Gallery_num*WordsNum*LRB;//Label_size=4*370*3
	int i;
	Label_sequence2[0]=Label_sequence1[0];
	for(i=1;i<Label_size;i++)
	{
		Label_sequence2[i]=Label_sequence2[i-1]+Label_sequence1[i];
	}
	/*
	Label_sequence2[ Label_size-1 ]��ʾ���йؼ�֡�ĸ��������濪�ٵĿռ伴Ϊ�ļ�Gallery_Data�д洢��double���ݵĸ���
	*/
	int HOG_size=Label_sequence2[ Label_size-1 ]*feature_dimension;//HOG_size
	p_gallery=new double[ HOG_size ];//p_gallery

	ifstream infile2;
	infile2.open(route2+"\\Gallery_DataNew.dat",ios::binary);
	infile2.read( (char *)p_gallery,HOG_size*sizeof(double) );
	infile2.close();
}
void DealProbeData()//��Probe�е����ݴӹؼ�֡��ͼƬ��ת����HOG����������
{
	Akeyframe=hand[0].keyframe_num + hand[1].keyframe_num + hand[2].keyframe_num;//����˫�֣����йؼ�֡�ĸ���
	p_probeimg=new IplImage* [ Akeyframe ];//�洢���йؼ�֡�Ĵ���ͼ��

	int i;
	p_probehog=new double*[ Akeyframe ];//��һ����ά���飬�洢���йؼ�֡�Ĵ���ͼ���HOG����
	for(i=0;i<Akeyframe;i++)
		p_probehog[i]=new double[ feature_dimension ];
} 
double Img_distance(IplImage *dst1,IplImage *dst2)//��������ͼ���ŷ����þ���
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
IplImage* Extract_image(int index1,int index2)//�ӹؼ�֡�����з��ش���ͼ��
{
	IplImage * ori_img;//ԭʼͼ��
	IplImage * avg_img=cvCreateImage( cvSize(SIZE,SIZE),8,1);//��ֵͼ��
	uchar *pp;
	uchar *qq;
	int Img_sum[SIZE][SIZE];//����ͼ�����
	memset( Img_sum,0,sizeof(Img_sum) );//������
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
bool GetHOGHistogram_Patch(IplImage *img,double hog_hist[])//ȡ��ͼ��img��HOG��������
{
	//HOGDescriptor *hog=new HOGDescriptor(cvSize(SIZE,SIZE),cvSize(8,8),cvSize(4,4),cvSize(4,4),9);
	HOGDescriptor *hog=new HOGDescriptor(cvSize(SIZE,SIZE),cvSize(16,16),cvSize(8,8),cvSize(8,8),9);
	/////////////////////window��СΪ64*64��block��СΪ8*8��block����Ϊ4*4��cell��СΪ4*4
	Mat handMat(img);
	vector<float> *descriptors = new std::vector<float>();
	hog->compute(handMat, *descriptors,Size(0,0), Size(0,0));
	////////////////////window����Ϊ0*0
	double total=0.0;
	int i;
	for(i=0;i<descriptors->size();i++)
		total+=abs((*descriptors)[i]);
	//	total=sqrt(total);
	for(i=0;i<descriptors->size();i++)
		hog_hist[i] = (*descriptors)[i]/total ;
	return true; 
}
void Get_Image_HOG()//����Probeͼ���HOG����
{
	IplImage * Rep_img;//�ؼ�֡�����еĴ���ͼ��
	int Temp=0;
	int i,j;
	for(i=0;i<LRB;i++)//����˫��
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
double Hisogram(double *vec1,double *vec2)//������������ֱ��ͼ���Ľ��
{
	double mat_score=0.0;
	int i;
	for(i=0;i<feature_dimension;i++)
		mat_score+=vec1[i]<vec2[i] ? vec1[i] : vec2[i];
	return mat_score;
}
int combination[6440][15];//�����ϵĽ����Ĭ�Ϲؼ�֡�ĸ���<=15. ��ʱ��C(15,7)=6435
int c_temp[15];//��ʱ����
int c_count;//��ϵĸ���
int c_choice;//�ؼ�֡������С���Ǹ�
void Combination(int pos,int m,int n)//������C(n,m)���������
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
	c_temp[pos]=n-1;///////////////////////�ǳ��ǳ���Ҫ������������������
	Combination(pos-1,m-1,n-1);
	Combination(pos,m,n-1);
}
double hand_distance(double* *xx,int x,double *yy,int y)//Probe��Gallery����(�һ�˫)��֮������ƶ�
{
	double Max=0.0;
	double result;
	if( x==0 || y==0 )//����������һ��û�йؼ�֡
		return 0.0;
	if( x>15 || y>15 )//��������һ���ؼ�֡��>15�򷵻�0, ��Ϊ�쳣����
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
	int x1,y1,z1;//�ֱ����Gallery������˫�ֵĹؼ�֡����
	int x2,y2,z2;//�ֱ����Probe������˫�ֵĹؼ�֡����

	int pos=gallery_pos*WordsNum*LRB+posture_pos*LRB;
	double *pos1;//���ֹؼ�֡����HOG������p_gallery�����е���ʼ��ַ
	double *pos2;//���ֹؼ�֡����HOG������p_gallery�����е���ʼ��ַ
	double *pos3;//˫�ֹؼ�֡����HOG������p_gallery�����е���ʼ��ַ

	if(pos==0)
		pos1=p_gallery;
	else
		pos1=p_gallery+Label_sequence2[pos-1]*feature_dimension;
	pos2=p_gallery+Label_sequence2[pos]*feature_dimension;
	pos3=p_gallery+Label_sequence2[pos+1]*feature_dimension;

	x1=*(Label_sequence1+pos);    //Gallery�����ֹؼ�֡����
	y1=*(Label_sequence1+pos+1);//Gallery�����ֹؼ�֡����
	z1=*(Label_sequence1+pos+2);//Gallery��˫�ֹؼ�֡����
	x2=hand[0].keyframe_num;      //Probe�����ֹؼ�֡����
	y2=hand[1].keyframe_num;      //Probe�����ֹؼ�֡����
	z2=hand[2].keyframe_num;      //Probe��˫�ֹؼ�֡����

	int weight=LRB;//
	double temp;
	double result=0.0;
	int i,j;
	double* *addresslist;///////////////////////////////////////////////////////////////The list of addresses
	for(i=0;i<LRB;i++)
	{
		if(i==0)//����
		{
			addresslist=new double*[x2];
			for(j=0;j<x2;j++)
				addresslist[j]=p_probehog[j];
			temp=hand_distance(addresslist,x2,pos1,x1);   //����probe��gallery���ֵ����ƶ�
		}
		else if(i==1)//����
		{
			addresslist=new double*[y2];
			for(j=0;j<y2;j++)
				addresslist[j]=p_probehog[x2+j];
			temp=hand_distance(addresslist,y2,pos2,y1);        //����probe��gallery���ֵ����ƶ�
		}
		else//˫��
		{
			addresslist=new double*[z2];
			for(j=0;j<z2;j++)
				addresslist[j]=p_probehog[x2+y2+j];
			temp=hand_distance(addresslist,z2,pos3,z1);//����probe��gallery˫�ֵ����ƶ�
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
	for(i=0;i<WordsNum;i++)//���Ƹ���
	{
		_max=0.0;
		for(j=0;j<Gallery_num;j++)//gallery����
		{
			similarity_matrix[j][i]=Posture_distance(j,i);//similarity_matrix[i][j]��ʾProbe�͵�i��Gallery�еĵ�j�����Ƶ����ƶ�
			if(similarity_matrix[j][i]>_max)
				_max=similarity_matrix[j][i];
		}
		result_matrix[i]=_max;//�ҳ�ͬһ�����ƶ�Ӧ������gallery�����ֵ�����ƶȣ�
		if( _max>last_max )
			last_max=_max;
	}
	if( last_max==0.0 )
		;
	else//��һ��!!!
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
	DealProbeData();   //�����Probe������Ҫ�����ݽṹ
	Get_Image_HOG();//��Probe�е�����д�뽨�õ����ݽṹ��
	Get_matrix();//������յĹ�һ������
	for(int i=0;i<WordsNum;i++)
		score[i]=result_matrix[i];
	Del();
}

void Arrayrankingindex(double a[],int length,int index[])//��������
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

//int main()
//{
//	//��Ҫ����ʦ���ṩ·��
//	CString route1="";//route1��ָGallery_Label��·��
//	CString route2="";//route2��ָGallery_Data��·��
//	ReadData(route1,route2);//�������ϲ��裬���ǽ�Gallery_Label��Gallery_Data�е����ݷֱ������int������Label_sequence1(Label_sequence2)��double�Ϳռ�p��
//	/*
//	����Ҫ�Ӳɼ�����ƵƬ���н������ݣ�Ȼ��д���ṹ������hand��
//	*/
//	DealProbeData();   //�����Probe������Ҫ�����ݽṹ
//	Get_Image_HOG();//��Probe�е�����д�뽨�õ����ݽṹ��
//	Get_matrix();//������յĹ�һ������
//	return 0;
//}