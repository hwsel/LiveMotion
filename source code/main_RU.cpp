

/*================================================================
file deccription:
corresponding name +"result.txt"+ parameters
0 :wrong 1:right distance unit:pixel
basic pridiction (rate+distance) || velocitybased (rate+distance) || enhanced A (rate+distance)|| enhanced B ((rate+More Tile number))|| tiles number || total tiles number

corresponding name+"overheadresult.txt"

Tracking || velocity based || velocity + recovery || velocity + Update

=================================================================*/
/*======================main==================================*/





#include<math.h>
#include <SDKDDKVer.h>
#include <Windows.h>
#include <ctime>

#pragma warning(disable:4819)
#pragma warning(disable:4996)

// for OpenCV2

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/gpu/gpu.hpp"
#include "opencv2/highgui/highgui.hpp"

#ifdef _DEBUG
#pragma comment(lib, "opencv_core330d.lib")
#pragma comment(lib, "opencv_imgproc330d.lib")
#pragma comment(lib, "opencv_objdetect330d.lib")
//#pragma comment(lib, "opencv_gpu320d.lib")
#pragma comment(lib, "opencv_highgui330d.lib")
#else
#pragma comment(lib, "opencv_core320.lib")
#pragma comment(lib, "opencv_imgproc320.lib")
#pragma comment(lib, "opencv_objdetect320.lib")
#pragma comment(lib, "opencv_gpu320.lib")
#pragma comment(lib, "opencv_highgui320.lib")
#endif

/*===========================================================*/

#include "opencv2/opencv.hpp"  
#include "opencv2/video/background_segm.hpp"  

#include <iostream>  
#include <string> 
#include <fstream>
#include <vector>
//#include <cv.h>
//#include <cxcore.h>
//#include <highgui.h>


//#include <iostream>
//#include <string>
//#include <vector>
//#include <fstream>
#include <sstream>

#include <stdio.h>    
#include <windows.h>  
#include "cv.h"    
#include "cxcore.h"    
#include "highgui.h"    
#include <opencv2\opencv.hpp>    
using namespace cv;
using namespace std;
static const double pi = 3.14159265358979323846;

#define Video_Divide 10      // (tile number)indicate how we devide the video, the total number should be Video_Divide*Video_Divide
#define Video_Buffer_Len 25*4   // based on the frame rate it should be 1,2,3,4,5 seconds and related value
#define Shrink_Speed 3     // set the value of HIGH in velocity+ update method

#define ShowFeature 1
#define MASK 1

#define RATEFRAME 30  //define the frame rate of each video
#define DataAnalyze false
#define DataShow false

int BlockStatus[Video_Divide*Video_Divide];

// make sure the minus number is correct

double stringToDouble(string num)
{
	bool minus = false;      //
	string real = num;       //real
	if (num.at(0) == '-')
	{
		minus = true;
		real = num.substr(1, num.size() - 1);
	}

	char c;
	int i = 0;
	double result = 0.0, dec = 10.0;
	bool isDec = false;       //
	unsigned long size = real.size();
	while (i < size)
	{
		c = real.at(i);
		if (c == '.')
		{//
			isDec = true;
			i++;
			continue;
		}
		if (!isDec)
		{
			result = result * 10 + c - '0';
		}
		else
		{//
			result = result + (c - '0') / dec;
			dec *= 10;
		}
		i++;
	}

	if (minus == true) {
		result = -result;
	}

	return result;
}

struct Video_Block
{
	int XH;
	int XL;
	int YH;
	int YL;
};

// coordinate calculation
// calculate the foward vector based on the unit quaternion
void cal_forward_ProjectToFrame(double * UQ, double *FV)
{
	double qx = UQ[0];
	double qy = UQ[1];
	double qz = UQ[2];
	double qw = UQ[3];

	double x = 2 * qx*qz + 2 * qy*qw;
	double y = 2 * qy*qz - 2 * qx*qw;
	double z = 1 - 2 * qx*qx - 2 * qy*qy;

	double a = acos(sqrt(x*x+z*z)/sqrt(x*x+y*y+z*z));
	if (y > 0)
	{
		FV[0] = a / pi * 180;
	}
	else
	{
		FV[0] = -a / pi * 180;
	}
	//cout << "show a=   " << a << endl;
	double b = acos(x/sqrt(x*x + z*z));
	if (z < 0)			//original is z<0
	{
		FV[1] = b / pi * 180;
	}
	else
	{
		FV[1] = (2-b / pi) * 180;
	}
	//cout << "show b=   " << b << endl;
}


int CalculateDistance(int A_X, int A_Y, int B_X, int B_Y)
{
	int Dist;
	Dist = int(sqrt((A_X - B_X)*(A_X - B_X) + (A_Y - B_Y)*(A_Y - B_Y)));
	return Dist;
}

//fix the error calculate the new bandwidth
int CheckErrorCorrecBlockNO(int * blocklist, IplImage *frame, int A_XH, int A_XL, int A_YH, int A_YL)
{
	int count_blocks = 0;
	int i = 0;
	int L = frame->width;
	int H = frame->height;
	int DL = int(L / Video_Divide);
	int DH = int(H / Video_Divide);
	int LN;
	int HN;
	int XH;
	int XL;
	int YH;
	int YL;
	for (i = 0; i < Video_Divide*Video_Divide; i++)
	{
		if (blocklist[i] == 1)continue;
		LN = int(i % Video_Divide);
		HN = int(i / Video_Divide);
		XH = (LN + 1)*DL - 1;
		YH = (HN + 1)*DH - 1;
		XL = LN*DL + 1;
		YL = HN*DH + 1;
		if (((XH <= A_XH) && (XH >= A_XL) && (YH <= A_YH) && (YH >= A_YL)) ||
			((XL <= A_XH) && (XL >= A_XL) && (YH <= A_YH) && (YH >= A_YL)) ||
			((XH <= A_XH) && (XH >= A_XL) && (YL <= A_YH) && (YL >= A_YL)) ||
			((XL <= A_XH) && (XL >= A_XL) && (YL <= A_YH) && (YL >= A_YL))
			)
		{
			count_blocks++;
		}
	}
	return count_blocks;
}

///struct Video_block VBlock[Video_Divide*Video_Divide];
void UpdateBlockByOther(int * blocklist, IplImage *frame,
	int A_XH, int A_XL, int A_YH, int A_YL,
	int B_XH, int B_XL, int B_YH, int B_YL
)
{
	int i = 0;
	int L = frame->width;
	int H = frame->height;
	int DL = int(L / Video_Divide);
	int DH = int(H / Video_Divide);
	int LN;
	int HN;
	int XH;
	int XL;
	int YH;
	int YL;
	for (i = 0; i < Video_Divide*Video_Divide; i++)
	{
		LN = int(i % Video_Divide);
		HN = int(i / Video_Divide);
		XH = (LN + 1)*DL - 1;
		YH = (HN + 1)*DH - 1;
		XL = LN*DL + 1;
		YL = HN*DH + 1;
		if (((XH <= A_XH) && (XH >= A_XL) && (YH <= A_YH) && (YH >= A_YL)) ||
			((XL <= A_XH) && (XL >= A_XL) && (YH <= A_YH) && (YH >= A_YL)) ||
			((XH <= A_XH) && (XH >= A_XL) && (YL <= A_YH) && (YL >= A_YL)) ||
			((XL <= A_XH) && (XL >= A_XL) && (YL <= A_YH) && (YL >= A_YL)) ||
			((XH <= B_XH) && (XH >= B_XL) && (YH <= B_YH) && (YH >= B_YL)) ||
			((XL <= B_XH) && (XL >= B_XL) && (YH <= B_YH) && (YH >= B_YL)) ||
			((XH <= B_XH) && (XH >= B_XL) && (YL <= B_YH) && (YL >= B_YL)) ||
			((XL <= B_XH) && (XL >= B_XL) && (YL <= B_YH) && (YL >= B_YL))
			)
		{
			blocklist[i] = 1;
			BlockStatus[i] = Shrink_Speed;
		}
	}
}
int CheckIfInList(int * blocklist, IplImage *frame, int x, int y)
{
	int L = frame->width;
	int H = frame->height;
	int DL = int(L / Video_Divide);
	int DH = int(H / Video_Divide);
	int X = int((x - 1) / DL);
	int Y = int((y - 1) / DH);
	int BlockIndex = X + (Y)*Video_Divide;
	return blocklist[BlockIndex];
}
void UpdateSmallBlocksSet(int * blocklist, IplImage *frame, int x, int y)
{
	//	int XH, XL, YH, YL;
	int L = frame->width;
	int H = frame->height;
	int DL = int(L / Video_Divide);
	int DH = int(H / Video_Divide);
	int X = int((x - 1) / DL);
	int Y = int((y - 1) / DH);
	int BlockIndex = X + (Y)*Video_Divide;
	blocklist[BlockIndex] = 1;
	BlockStatus[BlockIndex] = Shrink_Speed;
}
void MaintainBlocks(int * blocklist)
{
	int i = 0;
	for (i = 0; i < Video_Divide*Video_Divide; i++)
	{
		if (blocklist[i] == 1)
		{
			BlockStatus[i] = BlockStatus[i] - 1;
			if (BlockStatus[i] == 0)
			{
				blocklist[i] = 0;
			}
		}
	}
}
int GetInterestedBlockNO(int * blocklist)
{
	int i;
	int count = 0;
	for (i = 0; i < Video_Divide*Video_Divide; i++)
	{
		if (blocklist[i])
			count++;
	}
	return count;
}
void ShowSmallBlocks(int * blocklist, IplImage *frame, int k)
{
	int i, j;
	//	int XH, XL, YH, YL;
	int L = frame->width;
	int H = frame->height;
	int DL = int(L / Video_Divide);
	int DH = int(H / Video_Divide);
	int LN = int(k % Video_Divide);
	int HN = int(k / Video_Divide);
	if (blocklist[k] == 0)
	{
		cvRectangle(frame, cvPoint(LN*DL, HN*DH), cvPoint((LN + 1)*DL, (HN + 1)*DH), Scalar(255, 255, 0), 1, 8, 0);
	}
	else
	{
		cvRectangle(frame, cvPoint(LN*DL, HN*DH), cvPoint((LN + 1)*DL, (HN + 1)*DH), Scalar(255, 0, 255), 3, 8, 0);
	}
	/*	for (i = 0; i < Video_Divide*Video_Divide; i++)
	{
	;
	}*/
}

inline static double square(int a)
{
	return a * a;
}

inline static void allocateOnDemand(IplImage **img, CvSize size, int depth, int channels)
{
	if (*img != NULL) return;
	*img = cvCreateImage(size, depth, channels);
	if (*img == NULL)
	{
		fprintf(stderr, "Error: Couldn't allocate image.  Out of memory?\n");
		exit(-1);
	}
}
uchar CheckPixelValue(Mat *Image, int x, int y)
{
	int nr = Image->rows;
	int nc = Image->cols*Image->channels();
	uchar *data = Image->ptr<uchar>(x);
	uchar DataBack = data[y];
	return DataBack;
}

void CopySubImage(IplImage *sre, IplImage *dst, int xl,int yl, int xh, int yh)
{
//	xl = 360; yl = 360; xh = 600; yh = 600;
	CvRect roi_sre = cvRect(xl, yl, (xh - xl), (yh - yl));
	CvRect roi_dst = cvRect(xl, yl, (xh - xl), (yh - yl));
	cvSetImageROI(sre, roi_sre);
	cvSetImageROI(dst, roi_dst);
	cvCopy(sre, dst);
	cvResetImageROI(sre);
	cvResetImageROI(dst);
}
void AddMaskBlocks(int * Block_List, IplImage *sre, IplImage *dst, int k)
{
	int L = dst->width;
	int H = dst->height;
	int DL = int(L / Video_Divide);
	int DH = int(H / Video_Divide);
	int LN = int(k % Video_Divide);
	int HN = int(k / Video_Divide);
	int xl = LN*DL;
	int yl = HN*DH;
	int xh = (LN + 1)*DL;
	int yh = (HN + 1)*DH;
	if (Block_List[k] == 1)CopySubImage(sre, dst,  xl,  yl,  xh,  yh);

}

void createAlphaMat(Mat &mat)
{
	CV_Assert(mat.channels() == 4);
	for (int i = 0; i < mat.rows; ++i) {
		for (int j = 0; j < mat.cols; ++j) {
			Vec4b& bgra = mat.at<Vec4b>(i, j);
			bgra[0] = UCHAR_MAX; // Blue
			bgra[1] = saturate_cast<uchar>((float(mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX); // Green
			bgra[2] = saturate_cast<uchar>((float(mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX); // Red
			bgra[3] = saturate_cast<uchar>(0.5 * (bgra[1] + bgra[2])); // Alpha
		}
	}
}

int main(int argc, char *argv[])
{
	/*=================================================================================
	test csv writeer
	===================================================================================*/
	/*
	for (int i = 0; i < 3; i++)
	{
		ofstream outFile;
		outFile.open("dataToStoreK.csv", ios::app); 
		outFile << "basic pridiction" << ',' << "velocitybased" << ',' << "enhanced A" << ',' << "enhanced B" << ',' << "tiles number" << ',' << endl;
		outFile << "name" << ',' << "age" << ',' << "hobby" << endl;
		outFile << "Mike" << ',' << 18 << ',' << "paiting" << endl;
		outFile << "Tom" << ',' << 25 << ',' << "football" << endl;
		outFile << "Jack" << ',' << 21 << ',' << "music" << endl;
		outFile.close();
	}
	return 1;
	*/

	/*=========================================================================
	This part we read all user data and show it in the video and store the result for analysis
	==========================================================================*/
	if (DataShow)
	{
		//2-5-Fighting
		VideoCapture cap("2-4-FemaleBasketball.mp4");     // the orignal is   ElephantsOldB  ||  ElephantsC || Wild_Horses ||5cutecat || cow  ||Female Basketball Match ||2-4-FemaleBasketball

		if (!cap.isOpened())
		{
			return -1;
		}
		Mat frame;
		CvSize frame_size;
		frame_size.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		frame_size.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		double FPSFromeVideo = cap.get(CV_CAP_PROP_FPS);
		static IplImage *frameT = NULL;
		Mat frametT;
		Mat ImageforAll;
		
		int r = 20;    //	
		// this part test the format of the video
		Point centerT = Point(100, 100);
		//circle(frametT, centerT, r, Scalar(255, 0, 0), -1);//-1
		
		string Filename_AVI = "allUserView_4to3.avi";
		//Filename_AVI = NameStr + Filename_AVI + NameB.str() + "S_" + NameA.str() + "T_" + NameC.str() + "V" + ".avi";
		//	CvVideoWriter *writer = cvCreateVideoWriter("ElephantsOldB_prediction.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, cvSize(frame_size.width, frame_size.height));
		const char *Filename_AVI_C = Filename_AVI.c_str();
		CvVideoWriter *writer = cvCreateVideoWriter(Filename_AVI_C, CV_FOURCC('D', 'I', 'V', 'X'), FPSFromeVideo, cvSize(frame_size.width, frame_size.height));
		

		ifstream inFile("dataToStore_Change3to4B.csv", ios::in);
		string lineStr;
		int Fx[48];
		int Fy[48];
		int ii = 0;
		int flagImag = 1;
		int countNo = 0;

		//I want to draw all the user pots in one image
		Mat mat(frame_size.height, frame_size.width, CV_8UC4);
		createAlphaMat(mat);
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		imwrite("TestAllB.png", mat, compression_params);

		while (getline(inFile, lineStr))
		{
			//if (lineStr == NULL)
			//	break;
			countNo++;
			//  cap >> framet;  
			if (lineStr == "")
				break;
			cap.read(frametT);
			//Mat Image = imread("TestAll.png");
			stringstream ss(lineStr);
			string Qx;
			if (flagImag)
			{
				ImageforAll = frametT.clone();
				flagImag = 0;
			}
			for (ii = 0; ii < 48; ii ++ )
			{
				getline(ss, Qx, ',');
				Fx[ii] = (int)stringToDouble(Qx);
			}
			getline(inFile, lineStr);
			if (lineStr == "")
				break;
			stringstream ssb(lineStr);
			for (ii = 0; ii < 48; ii++)
			{
				getline(ssb, Qx, ',');
				Fy[ii] = (int)stringToDouble(Qx);
				Point center = Point(Fx[ii], Fy[ii]);
				circle(frametT, center, r, Scalar(123, 21, 32), -1);//-1
				Point centerT = Point(100, 200);
				circle(frametT, centerT, r, Scalar(123, 255, 0), -1);//-1
				circle(ImageforAll, center, r, Scalar(123, 21, 32), -1);//-1
			}
			//imwrite("TestAll.png", ImageforAll, compression_params);
			frameT = &IplImage(frametT);//
			cvNamedWindow("Test user video ", CV_WINDOW_NORMAL);
			cvShowImage("Test user video ", frameT);    //  Usr_View   NewVideoF
			cvWriteFrame(writer, frameT);
			cvWaitKey(33);		
			if (countNo >= 8192)
				break;			//5280
		}

		imwrite("TestAllCYCV.png", ImageforAll, compression_params);
		
		cap.release();
		return 1;

	}
	/*====================================================================
	This code block will read all csv files and show all the user data in one video
	=====================================================================*/
	if (DataAnalyze)
	{
		int CountUser = 48;
		
		for (int i = 1; i <= CountUser; i++)
		{
			stringstream NameC;
			NameC << i; 
			//NameC.str()
			string NameList = "video_3_"+ NameC.str()+".csv";
			ifstream inFile(NameList, ios::in);
			string lineStr;
			vector<vector<string>> strArray;
			int LineNo = 0;
			double TimeStempSedCur = 1;
			double TimeStempSedPre = 1;

			// output file======================================
			ofstream outFile;
			outFile.open("dataToStore_Change3to4.csv", ios::app);
			/*==========================The modified code block to read CSV file in to parts we want==============================*/
			//=read the first line(row in excel)   In CSV file the first line (row) is the title/legen
			getline(inFile, lineStr);
			cout << "The first line of notification" << lineStr << endl;
			//=read the second line which contains the real data we need 
			getline(inFile, lineStr);

			stringstream ss(lineStr);
			string Qx;
			string Qy;
			string Qz;
			string Qw;
			string Fx;
			string Fy;
			string Fz;
			vector<string> lineArray;
			// 
			getline(ss, Qx, ',');
			// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second 
			stringstream ssb(Qx);
			getline(ssb, Qx, ':');
			cout << "1   time stamp   hour   " << Qx << endl;
			getline(ssb, Qx, ':');
			cout << "2  time stamp   minutes   " << Qx << endl;
			getline(ssb, Qx, ':');
			cout << "3   time stamp    second   " << Qx << endl;
			TimeStempSedCur = stringToDouble(Qx);
			TimeStempSedCur = floor(TimeStempSedCur);
			TimeStempSedPre = TimeStempSedCur + 1;		//TimeStempSedCur represent the upper threshold 
														// play back time data "0.000s"
			getline(ss, Qx, ',');//delete the first two data which is the playback and time stemp
			cout << "4   play back time    second   " << Qx << endl;
			double PlayBacks;
			PlayBacks = stringToDouble(Qx);
			cout << "4   play back time    second (in double)  " << PlayBacks << endl;
			getline(ss, Qx, ',');
			getline(ss, Qy, ',');
			getline(ss, Qz, ',');
			getline(ss, Qw, ',');
			getline(ss, Fx, ',');
			getline(ss, Fy, ',');
			getline(ss, Fz, ',');
			cout << "unit quaternion x" << Qx << endl;
			cout << "unit quaternion y" << Qy << endl;
			cout << "unit quaternion z" << Qz << endl;
			cout << "unit quaternion w" << Qw << endl;
			cout << "HmdPosition.x" << Fx << endl;
			cout << "HmdPosition.y" << Fy << endl;
			cout << "HmdPosition.z" << Fz << endl;

			//cout << "The first line of notification" << lineStr << endl;
			// calculate the position in video
			//double Fx;
			//double Fy;
			double DQ[4], FLocation[2];
			DQ[0] = stringToDouble(Qx);
			DQ[1] = stringToDouble(Qy);
			DQ[2] = stringToDouble(Qz);
			DQ[3] = stringToDouble(Qw);
			cout << "unit quaternion x in number" << DQ[0] << endl;
			cout << "unit quaternion y in number" << DQ[1] << endl;
			cout << "unit quaternion z in number" << DQ[2] << endl;
			cout << "unit quaternion w in number" << DQ[3] << endl;

			cal_forward_ProjectToFrame(DQ, FLocation);
			cout << "Position y in frame" << FLocation[0] << endl;
			cout << "Position x in frame" << FLocation[1] << endl;
			double DoubleFx;
			double DoubleFy;
			DoubleFx = FLocation[1] / 360;
			DoubleFy = (90 - FLocation[0]) / 180;
			cout << "Position x in frame" << DoubleFx << endl;
			cout << "Position y in frame" << DoubleFy << endl;
			
			//2-5-Fighting
			VideoCapture cap("2-4-FemaleBasketball.mp4");     // the orignal is   ElephantsOldB  ||  ElephantsC || Wild_Horses ||5cutecat || cow  ||Female Basketball Match ||2-4-FemaleBasketball

			if (!cap.isOpened())
			{
				return -1;
			}
			Mat frame;
			CvSize frame_size;
			frame_size.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
			frame_size.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
			double FPSFromeVideo = cap.get(CV_CAP_PROP_FPS);

			//========================avoid the noise of 1 go back to 0======================
			double XmaxCSV = 0;
			double XminCSV = 1;
			double prePlayBack = 0;
			double curPlayBack;
			/*
			while (getline(inFile, lineStr))
			{
				stringstream ss(lineStr);
				string Qx;
				string Qy;
				string Qz;
				string Qw;
				string Fx;
				string Fy;
				string Fz;
				vector<string> lineArray;
				// 按照逗号分隔
				getline(ss, Qx, ',');
				// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second 
				stringstream ssb(Qx);
				getline(ssb, Qx, ':');
				//cout << "1   time stamp   hour   " << Qx << endl;
				getline(ssb, Qx, ':');
				//cout << "2  time stamp   minutes   " << Qx << endl;
				getline(ssb, Qx, ':');
				//cout << "3   time stamp    second   " << Qx << endl;
				TimeStempSedCur = stringToDouble(Qx);
				TimeStempSedCur = floor(TimeStempSedCur);
				TimeStempSedPre = TimeStempSedCur + 1;		//TimeStempSedCur represent the upper threshold 
															// play back time data "0.000s"
				getline(ss, Qx, ',');//delete the first two data which is the playback and time stemp
				curPlayBack = stringToDouble(Qx);
				if (curPlayBack < prePlayBack)
				{
					//prePlayBack = curPlayBack;
					break;
				}
				else
				{
					prePlayBack = curPlayBack;
				}
				//cout << "4   play back time    second   " << Qx << endl;
				double PlayBacks;
				PlayBacks = stringToDouble(Qx);
				//cout << "4   play back time    second (in double)  " << PlayBacks << endl;
				getline(ss, Qx, ',');
				getline(ss, Qy, ',');
				getline(ss, Qz, ',');
				getline(ss, Qw, ',');
				getline(ss, Fx, ',');
				getline(ss, Fy, ',');
				getline(ss, Fz, ',');
				//cout << "unit quaternion x" << Qx << endl;
				//cout << "unit quaternion y" << Qy << endl;
				//cout << "unit quaternion z" << Qz << endl;
				//cout << "unit quaternion w" << Qw << endl;
				//cout << "HmdPosition.x" << Fx << endl;
				//cout << "HmdPosition.y" << Fy << endl;
				//cout << "HmdPosition.z" << Fz << endl;

				//cout << "The first line of notification" << lineStr << endl;
				// calculate the position in video
				//double Fx;
				//double Fy;
				double DQ[4], FLocation[2];
				DQ[0] = stringToDouble(Qx);
				DQ[1] = stringToDouble(Qy);
				DQ[2] = stringToDouble(Qz);
				DQ[3] = stringToDouble(Qw);
				//cout << "unit quaternion x in number" << DQ[0] << endl;
				//cout << "unit quaternion y in number" << DQ[1] << endl;
				//cout << "unit quaternion z in number" << DQ[2] << endl;
				//cout << "unit quaternion w in number" << DQ[3] << endl;

				cal_forward_ProjectToFrame(DQ, FLocation);
				//cout << "Position y in frame" << FLocation[0] << endl;
				//cout << "Position x in frame" << FLocation[1] << endl;
				double DoubleFx;
				double DoubleFy;
				DoubleFx = FLocation[1] / 360;
				DoubleFy = (90 - FLocation[0]) / 180;
				//cout << "Position x in frame" << DoubleFx << endl;
				//cout << "Position y in frame" << DoubleFy << endl;
				//if (DoubleFx < XminCSV) XminCSV = DoubleFx;
				//if (DoubleFx > XmaxCSV) XmaxCSV = DoubleFx;

			}
			cout << "pre" << prePlayBack << endl;
			cout << "cur" << curPlayBack << endl;
			//getchar();

			*/

			/*================================================
			this part we will test the data from the public dataset
			================================================*/
			bool test = true;			//  true  or  false
			int FrameHeightT = frame_size.height;
			int FrameWidthT = frame_size.width;
			static IplImage *frameT = NULL;
			Mat frametT;
			int countTst = 1;
			int XinFrame;
			int YinFrame;
			int location[100][2];
			int indexLocation = 0;
			int indexLocationForVideo;
			int TotalInfo = 0;
			double VideoUserInter;
			bool flagOverOneMin = false;
			countTst = RATEFRAME + 1;
			int modifiedX;
			int modifiedXNo;
			int dropTwo = 0;
			int countTime = 0;
			int flagStop = 1;
			while (test)
			{
				// frame rate is 30, get 30 frame from the user data set
				if (countTst > RATEFRAME)
				{
					cout << "current time  " << countTime << endl;
					countTime++;

					/* ===== this part is the code block to add all user info in 1s====
					double TimeStempSedCur = 1;				this is current time threshold
					double TimeStempSedPre = 1;				this is the previous time threshold
					==================================================================*/
					indexLocation = 0;
					TotalInfo = 0;
					indexLocationForVideo = 0;
					while (TimeStempSedPre > TimeStempSedCur)
					{
						if (!getline(inFile, lineStr))
						{
							flagStop = 0;
							break;
						}
						//cout << "The  line of info" << lineStr << endl;
						stringstream ssa(lineStr);

						// 按照逗号分隔
						getline(ssa, Qx, ',');
						// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second 
						stringstream ssd(Qx);
						getline(ssd, Qx, ':');   //hour "yyyy-mm-dd hh"
						getline(ssd, Qx, ':');	//"mm"
						getline(ssd, Qx, ':');    //ss.ms
						TimeStempSedCur = stringToDouble(Qx);
						TimeStempSedCur = floor(TimeStempSedCur);
						if (TimeStempSedPre == 60 && TimeStempSedCur == 0) break;
						// play back time data "0.000s"
						getline(ssa, Qx, ',');		//delete the first two data which is the playback and time stemp
						PlayBacks = stringToDouble(Qx);

						getline(ssa, Qx, ',');
						getline(ssa, Qy, ',');
						getline(ssa, Qz, ',');
						getline(ssa, Qw, ',');
						getline(ssa, Fx, ',');
						getline(ssa, Fy, ',');
						getline(ssa, Fz, ',');

						//cout << "The first line of notification" << lineStr << endl;
						// calculate the position in video
						//double Fx;
						//double Fy;
						DQ[0] = stringToDouble(Qx);
						DQ[1] = stringToDouble(Qy);
						DQ[2] = stringToDouble(Qz);
						DQ[3] = stringToDouble(Qw);

						cal_forward_ProjectToFrame(DQ, FLocation);

						//DoubleFx = FLocation[1] / 360;
						//DoubleFy = (90 - FLocation[0]) / 180;

						DoubleFx = FLocation[1] / 360;
						DoubleFy = (90 - FLocation[0]) / 180;
						XinFrame = (int)round(DoubleFx*FrameWidthT);
						YinFrame = (int)round(DoubleFy*FrameHeightT);
						location[indexLocation][0] = XinFrame;
						location[indexLocation][1] = YinFrame;
						indexLocation++;
						TotalInfo++;
						if (indexLocation > 100)
						{
							cout << "This is where we have over 100" << endl;
							getchar();
						}
					}
					TimeStempSedPre = TimeStempSedCur + 1;

					if (flagStop == 0) break;
					countTst = 1;
					VideoUserInter = TotalInfo / RATEFRAME;
				}

				if (dropTwo < 0)        //2
				{
					countTst = RATEFRAME + 1;
					dropTwo++;
					continue;
				}

				
				modifiedX = location[indexLocationForVideo][0] - ((int)FrameWidthT / 4);
				if (modifiedX < 0)
				{
					modifiedX = modifiedX + FrameWidthT;
				}
				modifiedXNo = location[indexLocationForVideo][0];
				
				//============================write to a file=========================
				stringstream NameA;
				NameA << modifiedXNo;
				//NameC.str()
				stringstream NameB;
				NameB << location[indexLocationForVideo][1];
				//NameC.str()
				outFile << NameA.str() << ',' << NameB.str() << ',';

				//===========================show the result in video=================
				
				/*
				//  cap >> framet;  
				cap.read(frametT);
				//
				int r = 20;    //
				Point center = Point(modifiedXNo, location[indexLocationForVideo][1]);
				// this part test the format of the video
				Point centerT = Point(100, 100);
				//circle(frametT, centerT, r, Scalar(255, 0, 0), -1);//-1
				circle(frametT, center, r, Scalar(123, 21, 32), -1);//-1
																	
				frameT = &IplImage(frametT);
				cvNamedWindow("Test user video ", CV_WINDOW_NORMAL);
				cvShowImage("Test user video ", frameT);    //  Usr_View   NewVideoF
				cvWaitKey(33);
				*/

				indexLocationForVideo = round(VideoUserInter*countTst);
				if (indexLocationForVideo > indexLocation)indexLocationForVideo = indexLocation;
				countTst++;
			}
			outFile << endl;
			outFile.close();
			cap.release();
		}

		return 1;
	}
	
	//==========generate the name=============================================================================

	int VD = Video_Divide;
	int VBL = Video_Buffer_Len / 25;
	int SS = Shrink_Speed;

	stringstream NameA;
	stringstream NameB;
	stringstream NameC;
	NameA << VD;
	NameB << VBL;
	NameC << SS;
	
	int totalUserNo = 48;			//48
	int UserI;
	string VideoWaME = "video_4_";
	string VideoNameNew = "2-5-FightingB.mp4";
	for (UserI = 1; UserI <= totalUserNo; UserI++)
	{
		stringstream NameO;
		NameO << UserI;
		//NameC.str()
		string NameList = VideoWaME + NameO.str() + ".csv";
		string NameListO = VideoWaME + "Result_" + NameO.str() + "_"+NameB.str()+ "s.csv";
		//ifstream inFile(NameList, ios::in);
		// =================write data to CSV file =============================
		ofstream outFileB;
		outFileB.open(NameListO, ios::out); // 
		outFileB << "basic pridiction" << ',' << "velocitybased" << ',' << "enhanced A" << ',' << "enhanced B" << ',' << "tiles number" << ',' << "Total tile number" << endl;
		/*outFile << "name" << ',' << "age" << ',' << "hobby" << endl;
		outFile << "Mike" << ',' << 18 << ',' << "paiting" << endl;
		outFile << "Tom" << ',' << 25 << ',' << "football" << endl;
		outFile << "Jack" << ',' << 21 << ',' << "music" << endl;
		outFile.close();*/
		//======================read user view from csv file=====================
		//ifstream: read files from the disc and store in Memory
		ifstream inFile(NameList, ios::in);
		string lineStr;
		vector<vector<string>> strArray;
		int LineNo = 0;
		double TimeStempSedCur = 1;
		double TimeStempSedPre = 1;
		//getline check the EOF
		/*==========================The original code block to read CSV file==============================*/
		/*while (getline(inFile, lineStr))
		{
		// 
		cout << lineStr << endl;
		// 
		stringstream ss(lineStr);
		string str;
		vector<string> lineArray;
		// 
		while (getline(ss, str, ','))
		{
		lineArray.push_back(str);
		cout << str << "   <<<<====test what happened in inner while"<<endl;
		}

		strArray.push_back(lineArray);
		getchar();

		}*/

		/*==========================The modified code block to read CSV file in to parts we want==============================*/
		//=read the first line(row in excel)   In CSV file the first line (row) is the title/legen
		getline(inFile, lineStr);
		cout << "The first line of notification" << lineStr << endl;
		//=read the second line which contains the real data we need 
		getline(inFile, lineStr);

		stringstream ss(lineStr);
		string Qx;
		string Qy;
		string Qz;
		string Qw;
		string Fx;
		string Fy;
		string Fz;
		vector<string> lineArray;
		// 
		getline(ss, Qx, ',');
		// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second 
		stringstream ssb(Qx);
		getline(ssb, Qx, ':');
		cout << "1   time stamp   hour   " << Qx << endl;
		getline(ssb, Qx, ':');
		cout << "2  time stamp   minutes   " << Qx << endl;
		getline(ssb, Qx, ':');
		cout << "3   time stamp    second   " << Qx << endl;
		TimeStempSedCur = stringToDouble(Qx);
		TimeStempSedCur = floor(TimeStempSedCur);
		TimeStempSedPre = TimeStempSedCur + 1;		//TimeStempSedCur represent the upper threshold 
													// play back time data "0.000s"
		getline(ss, Qx, ',');//delete the first two data which is the playback and time stemp
		cout << "4   play back time    second   " << Qx << endl;
		double PlayBacks;
		PlayBacks = stringToDouble(Qx);
		cout << "4   play back time    second (in double)  " << PlayBacks << endl;
		getline(ss, Qx, ',');
		getline(ss, Qy, ',');
		getline(ss, Qz, ',');
		getline(ss, Qw, ',');
		getline(ss, Fx, ',');
		getline(ss, Fy, ',');
		getline(ss, Fz, ',');
		cout << "unit quaternion x" << Qx << endl;
		cout << "unit quaternion y" << Qy << endl;
		cout << "unit quaternion z" << Qz << endl;
		cout << "unit quaternion w" << Qw << endl;
		cout << "HmdPosition.x" << Fx << endl;
		cout << "HmdPosition.y" << Fy << endl;
		cout << "HmdPosition.z" << Fz << endl;

		//cout << "The first line of notification" << lineStr << endl;
		// calculate the position in video
		//double Fx;
		//double Fy;
		double DQ[4], FLocation[2];
		DQ[0] = stringToDouble(Qx);
		DQ[1] = stringToDouble(Qy);
		DQ[2] = stringToDouble(Qz);
		DQ[3] = stringToDouble(Qw);
		cout << "unit quaternion x in number" << DQ[0] << endl;
		cout << "unit quaternion y in number" << DQ[1] << endl;
		cout << "unit quaternion z in number" << DQ[2] << endl;
		cout << "unit quaternion w in number" << DQ[3] << endl;

		cal_forward_ProjectToFrame(DQ, FLocation);
		cout << "Position y in frame" << FLocation[0] << endl;
		cout << "Position x in frame" << FLocation[1] << endl;
		double DoubleFx;
		double DoubleFy;
		DoubleFx = FLocation[1] / 360;
		DoubleFy = (90 - FLocation[0]) / 180;
		cout << "Position x in frame" << DoubleFx << endl;
		cout << "Position y in frame" << DoubleFy << endl;

		//======================test===================================
		//double aatest = acos(-1);
		//cout << " acos(-1)==" << aatest/pi*180 << endl;

		//getchar();

		//I want to test the data set here=============================================avoid the noise of 
		/*
		double XmaxCSV=0;
		double XminCSV=1;
		double prePlayBack=0;
		double curPlayBack;
		while (getline(inFile, lineStr))
		{
		stringstream ss(lineStr);
		string Qx;
		string Qy;
		string Qz;
		string Qw;
		string Fx;
		string Fy;
		string Fz;
		vector<string> lineArray;
		// 
		getline(ss, Qx, ',');
		// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second
		stringstream ssb(Qx);
		getline(ssb, Qx, ':');
		//cout << "1   time stamp   hour   " << Qx << endl;
		getline(ssb, Qx, ':');
		//cout << "2  time stamp   minutes   " << Qx << endl;
		getline(ssb, Qx, ':');
		//cout << "3   time stamp    second   " << Qx << endl;
		TimeStempSedCur = stringToDouble(Qx);
		TimeStempSedCur = floor(TimeStempSedCur);
		TimeStempSedPre = TimeStempSedCur + 1;		//TimeStempSedCur represent the upper threshold
		// play back time data "0.000s"
		getline(ss, Qx, ',');//delete the first two data which is the playback and time stemp
		curPlayBack = stringToDouble(Qx);
		if (curPlayBack < prePlayBack)
		{
		//prePlayBack = curPlayBack;
		break;
		}
		else
		{
		prePlayBack = curPlayBack;
		}
		//cout << "4   play back time    second   " << Qx << endl;
		double PlayBacks;
		PlayBacks = stringToDouble(Qx);
		//cout << "4   play back time    second (in double)  " << PlayBacks << endl;
		getline(ss, Qx, ',');
		getline(ss, Qy, ',');
		getline(ss, Qz, ',');
		getline(ss, Qw, ',');
		getline(ss, Fx, ',');
		getline(ss, Fy, ',');
		getline(ss, Fz, ',');
		//cout << "unit quaternion x" << Qx << endl;
		//cout << "unit quaternion y" << Qy << endl;
		//cout << "unit quaternion z" << Qz << endl;
		//cout << "unit quaternion w" << Qw << endl;
		//cout << "HmdPosition.x" << Fx << endl;
		//cout << "HmdPosition.y" << Fy << endl;
		//cout << "HmdPosition.z" << Fz << endl;

		//cout << "The first line of notification" << lineStr << endl;
		// calculate the position in video
		//double Fx;
		//double Fy;
		double DQ[4], FLocation[2];
		DQ[0] = stringToDouble(Qx);
		DQ[1] = stringToDouble(Qy);
		DQ[2] = stringToDouble(Qz);
		DQ[3] = stringToDouble(Qw);
		//cout << "unit quaternion x in number" << DQ[0] << endl;
		//cout << "unit quaternion y in number" << DQ[1] << endl;
		//cout << "unit quaternion z in number" << DQ[2] << endl;
		//cout << "unit quaternion w in number" << DQ[3] << endl;

		cal_forward_ProjectToFrame(DQ, FLocation);
		//cout << "Position y in frame" << FLocation[0] << endl;
		//cout << "Position x in frame" << FLocation[1] << endl;
		double DoubleFx;
		double DoubleFy;
		DoubleFx = FLocation[1] / 360;
		DoubleFy = (90 - FLocation[0]) / 180;
		//cout << "Position x in frame" << DoubleFx << endl;
		//cout << "Position y in frame" << DoubleFy << endl;
		//if (DoubleFx < XminCSV) XminCSV = DoubleFx;
		//if (DoubleFx > XmaxCSV) XmaxCSV = DoubleFx;

		}
		cout << "pre" << prePlayBack << endl;
		cout << "cur" << curPlayBack << endl;
		//getchar();
		*/
		//============================test end=====================================

		//========================add timer=====================================
		long B_Start, B_end; //test the running time for basic method
		long V_Start, V_end; //test the running time for velocity based method
		long EA_Start, EA_end; //test the running time for Enhanced A method
		long EB_Start, EB_end; //test the running time for Enhanced B method
		long IMP_Start, IMP_end; //test the running time for Enhanced B method
								 //=========================other variable====================================
		int windw = 350; int windh = 350;    // the window size  350 350
		int windx = 250; int windy = 120;	// the location
		int xl; int xh; int yh; int yl;     // area for the usr veiw
		int x_o = 0; int y_o = 0; // the central location for feature clouds in previous frame
		int x_n; int y_n; // the central location for feature clouds in curr frame
		int x_m; int y_m; // the movement of the central location for feature clouds in two frames
		int feature_cloud_count = 0;   // the number of  feature clouds in one frames
		int framenumber;          //frame number sequence
		uchar PixelValue;
		int Video_w; int Video_h;  // record the video length and width
		int Block_NO = Video_Divide*Video_Divide;  //video will be cutted into many little part, indicate the size and the totall number
		int Block_List[Video_Divide*Video_Divide];			// list all the blocks and indicate the block if it belong to the interested view, 1->interested  0-> not interested
		int Display_count = 1;
		int i_count;
		//CLOCKS_PER_SEC;
		for (i_count = 0; i_count < Video_Divide*Video_Divide; i_count++)
		{
			Block_List[i_count] = 1;
			BlockStatus[i_count] = 2;
		}
		//=========================other variable for compare and analyze====================================
		int Usr_V_XH;      //usr view X-axis Highest 
		int Usr_V_YH;      //usr view Y-axis Highest 
		int Usr_V_XL;      //usr view X-axis Lowest 
		int Usr_V_YL;      //usr view Y-axis Lowest 
		int Usr_V_Count;    // counts for user frames
		int Usr_V_Gap_count; // eliminate the 24FPS and 25FPS

							 //=============velocity based prediction(VBP)============================
		int VBP_XH;     //velocity based prediction(VBP) location x highest
		int VBP_XL;     //velocity based prediction(VBP) location x highest
		int VBP_YH;     //velocity based prediction(VBP) location y highest
		int VBP_YL;     //velocity based prediction(VBP) location y highest
		int VBP_VX;		////velocity based prediction(VBP) frame speed in x
		int VBP_VY;		////velocity based prediction(VBP) frame speed in y
		int VBP_VX_Start;		////velocity based prediction(VBP) frame speed in x 
		int VBP_VY_Start;		////velocity based prediction(VBP) frame speed in y 
		int VBP_VX_End;		////velocity based prediction(VBP) frame speed in x     
		int VBP_VY_End;		////velocity based prediction(VBP) frame speed in y     

							/*=============basic method + first enhancement ===========================
							this method first use feature prediction and also add the user data feed back
							==========================================================================*/
		int ENHAN_A_XH;		// basic method + first enhancement location x highest
		int ENHAN_A_XL;		// basic method + first enhancement location x lowest
		int ENHAN_A_YH;		// basic method + first enhancement location Y highest
		int ENHAN_A_YL;		// basic method + first enhancement location Y lowest
		int ENHAN_A_collection_X;		// basic method + first enhancement location X
		int ENHAN_A_collection_Y;		// basic method + first enhancement location Y
		int ENHAN_A_collection_Count;		// basic method + first enhancement location points count

		string NameStr = "Cat";
		//==================code for GMM block============================================
		Ptr<BackgroundSubtractorMOG2> bgsubtractor = createBackgroundSubtractorMOG2();
		bgsubtractor->setVarThreshold(20);
		Mat GMM_mask;
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		/*=============================================================================
		Video list:
		
		
		=============================================================================*/
		//2-5-Fighting
		VideoCapture cap(VideoNameNew);     // the orignal is   ElephantsOldB  ||  ElephantsC || Wild_Horses ||5cutecat || cow  ||Female Basketball Match ||2-4-FemaleBasketball



		if (!cap.isOpened())
		{
			return -1;
		}
		Mat frame;
		/*===================read usr view and user view matching parameters&varable===================================*/
		VideoCapture capb("catrecord.mp4");            //cowrecord
		if (!capb.isOpened())
		{
			return -1;
		}
		Mat UsrFrame_RGB;
		Mat UsrFrame_GRY;
		Mat BgFrame_GRY;
		int match_method = CV_TM_CCORR_NORMED;
		cv::Mat result_mat;
		cv::Mat debug_img;
		double minVal; double maxVal;
		cv::Point minLoc, maxLoc, matchLoc;
		Mat UsrFrameSmall_GRY;
		double Usr_Scale = 0.5;

		capb.read(UsrFrame_RGB);
		cvtColor(UsrFrame_RGB, UsrFrame_GRY, CV_RGB2GRAY);

		/*========================================================================================================*/
		

		/*===================file operation===================================*/
		string Filename_TXT = "result_";
		Filename_TXT = VideoWaME + NameO.str() + Filename_TXT + NameB.str() + "S_" + NameA.str() + "T_" + NameC.str() + "V" + ".txt";
		//ofstream outfile("result.txt", ofstream::app);
		ofstream outfile(Filename_TXT, ofstream::app);
		string ErrResult;
		ErrResult.clear();

		//==============record the overhead====================================
		Filename_TXT = "result_overhead_";
		Filename_TXT = VideoWaME + NameO.str() + Filename_TXT + NameB.str() + "S_" + NameA.str() + "T_" + NameC.str() + "V" + ".txt";
		//ofstream outfile("result.txt", ofstream::app);
		ofstream overheadoutfile(Filename_TXT, ofstream::app);
		string overheadresult;
		overheadresult.clear();

		CvSize frame_size;
		frame_size.height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
		frame_size.width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
		double FPSFromeVideo = cap.get(CV_CAP_PROP_FPS);
		cout << "the frame rate read from video is :   " << FPSFromeVideo << endl;
		windw = 250; windh = 200;
		windx = int((frame_size.width) / 2 - windw / 2); windy = int((frame_size.height) / 2 - windh / 2);
		xl = windx; xh = windx + windw; yh = windy + windh; yl = windy;
		ENHAN_A_XH = xh; ENHAN_A_XL = xl; ENHAN_A_YH = yh; ENHAN_A_YL = yl;
		x_o = (frame_size.width) / 2; y_o = (frame_size.height) / 2;
		//	InitialBlockStructure(frame_size.height, frame_size.width);

		string Filename_AVI = "_prediction_";
		Filename_AVI = VideoWaME + NameO.str() + Filename_AVI + NameB.str() + "S_" + NameA.str() + "T_" + NameC.str() + "V" + ".avi";
		//	CvVideoWriter *writer = cvCreateVideoWriter("ElephantsOldB_prediction.avi", CV_FOURCC('D', 'I', 'V', 'X'), 25, cvSize(frame_size.width, frame_size.height));
		const char *Filename_AVI_C = Filename_AVI.c_str();
		CvVideoWriter *writer = cvCreateVideoWriter(Filename_AVI_C, CV_FOURCC('D', 'I', 'V', 'X'), 25, cvSize(frame_size.width, frame_size.height));
		framenumber = 0;
		Usr_V_Count = 0;
		Usr_V_Gap_count = 0;
		int Flag_Skip = 0;


		/*================================================
		this part we will test the data from the public dataset
		================================================*/
		bool test = false;			//  true  or  false
		int FrameHeightT = frame_size.height;
		int FrameWidthT = frame_size.width;
		static IplImage *frameT = NULL;
		Mat frametT;
		int countTst = 1;
		int XinFrame;
		int YinFrame;
		int location[100][2];
		int indexLocation = 0;
		int indexLocationForVideo;
		int TotalInfo = 0;
		double VideoUserInter;
		bool flagOverOneMin = false;
		countTst = RATEFRAME + 1;
		int modifiedX;
		int modifiedXNo;
		int dropTwo = 0;
		int countTime = 0;
		while (test)
		{
			// frame rate is 30, get 30 frame from the user data set
			if (countTst > RATEFRAME)
			{
				cout << "current time  " << countTime << endl;
				countTime++;

				/* ===== this part is the code block to add all user info in 1s====
				double TimeStempSedCur = 1;				this is current time threshold
				double TimeStempSedPre = 1;				this is the previous time threshold
				==================================================================*/
				indexLocation = 0;
				TotalInfo = 0;
				indexLocationForVideo = 0;
				while (TimeStempSedPre > TimeStempSedCur)
				{
					getline(inFile, lineStr);
					//cout << "The  line of info" << lineStr << endl;
					stringstream ssa(lineStr);

					// 按照逗号分隔
					getline(ssa, Qx, ',');
					// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second 
					stringstream ssd(Qx);
					getline(ssd, Qx, ':');   //hour "yyyy-mm-dd hh"
					getline(ssd, Qx, ':');	//"mm"
					getline(ssd, Qx, ':');    //ss.ms
					TimeStempSedCur = stringToDouble(Qx);
					TimeStempSedCur = floor(TimeStempSedCur);
					if (TimeStempSedPre == 60 && TimeStempSedCur == 0) break;
					// play back time data "0.000s"
					getline(ssa, Qx, ',');		//delete the first two data which is the playback and time stemp
					PlayBacks = stringToDouble(Qx);

					getline(ssa, Qx, ',');
					getline(ssa, Qy, ',');
					getline(ssa, Qz, ',');
					getline(ssa, Qw, ',');
					getline(ssa, Fx, ',');
					getline(ssa, Fy, ',');
					getline(ssa, Fz, ',');

					//cout << "The first line of notification" << lineStr << endl;
					// calculate the position in video
					//double Fx;
					//double Fy;
					DQ[0] = stringToDouble(Qx);
					DQ[1] = stringToDouble(Qy);
					DQ[2] = stringToDouble(Qz);
					DQ[3] = stringToDouble(Qw);

					cal_forward_ProjectToFrame(DQ, FLocation);

					//DoubleFx = FLocation[1] / 360;
					//DoubleFy = (90 - FLocation[0]) / 180;

					DoubleFx = FLocation[1] / 360;
					DoubleFy = (90 - FLocation[0]) / 180;
					XinFrame = (int)round(DoubleFx*FrameWidthT);
					YinFrame = (int)round(DoubleFy*FrameHeightT);
					location[indexLocation][0] = XinFrame;
					location[indexLocation][1] = YinFrame;
					indexLocation++;
					TotalInfo++;
					if (indexLocation > 100)
					{
						cout << "This is where we have over 100" << endl;
						getchar();
					}
				}
				TimeStempSedPre = TimeStempSedCur + 1;


				countTst = 1;
				VideoUserInter = TotalInfo / RATEFRAME;
			}

			if (dropTwo < 0)        //2
			{
				countTst = RATEFRAME + 1;
				dropTwo++;
				continue;
			}

			//  cap >> framet;  
			cap.read(frametT);
			//
			int r = 20;    //
			modifiedX = location[indexLocationForVideo][0] - ((int)FrameWidthT / 4);
			if (modifiedX < 0)
			{
				modifiedX = modifiedX + FrameWidthT;
			}
			modifiedXNo = location[indexLocationForVideo][0];
			Point center = Point(modifiedXNo, location[indexLocationForVideo][1]);
			// this part test the format of the video
			Point centerT = Point(100, 100);
			//circle(frametT, centerT, r, Scalar(255, 0, 0), -1);//-1
			circle(frametT, center, r, Scalar(123, 21, 32), -1);//-1
																//
			frameT = &IplImage(frametT);
			cvNamedWindow("Test user video ", CV_WINDOW_NORMAL);
			indexLocationForVideo = round(VideoUserInter*countTst);
			if (indexLocationForVideo > indexLocation)indexLocationForVideo = indexLocation;
			cvShowImage("Test user video ", frameT);    //  Usr_View   NewVideoF
			countTst++;

			cvWaitKey(33);
		}



		while (true)
		{
			/*===========================================================
			This code block get the user data in one second and stored in an arry
			after this, we process the video and get the user data based on the frams and total user info
			The variables are :
			countTst: count the frame. if countTst > frame rate (frames in one second), countTst set to 1 and process the usuer info for one second
			TimeStempSedPre: the upper threshold for the time stamp which is timestamp+1 in last round.

			=============================================================*/

			// frame rate is 30, get 30 frame from the user data set
			if (countTst > RATEFRAME)
			{
				/* ===== this part is the code block to add all user info in 1s====
				double TimeStempSedCur = 1;				this is current time threshold
				double TimeStempSedPre = 1;				this is the previous time threshold
				==================================================================*/
				indexLocation = 0;
				TotalInfo = 0;
				indexLocationForVideo = 0;
				while (TimeStempSedPre > TimeStempSedCur)
				{
					getline(inFile, lineStr);
					//cout << "The  line of info" << lineStr << endl;
					stringstream ssa(lineStr);

					// 
					getline(ssa, Qx, ',');
					// time data "yyyy-mm-dd hh:mm:ss.ms" save to Qx then to ssb to get second 
					stringstream ssd(Qx);
					getline(ssd, Qx, ':');   //hour "yyyy-mm-dd hh"
					getline(ssd, Qx, ':');	//"mm"
					getline(ssd, Qx, ':');    //ss.ms
					TimeStempSedCur = stringToDouble(Qx);
					TimeStempSedCur = floor(TimeStempSedCur);
					if (TimeStempSedPre == 60 && TimeStempSedCur == 0) break;
					// play back time data "0.000s"
					getline(ssa, Qx, ',');		//delete the first two data which is the playback and time stemp
					PlayBacks = stringToDouble(Qx);

					getline(ssa, Qx, ',');
					getline(ssa, Qy, ',');
					getline(ssa, Qz, ',');
					getline(ssa, Qw, ',');
					getline(ssa, Fx, ',');
					getline(ssa, Fy, ',');
					getline(ssa, Fz, ',');

					//cout << "The first line of notification" << lineStr << endl;
					// calculate the position in video
					//double Fx;
					//double Fy;
					DQ[0] = stringToDouble(Qx);
					DQ[1] = stringToDouble(Qy);
					DQ[2] = stringToDouble(Qz);
					DQ[3] = stringToDouble(Qw);

					cal_forward_ProjectToFrame(DQ, FLocation);

					//DoubleFx = FLocation[1] / 360;
					//DoubleFy = (90 - FLocation[0]) / 180;

					DoubleFx = FLocation[1] / 360;
					DoubleFy = (90 - FLocation[0]) / 180;
					XinFrame = (int)round(DoubleFx*FrameWidthT);
					YinFrame = (int)round(DoubleFy*FrameHeightT);
					location[indexLocation][0] = XinFrame;
					location[indexLocation][1] = YinFrame;
					indexLocation++;
					TotalInfo++;
					if (indexLocation > 100)
					{
						cout << "This is where we have over 100" << endl;
						getchar();
					}
				}
				TimeStempSedPre = TimeStempSedCur + 1;


				countTst = 1;
				VideoUserInter = TotalInfo / RATEFRAME;
			}


			if (Flag_Skip == 0)
			{
				Flag_Skip = 1;
			}
			else
			{
				Flag_Skip = 0;
			}
			IMP_Start = clock();
			overheadresult.clear();
			framenumber++;
			cout << "count" << framenumber << endl;
			static IplImage *frame = NULL, *frame1 = NULL, *frame1_1C = NULL, *NewVideoF = NULL,
				*frame2_1C = NULL, *eig_image = NULL, *temp_image = NULL, *NewBlack = NULL, *VideoM = NULL,
				*pyramid1 = NULL, *pyramid2 = NULL, *IP_GMM_mask = NULL, *Usr_View = NULL;

			Mat framet;

			//  cap >> framet;  
			cap.read(framet);
			countTst++;
			Mat edges;

			// cvtColor(framet, edges, CV_RGB2GRAY);  
			//  GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);  
			//  Canny(edges, edges, 0, 30, 3);  

			bgsubtractor->apply(framet, GMM_mask, 0.001);
			erode(GMM_mask, GMM_mask, cv::Mat());
			dilate(GMM_mask, GMM_mask, cv::Mat());
			dilate(GMM_mask, GMM_mask, cv::Mat());


			frame = &IplImage(framet);
			if (frame == NULL)
			{
				fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
				return -1;
			}


			allocateOnDemand(&frame1_1C, frame_size, IPL_DEPTH_8U, 1);


			cvConvertImage(frame, frame1_1C, 0);


			allocateOnDemand(&VideoM, frame_size, IPL_DEPTH_8U, 3);
			allocateOnDemand(&NewVideoF, frame_size, IPL_DEPTH_8U, 3);
			allocateOnDemand(&NewBlack, frame_size, IPL_DEPTH_8U, 3);
			allocateOnDemand(&frame1, frame_size, IPL_DEPTH_8U, 3);
			cvConvertImage(frame, frame1, 0);

			/*=================read one frame from usr view===============================*/
			//capb.read(UsrFrame_RGB);
			//cvtColor(UsrFrame_RGB, UsrFrame_GRY, CV_RGB2GRAY);
			Usr_V_Count++;
			Usr_V_Gap_count++;

			//cap >> framet;  
			cap.read(framet);
			countTst++;
			//  cvtColor(framet, edges, CV_RGB2GRAY);  
			//  GaussianBlur(edges, edges, Size(7, 7), 1.5, 1.5);  
			//  Canny(edges, edges, 0, 30, 3);  
			frame = &IplImage(framet);
			if (frame == NULL)
			{
				fprintf(stderr, "Error: Hmm. The end came sooner than we thought.\n");
				return -1;
			}
			bgsubtractor->apply(framet, GMM_mask, 0.001);
			erode(GMM_mask, GMM_mask, cv::Mat());
			dilate(GMM_mask, GMM_mask, cv::Mat());
			dilate(GMM_mask, GMM_mask, cv::Mat());
			IP_GMM_mask = &IplImage(GMM_mask);
			//		findContours(GMM_mask, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);



			allocateOnDemand(&frame2_1C, frame_size, IPL_DEPTH_8U, 1);
			cvConvertImage(frame, frame2_1C, 0);




			allocateOnDemand(&eig_image, frame_size, IPL_DEPTH_32F, 1);
			allocateOnDemand(&temp_image, frame_size, IPL_DEPTH_32F, 1);


			CvPoint2D32f frame1_features[400];
			int    number_of_features = 400;


			cvGoodFeaturesToTrack(frame1_1C, eig_image, temp_image,
				frame1_features, &number_of_features, .01, .01, NULL);




			CvPoint2D32f frame2_features[400];


			char optical_flow_found_feature[400];


			float optical_flow_feature_error[400];



			CvSize optical_flow_window = cvSize(5, 5);
			// CvSize optical_flow_window = cvSize(5, 5);  

			CvTermCriteria optical_flow_termination_criteria = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);


			allocateOnDemand(&pyramid1, frame_size, IPL_DEPTH_8U, 1);
			allocateOnDemand(&pyramid2, frame_size, IPL_DEPTH_8U, 1);


			cvCalcOpticalFlowPyrLK(frame1_1C, frame2_1C, pyramid1, pyramid2, frame1_features, frame2_features, number_of_features,
				optical_flow_window, 5, optical_flow_found_feature, optical_flow_feature_error, optical_flow_termination_criteria, 0);


			feature_cloud_count = 0; x_n = 0; y_n = 0;   //initial the parameters for location calculation
			ENHAN_A_collection_X = 0; ENHAN_A_collection_Y = 0; ENHAN_A_collection_Count = 0;

			IMP_end = clock();

			B_Start = clock();
			EA_Start = B_Start;

			for (int i = 0; i< number_of_features; i++)
			{
				//No feature detected
				if (optical_flow_found_feature[i] == 0)
					continue;
				int line_thickness;
				line_thickness = 1;


				CvScalar line_color;
				line_color = CV_RGB(255, 0, 0);

				CvPoint p, q;
				p.x = (int)frame1_features[i].x;
				p.y = (int)frame1_features[i].y;
				q.x = (int)frame2_features[i].x;
				q.y = (int)frame2_features[i].y;
				// check if the features are in the interested area
				//			if ((xl>p.x) || (p.x>xh) || (p.y>yh) || (p.y<yl))
				//				continue;
				GMM_mask;
				PixelValue = CheckPixelValue(&GMM_mask, p.y, p.x);
				if (PixelValue < 10)
					continue;
				feature_cloud_count++;
				//============update the blocklist===================================
				if (Flag_Skip)
				{
					UpdateSmallBlocksSet(Block_List, frame1, p.x, p.y);
				}

				//=============check if the features are in the EHAN_A area
				if ((ENHAN_A_XH>p.x) && (p.x>ENHAN_A_XL) && (p.y>ENHAN_A_YL) && (p.y<ENHAN_A_YH))
				{
					ENHAN_A_collection_X = ENHAN_A_collection_X + p.x;
					ENHAN_A_collection_Y = ENHAN_A_collection_Y + p.y;
					ENHAN_A_collection_Count++;
				}

				x_n = x_n + p.x;
				y_n = y_n + p.y;
				double angle;
				angle = atan2((double)p.y - q.y, (double)p.x - q.x);
				double hypotenuse;
				hypotenuse = sqrt(square(p.y - q.y) + square(p.x - q.x));


				q.x = (int)(p.x - 5 * hypotenuse * cos(angle));
				q.y = (int)(p.y - 5 * hypotenuse * sin(angle));

				// draw the arrow
				//			cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);


				p.x = (int)(q.x + 9 * cos(angle + pi / 4));
				p.y = (int)(q.y + 9 * sin(angle + pi / 4));
				//			cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);
				p.x = (int)(q.x + 9 * cos(angle - pi / 4));
				p.y = (int)(q.y + 9 * sin(angle - pi / 4));
				//			cvLine(frame1, p, q, line_color, line_thickness, CV_AA, 0);


			}

			//#if 1
#if ShowFeature
			//===show the result of prediction============================
			cvRectangle(frame1, cvPoint(windx + windw, windy + windh), cvPoint(windx, windy), Scalar(0, 255, 255), 2, 8, 0);
#endif 
			if (Flag_Skip)
			{
				if (feature_cloud_count != 0) x_n = int(x_n / feature_cloud_count);
				if (feature_cloud_count != 0) y_n = int(y_n / feature_cloud_count);
				x_m = x_n - x_o;
				y_m = y_n - y_o;
				windx = windx + x_m;
				windy = windy + y_m;
				x_o = x_n;  y_o = y_n;
			}


			//=============================
			/*		if (framenumber == 345)
			{
			windx = windx - 80;
			}
			*/
			//===============================
			xl = windx; xh = windx + windw; yh = windy + windh; yl = windy;
			//		cvRectangle(frame1, cvPoint(windx + windw, windy + windh), cvPoint(windx, windy), Scalar(255, 0, 255), 2, 8, 0);
			B_end = clock();
			/*================get the user view from the csv===================================================*/
			//
			int r = 20;    //
			modifiedX = location[indexLocationForVideo][0] - ((int)FrameWidthT / 4);
			if (modifiedX < 0)
			{
				modifiedX = modifiedX + FrameWidthT;
			}
			modifiedXNo = location[indexLocationForVideo][0];
			Point center = Point(modifiedXNo, location[indexLocationForVideo][1]);
			// this part test the format of the video
			//Point centerT = Point(100, 100);
			//circle(frametT, centerT, r, Scalar(255, 0, 0), -1);//-1为填充
			//circle(frametT, center, r, Scalar(123, 21, 32), -1);//-1为填充
			//转换图像至输出允许格式
			//frameT = &IplImage(frametT);
			//cvNamedWindow("Test user video ", CV_WINDOW_NORMAL);
			cvCircle(frame1, cvPoint(modifiedXNo, location[indexLocationForVideo][1]), r, CV_RGB(255, 0, 0), 1);
			indexLocationForVideo = round(VideoUserInter*countTst);
			if (indexLocationForVideo > indexLocation)indexLocationForVideo = indexLocation;
			/*=================read one frame from usr view & show the usr view===============================*/
			//capb.read(UsrFrame_RGB);
			//cvtColor(UsrFrame_RGB, UsrFrame_GRY, CV_RGB2GRAY);
			Usr_V_Count++; Usr_V_Gap_count++;
			UsrFrameSmall_GRY = UsrFrame_GRY(Range(150, 550), Range(100, 500));
			Size dsize = Size(UsrFrameSmall_GRY.cols*Usr_Scale, UsrFrameSmall_GRY.rows*Usr_Scale);
			Mat image2 = Mat(dsize, CV_32S);
			resize(UsrFrameSmall_GRY, image2, dsize);
			Usr_View = &IplImage(UsrFrameSmall_GRY);
			cvtColor(framet, BgFrame_GRY, CV_RGB2GRAY);
			cv::matchTemplate(BgFrame_GRY, image2, result_mat, match_method);
			cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());
			cv::minMaxLoc(result_mat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
			if (match_method == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED)  matchLoc = minLoc;
			else matchLoc = maxLoc;


			//====================here modify matchloc by the data we got from CSV file===========================
			matchLoc.x = modifiedXNo - windw / 2;
			matchLoc.y = location[indexLocationForVideo][1] - windh / 2;

			EA_Start = clock();
			/*========================================================================
			** Update and show the Basic and Enhanced prediction result
			========================================================================*/
			if (Flag_Skip)
			{
				if (ENHAN_A_collection_Count>0)
				{
					ENHAN_A_collection_X = int(ENHAN_A_collection_X / ENHAN_A_collection_Count);
					ENHAN_A_collection_Y = int(ENHAN_A_collection_Y / ENHAN_A_collection_Count);
					/*ENHAN_A_XH = ENHAN_A_collection_X + int(image2.cols / 2);
					ENHAN_A_XL = ENHAN_A_collection_X - int(image2.cols / 2);
					ENHAN_A_YH = ENHAN_A_collection_Y + int(image2.rows / 2);
					ENHAN_A_YL = ENHAN_A_collection_Y - int(image2.rows / 2);*/
					ENHAN_A_XH = ENHAN_A_collection_X + int(windw / 2);
					ENHAN_A_XL = ENHAN_A_collection_X - int(windw / 2);
					ENHAN_A_YH = ENHAN_A_collection_Y + int(windh / 2);
					ENHAN_A_YL = ENHAN_A_collection_Y - int(windh / 2);
				}
				else
				{
					ENHAN_A_XH = ENHAN_A_XH + VBP_VX;
					ENHAN_A_XL = ENHAN_A_XL + VBP_VX;
					ENHAN_A_YH = ENHAN_A_YH + VBP_VY;
					ENHAN_A_YL = ENHAN_A_YL + VBP_VY;

				}
			}

			//cout << ENHAN_A_XH<<"|||"<< ENHAN_A_collection_X <<"|||"<< ENHAN_A_YH<<"|||"<< ENHAN_A_collection_Y <<"|||"<< ENHAN_A_collection_Count << endl;
			/*========================================================================
			**considering user video 24 is FPS and the orignal is 25 FPS, so 24+1->25
			========================================================================*/
			if (Usr_V_Gap_count == 24)
			{
				Usr_V_Gap_count = 0;
				//			capb.read(UsrFrame_RGB);
				cap.read(framet);
				countTst++;
				cap.read(framet);
				countTst++;
				cap.read(framet);
				countTst++;
				cap.read(framet);
				countTst++;
				Usr_V_Count++;
			}
			/*========================================================================
			** velocity based prediction when one buffer lenth
			========================================================================*/
			if (Usr_V_Count >= Video_Buffer_Len)
			{
				VBP_VX_End = matchLoc.x;
				VBP_VY_End = matchLoc.y;
				Usr_V_Count = 0;
				VBP_VX = int(VBP_VX_End - VBP_VX_Start) / Video_Buffer_Len;
				VBP_VY = int(VBP_VY_End - VBP_VY_Start) / Video_Buffer_Len;
				//==============update the velocity based prediction================
				//VBP_XH = matchLoc.x + image2.cols;        
				VBP_XH = matchLoc.x + windw;
				VBP_XL = matchLoc.x;
				VBP_YH = matchLoc.y + windh;
				//VBP_YH = matchLoc.y + image2.rows;
				VBP_YL = matchLoc.y;
				//==============update the Basic + enhancedA prediction================
				//ENHAN_A_XH = matchLoc.x + image2.cols;
				ENHAN_A_XH = matchLoc.x + windw;
				ENHAN_A_XL = matchLoc.x;
				//ENHAN_A_YH = matchLoc.y + image2.rows;
				ENHAN_A_YH = matchLoc.y + windh;
				ENHAN_A_YL = matchLoc.y;
				//=============update the shrink prediction=============================
				MaintainBlocks(Block_List);
			}
			if ((Usr_V_Count >= 1) && (Usr_V_Count <= 4))
			{
				VBP_VX_Start = matchLoc.x;
				VBP_VY_Start = matchLoc.y;
			}
			EA_end = clock();
			/*========================================================================
			** Update and show the velocity based prediction result
			========================================================================*/
			VBP_XH = VBP_XH + VBP_VX;
			VBP_XL = VBP_XL + VBP_VX;
			VBP_YH = VBP_YH + VBP_VY;
			VBP_YL = VBP_YL + VBP_VY;
#if ShowFeature
			// show vilocity based prediction result
			cvRectangle(frame1, cvPoint(VBP_XH, VBP_YH), cvPoint(VBP_XL, VBP_YL), Scalar(0, 255, 0), 2, 8, 0);
#endif
			//cout << "=================" << endl;
			//cout << VBP_XH << "|||" << VBP_YH <<"|||"<< VBP_XL <<"|||"<< VBP_YL <<endl;
			//=======draw the usr view in the video===============================================
			/*cv::rectangle(
			frame1,
			matchLoc,
			cv::Point(matchLoc.x + UsrFrame_GRY.cols, matchLoc.y + UsrFrame_GRY.rows),
			CV_RGB(255, 0, 0),
			3);*/
#if ShowFeature
			//show the usr view
			cvRectangle(frame1, cvPoint(matchLoc.x + image2.cols, matchLoc.y + image2.rows), cvPoint(matchLoc.x, matchLoc.y), Scalar(255, 0, 0), 2, 8, 0);
			//show the basic+enhanceA method
			cvRectangle(frame1, cvPoint(ENHAN_A_XH, ENHAN_A_YH), cvPoint(ENHAN_A_XL, ENHAN_A_YL), Scalar(0, 0, 255), 2, 8, 0);
#endif 
			/*============================================================================*/
			/*========================================================================
			** Update and show the shrinking based prediction result
			========================================================================*/
			EB_Start = clock();
			UpdateBlockByOther(Block_List, frame1,
				VBP_XH, VBP_XL, VBP_YH, VBP_YL,
				ENHAN_A_XH, ENHAN_A_XL, ENHAN_A_YH, ENHAN_A_YL);
			EB_end = clock();
#if ShowFeature
			for (Display_count = 0; Display_count < Block_NO; Display_count++)
			{
				ShowSmallBlocks(Block_List, frame1, Display_count);
			}
#endif
#if MASK
			//if ((framenumber % 2) == 0)
			cvCopy(NewBlack, NewVideoF);
			if (1)
			{
				for (Display_count = 0; Display_count < Block_NO; Display_count++)
				{
					AddMaskBlocks(Block_List, frame1, NewVideoF, Display_count);
				}
			}
			//============pick one pice=======================================
			cvCopy(NewBlack, VideoM);
			CopySubImage(frame1, VideoM, 360, 360, 600, 600);

#endif
			/*================compare the usr view and predict vieaw============================*/
			// get usr data
			Usr_V_XH = matchLoc.x + image2.cols;      //usr view X-axis Highest 
			Usr_V_YH = matchLoc.y + image2.rows;      //usr view Y-axis Highest 
			Usr_V_XL = matchLoc.x;      //usr view X-axis Lowest 
			Usr_V_YL = matchLoc.y;      //usr view Y-axis Lowest
			Usr_V_XH = Usr_V_XH - 30;
			Usr_V_YH = Usr_V_YH - 30;
			Usr_V_XL = Usr_V_XL + 30;
			Usr_V_YL = Usr_V_YL + 30;
			//compare basic method
			if ((int(Usr_V_XL) > xl) && (int(Usr_V_XH) < xh) && (int(Usr_V_YL) > yl) && (int(Usr_V_YH) < yh))
			{
				//cout << "right"  << endl;
				ErrResult = ErrResult + "1" + "  " + "0" + "  ";
				outFileB << "1" << ',';
			}
			else
			{
				//cout << "wrong" << endl;
				int Dist = CalculateDistance(Usr_V_XL, Usr_V_YL, xl, yl);
				stringstream newstr;
				newstr << Dist;
				ErrResult = ErrResult + "0" + "  " + newstr.str() + "  ";
				outFileB << "0" << ',';
			}
			//compare velocity based method
			if ((int(Usr_V_XL) > VBP_XL) && (int(Usr_V_XH) < VBP_XH) && (int(Usr_V_YL) > VBP_YL) && (int(Usr_V_YH) < VBP_YH))
			{
				//cout << "right" << endl;
				ErrResult = ErrResult + "1" + "  " + "0" + "  ";
				outFileB << "1" << ',';
			}
			else
			{
				//cout << "wrong" << endl;
				int Dist = CalculateDistance(Usr_V_XL, Usr_V_YL, VBP_XL, VBP_YL);
				stringstream newstr;
				newstr << Dist;
				ErrResult = ErrResult + "0" + "  " + newstr.str() + "  ";
				outFileB << "0" << ',';
			}
			//compare enhance A
			if ((int(Usr_V_XL) > ENHAN_A_XL) && (int(Usr_V_XH) < ENHAN_A_XH) && (int(Usr_V_YL) > ENHAN_A_YL) && (int(Usr_V_YH) < ENHAN_A_YH))
			{
				//cout << "right" << endl;
				ErrResult = ErrResult + "1" + "  " + "0" + "  ";
				outFileB << "1" << ',';
			}
			else
			{
				//cout << "wrong" << endl;
				int Dist = CalculateDistance(Usr_V_XL, Usr_V_YL, ENHAN_A_XL, ENHAN_A_YL);
				stringstream newstr;
				newstr << Dist;
				ErrResult = ErrResult + "0" + "  " + newstr.str() + "  ";
				outFileB << "0" << ',';
			}

			cout << "Usr_V_XL" << Usr_V_XL << "Usr_V_XH" << Usr_V_XH << "Usr_V_YL" << Usr_V_YL << "Usr_V_YH" << Usr_V_YH << endl;

			if (Usr_V_XL < 0)Usr_V_XL = 0;
			if (Usr_V_XL > FrameWidthT)Usr_V_XL = FrameWidthT;
			if (Usr_V_XH < 0)Usr_V_XH = 0;
			if (Usr_V_XH > FrameWidthT)Usr_V_XH = FrameWidthT;
			if (Usr_V_YL < 0)Usr_V_YL = 0;
			if (Usr_V_YL > FrameHeightT)Usr_V_YL = FrameHeightT;
			if (Usr_V_YH < 0)Usr_V_YH = 0;
			if (Usr_V_YH > FrameHeightT)Usr_V_YH = FrameHeightT;


			//compare shrink
			//		if ((int(Usr_V_XL*1.1) > xl) && (int(Usr_V_XH*0.9) < xh) && (int(Usr_V_YL*1.1) > yl) && (int(Usr_V_YH*0.8) < yh))
			if (CheckIfInList(Block_List, frame1, Usr_V_XL, Usr_V_YL) &&
				CheckIfInList(Block_List, frame1, Usr_V_XL, Usr_V_YH) &&
				CheckIfInList(Block_List, frame1, Usr_V_XH, Usr_V_YL) &&
				CheckIfInList(Block_List, frame1, Usr_V_XH, Usr_V_YH))
			{
				//cout << "right" << endl;
				ErrResult = ErrResult + "1" + "  " + "0" + "  ";
				outFileB << "1" << ',';
			}
			else
			{
				//cout << "wrong" << endl;

				int Dist = CheckErrorCorrecBlockNO(Block_List, frame1, Usr_V_XH, Usr_V_XL, Usr_V_YH, Usr_V_YL);
				stringstream newstr;
				newstr << Dist;
				ErrResult = ErrResult + "0" + "  " + newstr.str() + "  ";
				outFileB << "0" << ',';
			}
			// add the analysis of shrinking algorithm
			int C = GetInterestedBlockNO(Block_List);
			stringstream newstrA;
			newstrA << C;
			C = Video_Divide*Video_Divide;
			stringstream newstrB;
			newstrB << C;
			ErrResult = ErrResult + "  " + newstrA.str() + "  " + newstrB.str();
			outFileB << newstrA.str() << ',' << newstrB.str() << endl;
			outfile << ErrResult << endl;
			ErrResult.clear();
			//============store the over head============================================
			//IMP_Start = clock();
			int countsecond = CLOCKS_PER_SEC;
			stringstream newoverhead;
			newoverhead << (B_end - B_Start);
			stringstream newoverheadB;
			newoverheadB << (IMP_end - IMP_Start);
			stringstream newoverheadEB;
			newoverheadEB << (EB_end - EB_Start);
			stringstream newoverheadEA;
			newoverheadEA << (EA_end - EA_Start);
			overheadresult = newoverhead.str() + "   " + newoverheadB.str() + "   " + newoverheadEB.str() + "  " + newoverheadEA.str();
			overheadoutfile << overheadresult << endl;
			cout << (B_end - B_Start) << "-------" << (IMP_end - IMP_Start) << endl;
			//cout << IMP_Start <<"-------"<< countsecond << endl;
			/*============================================================================*/
			//		cout << "int(9/10)" << int(9 / 10) <<"int(1/10)"<< int(1 / 10) << endl;
			//		cout << "int(9%10)" << int(9 % 10) << "int(1%10)" << int(1 % 10) << endl;



			cvNamedWindow("Optical Flow", CV_WINDOW_NORMAL);
			cvNamedWindow("User View", CV_WINDOW_NORMAL);
			//		cvFlip(frame1, NULL, 2);
			cvShowImage("User View", Usr_View);    //  Usr_View   NewVideoF
			cvShowImage("Optical Flow", frame1);     // background is IP_GMM_mask; tracking is frame1  NewVideoF
													 //cvShowImage("Optical Flow", IP_GMM_mask);     // background is IP_GMM_mask; tracking is frame1
			cvWriteFrame(writer, frame1);                   ////  frame1   NewVideoF frame1   NewBlack   VideoM

			cvWaitKey(33);

			//======404 frame no continue==========
			if (framenumber >= 1800)             // 3240 for female football   404 for tsest  1800 for 2_5 fighting
				break;



			//   cv::Mat m = cv::cvarrToMat(frame1);
			//   writer << m;//opencv3.0 version writer  

		}
		outFileB.close();
		overheadoutfile.close();
		outfile.close();
		cap.release();
		capb.release();
	}
	cvWaitKey(33);
	system("pause");
}


