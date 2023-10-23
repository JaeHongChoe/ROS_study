#include <iostream>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

Mat calcGrayHist(const Mat& img)
{
	CV_Assert(img.type() == CV_8U);

	Mat hist;
	int channels[] = { 0 };
	int dims = 1;
	const int histSize[] = { 256 };
	float graylevel[] = { 0, 256 };
	const float* ranges[] = { graylevel };

	calcHist(&img, 1, channels, noArray(), hist, dims, histSize, ranges, true);

	return hist;
}

Mat getGrayHistImage(const Mat& hist)
{
	CV_Assert(!hist.empty());
	CV_Assert(hist.type() == CV_32F);

	double histMax = 0.;
	minMaxLoc(hist, 0, &histMax);

	Mat imgHist(100, 256, CV_8UC1, Scalar(255));
	for (int i = 0; i < 256; i++) {
		line(imgHist, Point(i, 100),
			Point(i, 100 - cvRound(hist.at<float>(i) * 100 / histMax)), Scalar(0));
	}

	return imgHist;
}

int main()
{
	Mat src = imread("lenna.bmp", IMREAD_GRAYSCALE);

	if (src.empty()) {
		cerr << "Image load failed!" << endl;
		return -1;
	}

	double gmin, gmax;
	minMaxLoc(src, &gmin, &gmax);

	Mat dst = (src - gmin) * 255 / (gmax - gmin);

	imshow("src", src);
	imshow("dst", dst);
	imshow("hist_src", getGrayHistImage(calcGrayHist(src)));
	imshow("hist_dst", getGrayHistImage(calcGrayHist(dst)));

	int hist[256] = {0,};
	for(int y=0; y<src.rows;y++){
		for (int x =0; x < src.cols; x++){
			hist[src.at<uchar>(y,x)]++;
		}
	}

	int gminx = 255, gmaxx = 0;
	int ratio= int(src.cols * src.rows * 0.01);

	for (int i= 0, s = 0; i< 255; i++){
		if (ratio <= s){
			gminx = i;
			break;
		}
		s+= hist[i];
	}
	for (int i= 255, s = 0; i >= 0; i--) {
		if (ratio <= s){
			gmaxx = i;
			break;
		}
		s += hist[i];
	}

	cout << gminx << " "<< gmaxx << endl;
	Mat dstx = (src - gminx) * 255 / (gmaxx - gminx);

	imshow("dstx", dstx);
	imshow("hist_dstx", getGrayHistImage(calcGrayHist(dstx)));

	waitKey();
}
