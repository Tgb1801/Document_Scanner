#include<opencv2/opencv.hpp>
#include<opencv2/imgcodecs.hpp>
#include<opencv2/highgui.hpp>
#include<opencv2/imgproc.hpp>
#include<iostream>

using namespace std;
using namespace cv;

Mat Coloured_to_GrayScale(Mat img)		// This function converts coloured image to gray scale.
{
	vector<Mat> BGR;	// BGR is a vector holding blue, green and red components of the image.

	split(img, BGR);	// Split the image into its BGR components.

	// BGR[0] is blue component.

	// BGR[1] is green component.

	// BGR[2] is red component.

	//namedWindow("BLUE", WINDOW_NORMAL);

	//imshow("BLUE", BGR[0]);

	//namedWindow("GREEN", WINDOW_NORMAL);

	//imshow("GREEN", BGR[1]);

	//namedWindow("RED", WINDOW_NORMAL);

	//imshow("RED", BGR[2]);

	Mat gray_scaled_img = 0.3 * BGR[2] + 0.59 * BGR[1] + 0.11 * BGR[0];	// Image converted to gray scale.

	//namedWindow("Gray_Scale", WINDOW_NORMAL);

	//imshow("Gray_Scale", gray_scaled_img);

	//waitKey(0);

	return gray_scaled_img;
}

Mat GrayScale_to_Blur(Mat gray_scale)		// This function converts gray scale image to blurred image.
{
	float mat[49];

	for (int i = 0; i < 49; i++)
		mat[i] = 1.0 / 49.0;

	Mat k(7, 7, CV_32F, mat);

	Mat Blurred_img;

	filter2D(gray_scale, Blurred_img, -1, k);	// Gray Scale image converted to blurred image.

	return Blurred_img;
}

int main()
{
	Mat img = imread("ticket.jpg");		// To get image in the img variable.

	namedWindow("ticket", WINDOW_NORMAL);

	imshow("ticket", img);

	waitKey(0);

	Mat gray_scaled_img = Coloured_to_GrayScale( img );		// Gray Scaled image.

	Mat Blurred_img = GrayScale_to_Blur(gray_scaled_img);	// Blurred image.

	namedWindow("Blurred", WINDOW_NORMAL);

	imshow("Blurred", Blurred_img);

	waitKey(0);

	return 0;
}