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

/*Mat GrayScale_to_Blur(Mat gray_scale)		// This function converts gray scale image to blurred image.
{
	float mat[49];

	for (int i = 0; i < 49; i++)
		mat[i] = 1.0 / 49.0;

	Mat k(7, 7, CV_32F, mat);

	Mat Blurred_img;

	filter2D(gray_scale, Blurred_img, -1, k);	// Gray Scale image converted to blurred image.

	return Blurred_img;
}*/

bool compareContourAreas(vector<Point> contour1, vector<Point> contour2)
{
	double i = fabs(contourArea(Mat(contour1)));
	double j = fabs(contourArea(Mat(contour2)));
	return (i < j);
}

vector<Point> findBorderPoints(vector<vector<Point>> contours){
	int i = contours.size()-1;
	vector<Point> borderpts(4);

	long summax = LONG_MIN, summin = LONG_MAX, diffmax = LONG_MIN, diffmin = LONG_MAX;
	Point smax, smin, dmax, dmin;

	for(Point j : contours[i]){
		if(j.x + j.y >summax){
			summax = j.x + j.y;
			smax = j;
		}else if(j.x + j.y < summin){
			summin = j.x + j.y;
			smin = j;
		}
		if(j.y - j.x > diffmax){
			diffmax = j.y - j.x;
			dmax = j;
		}else if(j.y - j.x < diffmin){
			diffmin = j.y - j.x;
			dmin = j;
		}
	}
	borderpts[0] = smin; borderpts[1] = dmin; borderpts[2] = smax; borderpts[3] = dmax;
	return borderpts;
}

int main()
{
	Mat img = imread("ticket.jpg");		// To get image in the img variable.

	namedWindow("ticket", WINDOW_NORMAL);

	imshow("ticket", img);

	waitKey(0);

	Mat gray_scaled_img = Coloured_to_GrayScale(img);
	
	Mat Blurred_img(gray_scaled_img);

	GaussianBlur(gray_scaled_img, Blurred_img, Size(7, 7), 0);	// Gives Blurred image. 

	Mat edge_detected(Blurred_img);

	Canny(Blurred_img, edge_detected, 75, 200);	// Gives edge-detected image.

	namedWindow("edge detection", WINDOW_NORMAL);

	imshow("edge detection", edge_detected);

	Mat Copy_Edged = edge_detected;

	vector<vector<Point>> contours;

	vector<Vec4i> hierarchy;

	findContours(Copy_Edged, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);	

	sort(contours.begin(), contours.end(), compareContourAreas);

	vector<Point> bpoints = findBorderPoints(contours);

	Mat duplicate_image = img;

	for(Point j : bpoints){
		circle(duplicate_image, j, 10, Scalar( 0, 255, 0 ), FILLED);
	}

	namedWindow("Points", WINDOW_NORMAL);

	imshow("Points", duplicate_image);

	drawContours(img, contours, contours.size()-1 , Scalar(0, 255, 0), 2);

	namedWindow("Contours", WINDOW_NORMAL);

	imshow("Contours", img);

	waitKey(0);

	return 0;
}
