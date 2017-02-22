#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <vector>
#include <iostream>
#include <ctype.h>
#include <fstream>
#include <string>
#include <limits>

using namespace cv;
using namespace std;

static void displayResults(Mat& prevImage, Mat& image, vector<Point2f>* points,
		Mat inliers) {
	String displayType = "vectors"; //points, vectors are the options
	int waitTime = 10; //in ms
	Mat colorImage;
	cvtColor(prevImage, colorImage, CV_GRAY2RGB);

	if (displayType == "points") {
		Mat nextColorImage;
		cvtColor(image, nextColorImage, CV_GRAY2RGB);
		//circle original set of points
		for (int i = 0; i < points[0].size(); i++) {
			circle(colorImage, points[0][i], 3, Scalar(0, 200, 0), 3, 8);
		}

		//display first image with original points
		imshow("LK Demo", colorImage);
		waitKey(waitTime);

		//circle translated set of points
		for (int i = 0; i < points[1].size(); i++) {
			circle(nextColorImage, points[1][i], 3, Scalar(0, 200, 0), 3, 8);
		}
		waitKey(5);
		//display second image with translated points
		imshow("LK Demo", nextColorImage);
		waitKey(waitTime);
	} else if (displayType == "vectors") {
		//vector<Point2f> vectors[1];
		//cout<<endl<<points[0].size()<<" "<<points[1].size()<<endl;
		for (int i = 0; i < points[1].size(); i++) {
			//cout<<endl<<points[1][i]<<endl;
			//float x = points[1][i].x-points[0][i].x;
			//float y = points[1][i].y-points[0][i].y;
			//Point2f vector = Point2f(x, y);
			//vectors[0].push_back(vector);

			if (inliers.at<bool>(i))
				arrowedLine(colorImage, points[1][i], points[0][i],
						Scalar(0, 200, 0), 5, 8, 0, 0.3);
			else
				arrowedLine(colorImage, points[1][i], points[0][i],
						Scalar(0, 0, 200), 5, 8, 0, 0.3);
		}
		//cout<<endl<<"flag1"<<endl;
		imshow("LK Demo", colorImage);
		waitKey(waitTime);
	}
}

int bytesToInt(int8_t* bytes, int index) {
	int integerOut = (bytes[index] << 8) | (bytes[index + 1] & 0x0FF);
	return integerOut;
}

double bytesToDouble(int8_t* bytes, int index) {

	union {
		double doubleOut;
		char bytes[sizeof(double)];
	} dataUnion;
//char bytesCast[8];
	for (int i = 0; i < 8; i++) {
		dataUnion.bytes[7 - i] = bytes[i + index];
	}

	return dataUnion.doubleOut;
}

double* dcmToRpy(Mat dcm)
{
    static double rpy[3];
    rpy[2] = atan2(dcm.at<double>(0,1), dcm.at<double>(0,0));
    rpy[1] = -asin(dcm.at<double>(0,2));
    rpy[0] = atan2(dcm.at<double>(1,2), dcm.at<double>(2,2));
    return rpy;
}

double* mainVO(int8_t* inputData) {
	bool DISPLAY = true;
	bool DEBUGGING = true;
	bool UNDISTORT_IMAGES = true;

	//first split up bytes into ints, doubles, DCM's, and images
	int index = 0;
	double height = bytesToDouble(inputData, index);
	index += 8;
	double fx = bytesToDouble(inputData, index);
	index += 8;
	double fy = bytesToDouble(inputData, index);
	index += 8;
	double cx = bytesToDouble(inputData, index);
	index += 8;
	double cy = bytesToDouble(inputData, index);
	index += 8;
	double firstCamToNav[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			firstCamToNav[i][j] = bytesToDouble(inputData, index);
			index += 8;
		}
	}
	double secondCamToNav[3][3];
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			secondCamToNav[i][j] = bytesToDouble(inputData, index);
			index += 8;
		}
	}
	int ny = bytesToInt(inputData, index);
	index += 2;
	int nx = bytesToInt(inputData, index);
	index += 2;
	cv::Mat prevGray(ny, nx, CV_8UC1, cv::Scalar(255));
	int row;
	int col;
	for (row = 0; row < ny; ++row) {
		for (col = 0; col < nx; ++col) {
			prevGray.at<uchar>(row, col) = (uint8_t) inputData[index + col
					+ nx * row];
		}
	}
	index += nx * ny;
	cv::Mat gray(ny, nx, CV_8UC1, cv::Scalar(255));
	for (row = 0; row < ny; ++row) {
		for (col = 0; col < nx; ++col) {
			gray.at<uchar>(row, col) = (uint8_t) inputData[index + col
					+ nx * row];
		}
	}
	index += nx * ny;

	int bool1 = bytesToInt(inputData, index);
	index += 2;
	bool useFeatures = false;
	if (bool1 == 1) {
		useFeatures = true;
	}

	int bool2 = bytesToInt(inputData, index);
	index += 2;
	bool inertialAiding = false;
	if (bool2 == 1) {
		inertialAiding = true;
	}

	vector<Point2f> points[2];
	int numPoints;

	//undistort image
	if (UNDISTORT_IMAGES){
	Mat cameraMatrix = (Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	Mat distCoeffs  = (Mat_<double>(5,1) << -0.391873, 0.234790, 0,0, 0);
	Mat undistortGray, undistortPrevGray;
	undistort( gray,  undistortGray, cameraMatrix, distCoeffs, cameraMatrix );
	undistort( prevGray,  undistortPrevGray, cameraMatrix, distCoeffs, cameraMatrix );
	gray = undistortGray;
	prevGray = undistortPrevGray;
	}

	if (useFeatures) {

		//detect features
		bool nonmaxSuppression = false;
		int fastThreshold = 50; ///was 30
		vector<KeyPoint> keyPoints1,keyPoints2;
		//FAST(prevGray, keyPoints1, fastThreshold, nonmaxSuppression);
		//FAST(gray, keyPoints2, fastThreshold, nonmaxSuppression);


		//AKAZE descriptors
        Mat descriptors1, descriptors2;
        Ptr<AKAZE> akaze = AKAZE::create();
        //akaze->set("threshold", 3e-4);
        akaze->setThreshold(5e-5); //smaller is more loose threshold 5e-3 /////////////////////////////////////was 5e-6///////////////////////////////////////////////
        akaze->detectAndCompute(prevGray, noArray(), keyPoints1, descriptors1);
        akaze->detectAndCompute(gray, noArray(), keyPoints2, descriptors2);
        //akaze->compute(prevGray,keyPoints1,descriptors1);
        //akaze->compute(gray,keyPoints2,descriptors2);

        BFMatcher matcher(NORM_HAMMING);
        vector<vector<DMatch> > matches;
        matcher.knnMatch(descriptors1, descriptors2, matches, 2);

        //use FLANN matcher to match descriptors

        //FlannBasedMatcher matcher;
        //vector< DMatch > matches;
        //matcher.match( descriptors1, descriptors2, matches);
        float inlier_threshold = 0.8;
        //cout<<endl<<"Initial number of matches is "<<matches.size();
		numPoints = keyPoints1.size();
		for (int i = 0; i < numPoints; i++) {
		    int trainIndex = matches[i][0].trainIdx;
		    int queryIndex = matches[i][0].queryIdx;
		    //int trainIndex = matches[i].trainIdx;
		    //int queryIndex = matches[i].queryIdx;
		    if(matches[i][0].distance < inlier_threshold * matches[i][1].distance){
		    //if(matches[i].distance < inlier_threshold * matches[i].distance){
		        points[0].push_back(keyPoints1[queryIndex].pt);
		        points[1].push_back(keyPoints2[trainIndex].pt);
		    }
		}
		//cout<<endl<<"Final number of matches is "<<points[0].size();

	} else {
		//evenly distribute first set of points
		int xlim = gray.cols;
		int ylim = gray.rows;
		int dx = xlim / 22;
		int dy = ylim / 20;
		for (int j = 0; j < xlim - 1; j += dx) {
			for (int k = 0; k < ylim - 1; k += dy) {
				float x = j;
				float y = k;
				Point2f point = Point2f(x, y);
				vector<Point2f> tmp;
				tmp.push_back(point);
				//relocate points to nearby corners
				//cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
				points[0].push_back(tmp[0]);
			}
		}
		numPoints = points[0].size();
			//calculate location of points in next frame
        	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
        	Size winSize(31, 31);
        	vector<uchar> status;
        	vector<float> err;
        	calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err,
        			winSize, 3, termcrit, 0, 0.001);
        	//remove bad points
        	int numBadPoints = 0;
        	for (int i = 0; i < numPoints-numBadPoints; i++) {
        		bool failed = status.at(i) == 0;
        		bool runOffImage = (points[1][i].x < 0) || (points[1][i].y < 0)
        				|| (points[1][i].x > nx) || (points[1][i].y > ny);
        		bool badPoint = runOffImage || failed;
        		if (badPoint) {
        			//status.at(i+numBadPoints) = 0;
        			status.erase(status.begin() + (i - numBadPoints));
        			points[0].erase(points[0].begin() + (i - numBadPoints));
        			points[1].erase(points[1].begin() + (i - numBadPoints));
        			numBadPoints++;
        		}
        	}
	}
	numPoints = points[0].size();
	//Use RANSAC to find E matrix and to reject outliers
	Point2d pp(cx, cy);
	double focal = (fx + fy) / 2;
	int method = RANSAC; //RANSAC or MEDS (LMedS)
	double prob = 0.9999; //confidence level
	double threshold = 0.2; //RANSAC threshold (distance from epipolar line)
	Mat inliers;
	Mat E = findEssentialMat(points[0], points[1], focal, pp, method, prob,
			threshold, inliers);
	numPoints = inliers.rows;
	//cout << endl << "Number of features:" << "\t" << numPoints;
	/*
	 for (int i = 0; i<numPoints;i++){
	 cout<<"\t"<<inliers.at<bool>(i);
	 }
	 */

	 double cam1Tocam2[3][3];

	if (inertialAiding) {
	}
	else{
		//calculate the rotation from decomposing the Essential Matrix
		Mat R, t, mask;
		recoverPose(E, points[0], points[1], R, t, focal, pp, mask);
		//cout<<endl<<"Number of points that pass the cheirality check:"<<"\t"<<mask.rows;


		Mat R1, R2, t2;
		decomposeEssentialMat( E,  R1, R2, t2);

		double* tempRpy = dcmToRpy(R1);
		double rpy1[3];
		rpy1[0] = tempRpy[0];
		rpy1[1] = tempRpy[1];
		rpy1[2] = tempRpy[2];
		double* rpy2 = dcmToRpy(R2);
		//cout<<endl<<"RPY: "<<rpy1[0]<<rpy1[1]<<rpy1[2];

		//cout<<endl<<"RPY1: "<<rpy1[0]<<" "<<rpy1[1]<<" "<<rpy1[2];
        //cout<<endl<<"RPY2: "<<rpy2[0]<<" "<<rpy2[1]<<" "<<rpy2[2];

        int R1Score = 0;
        int R2Score = 0;







		//use recovered rotation to calculate the DCM from the first camera frame to the nav frame
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				firstCamToNav[i][j] =
				          secondCamToNav[i][0] * R.at<double>(0, j)
						+ secondCamToNav[i][1] * R.at<double>(1, j)
						+ secondCamToNav[i][2] * R.at<double>(2, j);

				cam1Tocam2[i][j] = R.at<double>(i,j);
			}
		}

if(DEBUGGING){
				cout << endl << "DEBUGGING12" << "\t";
        		cout << R1.at<double>(0,0) << "\t" << R1.at<double>(0,1) << "\t" << R1.at<double>(0,2) << "\t";
        		cout << R1.at<double>(1,0) << "\t" << R1.at<double>(1,1) << "\t" << R1.at<double>(1,2) << "\t";
        		cout << R1.at<double>(2,0) << "\t" << R1.at<double>(2,1) << "\t" << R1.at<double>(2,2) << "\t";
        		cout << "cam1 to cam2" << "\t" << "END";

				cout << endl << "DEBUGGING14" << "\t";
        		cout << R2.at<double>(0,0) << "\t" << R2.at<double>(0,1) << "\t" << R2.at<double>(0,2) << "\t";
        		cout << R2.at<double>(1,0) << "\t" << R2.at<double>(1,1) << "\t" << R2.at<double>(1,2) << "\t";
        		cout << R2.at<double>(2,0) << "\t" << R2.at<double>(2,1) << "\t" << R2.at<double>(2,2) << "\t";
        		cout << "cam1 to cam2" << "\t" << "END";
        		}
	}

	//calculate the normalized points
	vector<Point3f> pointsNorm[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			Point3f tempPoint;
			tempPoint.x = (points[j][i].x - cx) / fx;
			tempPoint.y = (points[j][i].y - cy) / fy;

			//normalize
			double magnitude = pow(
					pow(tempPoint.x, 2) + pow(tempPoint.y, 2) + 1, 0.5);
			tempPoint.x *= 1 / magnitude;
			tempPoint.y *= 1 / magnitude;
			tempPoint.z = 1 / magnitude;
			pointsNorm[j].push_back(tempPoint);
		}
	}

	//rotate the normalized points into the nav frame
	vector<Point3f> pointsNED[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			double dcm[3][3];
			if (j == 0) {
				memcpy(dcm, firstCamToNav, sizeof(dcm));
			} else {
				memcpy(dcm, secondCamToNav, sizeof(dcm));
			}
			float x0 = pointsNorm[j][i].x;
			float y0 = pointsNorm[j][i].y;
			float z0 = pointsNorm[j][i].z;
			Point3f tempPoint;
			tempPoint.x = dcm[0][0] * x0 + dcm[0][1] * y0 + dcm[0][2] * z0;
			tempPoint.y = dcm[1][0] * x0 + dcm[1][1] * y0 + dcm[1][2] * z0;
			tempPoint.z = dcm[2][0] * x0 + dcm[2][1] * y0 + dcm[2][2] * z0;
			pointsNED[j].push_back(tempPoint);
		}
	}

	//calculate the angle between the unit depth vector and each point-vector
	vector<double> theta[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			double tempTheta;
			double x = pointsNED[j][i].x;
			double y = pointsNED[j][i].y;
			double z = pointsNED[j][i].z;
			double sumOfSquares = pow(x, 2) + pow(y, 2) + pow(z, 2);
			tempTheta = acos((z) / pow(sumOfSquares, 0.5));
			theta[j].push_back(tempTheta);
		}
	}

	//calculate the depth vector to each point from the camera
	vector<double> depth[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			double tempDepth = height / cos(theta[j][i]);
			depth[j].push_back(tempDepth);
		}
	}

	//calculate the camera coordinates
	vector<Point3f> pointsCam[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			Point3f tempPoint;
			tempPoint.x = pointsNorm[j][i].x * depth[j][i];
			tempPoint.y = pointsNorm[j][i].y * depth[j][i];
			tempPoint.z = pointsNorm[j][i].z * depth[j][i];
			pointsCam[j].push_back(tempPoint);
		}
	}

	//rotate camera coordinates into nav frame (this time they're actually scaled)
	vector<Point3f> pointsNEDScaled[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			double dcm[3][3];
			if (j == 0) {
				memcpy(dcm, firstCamToNav, sizeof(dcm));
			} else {
				memcpy(dcm, secondCamToNav, sizeof(dcm));
			}
			float x0 = pointsCam[j][i].x;
			float y0 = pointsCam[j][i].y;
			float z0 = pointsCam[j][i].z;
			Point3f tempPoint;
			tempPoint.x = dcm[0][0] * x0 + dcm[0][1] * y0 + dcm[0][2] * z0;
			tempPoint.y = dcm[1][0] * x0 + dcm[1][1] * y0 + dcm[1][2] * z0;
			tempPoint.z = dcm[2][0] * x0 + dcm[2][1] * y0 + dcm[2][2] * z0;
			pointsNEDScaled[j].push_back(tempPoint);
		}
	}

	//difference the two sets of points to calculate a vector field
	vector<Point3f> translationField;
	for (int i = 0; i < numPoints; i++) {
		Point3f tempTranslation;
		tempTranslation.x = pointsNEDScaled[0][i].x - pointsNEDScaled[1][i].x;
		tempTranslation.y = pointsNEDScaled[0][i].y - pointsNEDScaled[1][i].y;
		tempTranslation.z = pointsNEDScaled[0][i].z - pointsNEDScaled[1][i].z;
		translationField.push_back(tempTranslation);
	}

	static double translation[3] = { 0, 0, 0 };
	//accumulate the translation in every direction
	int numInliers = 0;
	int numOutliers = 0;
	for (int i = 0; i < numPoints; i++) {
		if (inliers.at<bool>(i)) {
			translation[0] += translationField[i].x;
			translation[1] += translationField[i].y;
			translation[2] += translationField[i].z;
			numInliers++;
		} else {
			numOutliers++;
		}
	}
	//cout<<endl<<"Skipped points: "<<skippedPoints;
	//divide by number of points to complete the average
	translation[0] /= numInliers;
	translation[1] /= numInliers;
	translation[2] /= numInliers;
	//cout<<endl<<"Outliers: "<<numOutliers<<"\tInliers: "<<numInliers;

	//print out for debugging
	int indexD = 24;
	if (DEBUGGING) {

	static double translationPixels[2] = { 0, 0 };
    	//accumulate the translation in every direction
    	int numInliers = 0;
    	int numOutliers = 0;
    	for (int i = 0; i < numPoints; i++) {
    		if (inliers.at<bool>(i)) {
    			translationPixels[0] += points[0][i].x-points[1][i].x;
    			translationPixels[1] += points[0][i].y-points[1][i].y;
    			numInliers++;
    		} else {
    			numOutliers++;
    		}
    	}
    		translationPixels[0] /= numInliers;
        	translationPixels[1] /= numInliers;

		cout << endl << "DEBUGGING00" << "\t";
		cout << translationPixels[0] << "\t" << translationPixels[1] << "\t";
		cout << '0' << "\t" << '0' << "\t";
		cout << "pixel coordinates" << "\t" << "END";

		cout << endl << "DEBUGGING01" << "\t";
		cout << pointsNorm[0][indexD].x << "\t" << pointsNorm[0][indexD].y
				<< "\t" << pointsNorm[0][indexD].z << "\t";
		cout << pointsNorm[1][indexD].x << "\t" << pointsNorm[1][indexD].y
				<< "\t" << pointsNorm[1][indexD].z << "\t";
		cout << "normalized coordinates" << "\t" << "END";

		cout << endl << "DEBUGGING02" << "\t";
		cout << pointsNED[0][indexD].x << "\t" << pointsNED[0][indexD].y << "\t"
				<< pointsNED[0][indexD].z << "\t";
		cout << pointsNED[1][indexD].x << "\t" << pointsNED[1][indexD].y << "\t"
				<< pointsNED[1][indexD].z << "\t";
		cout << "unscaled nav coordinates" << "\t" << "END";

		cout << endl << "DEBUGGING03" << "\t";
		cout << theta[0][indexD] << "\t";
		cout << theta[1][indexD] << "\t";
		cout << "thetas" << "\t" << "END";

		cout << endl << "DEBUGGING04" << "\t";
		cout << depth[0][indexD] << "\t";
		;
		cout << depth[1][indexD] << "\t";
		;
		cout << "depth vectors" << "\t" << "END";

		cout << endl << "DEBUGGING05" << "\t";
		cout << pointsCam[0][indexD].x << "\t" << pointsCam[0][indexD].y << "\t"
				<< pointsCam[0][indexD].z << "\t";
		cout << pointsCam[1][indexD].x << "\t" << pointsCam[1][indexD].y << "\t"
				<< pointsCam[1][indexD].z << "\t";
		cout << "camera coordinates" << "\t" << "END";

		cout << endl << "DEBUGGING06" << "\t";
		cout << pointsNEDScaled[0][indexD].x << "\t"
				<< pointsNEDScaled[0][indexD].y << "\t"
				<< pointsNEDScaled[0][indexD].z << "\t";
		cout << pointsNEDScaled[1][indexD].x << "\t"
				<< pointsNEDScaled[1][indexD].y << "\t"
				<< pointsNEDScaled[1][indexD].z << "\t";
		cout << "scaled NED coordinates" << "\t" << "END";

		cout << endl << "DEBUGGING07" << "\t";
		cout << translationField[indexD].x << "\t" << translationField[indexD].y
				<< "\t" << translationField[indexD].z << "\t";
		cout << "translation vector" << "\t" << "END";

		cout << endl << "DEBUGGING08" << "\t";
		cout << firstCamToNav[0][0] << "\t" << firstCamToNav[0][1] << "\t"
				<< firstCamToNav[0][2] << "\t";
		cout << firstCamToNav[1][0] << "\t" << firstCamToNav[1][1] << "\t"
				<< firstCamToNav[1][2] << "\t";
		cout << firstCamToNav[2][0] << "\t" << firstCamToNav[2][1] << "\t"
				<< firstCamToNav[2][2] << "\t";
		cout << "first camToNav" << "\t" << "END";

		cout << endl << "DEBUGGING09" << "\t";
		cout << secondCamToNav[0][0] << "\t" << secondCamToNav[0][1] << "\t"
				<< secondCamToNav[0][2] << "\t";
		cout << secondCamToNav[1][0] << "\t" << secondCamToNav[1][1] << "\t"
				<< secondCamToNav[1][2] << "\t";
		cout << secondCamToNav[2][0] << "\t" << secondCamToNav[2][1] << "\t"
				<< secondCamToNav[2][2] << "\t";
		cout << "second camToNav" << "\t" << "END";

		cout << endl << "DEBUGGING10" << "\t";
		//int numOutliers = numPoints-numInliers;
		cout << inliers.at<bool>(indexD) << "\t" << numInliers << "\t" << numOutliers << "\t";
		cout << "num inliers" << "\t" << "END";

		cout << endl << "DEBUGGING11" << "\t";
		cout << translation[0] << "\t" << translation[1] << "\t"
				<< translation[2] << "\t";
		cout << "final translation vector" << "\t" << "END";

		cout << endl << "DEBUGGING13" << "\t";
        		cout << cam1Tocam2[0][0] << "\t" << cam1Tocam2[0][1] << "\t"
        				<< cam1Tocam2[0][2] << "\t";
        		cout << cam1Tocam2[1][0] << "\t" << cam1Tocam2[1][1] << "\t"
        				<< cam1Tocam2[1][2] << "\t";
        		cout << cam1Tocam2[2][0] << "\t" << cam1Tocam2[2][1] << "\t"
        				<< cam1Tocam2[2][2] << "\t";
        		cout << "cam1Tocam2" << "\t" << "END";
	}

	 if (DISPLAY){
	 namedWindow( "LK Demo", WINDOW_NORMAL   ); //create resizable window
	 //resizeWindow("LK Demo", nx/2,ny/2); //resize window to half of image resolution
	 displayResults(prevGray, gray, points,inliers);
	 waitKey(5);
	 }

	//clean up stuff
	points[1].clear();
	points[0].clear();

	return translation;
}

extern "C" double* VO(int8_t* inputData) {
	return mainVO(inputData);
}

