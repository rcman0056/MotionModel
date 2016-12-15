#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <ctype.h>
#include <fstream>
#include <string>
#include <limits>
using namespace cv;
using namespace std;
static void displayResults(Mat& prevImage, Mat& image,
		vector<Point2f>* points, bool* outliers) {
	String displayType = "vectors"; //points, vectors are the options
	int waitTime = 10; //in ms
	Mat colorImage;
	cvtColor(prevImage,colorImage,CV_GRAY2RGB);
	if (displayType == "points") {
	Mat nextColorImage;
	cvtColor(image,nextColorImage,CV_GRAY2RGB);
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
		waitKey(0);
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
            if(outliers[i])
			    arrowedLine(colorImage, points[1][i], points[0][i], Scalar(0, 0, 200), 5, 8, 0, 0.3);
			else
                arrowedLine(colorImage, points[1][i], points[0][i], Scalar(0, 200, 0), 5, 8, 0, 0.3);
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
		dataUnion.bytes[7-i] = bytes[i+index];
	}
	return dataUnion.doubleOut;
}
double* calcMean(vector<Point3f> translationField, bool* outliers){
    static double translation[3] = {0,0,0};
	//accumulate the translation in every direction
	//cout<<endl<<"X translation:";
	int numPoints = translationField.size();
	int numInliers = 0;
	int numOutliers = 0;
	//cout<<endl<<"Num Points: "<<numPoints<<endl;
	//cout<<endl;
	for (int i = 0; i < numPoints; i++) {
	    if(outliers[i]){
	    numOutliers++;
	    }
	    else{
		translation[0] += translationField[i].x;
		translation[1] += translationField[i].y;
		translation[2] += translationField[i].z;
		numInliers++;
		//cout<<"\t"<<outliers[i];
		}
	}
	//cout<<endl<<"Skipped points: "<<skippedPoints;
	//divide by number of points to complete the average
	translation[0] /= numInliers;
	translation[1] /= numInliers;
	translation[2] /= numInliers;
	//cout<<endl<<"Outliers: "<<numOutliers<<"\tInliers: "<<numInliers;
    return translation;
}
double* calcStanDev(vector<Point3f> translationField, double translation[3], bool* outliers){
    static double sigma[3] = {0,0,0};
    int numPoints = translationField.size();
    int numInliers = 0;
	for (int i = 0; i < numPoints; i++) {
	    if(outliers[i]){
	    }
	    else {
	        sigma[0] += pow(translationField[i].x-translation[0],2);
	        sigma[1] += pow(translationField[i].y-translation[1],2);
	        sigma[2] += pow(translationField[i].z-translation[2],2);
	        numInliers++;
        }
	}
	sigma[0] /= numInliers;
    sigma[1] /= numInliers;
    sigma[2] /= numInliers;
    sigma[0] = pow(sigma[0],0.5);
    sigma[1] = pow(sigma[1],0.5);
    sigma[2] = pow(sigma[2],0.5);
    return sigma;
}
double* mainOpticalFlow(int8_t* inputData) {
    bool DISPLAY = false;
    bool DEBUGGING = false;
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
			prevGray.at<uchar>(row, col) = (uint8_t)inputData[index + col + nx*row];
		}
	}
	index += nx * ny;
	cv::Mat gray(ny, nx, CV_8UC1, cv::Scalar(255));
	for (row = 0; row < ny; ++row) {
		for (col = 0; col < nx; ++col) {
			gray.at<uchar>(row, col) = (uint8_t)inputData[index + col + nx*row];
		}
	}
	TermCriteria termcrit(TermCriteria::COUNT | TermCriteria::EPS, 20, 0.03);
	Size winSize(31, 31);
	//Mat gray, prevGray//, prevImage, image, frame;
	vector<Point2f> points[2];
	vector<uchar> status;
	vector<float> err;
	//evenly distribute first set of points
	int xlim = gray.cols;
	int ylim = gray.rows;
	int dx = xlim / 10;
	int dy = ylim / 10;
	for (int j = dx; j < xlim - dx; j += dx) {
		for (int k = dy; k < ylim - dy; k += dy) {
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
	int numPoints = points[0].size();
	//calculate location of points in next frame
	calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err,
			winSize, 3, termcrit, 0, 0.001);
	//calculate the normalized points
	vector<Point3f> pointsNorm[2];
	for (int i = 0; i < numPoints; i++) {
		for (int j = 0; j < 2; j++) {
			Point3f tempPoint;
			tempPoint.x = (points[j][i].x - cx) / fx;
			tempPoint.y = (points[j][i].y - cy) / fy;
			//normalize
			double magnitude = pow(pow(tempPoint.x,2)+pow(tempPoint.y,2)+1,0.5);
			tempPoint.x *= 1/magnitude;
			tempPoint.y *= 1/magnitude;
			tempPoint.z = 1/magnitude;
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
    int numIterations = 5;
    int numOutliers;
    double *translation;
    double *sigma;
    bool outliers[numPoints];
    for (int j = 0; j < numPoints; j++) {
        outliers[j] = false;
    }
    for (int i = 0; i<numIterations;i++){
        //calculate the mean translation
    	translation = calcMean(translationField, outliers);
    	//calculate the standard deviation of the translation
    	sigma = calcStanDev(translationField, translation, outliers);
        //cout.precision(10);
        //cout<<endl<<"Sigma:"<<"\t"<<sigma[0]<<"\t"<<sigma[1]<<"\t"<<sigma[2];
        //cout<<endl<<"Mean translation:"<<"\t"<<translation[0]<<"\t"<<translation[1]<<"\t"<<translation[2]<<endl<<endl;
        numOutliers = 0;
        for (int j = 0; j < numPoints; j++) {
                outliers[j] = false;
        }
        //detect outliers
        for (int j = 0; j < numPoints; j++) {
            double deviance[3];
            double thresholdFactor = 2;
            deviance[0] = translationField[j].x-translation[0];
            deviance[1] = translationField[j].y-translation[1];
            deviance[2] = translationField[j].z-translation[2];
            if(abs(deviance[0])>thresholdFactor*sigma[0]){
                outliers[j] = true;
                numOutliers++;
            }
            else if(abs(deviance[1])>thresholdFactor*sigma[1]){
                outliers[j] = true;
                numOutliers++;
            }
            /*
            else if(abs(deviance[2])>thresholdFactor*sigma[2]){
                outliers[j] = true;
                numOutliers++;
            }
            */
            else {
                outliers[j] = false;
            }
        }
    }
    	//print out for debugging
    	int indexD = 24;
    	if (DEBUGGING){
    	cout<<endl<<"DEBUGGING00"<<"\t";
    	cout<<points[0][indexD].x<<"\t"<<points[0][indexD].y<<"\t";
    	cout<<points[1][indexD].x<<"\t"<<points[1][indexD].y<<"\t";
    	cout<<"pixel coordinates"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING01"<<"\t";
    	cout<<pointsNorm[0][indexD].x<<"\t"<<pointsNorm[0][indexD].y<<"\t"<<pointsNorm[0][indexD].z<<"\t";
    	cout<<pointsNorm[1][indexD].x<<"\t"<<pointsNorm[1][indexD].y<<"\t"<<pointsNorm[1][indexD].z<<"\t";
    	cout<<"normalized coordinates"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING02"<<"\t";
    	cout<<pointsNED[0][indexD].x<<"\t"<<pointsNED[0][indexD].y<<"\t"<<pointsNED[0][indexD].z<<"\t";
    	cout<<pointsNED[1][indexD].x<<"\t"<<pointsNED[1][indexD].y<<"\t"<<pointsNED[1][indexD].z<<"\t";
    	cout<<"unscaled nav coordinates"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING03"<<"\t";
    	cout<<theta[0][indexD]<<"\t";
    	cout<<theta[1][indexD]<<"\t";
    	cout<<"thetas"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING04"<<"\t";
    	cout<<depth[0][indexD]<<"\t";;
    	cout<<depth[1][indexD]<<"\t";;
    	cout<<"depth vectors"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING05"<<"\t";
    	cout<<pointsCam[0][indexD].x<<"\t"<<pointsCam[0][indexD].y<<"\t"<<pointsCam[0][indexD].z<<"\t";
    	cout<<pointsCam[1][indexD].x<<"\t"<<pointsCam[1][indexD].y<<"\t"<<pointsCam[1][indexD].z<<"\t";
    	cout<<"camera coordinates"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING06"<<"\t";
    	cout<<pointsNEDScaled[0][indexD].x<<"\t"<<pointsNEDScaled[0][indexD].y<<"\t"<<pointsNEDScaled[0][indexD].z<<"\t";
    	cout<<pointsNEDScaled[1][indexD].x<<"\t"<<pointsNEDScaled[1][indexD].y<<"\t"<<pointsNEDScaled[1][indexD].z<<"\t";
    	cout<<"scaled NED coordinates"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING07"<<"\t";
    	cout<<translationField[indexD].x<<"\t"<<translationField[indexD].y<<"\t"<<translationField[indexD].z<<"\t";
    	cout<<"translation vector"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING08"<<"\t";
    	cout<<firstCamToNav[0][0]<<"\t"<<firstCamToNav[0][1]<<"\t"<<firstCamToNav[0][2]<<"\t";
    	cout<<firstCamToNav[1][0]<<"\t"<<firstCamToNav[1][1]<<"\t"<<firstCamToNav[1][2]<<"\t";
    	cout<<firstCamToNav[2][0]<<"\t"<<firstCamToNav[2][1]<<"\t"<<firstCamToNav[2][2]<<"\t";
    	cout<<"first camToNav"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING09"<<"\t";
    	cout<<secondCamToNav[0][0]<<"\t"<<secondCamToNav[0][1]<<"\t"<<secondCamToNav[0][2]<<"\t";
    	cout<<secondCamToNav[1][0]<<"\t"<<secondCamToNav[1][1]<<"\t"<<secondCamToNav[1][2]<<"\t";
    	cout<<secondCamToNav[2][0]<<"\t"<<secondCamToNav[2][1]<<"\t"<<secondCamToNav[2][2]<<"\t";
    	cout<<"second camToNav"<<"\t"<<"END";
    	cout<<endl<<"DEBUGGING10"<<"\t";
        cout<<outliers[indexD]<<"\t"<<numOutliers<<"\t";
        cout<<"outlier"<<"\t"<<"END";
        cout<<endl<<"DEBUGGING11"<<"\t";
        cout<<translation[0]<<"\t"<<translation[1]<<"\t"<<translation[2]<<"\t";
        cout<<"final translation vector"<<"\t"<<"END";
    	}
if (DISPLAY){
    namedWindow( "LK Demo", WINDOW_NORMAL   ); //create resizable window
        //resizeWindow("LK Demo", nx/2,ny/2); //resize window to half of image resolution
	displayResults(prevGray, gray, points,outliers);
	waitKey(0);
	}
	//clean up stuff
	points[1].clear();
    points[0].clear();
    pointsCam[1].clear();
    pointsCam[0].clear();
    pointsNED[1].clear();
    pointsNED[0].clear();
    pointsNEDScaled[1].clear();
    pointsNEDScaled[0].clear();
    pointsNorm[1].clear();
    pointsNorm[0].clear();
	return translation;
}
extern "C" double* opticalFlow(int8_t* inputData) {
	return mainOpticalFlow(inputData);
}