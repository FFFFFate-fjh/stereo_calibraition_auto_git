#include<iostream>
#include<list>
#include<vector>
#include<stdio.h>
#include<fstream>
#include <time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include<opencv2/opencv.hpp>

using namespace std;
using namespace cv;


bool stereo_calibrate( vector<Mat> img_left, vector<Mat> img_right, int width, int height, double& error);
