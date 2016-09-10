/*
 * camera_calibrate.h
 *
 *  Created on: Aug 18, 2016
 *      Author: lzp
 */

#ifndef CAMERA_CALIBRATE_H_
#define CAMERA_CALIBRATE_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

/*
 * Calibrate a camera by using chess board.
 *
 *input:
 *		width: number of corners in a row of the chess board.
 *		height: number of corners in a column of the chess board.
 *		image_list: .xml file, used for recording chess board images path.
 *
 *output:
 *		camera_param_file: .yml file storing camera parameters.
 * */

bool camera_calibrate(int width, int height, vector<Mat> img,
						const string& camera_param_file);

#endif /* CAMERA_CALIBRATE_H_ */
