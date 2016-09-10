/*
 * binocular_rectify.h
 *
 *  Created on: Aug 17, 2016
 *      Author: lzp
 */

#ifndef BINOCULAR_RECTIFY_H_
#define BINOCULAR_RECTIFY_H_

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

	/*
	 * Undistort left and right cameras' captures.
	 *
	 * input:
	 * 		intrinsics_yml: intrinsics parameters.
	 * 		extrinsics_yml: extrinsics parameters.
	 * 		img_l: image from left camera.
	 * 		img_r: image from right camera.
	 *
	 * output:
	 * 		img_l_undistorted: undistorted left image.
	 * 		img_r_undistorted: undistorted right image.
	 * */
	bool stereo_rectify(	const string intrinsics_yml,
							const string extrinsics_yml,
							Mat img_l,
							Mat img_r,
							Mat& img_l_undistorted,
							Mat& img_r_undistorted);

#endif /* BINOCULAR_RECTIFY_H_ */
