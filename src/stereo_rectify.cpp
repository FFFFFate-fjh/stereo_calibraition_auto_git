/*
 * binocular_rectify.cpp
 *
 *  Created on: Aug 17, 2016
 *      Author: lzp
 */


#include "stereo_rectify.h"



bool stereo_rectify(	const string intrinsics_yml,const string extrinsics_yml,
							Mat img_l, Mat img_r, Mat& img_l_undistorted,
							Mat& img_r_undistorted)
{
		Size img_size;
		img_size=img_l.size();
		if(img_size!=img_r.size()){
			cout<<"The images have different sizes."<<endl;
			return false;
		}
		//reading intrinsic parameters
		FileStorage fs(intrinsics_yml, CV_STORAGE_READ);
		if (!fs.isOpened()) {
			cout << "Failed to open file: " << intrinsics_yml << endl;
			return false;
		}

		Mat M1, D1, M2, D2;
		fs["M1"] >> M1;
		fs["D1"] >> D1;
		fs["M2"] >> M2;
		fs["D2"] >> D2;

		//M1 *= scale;
		//M2 *= scale;

		fs.open(extrinsics_yml, CV_STORAGE_READ);
		if (!fs.isOpened()) {
			cout << "Failed to open file: " << extrinsics_yml << endl;
			return false;
		}

		Mat R1, P1, R2, P2;

		fs["R1"] >> R1;
		fs["P1"] >> P1;
		fs["R2"] >> R2;
		fs["P2"] >> P2;

		//单目矫正
		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11,
				map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21,
				map22);
        cout<<map11.at<short>(636,869)<<" "<<map12.at<short>(636,869)<<endl;
        cout<<map11.at<short>(637,865)<<" "<<map12.at<short>(637,865)<<endl;
//        cout<<img_l.at<short>(830,240)<<endl;
		Mat img1r, img2r;
		remap(img_l, img1r, map11, map12, INTER_LINEAR); //将矫正后图像映射到img1r上
        remap(img_r, img2r, map21, map22, INTER_LINEAR);
//        cout<<map11.at<short>(830,240)<<" "<<map12.at<short>(830,240)<<endl;
//        cout<<img1r.at<short>(map11.at<short>(830,240),map12.at<short>(830,240))<<endl;
		img_l_undistorted = img1r;
		img_r_undistorted = img2r;

	return true;
}
