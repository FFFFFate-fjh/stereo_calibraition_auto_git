/*
 * test_binocular_rectify.cpp
 *
 *  Created on: Aug 17, 2016
 *      Author: lzp
 */


#include "stereo_rectify.h"

static void help() {

	cout<<"Arguments: "<<endl
			<<"intrinsics_yml extrinsics_yml "
			<<"img_l_file img_r_file "
			<<"img_l_undistorted_file img_r_undistorted_file "
			<<endl;
}


int main(int argc, char* argv[]) {

	if(argc!=7){
		help();
		return 0;
	}

	Mat img_l, img_r, img_l_undistorted, img_r_undistorted;
	img_l=imread(argv[3]);
	img_r=imread(argv[4]);


	bool ok=stereo_rectify(argv[1],argv[2],img_l, img_r,
							  img_l_undistorted,img_r_undistorted);

	if(ok){
		cout<<"rectify successfully!"<<endl;
	}else{
		cout<<"rectify failed!"<<endl;
	}

	imwrite(argv[5],img_l_undistorted);
	imwrite(argv[6],img_r_undistorted);

	return 0;
}
