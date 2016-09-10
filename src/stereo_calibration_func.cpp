#include "stereo_calibration_func.h"


bool stereo_calibrate(vector<Mat> img_left, vector<Mat> img_right, int width, int height, double& error) {





    //default parameters
    string left_cam_param_file ="./data/camera_L.yml";
    string right_cam_param_file ="./data/camera_R.yml";
//    int width = 11;
//    int height = 8;
    string intrinsics_yml="./data/intrinsics.yml";
    string extrinsics_yml="./data/extrinsics.yml";

    vector<string> imagelist;
    Size boardSize;

    if (height <= 0 || width <= 0) {
        cout << "invalid chess board size" << endl;
    } else {
        boardSize.height = height;
        boardSize.width = width;
    }

//	bool ok = readStringList(chessBoard_img_list, imagelist);
//	if (!ok || imagelist.empty()) {
//		cout << "can not open " << chessBoard_img_list << "or the string list is empty"
//				<< endl;
//		return false;
//	}

//	if (imagelist.size() % 2 != 0) {
//		cout << "Error: the number of images is not even\n"; //如果图片数量不为偶则出错
//		return false;
//	}

    const int maxScale = 2; //the maximum scale for corner detection
    const float squareSize = 2.f; //Set this to your actual square size

    vector<vector<Point2f> > imagePoints[2]; //2D point vectors
    vector<vector<Point3f> > objectPoints; //3D point vectors for actual scale
    Size imageSize;

    //int i, j, k, nimages = (int) imagelist.size() / 2; //number of images for each side
    int i, j, k ; //number of images for each side
    int nimages = img_left.size();
    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);

    //cout<< nimages << endl;
    for (i = j = 0; i < nimages; i++)
    {
        for (k = 0; k < 2; k++)
        {
            if(k==0)
            {
                //const string& filename = imagelist[i * 2 + k]; //read left and right images alternatively
                //Mat img = imread(filename, 0); //read gray scale
                //Mat img=img_v[i];
                Mat img=img_left[i];
                //cout << "read image file:" << filename << endl;
                if (img.empty())
                    break;
                if (imageSize == Size()) //Size() represents type Size? Check imageSize's type is legal?
                    imageSize = img.size();
                else if (img.size() != imageSize) {
                    cout << "The image" "has the size different from the first image size."
                                    "Skipping the pair\n";
                    break;
                }
                bool found = false;
                vector<Point2f>& corners = imagePoints[k][j]; //number of corners in jth image for each side
                for (int scale = 1; scale <= maxScale; scale++) {
                    Mat timg;
                    if (scale == 1)
                        timg = img;
                    else
                        resize(img, timg, Size(), scale, scale);
                    found = findChessboardCorners(timg, boardSize, corners,
                    //the third parameter may need to adjust
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                    if (found) {
                        if (scale > 1) {
                            Mat cornersMat(corners);
                            cornersMat *= 1. / scale; //if it's found after scaled, then rescale it back to original
                        }
                        break;
                    }
                }
                if (!found)
                    break;
                cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                        TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, //在亚像素尺度微调，参数的含义？
                                30, 0.01));
            }
            else if(k==1)
            {
                //const string& filename = imagelist[i * 2 + k]; //read left and right images alternatively
                //Mat img = imread(filename, 0); //read gray scale
                //Mat img=img_v[i];
                Mat img=img_right[i];
                //cout << "read image file:" << filename << endl;
                if (img.empty())
                    break;
                if (imageSize == Size()) //Size() represents type Size? Check imageSize's type is legal?
                    imageSize = img.size();
                else if (img.size() != imageSize) {
                    cout << "The image" "has the size different from the first image size."
                                    "Skipping the pair\n";
                    break;
                }
                bool found = false;
                vector<Point2f>& corners = imagePoints[k][j]; //number of corners in jth image for each side
                for (int scale = 1; scale <= maxScale; scale++) {
                    Mat timg;
                    if (scale == 1)
                        timg = img;
                    else
                        resize(img, timg, Size(), scale, scale);
                    found = findChessboardCorners(timg, boardSize, corners,
                    //the third parameter may need to adjust
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                    if (found) {
                        if (scale > 1) {
                            Mat cornersMat(corners);
                            cornersMat *= 1. / scale; //if it's found after scaled, then rescale it back to original
                        }
                        break;
                    }
                }
                if (!found)
                    break;
                cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
                        TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, //在亚像素尺度微调，参数的含义？
                                30, 0.01));
            }

        }
        if (k == 2)
            j++;

    }
    cout << j << " pairs have been successfully detected.\n";
    nimages = j;
    if (nimages < 2) {
        cout << "Error: too little pairs to run the calibration\n";
        return false;
    }

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);

    for (i = 0; i < nimages; i++) {
        for (j = 0; j < boardSize.height; j++)
            for (k = 0; k < boardSize.width; k++)
                objectPoints[i].push_back(
                        Point3f(k * squareSize, j * squareSize, 0)); //实际棋盘大小（宽，长，0)
    }

    cout << "Running stereo calibration...\n";

    Mat cameraMatrix[2], distCoeffs[2];
    /*分别读取左右目摄像机的标定参数*/

    FileStorage fs_read(left_cam_param_file, CV_STORAGE_READ);
    if(!fs_read.isOpened()){
        cout<<"Failed to open file: "<<left_cam_param_file<<endl;
        return false;
    }
    //左目内参
    fs_read["camera_matrix"]>>cameraMatrix[0];
    //左目畸变
    fs_read["distortion_coefficients"]>>distCoeffs[0];



    fs_read.open(right_cam_param_file, CV_STORAGE_READ);
    if(!fs_read.isOpened()){
        cout<<"Failed to open file: "<<right_cam_param_file<<endl;
        return false;
    }
    //右目内参
    fs_read["camera_matrix"]>>cameraMatrix[1];
    //右目畸变
    fs_read["distortion_coefficients"]>>distCoeffs[1];
    /*---------------------------------------------*/


    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints[0],
            imagePoints[1], //参数的含义？
            cameraMatrix[0], distCoeffs[0], cameraMatrix[1], distCoeffs[1],
            imageSize, R, T, E, F,
            TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
            CV_CALIB_USE_INTRINSIC_GUESS   //自定义摄像机内参
            +CV_CALIB_FIX_ASPECT_RATIO
            + CV_CALIB_FIX_K3
            + CV_CALIB_FIX_K4
            + CV_CALIB_FIX_K5
            );
    cout << "done with RMS error=" << rms << endl; //方均根误差

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0

    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (i = 0; i < nimages; i++) {
        int npt = (int) imagePoints[0][i].size(); //一边的角点数
        Mat imgpt[2];
        for (k = 0; k < 2; k++) {
            imgpt[k] = Mat(imagePoints[k][i]); //一边的角点坐标（标定前）
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k],
                    Mat(), cameraMatrix[k]); //一边的角点坐标（标定后）
            computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]); //在另一幅图中寻找角点对应的极线
        }
        for (j = 0; j < npt; j++) {
            //取点（i,j),验证|al*xr+bl*yr+cl|+|ar*xl+br*yl+cr|=0
            double errij = fabs(
                    imagePoints[0][i][j].x * lines[1][j][0]
                            + imagePoints[0][i][j].y * lines[1][j][1]
                            + lines[1][j][2])
                    + fabs(
                            imagePoints[1][i][j].x * lines[0][j][0]
                                    + imagePoints[1][i][j].y * lines[0][j][1]
                                    + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }

    cout << "average reprojection err = " << err / npoints << endl;
    error=err / npoints;

    // save intrinsic parameters
    FileStorage fs_write(intrinsics_yml, CV_STORAGE_WRITE);
    if (fs_write.isOpened()) {
        fs_write << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] << "M2"
                << cameraMatrix[1] << "D2" << distCoeffs[1];
        fs_write.release();
    } else {
        cout << "Error: can not save the intrinsic parameters\n";
        return false;
    }

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
            distCoeffs[1], imageSize, R, T, R1, R2, P1, P2, Q,
            CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs_write.open(extrinsics_yml, CV_STORAGE_WRITE);
    if (fs_write.isOpened()) {
        fs_write << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1
                << "P2" << P2 << "Q" << Q << "ROI1" << validRoi[0] << "ROI2"
                << validRoi[1];
        fs_write.release();
    } else {
        cout << "Error: can not save the extrinsic parameters\n";
        return false;
    }
    return true;
}

