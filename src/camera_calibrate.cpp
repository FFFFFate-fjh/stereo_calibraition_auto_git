/*
 * camera_calibrate.cpp
 *
 *  Created on: Aug 18, 2016
 *      Author: lzp
 */

#include "camera_calibrate.h"

//计算标定误差
static double computeReprojectionErrors(
		const vector<vector<Point3f> >& objectPoints,
		const vector<vector<Point2f> >& imagePoints, const vector<Mat>& rvecs,
		const vector<Mat>& tvecs, const Mat& cameraMatrix,
		const Mat& distCoeffs, vector<float>& perViewErrors) {
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int) objectPoints.size(); i++) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
				distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int) objectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

//计算正面视觉的棋盘角点坐标（考虑实际格子大小）
static void calcChessboardCorners(Size boardSize, float squareSize,
		vector<Point3f>& corners) {
	corners.resize(0);

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(
					Point3f(float(j * squareSize), float(i * squareSize), 0));

}

static bool runCalibration(vector<vector<Point2f> > imagePoints, Size imageSize,
		Size boardSize, float squareSize, float aspectRatio, int flags,
		Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs,
		vector<Mat>& tvecs, vector<float>& reprojErrs, double& totalAvgErr) {
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CV_CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0]);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize,
			cameraMatrix, distCoeffs, rvecs, tvecs,
			flags | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);
	///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

//	cout << distCoeffs << endl;
//	cout << cameraMatrix << endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, rvecs,
			tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

static void saveCameraParams(const string& filename, Size imageSize,
		Size boardSize, float squareSize, float aspectRatio, int flags,
		const Mat& cameraMatrix, const Mat& distCoeffs, double totalAvgErr) {
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CV_CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0) {
		sprintf(buf, "flags: %s%s%s%s",
				flags & CV_CALIB_USE_INTRINSIC_GUESS ?
						"+use_intrinsic_guess" : "",
				flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
				flags & CV_CALIB_FIX_PRINCIPAL_POINT ?
						"+fix_principal_point" : "",
				flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		cvWriteComment(*fs, buf, 0);
	}

	fs << "flags" << flags;
	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;
	fs << "avg_reprojection_error" << totalAvgErr;

}

static bool readStringList(const string& filename, vector<string>& l) {
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string) *it);
	return true;
}

static bool runAndSave(const string& outputFilename,
		const vector<vector<Point2f> >& imagePoints, Size imageSize,
		Size boardSize, float squareSize, float aspectRatio, int flags,
        Mat& cameraMatrix, Mat& distCoeffs) {
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
    double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, imageSize, boardSize, squareSize,
			aspectRatio, flags, cameraMatrix, distCoeffs, rvecs, tvecs,
			reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n",
			ok ? "Calibration succeeded" : "Calibration failed", totalAvgErr);

	if (ok)
		saveCameraParams(outputFilename, imageSize, boardSize, squareSize,
				aspectRatio, flags, cameraMatrix, distCoeffs, totalAvgErr);
	return ok;
}

bool camera_calibrate(int width, int height, vector<Mat> img, const string& camera_param_file) {
	Size boardSize, imageSize;
	float squareSize = 2.f; //棋盘格大小，单位自定。
	float aspectRatio = 1.f;
	Mat cameraMatrix, distCoeffs;

	int i, nframes;
	int flags = 0;
	vector<vector<Point2f> > imagePoints;
    //vector<string> imageList;

	boardSize.width = width;
	boardSize.height = height;
//    readStringList(inputFilename, imageList);
//	nframes = (int) imageList.size();

    nframes = img.size();
	for (i = 0;; i++) {
		Mat view, viewGray;

//		if (i < (int) imageList.size())
//			view = imread(imageList[i], 1);
//		cout << "read image: " << imageList[i] << endl;
//        cout<<img[i].rows<<img[i].cols<<endl;
        img[i].copyTo(view);
        imageSize = view.size();
//        cout<<"imageSize:"<<" "<<imageSize<<endl;
        vector<Point2f> pointbuf;
//        cvtColor(view, viewGray, COLOR_BGR2GRAY);

		bool found;

        found = findChessboardCorners(view, boardSize, pointbuf,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK
						| CV_CALIB_CB_NORMALIZE_IMAGE);

		// improve the found corners' coordinate accuracy
        cornerSubPix(view, pointbuf, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		if (found) {
			imagePoints.push_back(pointbuf);
			//cout << "found chess board " << i + 1 << endl;
		} else{
			cout << "No chess board corner was found!" << endl;
			return false;
		}

		if (imagePoints.size() >= (unsigned)nframes) {
			//cout << "Start runAndSave..." << endl;
			if (imagePoints.size() > 0){
				bool done = runAndSave(camera_param_file, imagePoints, imageSize, boardSize,
										squareSize, aspectRatio, flags, cameraMatrix,
                                        distCoeffs);
				if(!done) cout<<"runAndSave failed!"<<endl;
			}else{
				cout<<"in sufficient imagePoints!"<<endl;
				return false;
			}

			break;

		}
	}

	return true;
}
