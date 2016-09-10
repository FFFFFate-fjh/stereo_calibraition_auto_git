#include<iostream>
#include<list>
#include<vector>
#include<stdio.h>
#include<fstream>
#include <time.h>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include<opencv2/opencv.hpp>
#include "stereo_calibration_func.h"
#include "camera_calibrate.h"

using namespace std;
using namespace cv;

void combineK(list<int> &Lis,vector<int> &Vec,int nK,int nN,int k);
void sorterror(vector< vector<double> > &errfile, int &cols);
void saveerrfile(vector< vector<double> > &errfile, int cols);

vector<Mat> img_left;                //总图片矩阵 以1对为单位
vector<Mat> img_right;
//vector<Mat> img_left_rgb;
//vector<Mat> img_right_rgb;
vector<Mat> img_cur_left;            //正在使用的图片矩阵
vector<Mat> img_cur_right;
//vector<Mat> img_cur_left_rgb;
//vector<Mat> img_cur_right_rgb;
vector<Mat> img_com_left;            //组合图片矩阵
vector<Mat> img_com_right;
//vector<Mat> img_com_left_rgb;
//vector<Mat> img_com_right_rgb;
vector<Mat> img_opt_left;            //最优化图片矩阵
vector<Mat> img_opt_right;
//vector<Mat> img_opt_left_rgb;
//vector<Mat> img_opt_right_rgb;

vector<int> dis_img_left;         //去除的图片
vector<int> dis_img_right;        //去除的图片
string filename;                  //写入文件名
vector< vector<double> > errfile; //统计文件
bool exitflag =false;             //停止循环
double exerr = 0;                 //预期误差

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout<<" Usage: stereo_calibration  num_of_imgs  expected_error."<<endl;
        return 1;
    }

    int64_t st = getTickCount();

    std::string param1(argv[1]);
    int img_num = atoi(param1.c_str());    //照片数量(照片以1对为单位)
    std::string param2(argv[2]);
    int img_cnum = img_num -1;              //标定照片数量(照片以1对为单位)
    exerr = atof(param2.c_str());
    time_t curtime = time(0);
    char cttemp[64];
    strftime( cttemp, sizeof(cttemp), "%Y%m%d%X",localtime(&curtime) );
    char filenamebuf[30];
    sprintf(filenamebuf, "./output/stereo_calibration_%s.txt", cttemp);
    filename=filenamebuf;

    list<int> img_list;

    for(int i=1;i<=img_num;i++)  //读左图
    {
        char addr_buf[100];
        char img_buf[100];
        sprintf(addr_buf, "./data/left/");
        sprintf(img_buf, "%s%d.jpg", addr_buf,i);
        string img_addr = img_buf;
        cout<<img_addr<<endl;
        Mat src=imread(img_addr,0);
        img_left.push_back(src);
        vector<Mat> tmp;
        tmp.push_back(src);
        string camera_param = "./data/camera_L.yml";
        if(!camera_calibrate(11, 8, tmp, camera_param))
        {
            dis_img_left.push_back(i);
        }
    }

    for(int i=1;i<=img_num;i++)  //读右图
    {
        char addr_buf[100];
        char img_buf[100];
        sprintf(addr_buf, "./data/right/");
        sprintf(img_buf, "%s%d.jpg", addr_buf,i);
        string img_addr = img_buf;
        cout<<img_addr<<endl;
        Mat src=imread(img_addr,0);
        img_right.push_back(src);
        vector<Mat> tmp;
        tmp.push_back(src);
        string camera_param = "./data/camera_R.yml";
        if(!camera_calibrate(11, 8, tmp, camera_param))
        {
            dis_img_right.push_back(i);
        }
    }

    cout<<"img size: "<<img_right.size()<<endl;

    for(int i=1;i<=img_num;i++)
    {
        img_list.push_back(i);
        for(int j=0; j<dis_img_left.size();j++)
        {
            if(i == dis_img_left[j])
            {
                 img_list.pop_back();
                 break;
            }
        }
        for(int j=0; j<dis_img_right.size();j++)
        {
            if(i == dis_img_right[j])
            {
                 img_list.pop_back();
                 break;
            }
        }
    }

    cout<<"discarded left image size: "<<dis_img_left.size()<<",  "<<"discarded left image NO: ";
    for(int j=0; j<dis_img_left.size();j++)
    {
        cout<<dis_img_left[j]<<"  ";
    }
    cout<<endl;
    cout<<"discarded right image size: "<<dis_img_right.size()<<",  "<<"discarded right image NO: ";
    for(int j=0; j<dis_img_right.size();j++)
    {
        cout<<dis_img_right[j]<<"  ";
    }
    cout<<endl;

    img_num = img_num -max(dis_img_left.size(),dis_img_right.size());
    img_cnum = img_cnum -max(dis_img_left.size(),dis_img_right.size());
    vector<int> img_cap(img_cnum);         //排序照片容量

    list<int>::iterator it;
    img_cur_left.clear();
    img_cur_right.clear();
    cout<<"current pics size:"<< img_list.size()<<endl;
    cout<<"current pics: ";
    for(it = img_list.begin();it != img_list.end();it++)
    {
        img_cur_left.push_back(img_left[*it-1]);
        img_cur_right.push_back(img_right[*it-1]);
        cout<<*it<<" ";
    }
    cout<<endl;

    string camera_l_param_file = "./data/camera_L.yml";
    cout<<"mono_calibrate..."<<endl;
    camera_calibrate(11, 8, img_cur_left, camera_l_param_file);

    string camera_r_param_file = "./data/camera_R.yml";
    cout<<"mono_calibrate..."<<endl;
    camera_calibrate(11, 8, img_cur_right, camera_r_param_file);

    combineK(img_list,img_cap,img_cnum,img_num,0); //所有照片顺序列表，所求照片数量向量，所求照片数量，照片总数
    int cols= img_cnum+1;
    sorterror(errfile,cols);
    for(int i=0; i < errfile.size(); i++)
    {
        for(int j=0; j<cols; j++)
        {
            cout << errfile[i][j] <<" ";
        }
        cout << endl;
    }
    saveerrfile(errfile,cols);

    while(img_cnum>8&&exitflag == false)   //最少8张图
    {
        img_list.clear();
        for(int i=0;i<img_cnum;i++)
        {
            img_list.push_back(errfile[0][i]);
        }

        list<int>::iterator it;
        cout<<"current pics size:"<< img_list.size()<<endl;
        cout<<"current pics: ";
        img_cur_left.clear();
        img_cur_right.clear();
//        img_cur_left_rgb.clear();
//        img_cur_right_rgb.clear();
        for(it = img_list.begin();it != img_list.end();it++)
        {
            img_cur_left.push_back(img_left[*it-1]);
            img_cur_right.push_back(img_right[*it-1]);
//            img_cur_left_rgb.push_back(img_left_rgb[*it-1]);
//            img_cur_right_rgb.push_back(img_right_rgb[*it-1]);
            cout<<*it<<" ";
        }
        cout<<endl;

        cout<<"img_cur_left size: "<<img_cur_left.size()<<endl;
//        cout<<"img_cur_left_rgb size: "<<img_cur_left_rgb.size()<<endl;

        string camera_l_param_file = "./data/camera_L.yml";
        cout<<"mono_calibrate..."<<endl;
        camera_calibrate(11, 8, img_cur_left, camera_l_param_file);
        string camera_r_param_file = "./data/camera_R.yml";
        cout<<"mono_calibrate..."<<endl;
        camera_calibrate(11, 8, img_cur_right, camera_r_param_file);
        cout<<endl;
        img_num = img_num-1;      //图片数量减1
        img_cnum = img_cnum -1;
        vector<int> img_cap2(img_cnum);
        errfile.clear();
        combineK(img_list,img_cap2,img_cnum,img_num,0); //所有照片顺序列表，所求照片数量向量，所求照片数量，照片总数

        cols= img_cnum+1;
        sorterror(errfile,cols);
        for(int i=0; i < errfile.size(); i++)
        {
            for(int j=0; j<cols; j++)
            {
                cout << errfile[i][j] <<" ";
            }
            cout << endl;
        }
        saveerrfile(errfile,cols);
        if(exitflag == true)
            break;
    }

    if(exitflag == true)
    {
        double error;
        string camera_l_param = "./data/camera_L.yml";
        cout<<"mono_calibrate..."<<endl;
        camera_calibrate(11, 8, img_opt_left, camera_l_param);
        string camera_r_param = "./data/camera_R.yml";
        cout<<"mono_calibrate..."<<endl;
        camera_calibrate(11, 8, img_opt_right, camera_r_param);
        cout<<"stereo_calibrate..."<<endl;
        stereo_calibrate(img_opt_left, img_opt_right, 11, 8, error);
    }
    else
    {
        cols= img_cnum;
        for(int i=0; i < cols; i++)
        {
            img_opt_left.push_back(img_left[errfile[0][i]-1]);
            img_opt_right.push_back(img_right[errfile[0][i]-1]);
//            img_opt_left_rgb.push_back(img_left_rgb[errfile[0][i]-1]);
//            img_opt_right_rgb.push_back(img_right_rgb[errfile[0][i]-1]);
            cout<<errfile[0][i]-1<<"  ";
        }
        cout<<endl;
        double error;
        string camera_l_param = "./data/camera_L.yml";
        cout<<"mono_calibrate..."<<endl;
        camera_calibrate(11, 8, img_opt_left, camera_l_param);
        string camera_r_param = "./data/camera_R.yml";
        cout<<"mono_calibrate..."<<endl;
        camera_calibrate(11, 8, img_opt_right, camera_r_param);
        cout<<"stereo_calibrate..."<<endl;
        stereo_calibrate(img_opt_left, img_opt_right, 11, 8, error);
    }

    int64_t et = getTickCount();
    cout<<"total time: "<<(et-st)*1000/getTickFrequency()/1000<<"s"<<endl;

    return 0;
}

void combineK(list<int> &Lis,vector<int> &Vec,int nK,int nN,int k)
{
    if(exitflag == false)
    {
        int nSize = Lis.size();
        list<int>::iterator iter;
        for(iter = Lis.begin();iter != Lis.end();iter++)
        {
            if(nN - nSize == 0)
                k = 0;
            Vec[k] = *iter;
            if( k + 1 == nK)
            {
                img_com_left.clear();
                img_com_right.clear();
//                img_com_left_rgb.clear();
//                img_com_right_rgb.clear();
                for(int j=0;j<nK;j++)
                {
                    cout<<Vec[j]<<"  ";
                    img_com_left.push_back(img_left[Vec[j]-1]);
                    img_com_right.push_back(img_right[Vec[j]-1]);
//                    img_com_left_rgb.push_back(img_left_rgb[Vec[j]-1]);
//                    img_com_right_rgb.push_back(img_right_rgb[Vec[j]-1]);
                }
                cout<<endl;

                double err;
                cout<<"stereo_calibrate..."<<endl;
                stereo_calibrate(img_com_left, img_com_right, 11, 8, err);

                vector<double> plugin;
                for(int k=0;k<Vec.size();k++)
                {
                    plugin.push_back(Vec[k]);
                }
                plugin.push_back(err);
                errfile.push_back(plugin);
                if(err < exerr)
                {
                    exitflag = true;
                    for(int k=0 ;k<img_com_left.size(); k++)
                    {
                       img_opt_left.push_back(img_com_left[k]);
                       img_opt_right.push_back(img_com_right[k]);
//                       img_opt_left_rgb.push_back(img_com_left_rgb[k]);
//                       img_opt_right_rgb.push_back(img_com_right_rgb[k]);
                    }
                }
            }
            else
            {
                list<int> Lisc( ++iter,Lis.end() );
                iter--;
                k++;
                combineK(Lisc,Vec,nK,nN,k);
                k--;
            }
        }
    }
}

void sorterror(vector< vector<double> > &errfile, int &cols)
{
    for(int i=0; i < errfile.size(); i++)
        for(int j=i+1; j<errfile.size(); j++)
        {
            if(errfile[j][cols-1]<errfile[i][cols-1])
            {
                for(int k=0; k< cols; k++)
                {
                    double temp;
                    temp=errfile[i][k];
                    errfile[i][k]=errfile[j][k];
                    errfile[j][k]=temp;
                }
            }
        }
}

void saveerrfile(vector< vector<double> > &errfile, int cols)
{
    ofstream f;
    f.open(filename.c_str(),ios::app);
    for(int i=0; i < errfile.size(); i++)
    {
        for(int j=0; j<cols; j++)
        {
            f << errfile[i][j] <<" ";
        }
        f << endl;
    }
    f << endl;
    f.close();
}
