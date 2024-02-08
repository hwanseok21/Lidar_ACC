#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <yolov5_delivery/BoundingBox.h>
#include <yolov5_delivery/BoundingBox_vector.h>
#include <pcl/point_types.h>
#include <vector>
#include "car_struct.h"

using namespace cv;

struct box_size{
	int id;
	int x_coord;
	int y_coord;
	int w_coord;
	int h_coord;
    int sort_id;
};

class Camera_Func{
    private:
        float rotation_vector[3] = { 1.222192636079691f, -1.238407760588357f, 1.200576479947369f};
	    float translation_vector[3] = {0.05382344141744489f ,-0.1270560244311811f, -1.863598947777617f};
        cv::Mat rotation_mat = Mat(3, 1, CV_32F, rotation_vector);
        cv::Mat translation_mat = Mat(3, 1, CV_32F, translation_vector);
        
        float distMat[9] = {790.465826f, 0.000000f, 647.075400f, 0.000000f, 1086.082578f, 381.249308f, 0.000000f, 0.000000f, 1.000000f};
        float distCoef[5] = {0.002261, -0.003432, -0.000268, 0.003129, 0.000000};
        cv::Mat cameraMatrix = Mat(3,3, CV_32F,distMat);
        cv::Mat distCoeffs = Mat(5,1, CV_32F,distCoef);

    public:
        bool is_in_box(int x, int y,int l,int r,int u,int d);
        double GetIOU(const Rect_<float> bb_test, const Rect_<float> bb_gt);
        bool check_box(const Rect_<float> box_lidar,const box_size &obj);
        bool check_box_2(const Point2d lidar_2d, const box_size &obj);
        bool matching(vector<PointXYZV> &center_,const vector<box_size> &boxes, vector<PointXYZV> &result_);
        bool matching_check(const PointXYZV &center_, const box_size &obj);
};

double Camera_Func::GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

bool Camera_Func::is_in_box(int x, int y,int l,int r,int u,int d){
    return ((l < x && x < r) && (u < y && y < d));
}

bool Camera_Func::check_box(const Rect_<float> box_lidar, const box_size &obj){
    int x ,y;
    int box_l,box_r,box_d,box_u;// l : 왼쪽 r : 오른쪽 u : 위 d : 아래 
    
    x = box_lidar.x;
    y = box_lidar.y;

    if((0< x && x < 1280) && (0 < y && y < 720)){
        //for(int j = 0; j < obj.size();j++){
            Rect_<float> box_cam = Rect_<float>(obj.x_coord, obj.y_coord, obj.w_coord, obj.h_coord);
            if(GetIOU(box_cam, box_lidar)>0.3)  return true;
        //}
    }
    return false;
}

bool Camera_Func::check_box_2(const Point2d lidar_2d, const box_size &obj){
    int x ,y;
    int box_l,box_r,box_d,box_u;// l : 왼쪽 r : 오른쪽 u : 위 d : 아래 
    
    x = lidar_2d.x;
    y = lidar_2d.y;

    if((0< x && x < 1280) && (0 < y && y < 720)){
        box_l = obj.x_coord;
        box_r = obj.x_coord + obj.w_coord;
        box_u = obj.y_coord;
        box_d = obj.y_coord+ obj.h_coord;
        if(is_in_box(x,y,box_l,box_r,box_u,box_d)) return true;
    }
    return false;
}

bool Camera_Func::matching(vector<PointXYZV> &center_, const vector<box_size> &boxes, vector<PointXYZV> &result_){
    vector<cv::Point3d>	point3D_top_left;
    vector<cv::Point3d>	point3D_down_right;

    vector<cv::Point2d>	point2D_top_left;
    vector<cv::Point2d>	point2D_down_right;

	for (int i = 0; i < center_.size(); i++) {
        double x1 = center_[i].x;
        double y1 = center_[i].y; +center_[i].height/2;
        double z1 = center_[i].z; +center_[i].z_height/2;

        double x2 = center_[i].x;
        double y2 = center_[i].y-center_[i].height/2;
        double z2 = center_[i].z-center_[i].z_height/2;

        point3D_top_left.emplace_back(cv::Point3d(x1, y1, z1));
        point3D_down_right.emplace_back(cv::Point3d(x2, y2, z2));
    }
	
    if (point3D_top_left.size() == 0) return false;
    
    cv::projectPoints(point3D_top_left, rotation_mat, translation_mat, cameraMatrix, distCoeffs, point2D_top_left);//왼쪽점들 3d->2d변환
    cv::projectPoints(point3D_down_right, rotation_mat, translation_mat, cameraMatrix, distCoeffs, point2D_down_right);//왼쪽점들 3d->2d변환

    // for(int i = 0; i < point2D_top_left.size();i++){
    //     double tpx = point2D_top_left[i].x;
	// 	double tpy = point2D_top_left[i].y;
	// 	double tpw = point2D_down_right[i].x-point2D_top_left[i].x;
	// 	double tph = point2D_down_right[i].y-point2D_top_left[i].y;
    //     const Rect_<float> box_lidar = Rect_<float>(tpx,tpy,tpw,tph);
    //     if(check_box(box_lidar, boxes)) 
    //         result_.emplace_back(center_[i]);trash:///CM_LC/include/camera_func/camera_func.hpp
    // }

    for(int i = 0; i < boxes.size();i++){
        for(int j = 0; j < point2D_top_left.size(); j++){
            double tpx = point2D_top_left[j].x;
            double tpy = point2D_top_left[j].y;
            double tpw = point2D_down_right[j].x-point2D_top_left[j].x;
            double tph = point2D_down_right[j].y-point2D_top_left[j].y;
            const Rect_<float> box_lidar = Rect_<float>(tpx,tpy,tpw,tph);
            //if(point3D_top_left[j].x > 3 && check_box_2(point2D_top_left[j], boxes[i])){
            if(point3D_top_left[j].x > 2 && check_box(box_lidar, boxes[i])){
                int id = boxes[i].sort_id;
                center_[j].id = id;
                cout <<"matching id: " << id << endl << endl;
                result_.emplace_back(center_[j]);
                break;
            }
        }
    }

    if(result_.size()>0) return true;

    return false;
}

bool Camera_Func::matching_check(const PointXYZV &center_, const box_size &obj){
    vector<cv::Point3d>	point3D_top_left;
    vector<cv::Point3d>	point3D_down_right;

    vector<cv::Point2d>	point2D_top_left;
    vector<cv::Point2d>	point2D_down_right;

    double x1 = center_.x;
    double y1 = center_.y+center_.height/2;
    double z1 = center_.z+center_.z_height/2;

    double x2 = center_.x;
    double y2 = center_.y-center_.height/2;
    double z2 = center_.z-center_.z_height/2;

    point3D_top_left.emplace_back(cv::Point3d(x1, y1, z1));
    point3D_down_right.emplace_back(cv::Point3d(x2, y2, z2));
    

	if (point3D_top_left.size() == 0) return false;

    cv::projectPoints(point3D_top_left, rotation_mat, translation_mat, cameraMatrix, distCoeffs, point2D_top_left);//왼쪽점들 3d->2d변환
    cv::projectPoints(point3D_down_right, rotation_mat, translation_mat, cameraMatrix, distCoeffs, point2D_down_right);//왼쪽점들 3d->2d변환

    double tpx = point2D_top_left[0].x;
    double tpy = point2D_top_left[0].y;
    double tpw = point2D_down_right[0].x-point2D_top_left[0].x;
    double tph = point2D_down_right[0].y-point2D_top_left[0].y;
    const Rect_<float> box_lidar = Rect_<float>(tpx,tpy,tpw,tph);

    if(point3D_top_left[0].x > 0.3 && check_box(box_lidar, obj)) return true;

    return false;
}