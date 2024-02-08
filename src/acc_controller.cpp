#include <iostream>
#include <string>
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <waypoint_maker/State.h>	

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include "lidar_lib/lidar_lib.hpp"
#include "acc_controller/jhs_msg.h"
#include "acc_controller/jhs_child.h"

using namespace std;				

//=======================================GLOBAL================================================
boost::shared_ptr<Lidar_Func> LF; //lidar_func
boost::shared_ptr<Camera_Func> CF; //camera_func
ros::Publisher pub_center, lane_flag; //publisher
double z_max_,x_max_,x_min_,y_max_,y_min_; //ROI param
double roi[5]; //0: z_max, 1: x_max, 2: x_min, 3: y_max, 4:; y_min
double lpf_offset_, ang_vel_offset_;
vector<PointXYZV> ex_car_; ros::Time ex_time_; double cur_vx_, cur_vy_, ang_vel_, ang_acc_; //to get speed
vector<int> car_ind ; //stored car idsss
deque<double> speed_avg;
int ex_car_id_=-2; 

bool cmp(const PointXYZV &x1 ,const PointXYZV &x2){
  return sqrt(x1.x*x1.x+x1.y*x1.y) < sqrt(x2.x*x2.x+x2.y*x2.y);
}

double LPF(double cur_data, double prev_lpf)
{
    double alpha = lpf_offset_;
    double lpf_data;
    lpf_data = alpha * prev_lpf + (1 - alpha) * cur_data;
	//if(prev_lpf != 0 && lpf_data > prev_lpf*1.2) return prev_lpf;
    return lpf_data;
}

double movAvgFilter(std::deque<double>& x_n, double x_meas) {
    int n = x_n.size();
    for (int i = 0; i < n - 1; i++) {
        x_n[i] = x_n[i + 1];
    }
    x_n[n - 1] = x_meas;
    double sum = 0.0;
    for (int i = 0; i < n; i++) {
        sum += x_n[i];
    }
    double x_avg = sum / n;
    return x_avg;
}

void get_speed(vector<PointXYZV> &car_pts, double dt){
	if (ex_car_.size()==0) return;

	//dt = 0.05;
	for (int i = 0; i < car_pts.size(); i++){
		for (int j = 0; j < ex_car_.size(); j++){
			if(car_pts[i].id == ex_car_[j].id){ 
				if (ex_car_id_ == -2) ex_car_id_ = car_pts[i].id; //@@@@@@@@@@@@@@@@
				double dx = car_pts[i].x - ex_car_[j].x;
				//double dy = car_pts[i].y - ex_car_[j].y;
				double dy = 0;

				//cout << "ang_vel_ : " << ang_vel_ << endl;
				double cur_vel = sqrt(pow(dx/dt + cur_vx_,2)+pow(dy/dt + cur_vy_, 2)); //: ex_car_[j].vel;
				//double cur_vel =floor((dx/dt+cur_vx_));
				double ex_vel = ex_car_[j].vel;
				if (dx > 0) car_pts[i].vel = LPF(cur_vel , ex_vel);

				if(car_pts.size()==1){ //@@@@@@@@@@@@@@@@
					if (ex_car_id_ != car_pts[i].id) { cout <<"another car" << endl; speed_avg.clear(); }
					speed_avg.emplace_back(car_pts[i].vel);
					if (speed_avg.size()>=20){ 
						speed_avg.pop_front();
					}
					if (dx > 0) car_pts[i].vel = movAvgFilter(speed_avg, car_pts[i].vel);
					ex_car_id_ = car_pts[i].id;
				}

				cout << "car_vel: "<< car_pts[i].vel << endl;
				cout << "car_id: "<< car_pts[i].id << endl;
			}
		}
	}
}

void push_back_car_ind(vector<int> &car_ind, int id){
	if (id == -1) return;
	for (int i = 0; i < car_ind.size(); i++){
		if(car_ind[i]==id) return;
	}
	car_ind.emplace_back(id);
}

void callback_all(const sensor_msgs::PointCloud2ConstPtr &cloud_in, const yolov5_delivery::BoundingBox_vector::ConstPtr &darknet_msg){
	
	double start = ros::Time::now().toSec();
//-------------------------------darknet callback-------------------------------
	vector<box_size> boxes_;
  	int size = darknet_msg->delivery.size();
	box_size box;
    for(int i=0; i<size; i++){
    	yolov5_delivery::BoundingBox bounding_box = darknet_msg->delivery[i];
		if (bounding_box.id == 0){ //if car
			box.id = bounding_box.id;
			box.x_coord = bounding_box.x;
			box.y_coord = bounding_box.y;
			box.w_coord = bounding_box.w;
			box.h_coord = bounding_box.h;

      	if(bounding_box.w*bounding_box.h<150000){
			boxes_.emplace_back(box);
		}
   		}
    }
	//LF->CameraSORT(boxes_);

//-------------------------------lidar callback----------------------------------
	pcl::PointCloud<PointType>::Ptr pc_curr(new pcl::PointCloud<PointType>);
    pcl::fromROSMsg(*cloud_in, *pc_curr);

	vector<PointXYZV> center_; 
	string frame_id= "Lidar_64_1";

	LF->clustering(pc_curr, center_,roi);
	sort(center_.begin(),center_.end(),cmp);
	//LF->TestSORT(center_);

//-------------------------------mathing with camera bounding box--------------------------------------
	vector<PointXYZV> car_pts;
	for (int i = 0 ; i< boxes_.size(); i++){
		for (int j = 0 ; j < center_.size() ; j++){
			if (CF->matching_check(center_[j],boxes_[i])){
				car_pts.emplace_back(center_[j]);
				push_back_car_ind(car_ind, center_[j].id);
				center_[j].id = -1;				
				break;
			}
		}
	}
				
	for (int j = 0 ; j<center_.size() ; j++){
		for (int k = 0 ; k <car_ind.size() ; k++){
			if (center_[j].id == car_ind[k]) {
				cout << "matching id: "<< center_[j].id << endl;
				if(fabs(center_[j].x)<0.5 && fabs(center_[j].y)<1) continue;
				car_pts.emplace_back(center_[j]);
				break;
			}
		}
	}
	
	sort(car_pts.begin(),car_pts.end(),cmp);
	LF->TestSORT(car_pts);
	
//--------------------------------tracking && get speed-------------------------------------------------
	ros::Time cur_time = cloud_in->header.stamp;
	//ros::Time cur_time = ros::Time::now();
	ros::Duration dt_ = cur_time - ex_time_;
	double dt = dt_.toSec();

	get_speed(car_pts, dt);
	
	ex_time_ = cur_time;

//----------------------------store ex_car state && pub car state---------------------------------------
	acc_controller::jhs_msg jhs;
	PointXYZV tmp;
	ex_car_.clear();
	for (int i = 0 ; i < car_pts.size(); i++){
		float x = car_pts[i].x + car_pts[i].width/2;
		float y = car_pts[i].y;
		int id = car_pts[i].id;
		double vel = car_pts[i].vel;

		tmp.x = x - car_pts[i].width/2;
		tmp.y = y;
		tmp.id = id;
		tmp.vel = vel;
		ex_car_.emplace_back(tmp);

		acc_controller::jhs_child car;
		car.x = x;
		car.y = y;
		car.id = id;
		car.speed = vel;
		car.is_on_lane = true;

		jhs.obst.emplace_back(car);
	}
	lane_flag.publish(jhs);

//------------------------------visualize_rviz---------------------------
	LF->rviz_clear(pub_center, frame_id, cloud_in->header.stamp);
	LF->visualize_rviz(car_pts, pub_center, frame_id, cloud_in->header.stamp);

//----------------------------END----------------------------------------
	if(car_ind.size()>500) car_ind.clear();
	double end = ros::Time::now().toSec();

//------------------------------COUT-------------------------------------
	// cout << "OBJ_SIZE: "<< center_.size()<< endl;
	// cout << "CAR: "<< car_pts.size() << endl;
	// cout << "BOX: "<< boxes_.size() << endl;
	// cout << " (running_time: " << end-start << " sec)" << endl;
	cout << "=======================================" << endl;
}

//-------------------------------my car speed callback-------------------------------
void SpeedCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr &course_msg){
	cur_vx_ = course_msg->drive.speed;
	cur_vy_ = 0;
}

int main(int argc, char**argv) {
    ros::init(argc, argv, "acc_controller_node");
    ros::NodeHandle nh;
	LF.reset(new Lidar_Func(&nh));
	CF.reset(new Camera_Func());

	//setup
	nh.param("/acc_controller/z_max", z_max_, 2.0);
	nh.param("/acc_controller/x_max", x_max_, 30.0);
	nh.param("/acc_controller/x_min", x_min_, -1.0);
	nh.param("/acc_controller/y_max", y_max_, 0.0);
	nh.param("/acc_controller/y_min", y_min_, 0.0);
	nh.param("/acc_controller/lpf_offset", lpf_offset_, 0.3);
	nh.param("/acc_controller/ang_vel_offset", ang_vel_offset_, 0.3);

	ROS_INFO("z_max: %f", z_max_);
	ROS_INFO("x_max: %f", x_max_);
	ROS_INFO("x_min: %f", x_min_);
	ROS_INFO("y_max: %f", y_max_);
	ROS_INFO("y_min: %f", y_min_);
	ROS_INFO("lpf_offset: %f", lpf_offset_);
	ROS_INFO("ang_vel_offset: %f", ang_vel_offset_);

	roi[0]=z_max_;
	roi[1]=x_max_;
	roi[2]=x_min_;
	roi[3]=y_max_;
	roi[4]=y_min_;

    pub_center  = nh.advertise<visualization_msgs::MarkerArray>("/objects", 1, true);
	lane_flag = nh.advertise<acc_controller::jhs_msg>("/JHS", 10, true);

	//callback
	ros::Subscriber sub_speed = nh.subscribe("course", 1, &SpeedCallback);

	message_filters::Subscriber <sensor_msgs::PointCloud2> sub_cloud(nh, "/demo/nonground", 100);
	message_filters::Subscriber <yolov5_delivery::BoundingBox_vector> sub_darknet(nh, "/boundingbox", 6);

	typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::PointCloud2, yolov5_delivery::BoundingBox_vector> MySyncPolicy;
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cloud, sub_darknet);
	sync.registerCallback(boost::bind(&callback_all, _1, _2));


    ros::Rate loop_rate(20);
    while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
