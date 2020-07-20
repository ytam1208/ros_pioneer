#include <ros/ros.h>
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <mutex>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <cona_msgs/cona_BodyTrackerArray.h>
#include <cona_msgs/gui.h>
#include <cona_msgs/lsb.h>
#include <cona_msgs/GoalPose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
//#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "laser_tracker/Human.hpp"
#include "laser_tracker/FLAG.hpp"
#include "laser_tracker/Scan_receiver.hpp"



#define PI 3.14159265
#define RAD2DEG(x) ((x)*180./M_PI)


class Dynamixel
{
    public:
        struct Dynamixel_param
        {
            int min_vel, max_vel, center_vel, degree_center_vel;
            Dynamixel_param() : min_vel(300), max_vel(3795), center_vel(512), degree_center_vel(90)
            {

            }
        };
        Dynamixel_param Dynamixel_;

    Dynamixel()
    {

    }
    ~Dynamixel()
    {

    }
};

class laser_tracker
{
private:
    Scan_receiver sr;
    FLAG flag_laser;
    Human Target;
    Dynamixel Dynamixel_;
    struct Grid_param 
    {   
        int robot_col, robot_row;
        float mm2pixel;
        cv::Mat occup, free;
    };
    Grid_param grid; 

    struct param_data
    {
        int count_laser = 0;
        int out_count_laser = 0;
        int lsb_radius = 25;
        int target_threshold = 30;

    };
    param_data point_param;

    struct dynamixel_param
    {
        int min_vel, max_vel, center_val;
        dynamixel_param(): min_vel(100), max_vel(923), center_val(512)
        {

        }
    };
    dynamixel_param Dynamixel_param;

    struct SearchBoundary
    {
        cv::Point2f lsb_pt;
        float lsb_radius;
        std::vector<cv::Point2f> lsb_points;

        SearchBoundary()
        {
        }
    };

    ros::NodeHandle nh_;
    //Publisher
    ros::Publisher lsb_pub_;
    ros::Publisher goal_pub_;
	//Subscriber
    ros::Subscriber scan_sub_;
    //Survice Client
    ros::ServiceClient dynamixel_srv_;

    const float safty_radius = 900.0f;

    const int grid_row, grid_col, depth_row, depth_col;

    cv::Mat Grid;
    cv::Point2f lsb_points;
    std::string name_;
    std::vector<cv::Point2f> laser_pt, near_laser_pt;

public:
    laser_tracker(std::string _name) :
        name_(_name),
        grid_row(500), grid_col(500),
        depth_row(480), depth_col(640)
    {
		ROS_INFO("%s: Initializing", name_.c_str());
        initSubscriber();
        initPublisher();
        initServiceClient();


        Grid = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
        grid.occup = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));
        grid.free = cv::Mat(grid_row, grid_col, CV_8UC3, cv::Scalar(0,0,0));

        grid.robot_col = 250;
        grid.robot_row = 400;

        grid.mm2pixel = 50.0f/1000.0f;  // 1000mm(1m) -> 50 pixel, 20mm -> 1 pixel

        Target.lsb_radius = 20.0f;          // 300mm
        Target.lsb_pt.x = -1.0f;
        Target.lsb_pt.y = -1.0f;

        sr.parent_frame = "map";
        sr.child_frame = "laser";
        
        windowInitializer(true);
        InitDynamixel();
    }
    ~laser_tracker()
    {}

    void runLoop(void);
private:
    void initSubscriber(void);
    void initPublisher(void);
    void initServiceClient(void);
    void InitDynamixel(void);
    void windowInitializer(bool _flag);
    
    void scan_sick_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
    void make_grid(std::vector<cv::Point2f> &_laser_pt);
    void initTarget(std::vector<cv::Point2f> &_laser_pt, cv::Mat &_Grid, Human &_Target);
    void update_point(cv::Mat &_Grid, std::vector<cv::Point2f> & _near_laser_pt, Human &_Target);
    void MoveHead(int _value);
    void DisplayAll(bool _flag);

    void MoveWheel(cv::Point2f _target_grid_pt, float _target_degree, bool valid_safty);
    float RadiantoDegree(cv::Point2f _lsb_pt);
    int DegreetoPosition(float _degree);
    float velocity_ratio(cv::Point2f _target_grid_pt, bool valid_safty);
    bool safe_distance(cv::Point2f _target_grid_pt, float _target_degree, float _safty_radius, cv::Mat &_Grid);
};
