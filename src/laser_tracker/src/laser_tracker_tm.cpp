#include "laser_tracker/laser_tracker.hpp"

void laser_tracker::initSubscriber(void)
{
    scan_sub_ = nh_.subscribe("/scan", 10, &laser_tracker::scan_sick_callback, this);
}

void laser_tracker::initPublisher(void)
{
    lsb_pub_ = nh_.advertise<cona_msgs::lsb>("/sick_laser/lsb", 10);
    goal_pub_ = nh_.advertise<cona_msgs::GoalPose>("/goalpose", 10);
}

void laser_tracker::initServiceClient(void)
{
    dynamixel_srv_ = nh_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench/dynamixel_command");
}



void laser_tracker::InitDynamixel(void)
{
    ROS_INFO("InitDynamixel");
    dynamixel_workbench_msgs::DynamixelCommand _srv1; 
    dynamixel_workbench_msgs::DynamixelCommand _srv2; 
    dynamixel_workbench_msgs::DynamixelCommand _srv3; 

    std::string addr_name1, addr_name2, addr_name3;
    addr_name1 = "CW_Compliance_Slope";
    addr_name2 = "CCW_Compliance_Slope";
    addr_name3 = "Moving_Speed";
    
    //CW Slope
   _srv1.request.id = 2;					
   _srv1.request.addr_name = addr_name1;
   _srv1.request.value = 0x80;

   if(dynamixel_srv_.call(_srv1))
       ROS_INFO("Call Dynamixel Compliance result: %d", _srv1.response.comm_result);
   else
       ROS_ERROR("Fail to call service DynamixelCommand");

	//CCW Slope
   _srv2.request.id = 2;
   _srv2.request.addr_name = addr_name2;
   _srv2.request.value = 0x80;
   if(dynamixel_srv_.call(_srv2))
       ROS_INFO("Call Dynamixel CCW_Compliance result: %d", _srv2.response.comm_result);
   else
       ROS_ERROR("Fail to call service DynamixelCommand");

	//Moving Speed
   _srv3.request.id = 2;
   _srv3.request.addr_name = addr_name3;
   _srv3.request.value = 0x3FF;
   if(dynamixel_srv_.call(_srv3))
       ROS_INFO("Call Dynamixel Speed result: %d", _srv3.response.comm_result);
   else
       ROS_ERROR("Fail to call service DynamixelCommand");

	MoveHead(90);

}

void laser_tracker::scan_sick_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if (!sr.get_tf_flag)
        sr.get_tf_flag = sr.get_tf();

    int size = std::min((int)msg->ranges.size(), 1440);
    float angle_min = msg->angle_min;
    float angle_max = msg->angle_max;
    float angle_increment = msg->angle_increment;

    float range_min = (float)msg->range_min;
    float range_max = (float)msg->range_max;

    std::vector<cv::Point2f> tmp_laser_pt;
    for (int i = 0; i < size; i++)
    {
        float val = msg->ranges[i];
        if (val <= range_min || val >= range_max || !std::isfinite(val) || val == 0.0)
            continue;

        float angle = angle_min + angle_increment * (float)i;
        float digree = angle * 180 / 3.14;
            float x = cos(angle) * val;
            float y = sin(angle) * val;
            tf::Vector3 p(x, y, 0);
            tf::Vector3 reprj_p = sr.R * p + sr.T;

            cv::Point2f tmp_pt;
            tmp_pt.x = 1000.0f * reprj_p.getX();
            tmp_pt.y = 1000.0f * reprj_p.getY();
            
            tmp_laser_pt.push_back(tmp_pt);
    }

    make_grid(tmp_laser_pt);
}

void laser_tracker::MoveHead(int _value)
{
    int GoalPosition_max = 923; //CCW
    int GoalPosition_min = 100; //CW
    
    if(_value < 0)    return;
    else ROS_INFO("MoveHEAD");

    int value = _value * 5.68;
    
    if(value > GoalPosition_max)
        value = GoalPosition_max;
    else if(value < GoalPosition_min)
        value = GoalPosition_min;

    dynamixel_workbench_msgs::DynamixelCommand _srv;

    std::string addr_name;
    addr_name = "Goal_Position";

    _srv.request.id = 2;
    _srv.request.addr_name = addr_name;
    _srv.request.value = value;

   if(dynamixel_srv_.call(_srv))
       ROS_INFO("Call Dynamixel degree result: %d", _srv.request.value);
   else
       ROS_ERROR("Fail to call service DynamixelCommand");
}


void laser_tracker::MoveWheel(cv::Point2f _target_grid_pt, float _target_degree, bool valid_safty)
{
    if(std::isinf(_target_degree))  return;


    float _grid_pt_x = _target_grid_pt.x;
    float _grid_pt_y = _target_grid_pt.y;

    std::pair<float, float> world_grid_point;

    float output_degree = _target_degree;
    float world_x_ori = -(_grid_pt_x - grid.robot_col) / (1000.0 * grid.mm2pixel);
    float world_z_ori = -(_grid_pt_y - grid.robot_row) / (1000.0 * grid.mm2pixel);
    world_grid_point.first = world_x_ori;
    world_grid_point.second = world_z_ori;
    bool output_detour_on = false;

    cona_msgs::GoalPose msg_goal_pub;
    msg_goal_pub.target_x_mm_from_robot = world_grid_point.second * 1000.0f;
	msg_goal_pub.target_y_mm_from_robot = world_grid_point.first * 1000.0f;
	msg_goal_pub.target_degree_from_robot = output_degree;
	msg_goal_pub.vel_rate = velocity_ratio(_target_grid_pt, valid_safty);
	msg_goal_pub.detour_on = output_detour_on;

	goal_pub_.publish(msg_goal_pub);
	ROS_INFO("Publish GoalPose: [x: %f, y: %f][degree: %f][vel rate: %f]", 
	msg_goal_pub.target_x_mm_from_robot, msg_goal_pub.target_y_mm_from_robot, msg_goal_pub.target_degree_from_robot, msg_goal_pub.vel_rate);
}

bool laser_tracker::safe_distance(cv::Point2f _target_grid_pt, float _target_degree, float _safty_radius, cv::Mat &_Grid)
{
	float _mm2pixel = _safty_radius * grid.mm2pixel;				//1400mm -> 80pixel
    bool valid_target = false;

    int robot_col = grid.robot_col;
    int robot_row = grid.robot_row;

    float dist = std::sqrt((_target_grid_pt.x - (float)robot_col) * (_target_grid_pt.x - (float)robot_col)
                            + (_target_grid_pt.y - (float)robot_row) * (_target_grid_pt.y - (float)robot_row));
    if(std::fabs(_target_degree) <= 90.0f)
    {
        if(dist >= _mm2pixel)   //1600mm
            valid_target = true;
        else
            valid_target = false;
    }
    
    cv::Point2f robot_pt(robot_col, robot_row);
    cv::circle(_Grid, robot_pt, _mm2pixel, cv::Scalar(0,0,255), 1);

    return valid_target;
}


float laser_tracker::velocity_ratio(cv::Point2f _target_grid_pt, bool valid_safty) //속도 비율
{
    float dist = std::sqrt((_target_grid_pt.x - (float)grid.robot_col) * (_target_grid_pt.x - (float)grid.robot_col)
    + (_target_grid_pt.y - (float)grid.robot_row) * (_target_grid_pt.y - (float)grid.robot_row));

    float vel_ratio = 0.00035;
    float output_vel_rate = vel_ratio * (dist / grid.mm2pixel);

	if(output_vel_rate <= 0.2f)			//min
		output_vel_rate = 0.2f;

	if(output_vel_rate >= 1.0f)			//max
		output_vel_rate = 1.0f;

    if(valid_safty == false)
        output_vel_rate = 0.0f;

    return output_vel_rate;
}    

void laser_tracker::make_grid(std::vector<cv::Point2f> &_laser_pt)
{
    Grid.setTo(125);
    grid.occup.setTo(0);
    grid.free.setTo(0);

    std::vector<cv::Point2f> _tmp_laser_pt, near_pt;

    //grid(500, 500), grid_robot (250, 350)
    cv::Point2f center_line(grid.robot_col - sr.T.getY() * grid.mm2pixel,
                            grid.robot_row - sr.T.getX() * grid.mm2pixel);
    cv::Point2f grid_pt;

    for (int i = 0; i < (int)_laser_pt.size(); i++)
    {
        if (_laser_pt[i].x == 0.0f || _laser_pt[i].y == 0.0f)   continue;
        cv::Point2f pix(grid.robot_col - _laser_pt[i].y * grid.mm2pixel,
                            grid.robot_row - _laser_pt[i].x * grid.mm2pixel);

        bool valid_dist = true;
        if (pix.x <= 0 || pix.y <= 0 || pix.x >= grid_col || pix.y >= grid_row)
            valid_dist = false;
        if (valid_dist)
        {
            grid_pt.x = pix.x;
            grid_pt.y = pix.y;
            
            if(flag_laser.flag_param.search_target == true && Target.lsb_pt.x != -1.0f && Target.lsb_pt.y != -1.0f)
            {
                    bool valid_target = false;
                    float dist = std::sqrt((Target.lsb_pt.x - grid_pt.x) * (Target.lsb_pt.x - grid_pt.x) 
                                            + (Target.lsb_pt.y - grid_pt.y) * (Target.lsb_pt.y - grid_pt.y));
                    if(dist <= Target.lsb_radius)       valid_target = true;
                    if(valid_target)        near_pt.push_back(grid_pt);
            }
            
            cv::circle(grid.occup, pix, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(grid.free, center_line, pix, cv::Scalar(255, 255, 255), 3);
         
            _tmp_laser_pt.push_back(pix);
        }
    }
    Grid += grid.free;
    Grid -= grid.occup;

    cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid_col, grid.robot_row), cv::Scalar(255, 0, 0));
    cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid_row), cv::Scalar(255, 0, 0));

    laser_pt.swap(_tmp_laser_pt);
    near_laser_pt.swap(near_pt);                 //가까운 레이저 포인터
}

void laser_tracker::initTarget(std::vector<cv::Point2f> &_laser_pt, cv::Mat &_Grid, Human &_Target)
{
    flag_laser.flag_param.search_target = false;
    if(_laser_pt.size() == 0)
    {
        // ROS_ERROR("NO_initTarget laser_point!!!");
        return;
    }
    
    cv::Point2f rect_1(grid.robot_col - 50, grid.robot_row - 50);
    cv::Point2f rect_2(grid.robot_col + 50, grid.robot_row);    
    cv::rectangle(_Grid, rect_1, rect_2, cv::Scalar(255, 255, 0), 3);

    std::pair<int, int> boud_col;
    std::pair<int, int> boud_row;

    float hori_mm = 900.0f;
    float verti_mm = 1200.0f;
    float hori_pixel = grid.mm2pixel * hori_mm;
    float verti_pixel = grid.mm2pixel * verti_mm;

    boud_col.first = grid.robot_col - hori_pixel;
    boud_col.second = grid.robot_col + hori_pixel;
    boud_row.first = grid.robot_row - verti_pixel;
    boud_row.second = grid.robot_row + verti_pixel;

    point_param.count_laser = 0;
    float point_x_total = 0.0f;
    float point_y_total = 0.0f;
    std::vector<cv::Point2f> initTarget_near_pt;

    for(int i = 0; i < _laser_pt.size(); i++)
    {
        bool valid_laser = false;
        float x = _laser_pt[i].x;
        float y = _laser_pt[i].y;
        if(boud_col.first <= x && x <= boud_col.second)
            if(boud_row.first <= y && y <= boud_row.second)
                valid_laser = true;
        else
        {
            flag_laser.flag_param.out_target = true;
            flag_laser.flag_param.search_target = true;
            flag_laser.goal_param.go = true;
        }

        if(valid_laser)
        {
            initTarget_near_pt.push_back(_laser_pt[i]);
            point_param.count_laser++;
        }

    }

    bool valid_laserOn = false;
    if(point_param.count_laser >= point_param.target_threshold)  valid_laserOn = true;
    if(valid_laserOn)
    {
        for(int i = 0; i < initTarget_near_pt.size(); i++)
        {
            float x = initTarget_near_pt[i].x;
            float y = initTarget_near_pt[i].y;

            point_x_total += x;
            point_y_total += y;
        }
        if(point_param.count_laser != 0)
        {
            _Target.lsb_pt.x = point_x_total / (float)point_param.count_laser;
            _Target.lsb_pt.y = point_y_total / (float)point_param.count_laser;

            flag_laser.flag_param.search_target = true;
            flag_laser.flag_param.init_on_target = true;
            ROS_INFO("Init Target!");

            cona_msgs::lsb lsb_msgs;
            lsb_msgs.Target_grid_lsb.x = _Target.lsb_pt.x;
            lsb_msgs.Target_grid_lsb.y = _Target.lsb_pt.y;
            lsb_msgs.Target_radius_lsb = _Target.lsb_radius;
            lsb_pub_.publish(lsb_msgs);
        }
    }
}

void laser_tracker::update_point(cv::Mat &_Grid, std::vector<cv::Point2f> & _near_laser_pt, Human &_Target)
{
    if((int)_near_laser_pt.size() == 0)
    {
        // ROS_ERROR("NO near_laser point!");
        // ROS_WARN("_near_laser_pt.size = %d", (int)_near_laser_pt.size());       
        return;
    }
    std::vector<cv::Point2f> tmp_near_laser_pt;
    
    tmp_near_laser_pt = _near_laser_pt;

    float point_x_total = 0.0f;
    float point_y_total = 0.0f;
    int pt_size = (int)tmp_near_laser_pt.size();

    int Degree = 0;

    std::vector<cv::Point2f> tmp_lsb_pt;

    for(int i = 0; i < pt_size; i++)
    {
        float x = tmp_near_laser_pt[i].x;
        float y = tmp_near_laser_pt[i].y;

        point_x_total += x;
        point_y_total += y;

        cv::Point2f lsb_p;

        lsb_p.x = x;
        lsb_p.y = y;

        tmp_lsb_pt.push_back(lsb_p);
    }
    if(pt_size != 0)
    {
        _Target.lsb_pt.x = point_x_total / (float)pt_size;
        _Target.lsb_pt.y = point_y_total / (float)pt_size;
    }
    _Target.lsb_points.swap(tmp_lsb_pt);
    
    flag_laser.goal_param.safty = safe_distance(_Target.lsb_pt, _Target.target_degree, safty_radius, _Grid);

    cona_msgs::lsb lsb_msgs;
    lsb_msgs.Target_grid_lsb.x = _Target.lsb_pt.x;
    lsb_msgs.Target_grid_lsb.y = _Target.lsb_pt.y;
    lsb_msgs.Target_radius_lsb = _Target.lsb_radius;

    for(int i = 0; i < (int)_Target.lsb_points.size(); i++)
    {
        geometry_msgs::Pose _pose;
        _pose.position.x = _Target.lsb_points[i].x;
        _pose.position.y = _Target.lsb_points[i].y;
        lsb_msgs.Target_grid_pts.poses.push_back(_pose);
    }
    lsb_pub_.publish(lsb_msgs);
}

void laser_tracker::windowInitializer(bool _flag)
{
    cv::Point grid_pt;
    if (_flag == true)
    {
        grid_pt.x = 500;
        grid_pt.y = 500;
    }
    else
    {
        grid_pt.x = 500;
        grid_pt.y = 500;
    }

    cv::namedWindow("grid_laser");
    cv::moveWindow("grid_laser", grid_pt.x, grid_pt.y);
}

void laser_tracker::DisplayAll(bool _flag)
{
    if(Target.lsb_pt.x != -1.0f && Target.lsb_pt.y != -1.0f)
    {
        cv::circle(Grid, Target.lsb_pt, point_param.lsb_radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(Grid, Target.lsb_pt, 8, cv::Scalar(0, 255, 0), 2);
    }

    cv::imshow("grid_laser", Grid);
    cv::waitKey(1);
}

float laser_tracker::RadiantoDegree(cv::Point2f _lsb_pt)
{
    float x, y;
    cv::Point2f grid_tf_target;
    grid_tf_target.x = _lsb_pt.x;
    grid_tf_target.y = _lsb_pt.y + (100.0f * grid.mm2pixel);
    
    x = (grid_tf_target.x - (float)grid.robot_col);
    y = ((float)grid.robot_row - grid_tf_target.y);

    float body_radian = std::atan2(y, x);
    float body_angle = body_radian * (180 / M_PI);
    Target.td.target_degree = body_radian;
    
}

int laser_tracker::DegreetoPosition(float _degree)
{
    int curr_dynami_position = 512.0f + (512.0f / 150) * _degree;
    bool valid_position = true;
    if(curr_dynami_position < Dynamixel_param.min_vel)  valid_position = false;
    if(curr_dynami_position > Dynamixel_param.max_vel)  valid_position = false;

    if(valid_position)
        return  curr_dynami_position;
    else
        return -1;
}

void laser_tracker::runLoop()
{
    ros::Rate r(10);
    do
    {
        if(flag_laser.flag_param.init_on_target == false)
            initTarget(laser_pt, Grid, Target);

        if(flag_laser.flag_param.search_target == true && flag_laser.flag_param.init_on_target == true)
            {
                update_point(Grid, near_laser_pt, Target);
                RadiantoDegree(Target.lsb_pt);
                int target_position = Target.td.target_degree;
                //int target_position = DegreetoPosition(Target.td.target_degree);
                MoveHead(target_position);

                if(flag_laser.goal_param.go == true)
                    MoveWheel(Target.lsb_pt, Target.target_degree, flag_laser.goal_param.safty);
            }

        DisplayAll(true);
        
        r.sleep();
        ros::spinOnce();
    } while (ros::ok());
}