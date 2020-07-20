#include "laser_tracker/laser_tracker.hpp"

void laser_tracker::initSubscriber(void)
{
    scan_sub_ = nh_.subscribe("/scan", 10, &laser_tracker::scan_rp_callback, this);
    imgOn_sub_ = nh_.subscribe("/cona_hf/ImgOn", 10, &laser_tracker::imgOn_callback, this);
    window_sub_ = nh_.subscribe("cona_hf/windowInit", 10, &laser_tracker::windowInit_callback, this);
    weighted_mean_sub_ = nh_.subscribe("cona_hf/weighted_mean", 10, &laser_tracker::weighted_mean_callback, this);
}

void laser_tracker::weighted_mean_callback(const geometry_msgs::Pose::ConstPtr &msg)
{
	cv::Point2f msg_pose; 
	msg_pose.x = msg->position.x;
	msg_pose.y = msg->position.y;

	Target.weighted_mean_pt.x = msg_pose.x;
	Target.weighted_mean_pt.y = msg_pose.y;
}

void laser_tracker::imgOn_callback(const std_msgs::Bool::ConstPtr &msg)
{
    bool msg_data = msg -> data;
    flag.data[FLAG::Name::imgOn].second = msg_data;

    windowInitializer(flag.data[FLAG::Name::window_init].second);
    ROS_INFO("Callback imgOn is Laser_tracker!: %d", msg_data);
}
void laser_tracker::scan_rp_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
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
    //publish_grid_points(laser_pt);
}

void laser_tracker::make_grid(std::vector<cv::Point2f> &_laser_pt)
{
    Grid.setTo(125);
    grid.occup.setTo(0);
    grid.free.setTo(0);

    std::vector<cv::Point2f> _tmp_laser_pt, near_pt, near_weight_pt;

    cv::Point2f l_center(grid.robot_col - sr.T.getY() * grid.mm2pixel,
                         grid.robot_row - sr.T.getX() * grid.mm2pixel);
    cv::Point2f grid_pt;

    for (int i = 0; i < (int)_laser_pt.size(); i++) {
        if (_laser_pt[i].x == 0.0f || _laser_pt[i].y == 0.0f)
            continue;
        cv::Point2f p(grid.robot_col - laser_pt[i].y * grid.mm2pixel,
                      grid.robot_row - laser_pt[i].x * grid.mm2pixel);
        bool valid_dist = true;
        if (p.x <= 0 || p.y <= 0 || p.x >= grid_col || p.y >= grid_row)
            valid_dist = false;
        if (valid_dist)
        {
            grid_pt.x = p.x;
            grid_pt.y = p.y;

            if (flag.get_flag(FLAG::target_selected) == true && Target.lsb_pt.x != -1.0f && Target.lsb_pt.y != -1.0f)
            {
                if (Target.weighted_mean_pt.x != -1.0f && Target.weighted_mean_pt.y != -1.0f)
                {
                    bool valid_target = false;
                    float dist = std::sqrt((Target.weighted_mean_pt.x - grid_pt.x) * (Target.weighted_mean_pt.x - grid_pt.x) 
                                        + (Target.weighted_mean_pt.y - grid_pt.y) * (Target.weighted_mean_pt.y - grid_pt.y));
                    if (dist <= Target.lsb_radius)  valid_target = true;
                    if (valid_target)
                        near_weight_pt.push_back(grid_pt);
                }
                else
                {
                    bool valid_target = false;
                    float dist = std::sqrt((Target.lsb_pt.x - grid_pt.x) * (Target.lsb_pt.x - grid_pt.x) 
                                        + (Target.lsb_pt.y - grid_pt.y) * (Target.lsb_pt.y - grid_pt.y));
                    if (dist <= Target.lsb_radius)  valid_target = true;
                    if (valid_target)
                        near_pt.push_back(grid_pt);
                }
            }
            cv::circle(grid.occup, p, 5, cv::Scalar(255, 255, 255), -1);
            cv::line(grid.free, l_center, p, cv::Scalar(255, 255, 255), 3);

            _tmp_laser_pt.push_back(p);
        }
    }

    Grid += grid.free;
    Grid -= grid.occup;

    cv::line(Grid, cv::Point(0, grid.robot_row), cv::Point(grid_col, grid.robot_row), cv::Scalar(0, 0, 255));
    cv::line(Grid, cv::Point(grid.robot_col, 0), cv::Point(grid.robot_col, grid_row), cv::Scalar(0, 0, 255));

    laser_pt.swap(_tmp_laser_pt);
    near_laser_pt.swap(near_pt);
    near_weighted_laser_pt.swap(near_weight_pt);
}

void laser_tracker::windowInit_callback(const std_msgs::Bool::ConstPtr &msg)
{
	bool msg_data = msg->data;
	flag.data[FLAG::Name::window_init].second = msg_data;

	windowInitializer(flag.data[FLAG::Name::window_init].second);	// true-> desktop_mode, false-> robot_mode

	ROS_INFO("Callback windowInit in laser_tracker!: %d", msg_data);
}

void laser_tracker::windowInitializer(bool _flag)
{
    cv::Point grid_pt;
    if(_flag == true)
    {
        grid_pt.x = 100;
        grid_pt.y = 500;
    }
    
    else
    {
        grid_pt.x = 100;
        grid_pt.y = 100;
    }

    cv::namedWindow("grid_laser");
    cv::moveWindow("grid_laser", grid_pt.x, grid_pt.y);
}

void laser_tracker::DisplayAll(bool _flag)
{
    if(_flag == true)
    {
        flag.set_flag_on(FLAG::Name::window_created);

        if(Target.lsb_pt.x != -1.0f && Target.lsb_pt.y != -1.0f)
        {
            cv::circle(Grid, Target.lsb_pt, Target.lsb_radius, cv::Scalar(0, 165, 255), 2);
            cv::circle(Grid, Target.lsb_pt, 8, cv::Scalar(0, 255, 0), 2);
        }

        cv::imshow("grid_laser", Grid);
        cv::waitKey(1);
    }
    else
    {
        if(flag.data[FLAG::Name::window_created].second == true)
        {
            cv::destroyWindow("grid_laser");
            flag.data[FLAG::Name::window_created].second == false;
        }
    }
}
void laser_tracker::SearchTarget(std::vector<cv::Point2f> &_laser_pt, cv::Mat &_Grid, std::vector<Human> &_observation, Human &_Target)
{
	flag.set_flag_off(FLAG::Name::target_selected);

	if(_laser_pt.size() == 0) return;
	cv::Mat tmp_Grid = _Grid.clone();

	std::pair<int, int> boundary_col;
	std::pair<int, int> boundary_row;

	float hori_mm = 900.0f;
	float verti_mm = 1200.0f;
	float hori_pixel = grid.mm2pixel * hori_mm;
	float verti_pixel = grid.mm2pixel * verti_mm;

	boundary_col.first = grid.robot_col - hori_pixel;		//col: 200 ~ 300
	boundary_col.second = grid.robot_col + hori_pixel;
	boundary_row.first = grid.robot_row;					//row: 250 ~ 300 
	boundary_row.second = grid.robot_row - verti_pixel;		 

	//vertical
	cv::line(_Grid, cv::Point(boundary_col.first, boundary_row.second), 
				cv::Point(boundary_col.first, boundary_row.first), cv::Scalar(255,0,0));
	cv::line(_Grid, cv::Point(boundary_col.second, boundary_row.second), 
				cv::Point(boundary_col.second, boundary_row.first), cv::Scalar(255,0,0));
	//horizontal
	cv::line(_Grid, cv::Point(boundary_col.first, boundary_row.second), 
				cv::Point(boundary_col.second, boundary_row.second), cv::Scalar(255,0,0));
	cv::line(_Grid, cv::Point(boundary_col.first, boundary_row.first), 
				cv::Point(boundary_col.second, boundary_row.first), cv::Scalar(255,0,0));

	int cnt_laser = 0;
	int target_ini_count = 30;
	float x_sum = 0.0f;
	float y_sum = 0.0f;
	std::vector<cv::Point2f> ini_near_pt;

	for(int i = 0; i < (int)_laser_pt.size(); i++)
	{
		bool valid_laser = false;
		float x = _laser_pt[i].x;
		float y = _laser_pt[i].y;
		if(boundary_col.first <= x && x <= boundary_col.second)
			if(boundary_row.second <= y && y <= boundary_row.first)
				valid_laser = true;
		if(valid_laser)
		{
			ini_near_pt.push_back(_laser_pt[i]);
			cnt_laser++;
		}
	}
	ROS_INFO("cnt_laser: %d", cnt_laser);
	bool valid_laserOn = false;
	if(cnt_laser >= target_ini_count)	valid_laserOn = true;
	if(valid_laserOn)
	{
		for(int i = 0; i < (int)ini_near_pt.size(); i++)
		{
			float x = ini_near_pt[i].x;
			float y = ini_near_pt[i].y;
			x_sum += x;
			y_sum += y;
		}
		if(cnt_laser != 0)
		{
			_Target.lsb_pt.x = x_sum / (float)cnt_laser;
			_Target.lsb_pt.y = y_sum / (float)cnt_laser;

			flag.set_flag_off(FLAG::Name::init_target);
			flag.set_flag_on(FLAG::Name::target_selected);
			//Publish target ini lsb point
			cona_msgs::lsb lsb_msgs;
			lsb_msgs.Target_grid_lsb.x = _Target.lsb_pt.x;
			lsb_msgs.Target_grid_lsb.y = _Target.lsb_pt.y;
			lsb_msgs.Target_radius_lsb = _Target.lsb_radius;
			lsb_pub_.publish(lsb_msgs);
			ROS_INFO("[initial]Publish lsb[x: %f, y: %f], radius[%f]", _Target.lsb_pt.x, _Target.lsb_pt.y, _Target.lsb_radius);
			//Publish target_on
			std_msgs::Bool target_on_msgs;
			target_on_msgs.data = true;
			target_on_pub_.publish(target_on_msgs);
			ROS_INFO("Publish target_on: %d!", target_on_msgs.data);
		}
	}

}

void laser_tracker::runLoop()
{
    ros::Rate r(10);
    do
    {
		SearchTarget(laser_pt, Grid, observation, Target);

        DisplayAll(flag.get_flag(FLAG::Name::imgOn));

        r.sleep();
        ros::spinOnce();
    } while (ros::ok());
    
}
