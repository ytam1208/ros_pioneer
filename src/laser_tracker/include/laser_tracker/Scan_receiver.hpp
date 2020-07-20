
class Scan_receiver
{
public:
    bool get_tf_flag;
    tf::Matrix3x3 R;
    tf::Vector3 T;
    std::string parent_frame, child_frame;
    tf::TransformListener listener;

    Scan_receiver() :
        get_tf_flag(false)
    {

    }
    ~Scan_receiver()
    {}

public:
    bool get_tf(void)
    {
		if(parent_frame.empty() || child_frame.empty())
		{
			ROS_ERROR("There is no frame name");
			return 0;
		}

		try
		{
			tf::StampedTransform tf_msg;
			ros::Time now = ros::Time::now();
			listener.lookupTransform(parent_frame, child_frame, ros::Time(0), tf_msg);

			R = tf::Matrix3x3(tf_msg.getRotation());
			T = tf::Vector3(
				tf_msg.getOrigin().x(),
				tf_msg.getOrigin().y(),
				tf_msg.getOrigin().z());
		}
		catch(...)
		{
			ROS_ERROR("Fail to get tf");
			return 0;
		}
		
		ROS_INFO("success to get tf");
		return 1;


    }
};

