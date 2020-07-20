class Human
{
public:
    float lsb_radius;
    cv::Point2f lsb_pt;
    float target_degree = 25.0f;
    std::vector<cv::Point2f> lsb_points;

    struct Target_data
    {
        cv::Point2f target_pt;
        float target_degree = 0;
    };

    Target_data td;

    Human(/* args */)
    {
    }
    ~Human()
    {}
public:
    //TODO
    void reset_target()
    {
        lsb_pt.x = -1.0f;
        lsb_pt.y = -1.0f;
        lsb_radius = 25.0f;
        //target_degree = std::numeric_limits<float>INFINITY();
    }

};

