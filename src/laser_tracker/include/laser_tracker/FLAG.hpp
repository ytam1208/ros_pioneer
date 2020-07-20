class FLAG
{
    public:
        struct set_flag
        {
            //bool set_init_laser = false;
            bool search_target = false;
            bool out_target = false;
            bool init_on_target = false;
        };
        set_flag flag_param;
    
        struct valid_flag
        {
            bool valid_outdist = false;
        };
        valid_flag valid_param;

        struct goal_flag
        {
            bool go = false;
            bool stop = false;
            bool safty = false;
        };
        goal_flag goal_param;

    FLAG()
    {

    }
    ~FLAG()
    {

    }
};