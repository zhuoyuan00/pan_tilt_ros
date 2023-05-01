#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pan_tilt_msgs/PanTiltCmdDeg.h"
#include "pan_tilt_msgs/PanTiltStatus.h"
// #include "pan_tilt_driver/PanTiltDriver.h"

bool pub_traj = false;
float init_yaw_, init_pitch_, pitch_gap_;
int rotation_speed_;
pan_tilt_msgs::PanTiltCmdDeg set_traj;

void reset_pose(pan_tilt_msgs::PanTiltCmdDeg &reset_)
{
    reset_.yaw = 0;
    reset_.pitch = 0;
    reset_.speed = 10;
    ROS_INFO("reset the pose");
}
void check_dest(const pan_tilt_msgs::PanTiltStatus &msg)
{
    //    ROS_INFO("reseiving status, yaw:[%f] pitch:[%f] speed[%i]",msg.yaw_now,msg.pitch_now,msg.speed);
    if (fabs(msg.yaw_now - set_traj.yaw) < 0.05 && fabs(msg.pitch_now - set_traj.pitch) < 0.05)
    {
        ROS_INFO("reached destination, next point will be published", msg.yaw_now, msg.pitch_now, msg.speed);
        pub_traj = true;
    }
}
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");

    ros::init(argc, argv, "trajPubNode");
    ros::NodeHandle trajPub;
    ros::Publisher traj_pub = trajPub.advertise<pan_tilt_msgs::PanTiltCmdDeg>("/pan_tilt_cmd_deg", 1); // pan_tilt_traj
    //    std::cout<<"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
    ros::Subscriber state_sub = trajPub.subscribe("/pan_tilt_status", 1, check_dest);

    //    get_ros_parameter<std::string>(m_ros_node_handle, "/LiDAR_pointcloud_topic", LiDAR_pointcloud_topic, std::string("/laser_cloud_flat") );

    trajPub.param<float>("common/init_yaw", init_yaw_, 60);
    trajPub.param<float>("common/init_pitch", init_pitch_, -20);
    trajPub.param<int>("common/rotation_speed", rotation_speed_, 5);
    // private_nh_.param<std::string>("top_pitch", top_pitch_, "50");
    trajPub.param<float>("common/pitch_gap", pitch_gap_, 20);
    ROS_INFO("start to  set the trajectory");

    reset_pose(set_traj);
    ros::Duration(1).sleep();
    traj_pub.publish(set_traj);
    // ros::Duration(1).sleep();
    // traj_pub.publish(set_traj);
    bool first_time = true;
    while (ros::ok())
    {
        if (first_time)
        {
            ROS_INFO("reseting, flag now is : [%i]", pub_traj);
            first_time = false;
        }
        ros::spinOnce();
        if (pub_traj == true)
        {
            ROS_INFO("flag_changed");
            pub_traj = false;
            break;
        }
    }

    float last_yaw = 0;
    // ros::Rate r(0.05);

    while (ros::ok())
    {
        ROS_INFO("rosok");
        // set initial pose
        set_traj.yaw = init_yaw_;
        set_traj.pitch = init_pitch_;
        set_traj.speed = rotation_speed_;
        if (set_traj.yaw > 60.0 || set_traj.yaw < -60.0 || set_traj.pitch > 60.0 || set_traj.pitch < -60.0 || set_traj.speed > 30 || set_traj.speed <= 0)
        {
            ROS_WARN_STREAM("Input param error, yaw:[-60, 60] pitch:[-60, 60] speed[1, 30]");
            return 0;
        }
        traj_pub.publish(set_traj);
        ROS_INFO("set init pose : [%f,%f,%i]", set_traj.yaw, set_traj.pitch, set_traj.speed);
        while (ros::ok())
        {
            ros::spinOnce();
            if (pub_traj == true)
            {
                pub_traj = false;
                break;
            }
        }
        // r.sleep();

        // initial pose 2 start pose
        set_traj.yaw = 60;
        set_traj.pitch = init_pitch_;
        set_traj.speed = rotation_speed_;
        traj_pub.publish(set_traj);
        ROS_INFO("start place : [%f,%f,%i]", set_traj.yaw, set_traj.pitch, set_traj.speed);
        ROS_INFO("flag now is : [%i]", pub_traj);
        while (ros::ok())
        {
            ros::spinOnce();
            if (pub_traj == true)
            {
                ROS_INFO("finish, flag changed");
                pub_traj = false;
                break;
            }
        }
        // r.sleep();

        while (set_traj.pitch <= -init_pitch_)
        {
            if (set_traj.yaw == -60.0 && last_yaw == 60)
            {
                ROS_INFO("往下"); //, set_traj.yaw:[%f]  last_yaw:[%f]",set_traj.yaw,last_yaw
                last_yaw = set_traj.yaw;
                set_traj.pitch = set_traj.pitch + pitch_gap_;
            }
            else if (set_traj.yaw == -60.0 && last_yaw == -60)
            {
                ROS_INFO("往左");
                last_yaw = set_traj.yaw;
                set_traj.yaw = 60.0;
            }
            else if (set_traj.yaw == 60.0 && last_yaw == -60)
            {
                ROS_INFO("往下");
                last_yaw = set_traj.yaw;
                set_traj.pitch = set_traj.pitch + pitch_gap_;
            }
            else if (set_traj.yaw == 60.0 && (last_yaw == 60 || last_yaw == 0))
            {
                ROS_INFO("往右");
                last_yaw = set_traj.yaw;
                set_traj.yaw = -60.0;
            }
            if (set_traj.yaw > 60.0 || set_traj.yaw < -60.0 || set_traj.pitch >= -init_pitch_ || set_traj.pitch < init_pitch_ || set_traj.speed > 30 || set_traj.speed <= 0)
            {
                if (set_traj.pitch == -init_pitch_ && set_traj.yaw == last_yaw)
                {
                    traj_pub.publish(set_traj);
                    ROS_INFO("next place : [%f,%f,%i]", set_traj.yaw, set_traj.pitch, set_traj.speed);
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        if (pub_traj == true)
                        {
                            ROS_INFO("finish, flag changed");
                            pub_traj = false;
                            break;
                        }
                    }
                    ROS_INFO("continue");
                    continue;
                }
                else if(set_traj.pitch == -init_pitch_ && set_traj.yaw != last_yaw)
                {
                    traj_pub.publish(set_traj);
                    ROS_INFO("next place : [%f,%f,%i]", set_traj.yaw, set_traj.pitch, set_traj.speed);
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        if (pub_traj == true)
                        {
                            ROS_INFO("finish, flag changed");
                            pub_traj = false;
                            break;
                        }
                    }
                    ROS_INFO("finished scanning up");
                    reset_pose(set_traj);
                    traj_pub.publish(set_traj);
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        if (pub_traj == true)
                        {
                            ROS_INFO("finish, flag changed");
                            pub_traj = false;
                            break;
                        }
                    }
                    return 0;
                }
                else
                {
                    ROS_INFO("finished scanning down");
                    reset_pose(set_traj);
                    traj_pub.publish(set_traj);
                    while (ros::ok())
                    {
                        ros::spinOnce();
                        if (pub_traj == true)
                        {
                            ROS_INFO("finish, flag changed");
                            pub_traj = false;
                            break;
                        }
                    }
                    return 0;
                }
            }
            else
            {
                traj_pub.publish(set_traj);
                ROS_INFO("next place : [%f,%f,%i]", set_traj.yaw, set_traj.pitch, set_traj.speed);
                ROS_INFO("flag now is : [%i]", pub_traj);
                // r.sleep();
                while (ros::ok())
                {
                    ros::spinOnce();
                    if (pub_traj == true)
                    {
                        ROS_INFO("finish, flag changed");
                        pub_traj = false;
                        break;
                    }
                }
            }
        }

        ros::spinOnce();
    }
    return 0;
}