#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <cmath>


//std::vector<geometry_msgs::Pose> msg_vector;
geometry_msgs::Pose poses[16];
int robot_idx;

// Callback function used by the subscribers
void LocalizationFilterCallback(const geometry_msgs::Pose& msg)
{
    poses[robot_idx] = msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "soccer_localization_filter");
    ros::NodeHandle nh;
    ros::Subscriber subs[16];
    ros::Publisher pub[4];
    ros::Rate rate(1);
    ROS_INFO_STREAM("Hello, starting up");

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
        {
            robot_idx = i * 4 + j;
            subs[robot_idx] = nh.subscribe("/robot" + std::to_string(j) + "/estimate_pose/robot" + std::to_string(i), 1, &LocalizationFilterCallback);

        }

        pub[i] = nh.advertise<geometry_msgs::Pose>("/boss/robot" + std::to_string(i),1);
    }



    while (ros::ok()) {
        geometry_msgs::Pose msgs[4];

        for(int i = 0; i < 4; i++)
        {
            geometry_msgs::Pose current_msg;

            for (int j = 0; j < 4; j++)
            {
                int idx = i*4 + j;
                current_msg.position.x += poses[idx].position.x / 4;
                current_msg.position.y += poses[idx].position.y / 4;
                current_msg.position.z += poses[idx].position.z / 4;
                current_msg.orientation.x += poses[idx].orientation.x / 4;
                current_msg.orientation.y += poses[idx].orientation.y / 4;
                current_msg.orientation.z += poses[idx].orientation.z / 4;
                current_msg.orientation.w += poses[idx].orientation.w / 4;
            }

            float orientation_norm = std::sqrt(pow(current_msg.orientation.x,2) + pow(current_msg.orientation.y,2) + pow(current_msg.orientation.z,2) +
                                         pow(current_msg.orientation.w,2));

            current_msg.orientation.x /= orientation_norm;
            current_msg.orientation.y /= orientation_norm;
            current_msg.orientation.z /= orientation_norm;
            current_msg.orientation.w /= orientation_norm;
            pub[i].publish(current_msg);

        }

        rate.sleep();
        ros::spin();

    }
}

