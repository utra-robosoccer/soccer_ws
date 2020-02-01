#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


//std::vector<geometry_msgs::Pose> msg_vector;
//geometry_msgs::Pose poses[16];
//int robot_idx;

// Callback function used by the subscribers
//void LocalizationFilterCallback(const geometry_msgs::Pose& msg)
//{
//    poses[robot_idx] = msg;
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "soccer_localization_filter");
    ros::NodeHandle nh;
//    ros::Subscriber subs[16];
//    ros::Publisher pub[4];
    ros::Rate rate(1);

//    ros::Publisher pub = nh.advertise<
    static tf2_ros::TransformBroadcaster br;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    while (nh.ok()) {

        ros::Time time = ros::Time::now();

        geometry_msgs::TransformStamped transformStamped[4];

        try {
            transformStamped[0] = tfBuffer.lookupTransform("/robot1/estimate_ball_pose", "/ball_pose", ros::Time(0));
            transformStamped[1] = tfBuffer.lookupTransform("/robot2/estimate_ball_pose", "/ball_pose", ros::Time(0));
            transformStamped[2] = tfBuffer.lookupTransform("/robot3/estimate_ball_pose", "/ball_pose", ros::Time(0));
            transformStamped[3] = tfBuffer.lookupTransform("/robot4/estimate_ball_pose", "/ball_pose", ros::Time(0));
        }

        catch (tf2::TransformException &ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        geometry_msgs::Twist ball_twist;

        for (int i = 0; i < 4; i++) {
            ball_twist.linear.x = 0.25 * transformStamped[i].transform.translation.x;
            ball_twist.linear.y = 0.25 * transformStamped[i].transform.translation.y;
            ball_twist.linear.z = 0.25 * transformStamped[i].transform.translation.z;
            ball_twist.angular.x = 0.25 * transformStamped[i].transform.rotation.x;
            ball_twist.angular.y = 0.25 * transformStamped[i].transform.rotation.y;
            ball_twist.angular.z = 0.25 * transformStamped[i].transform.rotation.z;
        }

//        br.sendTransform(tf::StampedTransform())


    }

    /*ROS_INFO_STREAM("Hello, starting up");

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

    }*/
}

