#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>



#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

class BallDetector {
    ros::NodeHandle n;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Publisher head_rotator;


    float frequency = 0.2f;
    float max_angle = M_PI / 4.f;
    int last_t = 0;



public:


    BallDetector() : tfListener(tfBuffer) {
        //signal(SIGINT, BallDetector::quit);
        head_rotator = n.advertise<sensor_msgs::JointState>("all_motor", 10);


    }
    // catch names of the controllers availables on ROS network

    void move_head(){
        geometry_msgs::TransformStamped ball_pose;

        bool has_pose = false;
        try {
            ball_pose = tfBuffer.lookupTransform("ball", "torso",ros::Time(0));
            //has_pose = true;
        } catch (tf2::TransformException &ex) {
            has_pose = false;
        }
        /*ros::Duration last_pose = ros::Time::now() - ball_pose.header.stamp;
        if (last_pose < ros::Duration(1)) {
            has_pose = true;
            return;
        }*/

        if (!has_pose) {
            sensor_msgs::JointState js;
            js.name.push_back("head_motor_0");
            js.name.push_back("head_motor_1");
            js.position.push_back(max_angle * std::sin(static_cast<float>(last_t) / 100.f * frequency));
            js.position.push_back(0.6f);
            head_rotator.publish(js);
            last_t += 1;

        }
        else {
            has_pose = false;
        }


    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector");
    BallDetector ballDetector;

    ros::Rate r(100);
    while (ros::ok()) {
        ballDetector.move_head();
        r.sleep();
    }
    ros::spin();


    return 0;
}
/*static int controllerCount;
static std::vector<std::string> controllerList;
static std::string controllerName;
static double lposition = 0;
static double rposition = 0;
float frequency = 0.2f;
float max_angle = M_PI / 4.f;
int last_t = 0;
ros::ServiceClient leftWheelClient;
webots_ros::set_float leftWheelSrv;

ros::ServiceClient rightWheelClient;
webots_ros::set_float rightWheelSrv;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;


// catch names of the controllers availables on ROS network
void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
    controllerCount++;
    controllerList.push_back(name->data);
    ROS_INFO("Controller #%d: %s.", controllerCount, controllerList.back().c_str());
}

void quit(int sig) {

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ROS_INFO("User stopped the 'keyboard_teleop' node.");
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ball_detector", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    signal(SIGINT, quit);

    // subscribe to the topic model_name to get the list of availables controllers
    ros::Subscriber nameSub = n.subscribe("/model_name", 100, controllerNameCallback);

    while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {

        ros::spinOnce();
        ros::Duration(0.5).sleep();


    }
    ros::spinOnce();

    // if there is more than one controller available, let the user choose


    if (controllerCount == 1)
        controllerName = controllerList[0];
    else {
        int wantedController = 0;
        std::cout << "Choose the # of the controller you want to use:\n";
        std::cin >> wantedController;
        if (1 <= wantedController && wantedController <= controllerCount)
            controllerName = controllerList[wantedController - 1];
        else {
            ROS_ERROR("Invalid number for controller choice.");
            return 1;
        }
    }

    // leave topic once it's not necessary anymore
    nameSub.shutdown();
    leftWheelClient = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/head_motor_0/set_position");
    rightWheelClient = n.serviceClient<webots_ros::set_float>("/" + controllerName + "/head_motor_1/set_position");

    timeStepClient = n.serviceClient<webots_ros::set_int>("/" + controllerName + "/robot/time_step");
    timeStepSrv.request.value = TIME_STEP;

        // main loop
        while (ros::ok()) {


            if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
                ROS_ERROR("Failed to call service time_step for next step.");
            lposition = max_angle * std::sin(static_cast<float>(last_t) / 100.f * frequency);
            rposition = 0.6f;
            last_t += 1;
            leftWheelSrv.request.value = lposition;
            rightWheelSrv.request.value = rposition;

            if (!leftWheelClient.call(leftWheelSrv) || !rightWheelClient.call(rightWheelSrv) || !leftWheelSrv.response.success || !rightWheelSrv.response.success)
                   ROS_ERROR("Failed to send new position commands to the robot.");

            ros::spinOnce();


        }

    timeStepSrv.request.value = 0;
    timeStepClient.call(timeStepSrv);
    ros::shutdown();
    return 0;
}
    BallDetector ballDetector;

    ros::Rate r(100);
    while (ros::ok()) {
        ballDetector.move_head();
        r.sleep();
    }
    ros::spin();
    return 0;
}*/
