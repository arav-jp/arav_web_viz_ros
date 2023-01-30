#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "./MainControlParams0.hpp"

#include <can_msgs/Frame.h>
#include <IO/CAN/OpenIMU/OpenIMU.hpp>
#include <Process/Process.hpp>
#include <Process/Conversion/AccToJoint.hpp>
#include <Process/Filter/LowPass.hpp>

std::vector<arav::IO*> can1In;
std::vector<arav::Process*> a2j;
std::vector<arav::Process*> lpf;

void debugRosCan(const can_msgs::Frame::ConstPtr& msg) {
    if (msg->id < 0x10000000)
        std::cout << "ID:0" << std::hex << msg->id << std::dec << " Data:";
    else
        std::cout << "ID:" << std::hex << msg->id << std::dec << " Data:";
    
    for (int v : msg->data)
    {
        if (v < 10)
            std::cout << "  " << v << " ";
        else if (v < 100)
            std::cout << " " << v << " ";
        else
            std::cout << v << " ";
    }
    std::cout << std::endl;
}


void cbReceivedMsgs(const can_msgs::Frame::ConstPtr& msg) {
    CAN_message_t arav_can_msg;
    arav_can_msg.id = msg->id;
    for (int i = 0; i < 8; ++i)
        arav_can_msg.buf[i] = msg->data[i];

    // debugRosCan(msg);

    static auto *frame = new arav::type::can::Frame();
    frame->as<arav::type::can::Frame>()->data = arav_can_msg;

    // Read
    for(auto* io : can1In)
    {
        if(io->decode(frame))
        {
            break;
        }
    }
}

/**
    * @brief 角度を-piから+piに正規化する
    * @param angle: 入力角度[rad]
    * @return -pi～+piに正規化された角度[rad]
**/
double normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        return result + M_PI;
    return result - M_PI;
}



int main(int argc, char **argv) 
{
    // CAN //
    can1In.emplace_back(new arav::io::can::OpenIMU(3)); // body
    can1In.emplace_back(new arav::io::can::OpenIMU(2)); // boom
    can1In.emplace_back(new arav::io::can::OpenIMU(0)); // arm
    can1In.emplace_back(new arav::io::can::OpenIMU(1)); // bucket

    // Acc2Joint //
    arav::process::conversion::AccToJoint::Parameter a2j_param;
    // body
    a2j_param.axis = BODY_PITCH_IMU_AXIS;
    a2j_param.direction = BODY_PITCH_IMU_DIR;
    a2j_param.avoid_jump = BODY_PITCH_IMU_AVOID_JUMP;
    a2j_param.offset = BODY_IMU_OFFSET;
    a2j.emplace_back(new arav::process::conversion::AccToJoint(a2j_param));
    // boom
    a2j_param.axis = BOOM_IMU_AXIS;
    a2j_param.direction = BOOM_IMU_DIR;
    a2j_param.avoid_jump = BOOM_IMU_AVOID_JUMP;
    a2j_param.offset = BOOM_IMU_OFFSET;
    a2j.emplace_back(new arav::process::conversion::AccToJoint(a2j_param));
    // arm
    a2j_param.axis = ARM_IMU_AXIS;
    a2j_param.direction = ARM_IMU_DIR;
    a2j_param.avoid_jump = ARM_IMU_AVOID_JUMP;
    a2j_param.offset = ARM_IMU_OFFSET;
    a2j.emplace_back(new arav::process::conversion::AccToJoint(a2j_param));
    // bucket
    a2j_param.axis = BUCKET_IMU_AXIS;
    a2j_param.direction = BUCKET_IMU_DIR;
    a2j_param.avoid_jump = BUCKET_IMU_AVOID_JUMP;
    a2j_param.offset = BUCKET_IMU_OFFSET;
    a2j.emplace_back(new arav::process::conversion::AccToJoint(a2j_param));

    // Low Pass Filter //
    arav::process::filter::LowPass::Parameter lpf_param;
    // body
    lpf_param.cut_off_freq = BODY_IMU_COF;
    lpf_param.gain = BODY_IMU_GAIN;
    lpf.emplace_back(new arav::process::filter::LowPass(lpf_param, 1. / RATE));
    // boom
    lpf_param.cut_off_freq = BOOM_IMU_COF;
    lpf_param.gain = BOOM_IMU_GAIN;
    lpf.emplace_back(new arav::process::filter::LowPass(lpf_param, 1. / RATE));
    // arm
    lpf_param.cut_off_freq = ARM_IMU_COF;
    lpf_param.gain = ARM_IMU_GAIN;
    lpf.emplace_back(new arav::process::filter::LowPass(lpf_param, 1. / RATE));
    // bucket
    lpf_param.cut_off_freq = BUCKET_IMU_COF;
    lpf_param.gain = BUCKET_IMU_GAIN;
    lpf.emplace_back(new arav::process::filter::LowPass(lpf_param, 1. / RATE));

    ros::init(argc, argv, "arav_web_viz_ros_node");
    ros::NodeHandle nh;
    ros::Rate rate(RATE);

    ros::Subscriber sub_received_msgs = nh.subscribe("/received_messages", 10, cbReceivedMsgs);
    ros::Publisher pub_joint_states = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    while (ros::ok()) 
    {
        for (uint i = 0; i < NUM_OF_AUTOMATION_JOINTS; i++)
        {
            a2j[i]->execute(can1In[i]->decoded_data());
            lpf[i]->execute(a2j[i]->output());
        }

        double body_link   = lpf[0]->output()->as<arav::type::base::Float64>()->data;
        double boom_link   = lpf[1]->output()->as<arav::type::base::Float64>()->data;
        double arm_link    = lpf[2]->output()->as<arav::type::base::Float64>()->data;
        double bucket_link = lpf[3]->output()->as<arav::type::base::Float64>()->data;
        body_link = 0.0;
        // std::cout << body_link << " " << boom_link << " " << arm_link << " " << bucket_link << std::endl;

        std::vector<double> joint_value;
        joint_value.emplace_back(normalize_angle(0));
        joint_value.emplace_back(normalize_angle(boom_link + BOOM_JOINT_OFFSET));
        joint_value.emplace_back(normalize_angle(arm_link - boom_link + ARM_JOINT_OFFSET));
        joint_value.emplace_back(normalize_angle(bucket_link - arm_link - boom_link + BUCKET_JOINT_OFFSET));
        std::cout << joint_value[0] << " " << joint_value[1] << " " << joint_value[2] << " " << joint_value[3] << std::endl;

        sensor_msgs::JointState joint_states_msg;
        joint_states_msg.header.stamp = ros::Time::now();
        joint_states_msg.name.resize(4);
        joint_states_msg.name[0] = "body_joint";
        joint_states_msg.name[1] = "boom_joint";
        joint_states_msg.name[2] = "arm_joint";
        joint_states_msg.name[3] = "bucket_joint";
        joint_states_msg.position.resize(4);
        joint_states_msg.position = joint_value;
        pub_joint_states.publish(joint_states_msg);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
