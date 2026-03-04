#include <cmath>
#include <iostream>
#include <chrono>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>
#include <future> //?
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <yaml-cpp/yaml.h> 

//ros_communication_message type
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_srvs/srv/empty.hpp"

#include "edie_msgs/srv/reinforcement.hpp"
#include "edie_msgs/srv/file_path.hpp"
#include "edie_msgs/srv/bt_rl.hpp"

using namespace std::chrono_literals;

//DEFINE
#define HZ 100
#define UNITY_HZ 50
namespace EDIE
{
    class EDIE_rl_bt : public rclcpp::Node
    {
    public: 
        EDIE_rl_bt();
        ~EDIE_rl_bt();
        void BtService();
        void BtGetResult();
        void BtSequence();
        void BtWait(int tick);

        //Variables
        int edie_action_emotion;
        int bt_sequence_step;
        bool episode_done;
        bool bt_sequence_step_done;
        int countdown;

        //parameters

        //ROS topic
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_rl_step_info;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_episode_done;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_person_emotion;

        //ROS Service
        rclcpp::Client<edie_msgs::srv::BtRl>::SharedPtr rl_bt_client;

        //Msgs
        std_msgs::msg::Int8 step_info_msg;

        //Srvs

        //Function

        //Callback
        void EpisodeDoneCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void BtRlCallback(rclcpp::Client<edie_msgs::srv::BtRl>::SharedFuture future);
        void PersonEmotionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        //Service
        //------------------------------------------------------------------------------
    private:
    };
}   