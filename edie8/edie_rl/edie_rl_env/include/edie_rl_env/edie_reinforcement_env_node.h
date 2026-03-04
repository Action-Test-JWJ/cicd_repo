#ifndef EDIE_REINFORCEMENT_ENV_NODE_H
#define EDIE_REINFORCEMENT_ENV_NODE_H

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <time.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdio.h>
#include <math.h>
#include <cmath>
#include <random> //for random

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

//DEFINE
#define HZ 100
#define UNITY_HZ 50

namespace EDIE
{
    class EDIE_reinforcement_env : public rclcpp::Node
    {
    public: 
        EDIE_reinforcement_env();
        ~EDIE_reinforcement_env();

        #define NORMAL 0
        #define HAPPY 1
        #define SAD 2
        #define ANGRY 3
        #define DISGUST 4
        #define FEAR 5
        #define SURPRISE 6

        //Variables
        std::ofstream out;
        int state_number;
        int action_number;
        int state_sector_number;
        int max_step;
        int max_time;
        int main_input_type;
        double table_value_chanegr;
        std::vector<double> vision_emotion;
        std::vector<double> origin_state;
        std::vector<double> state;
        std::vector<double> action;
        std::vector<double> input_state;
        std::vector<double> reaction_state;
        int episode;
        int step;
        int max_state;    //for personality table
        std::vector<std::vector<double>> personality_table_copy;  //copy personality table
        std::string person_emotion[7] = {"Normal", "Happy", "Sad", "Angry", "Disgust", "Fear", "Surprise"};
        std::string edie_emotion_pool[16] = {"00",
                                         "01", "02", "03", "04", "05",
                                         "06", "07", "08", "09", "10",
                                         "11", "12", "13", "14", "15"};
        //std::string edie_emotion_pool_text[15] = {"Amuse", "Crying", "Worried",
        //                                      "Curious", "Disappointed", "Surprised", "Dizzy",
        //                                      "Anxious", "Angry", "Doubt", "Happy",
        //                                      "Hopeful", "Scared", "Embarrassed", "Sleepy/Boring"};
        std::string edie_emotion_pool_text[8] = {"Curious", "Sleepy/Boring", "Amuse", "Hopeful",
                                                 "Crying", "Surprised", "Disappointed", "Dizzy"};
        std::string edie_character_pool[6] = {"00", "01", "02", "03", "04"};
        std::string edie_character_pool_text[5] = {"INTP", "ESFI", "Cat_Like", "Normal", "ISTJ"};

        //Variables for input & reaction
        int main_emotion;
        double main_emotion_value;

        //Variables for reward
        double reward;
        double input_emotion_sum;
        double reaction_emotion_sum;

        //parameters
        int edie_rl_chracter_param;

        //ROS topic
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_display;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_output;
        rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_info;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_input;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_reaction;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_episode_done;
        rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pub_rl_step_info;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_person_emotion;

        //ROS Service
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service;
        rclcpp::Service<edie_msgs::srv::Reinforcement>::SharedPtr state_service;
        rclcpp::Service<edie_msgs::srv::Reinforcement>::SharedPtr action_service;
        rclcpp::Service<edie_msgs::srv::Reinforcement>::SharedPtr reward_service;
        rclcpp::Service<edie_msgs::srv::Reinforcement>::SharedPtr done_service;
        rclcpp::Service<edie_msgs::srv::FilePath>::SharedPtr path_service;

        //Msgs
        std_msgs::msg::Int8 display_msg;
        std_msgs::msg::String output_msg;
        std_msgs::msg::Int16MultiArray info_msg;  
        std_msgs::msg::Float32MultiArray input_msg;
        std_msgs::msg::Bool episode_done_msg;
        std_msgs::msg::Float32MultiArray reaction_msg;
        std_msgs::msg::Int8 step_info_msg;

        //Srvs
        std_srvs::srv::Empty empty;

        //Function
        std::string GetMotion(int num);
        void GetEdieYaml();

        //Callback
        void PersonEmotionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

        //Service
        bool ResetServiceCallback([[maybe_unused]]std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                  [[maybe_unused]]std::shared_ptr<std_srvs::srv::Empty::Response> res);
        bool StateServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                  std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res);
        bool ActionServiceCallback(std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                   [[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res);
        bool RewardServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                   std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res);
        bool DoneServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                   std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res);
        bool PathServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::FilePath::Request> req, 
                                 std::shared_ptr<edie_msgs::srv::FilePath::Response> res);
        //------------------------------------------------------------------------------
    private:
    };
}   

#endif // EDIE_REINFORCEMENT_ENV_NODE_H