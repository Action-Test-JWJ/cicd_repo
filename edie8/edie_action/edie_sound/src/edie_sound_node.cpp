#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include <edie_sound/sound.hpp>
#include <memory>
#include <sys/wait.h>  // for waitpid

using namespace Sound;

class EdieSoundNode : public rclcpp::Node
{
public:
    EdieSoundNode()
        : Node("edie_sound_node"), is_pressed(false), order(0)
    {
        sub_sound_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/edie8/emotion/sound_index", 10, std::bind(&EdieSoundNode::SoundCallback, this, std::placeholders::_1));
        
        pub_sound_done_ = this->create_publisher<std_msgs::msg::Bool>("/edie8/emotion/sound_done", 10);

        path_ = ament_index_cpp::get_package_share_directory("edie_sound") + "/sounds";
        sounds_ = std::make_shared<SoundManager>(path_);

        SoundFileAdd();
    }

    bool GetIsPressed() const
    {
        return is_pressed;
    }

    uint8_t GetOrder() const
    {
        return order;
    }

    void SetIsPressed(bool pressed)
    {
        is_pressed = pressed;
    }

    void PublishDoneMsg(const std_msgs::msg::Bool::SharedPtr& msg)
    {
        pub_sound_done_->publish(*msg);
    }

    std::shared_ptr<SoundManager> sounds_;

private:
    void SoundFileAdd()
    {
                                                    // BLINK와 SLEEPY는 NO SOUND
        sounds_->Add("/hmm_001.wav");               // if emotionstate==2 CURIOUS       -> sound index : 0
        sounds_->Add("/laugh_001.wav");             // if emotionstate==3 AMUSED        -> sound index : 1 
        sounds_->Add("/laugh_002.wav");             // if emotionstate==3 AMUSED        -> sound index : 2 
        sounds_->Add("/admiration_001.wav");        // if emotionstate==4 HOPEFUL       -> sound index : 3
        sounds_->Add("/question_001.wav");          // if emotionstate==5 CRYING        -> sound index : 4
        sounds_->Add("/grouchy_002.wav");           // if emotionstate==5 CRYING        -> sound index : 5
        sounds_->Add("/surprise_001.wav");          // if emotionstate==6 SURPRISED     -> sound index : 6
        sounds_->Add("/grouchy_001.wav");           // if emotionstate==7 DISAPPOINTED  -> sound index : 7
        sounds_->Add("/grouchy_003.wav");           // if emotionstate==7 DISAPPOINTED  -> sound index : 8
        sounds_->Add("/mumble_002.wav");            // if emotionstate==8 DIZZY         -> sound index : 9
    }

    void SoundCallback(const std_msgs::msg::UInt8::SharedPtr msg)
    {   
        if (sounds_->ValidCheck(msg->data))
        {
            is_pressed = true;
            order = msg->data;
        }
    }

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_sound_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_sound_done_;

    std::string path_;
    bool is_pressed;
    uint8_t order;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieSoundNode>();

    rclcpp::Rate rate(100); 

    pid_t current_sound_pid = -1;

    while (rclcpp::ok())
    {
        if (node->GetIsPressed())
        {
            node->SetIsPressed(false);
            node->sounds_->Play(node->GetOrder());

            current_sound_pid = node->sounds_->GetPid();

            int status;
            waitpid(current_sound_pid, &status, 0);  

            if (WIFEXITED(status) && WEXITSTATUS(status) == 0) 
            {
                auto done_msgs = std::make_shared<std_msgs::msg::Bool>();
                done_msgs->data = 1;
                node->PublishDoneMsg(done_msgs);
            }
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
