#include <edie_rl_env/edie_reinforcement_env_node.h>
//######################################################################################
EDIE::EDIE_reinforcement_env::EDIE_reinforcement_env() : Node("edie_reinforcement_env_node"){
  state_number            =   9;//9;
  action_number           =   8;//15;
  state_sector_number     =   10;//10;
  max_step                =   30;
  reward                  =   0;
  episode                 =   0;
  main_input_type         =   0;
  info_msg.data.resize(4);
  input_msg.data.resize(9); 
  reaction_msg.data.resize(9);
  pub_display             =   this->create_publisher<std_msgs::msg::Int8>("/edie8/display", 10);
  pub_output              =   this->create_publisher<std_msgs::msg::String>("/edie8/rl/output", 10);
  pub_info                =   this->create_publisher<std_msgs::msg::Int16MultiArray>("/edie8/rl/info", 10);
  pub_input               =   this->create_publisher<std_msgs::msg::Float32MultiArray>("/edie8/rl/input", 10);
  pub_reaction            =   this->create_publisher<std_msgs::msg::Float32MultiArray>("/edie8/rl/reaction", 10);
  pub_episode_done        =   this->create_publisher<std_msgs::msg::Bool>("/edie8/rl/episode_done", 10);
  pub_rl_step_info        =   this->create_publisher<std_msgs::msg::Int8>("/edie8/rl/step_info", 10);
  sub_person_emotion      =   this->create_subscription<std_msgs::msg::Float64MultiArray>("/emotion_publisher", 
                              10, std::bind(&EDIE::EDIE_reinforcement_env::PersonEmotionCallback, this, 
                              std::placeholders::_1));

  reset_service           =   create_service<std_srvs::srv::Empty>("/edie8/rl/reset_service", 
                              std::bind(&EDIE_reinforcement_env::ResetServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  state_service           =   create_service<edie_msgs::srv::Reinforcement>("/edie8/rl/state_service", 
                              std::bind(&EDIE_reinforcement_env::StateServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  action_service          =   create_service<edie_msgs::srv::Reinforcement>("/edie8/rl/action_service", 
                              std::bind(&EDIE_reinforcement_env::ActionServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  reward_service          =   create_service<edie_msgs::srv::Reinforcement>("/edie8/rl/reward_service", 
                              std::bind(&EDIE_reinforcement_env::RewardServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  done_service            =   create_service<edie_msgs::srv::Reinforcement>("/edie8/rl/done_service", 
                              std::bind(&EDIE_reinforcement_env::DoneServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  path_service            =   create_service<edie_msgs::srv::FilePath>("/edie8/rl/path_service", 
                              std::bind(&EDIE_reinforcement_env::PathServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  //----------------------------------------------------------------------------------------
  vision_emotion.clear();
  for(int i = 0; i < state_number-2; i++)
  {
    if(i == 0) vision_emotion.push_back(0.5);
    else vision_emotion.push_back(0);
  }
}
EDIE::EDIE_reinforcement_env::~EDIE_reinforcement_env(){ 
  out.close();
}
//****************************************************************************************************
void EDIE::EDIE_reinforcement_env::GetEdieYaml(){
    std::string edie_package = SOURCE_DIR;
    std::string edie_path = edie_package + "/../../" + "edie_bringup/config/edie_rl_config.yaml"; //AB param yaml
    YAML::Node edie_doc = YAML::LoadFile(edie_path);
    edie_rl_chracter_param     = edie_doc["edie_rl_character"].as<int>();
}
std::string EDIE::EDIE_reinforcement_env::GetMotion(int num){
    EDIE_reinforcement_env::GetEdieYaml();
    int num_emotion = (num) % 15;
    int num_character = edie_rl_chracter_param;
    //std::cout <<  "action: " << num << std::endl;
    //std::cout <<  "emotion: " << num_emotion << std::endl;
    //std::cout <<  "character: " << num_character << std::endl;
    //std::cout <<  "motion: " << edie_emotion_pool[num_emotion+1] + edie_character_pool[num_character+1] << std::endl;
    return edie_emotion_pool[num_emotion+1] + edie_character_pool[num_character];
}
//****************************************************************************************************
void EDIE::EDIE_reinforcement_env::PersonEmotionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    vision_emotion.clear();
    if(msg->data[0] == 0 && msg->data[1] == 0 && msg->data[2] == 0 &&
       msg->data[3] == 0 && msg->data[4] == 0 && msg->data[5] == 0 && msg->data[6] == 0)
    {
    }
    else{
        for(int i = 0; i < state_number-2; i++)
        {
            vision_emotion.push_back(msg->data[i]);
        }
    }
}
//****************************************************************************************************
bool EDIE::EDIE_reinforcement_env::ResetServiceCallback([[maybe_unused]]std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                                          [[maybe_unused]]std::shared_ptr<std_srvs::srv::Empty::Response> res){
    //RCLCPP_INFO(this->get_logger(),"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    //RCLCPP_INFO(this->get_logger(),"State Initialize.");
    step = 0;

    state.clear();          //RL node로 보내줄 감정 데이터 형식
    input_state.clear();    //env node에서 사용할 감정 데이터 형식
    for(int i = 0; i < state_number-2; i++)
    {
        input_state.push_back(vision_emotion.at(i));
    }

    std::cout <<  "----------initial_state----------" << std::endl;
    for(int i = 0; i < state_number-2; i++)
    {
        std::cout << input_state.at(i) << "  ";
    }
    std::cout << std::endl;

    max_state = max_element(input_state.begin(), input_state.end()) - input_state.begin();
    std::cout <<  "max state: " << max_state << std::endl;
    std::cout <<  "-----------------------------------" << std::endl;
    for(int i = 0; i < state_number-2; i++)
    {
        if(i == max_state)  state.push_back(input_state.at(i));
        else state.push_back(0);
    }
    reward = 0;
    return true;
}
bool EDIE::EDIE_reinforcement_env::StateServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                          std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    //RCLCPP_INFO(this->get_logger(),"StateServiceCallback");
    res->state.clear();

    for(int i=0; i < state_number-2; i++)res->state.push_back(state.at(i));
    res->result = true;
    return true;
}
bool EDIE::EDIE_reinforcement_env::ActionServiceCallback(std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                           [[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    //RCLCPP_INFO(this->get_logger(),"ActionServiceCallback");
    //action 산출
    action.clear();
    int max_action_index = req->action[0];
    output_msg.data = EDIE_reinforcement_env::GetMotion(max_action_index);
    pub_output->publish(output_msg);
    std::cout <<  "action_index: " << max_action_index << std::endl;
    display_msg.data = max_action_index+1;
    pub_display->publish(display_msg);

    //next state 계산
    reaction_state.clear();
    for(int i = 0; i < state_number-2; i++)
    {
        reaction_state.push_back(vision_emotion.at(i));
    }

    //show input and reaction state
    std::cout <<  "----------input_state----------" << std::endl;
    for(int i = 0; i < state_number-2; i++)
    {
        std::cout << input_state.at(i) << "  ";
    }
    std::cout << std::endl;
    std::cout <<  "----------reaction_state----------" << std::endl;
    for(int i = 0; i < state_number-2; i++)
    {
        std::cout << reaction_state.at(i) << "  ";
    }
    std::cout << std::endl;

    reaction_state.clear();
    for(int i = 0; i < state_number-2; i++)
  {
    if(i == 2) reaction_state.push_back(0.5);
    else reaction_state.push_back(0);
  }

    //reward 계산
    reward = 0;
    //std::cout <<  "----------reward----------" << std::endl;
    int max_next_state_index = max_element(reaction_state.begin(), reaction_state.end()) - reaction_state.begin();
    int max_state_index = max_element(input_state.begin(), input_state.end()) - input_state.begin();
    //긍정적인 감정일때
    if(max_state_index == 1 || max_state_index == 6){
        if(input_state[max_state_index] == reaction_state[max_next_state_index])
            reward = -0.1;
        else if(input_state[max_state_index] > reaction_state[max_next_state_index])
            reward = -0.1;
        else if(input_state[max_state_index] < reaction_state[max_next_state_index])
            reward = 0.1;
    }
    //부정적인 감정일때
    else if(max_state_index == 0 || max_state_index == 2 || max_state_index == 3 || max_state_index == 4 || max_state_index == 5){
        if(input_state[max_state_index] == reaction_state[max_next_state_index])
            reward = -0.1;
        else if(input_state[max_state_index] > reaction_state[max_next_state_index])
            reward = 0.1;
        else if(input_state[max_state_index] < reaction_state[max_next_state_index])
            reward = -0.1;
    }
    reward = reward*1;
    std::cout <<  "final reward: " << reward << std::endl;

    //make state for ros learn node
    state.clear();
    int input_max_state_now = max_element(input_state.begin(), input_state.end()) - input_state.begin();
    int reaction_max_state_now = max_element(reaction_state.begin(), reaction_state.end()) - reaction_state.begin();
    for(int i = 0; i < state_number-2; i++)
    {
        if(i == reaction_max_state_now)  state.push_back(reaction_state.at(i));
        else state.push_back(0);
    }

    //pub state msgs
    //for(int i = 0; i < state_number-2; i++)
    //{
    //    if(i == input_max_state_now)  input_msg.data[i] = input_state.at(i);
    //    else input_msg.data[i] = 0;
    //}
    //for(int i = 0; i < state_number-2; i++)
    //{
    //    if(i == reaction_max_state_now)  reaction_msg.data[i] = reaction_state.at(i);
    //    else reaction_msg.data[i] = 0;
    //}
    input_msg.data[0] = input_max_state_now;
    reaction_msg.data[0] = reaction_max_state_now;
    pub_input->publish(input_msg);
    pub_reaction->publish(reaction_msg);
    
    return true;
}
bool EDIE::EDIE_reinforcement_env::RewardServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                            std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    //RCLCPP_INFO(this->get_logger(),"RewardServiceCallback");
    res->reward.clear();
    res->reward.push_back(reward);
    return true;
}
bool EDIE::EDIE_reinforcement_env::DoneServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                            std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    //RCLCPP_INFO(this->get_logger(),"doneServiceCallback");
    if(step >= max_step-1){
        episode = episode + 1;
        res->done = true;
        std::cout << step << " step done---------------------------------------------------------------------------------------" << std::endl;
        step = 0;
        episode_done_msg.data = true;
        pub_episode_done->publish(episode_done_msg);
        step_info_msg.data = 4;
        pub_rl_step_info->publish(step_info_msg);
    }
    else{
        std::cout << step << " step done---------------------------------------------------------------------------------------" << std::endl;
        step = step+1;
        res->done = false;
    }
    if(episode >= 500){
        episode_done_msg.data = true;
        pub_episode_done->publish(episode_done_msg);
    }
    //input_state.clear();
    //for(int i = 0; i < state_number-2; i++)
    //{
    //    input_state.push_back(reaction_state.at(i));
    //}
    info_msg.data[0] = episode;
    info_msg.data[1] = 4;
    info_msg.data[2] = step;
    info_msg.data[3] = reward*(double)1000;
    std::cout <<  "pub reward: " << reward << std::endl;
    pub_info->publish(info_msg);
    return true;
}
bool EDIE::EDIE_reinforcement_env::PathServiceCallback([[maybe_unused]]std::shared_ptr<edie_msgs::srv::FilePath::Request> req, 
                                                            std::shared_ptr<edie_msgs::srv::FilePath::Response> res){
    RCLCPP_INFO(this->get_logger(),"PathServiceCallback");
    res->path = SOURCE_DIR;
    RCLCPP_INFO(this->get_logger(),SOURCE_DIR);
    return true;
}
//######################################################################################

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto EDIE_reinforcement_env = std::make_shared<EDIE::EDIE_reinforcement_env>();

  rclcpp::WallRate loop_rate(HZ);
  while (rclcpp::ok()) {
    //---------------------------------------------------------
    
    //---------------------------------------------------------
    rclcpp::spin_some(EDIE_reinforcement_env);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}