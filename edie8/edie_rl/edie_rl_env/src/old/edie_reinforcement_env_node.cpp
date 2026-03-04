#include <edie_rl_env/edie_reinforcement_env_node.h>
template<typename dataType>
//######################################################################################
double softmax(std::vector<dataType>& arr, dataType sj){
	if(!std::any_of(arr.begin(), arr.end(), [&sj](dataType& j){ return j == sj; })) throw std::runtime_error("Invalid value");
	
	dataType maxElement = *std::max_element(arr.begin(), arr.end());
	double sum = 0.0;
	for(auto const& i : arr) sum += std::exp(i - maxElement);
	
	return (std::exp(sj - maxElement) / sum);
}
//######################################################################################
Dummy::Dummy(int state_number_, int action_number_)
{
    state_number = state_number_;
    action_number = action_number_;
    srand((unsigned)time(NULL));
    double tmp_value;
    tmp_value = ((double)(rand()%20) / 10) - 1.0;

    std::cout <<  "Dummy personality table was created." << std::endl;
   for(int i = 0; i < state_number*10; i++)
    {
        std::vector<double> tmp_vector;
        personality_table.push_back(tmp_vector);
        for(int j = 0; j < action_number; j++)
        {
            double tmp_value;
            tmp_value = ((double)(rand()%200) / 100) - 1.0;
            personality_table.at(i).push_back(tmp_value);
            std::cout << tmp_value << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}
Dummy::~Dummy(){}
//######################################################################################
EDIE::EDIE_reinforcement_env::EDIE_reinforcement_env() : Node("edie_reinforcement_env_node"){
  state_number            =   9;
  action_number           =   15;
  reward                  =   0;
  episode                 =   0;
  main_input_type         =   0;
  dummy_instance          =   new Dummy(state_number, action_number);
  get_state_flag          =   false;
  state_service_result    =   false;
  learning_step           =   0;
  info_msg.data.resize(3);
  state_msg.data.resize(9); 
  pub_output              =   this->create_publisher<std_msgs::msg::String>("/edie8/rl/output", 10);
  pub_info                =   this->create_publisher<std_msgs::msg::Int16MultiArray>("/edie8/rl/info", 10);
  sub_state               =   this->create_subscription<std_msgs::msg::Float32MultiArray>(
                              "/edie8/rl/input", 10,
                              std::bind(&EDIE_reinforcement_env::GetStateCallback, this, std::placeholders::_1));
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
  path_service            =   create_service<edie_msgs::srv::FilePath>("/edie8/rl/path_service", 
                              std::bind(&EDIE_reinforcement_env::PathServiceCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
  //----------------------------------------------------------------------------------------
}
EDIE::EDIE_reinforcement_env::~EDIE_reinforcement_env(){ 
  out.close();
  delete dummy_instance;
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
    std::cout <<  "action: " << num << std::endl;
    std::cout <<  "emotion: " << num_emotion << std::endl;
    std::cout <<  "character: " << num_character << std::endl;
    std::cout <<  "motion: " << edie_emotion_pool[num_emotion+1] + edie_character_pool[num_character+1] << std::endl;
    return edie_emotion_pool[num_emotion+1] + edie_character_pool[num_character];
}
//****************************************************************************************************
void EDIE::EDIE_reinforcement_env::GetStateCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    //RCLCPP_INFO(this->get_logger(),"Sub states");
    if(!get_state_flag){
        if(learning_step == 1){
            //RCLCPP_INFO(this->get_logger(),"input state");
            for(int i = 0; i < state_number; i++) state_msg.data[i] = msg->data[i];
            for(int i = 0; i < state_number; i++) input_state.push_back(state_msg.data[i]);
            get_state_flag = true;
        }
        else if(learning_step == 2){
            //RCLCPP_INFO(this->get_logger(),"Reaction state");
            for(int i = 0; i < state_number; i++) state_msg.data[i] = msg->data[i];
            for(int i = 0; i < state_number; i++) reward_state.push_back(state_msg.data[i]);
            get_state_flag = true;
        }
        //else RCLCPP_INFO(this->get_logger(),"none state");
    }
}
//****************************************************************************************************
bool EDIE::EDIE_reinforcement_env::ResetServiceCallback(std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                                          std::shared_ptr<std_srvs::srv::Empty::Response> res){
    RCLCPP_INFO(this->get_logger(),"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    RCLCPP_INFO(this->get_logger(),"State Initialize.");
    input_state.clear();
    reward_state.clear();
    get_state_flag = false;
    learning_step = 1;
    return true;
}
bool EDIE::EDIE_reinforcement_env::StateServiceCallback(std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                          std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    res->state.clear();
    if(get_state_flag){
        RCLCPP_INFO(this->get_logger(),"StateServiceCallback");
        for(int i=0; i < state_number; i++)res->state.push_back(state_msg.data[i]);
        res->result = true;
        return true;
    }
    else{
        //RCLCPP_INFO(this->get_logger(),"Not Yet");
        if(learning_step == 1){
            info_msg.data[0] = episode;
            info_msg.data[1] = 0;
            pub_info->publish(info_msg);
        }
        else if(learning_step == 2){
            info_msg.data[0] = episode;
            info_msg.data[1] = 2;
            pub_info->publish(info_msg);
        }
        res->result = false;
    }
}
bool EDIE::EDIE_reinforcement_env::ActionServiceCallback(std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                           std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    RCLCPP_INFO(this->get_logger(),"ActionServiceCallback");
    action.clear();
    learning_step = 2;
    get_state_flag = false;
    reward = 0;

    for(int i=0; i < action_number; i++)action.push_back(req->action[i]);

    double max_action = *max_element(action.begin(), action.end());
    int max_action_index = max_element(action.begin(), action.end()) - action.begin();

    std::vector<double> state_emotion = { 0, 0, 0, 0, 0, 0, 0};
    state_emotion[0] = input_state.at(0);
    double tmp[state_number];
    tmp[7] = input_state.at(7);
    tmp[8] = input_state.at(8);
    std::cout <<  "----------reward----------" << std::endl;
    std::cout <<  "max_action_index: " << max_action_index << std::endl;
    output_msg.data = EDIE_reinforcement_env::GetMotion(max_action_index);
    pub_output->publish(output_msg);
    info_msg.data[0] = episode;
    info_msg.data[1] = 1;
    pub_info->publish(info_msg);
   for(int i = 1; i<state_number-2; i++)
    {
        if(input_state.at(i) >= 0 && input_state.at(i) < 0.1){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i][max_action_index];
        }
        else if(input_state.at(i) >= 0.1 && input_state.at(i) < 0.2){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+1][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+1][max_action_index];
        }
        else if(input_state.at(i) >= 0.2 && input_state.at(i) < 0.3){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+2][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+2][max_action_index];
        }  
        else if(input_state.at(i) >= 0.3 && input_state.at(i) < 0.4){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+3][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+3][max_action_index];
        }
        else if(input_state.at(i) >= 0.4 && input_state.at(i) < 0.5){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+4][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+4][max_action_index];
        }
        else if(input_state.at(i) >= 0.5 && input_state.at(i) < 0.6){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+5][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+5][max_action_index];
        }
        else if(input_state.at(i) >= 0.6 && input_state.at(i) < 0.7){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+6][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+6][max_action_index];
        }
        else if(input_state.at(i) >= 0.7 && input_state.at(i) < 0.8){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+7][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+7][max_action_index];
        }
        else if(input_state.at(i) >= 0.8 && input_state.at(i) < 0.9){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+8][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+8][max_action_index];
        }
        else if(input_state.at(i) >= 0.9 && input_state.at(i) < 1){
            state_emotion[i] = input_state.at(i) + dummy_instance->personality_table[i+9][max_action_index];
            if(state_emotion[i]>1) state_emotion[i]=1;
            if(state_emotion[i]<0) state_emotion[i]=0;
            //reward += dummy_instance->personality_table[i+9][max_action_index];
        }
    }
    for(int i = 0; i < state_number-2; i++){
        //tmp[i] = softmax<double>(state_emotion, state_emotion[i]);
        tmp[i] = state_emotion[i];
    }
    for(int i = 0; i < state_number; i++){
        reward += tmp[i] - input_state.at(i);
        std::cout <<  "reward: " << reward << std::endl; 
        std::cout <<  "--------------------------" << std::endl;
    }

    reward_state.clear();
    for(int i = 0; i < state_number; i++)
    {
        reward_state.push_back(tmp[i]);
        std::cout << tmp[i] << " ";
    }
    res->state.clear();
    res->reward.clear();
    for(int i=0; i < state_number; i++)res->state.push_back(reward_state.at(i));
    res->reward.push_back(reward);
    return true;
}
bool EDIE::EDIE_reinforcement_env::RewardServiceCallback(std::shared_ptr<edie_msgs::srv::Reinforcement::Request> req, 
                                                            std::shared_ptr<edie_msgs::srv::Reinforcement::Response> res){
    RCLCPP_INFO(this->get_logger(),"RewardServiceCallback");
    learning_step = 0;
    res->reward.clear();
    res->reward.push_back(reward);

    episode = episode + 1;
    // ROS_INFO("<<<reward>>>");
    // printf("episode             :   %d\n", episode);
    // printf("reward              :   %f\n", reward);
    // printf("similarity reward   :   %f\n", similarity_reward);
    // printf("gradient reward     :   %f\n", gradient_reward);
    // printf("length reward       :   %f\n", length_reward);
    // printf("time reward         :   %f\n", time_reward);
    // out << episode << " "
    //     << reward << " "
    //     << similarity_reward << " "
    //     << gradient_reward << " "
    //     << length_reward << " "
    //     << time_reward << " "
    //     << foot_step_command_msg.step_length << " "
    //     << foot_step_command_msg.step_time <<std::endl;
    info_msg.data[0] = episode;
    info_msg.data[1] = 3;
    pub_info->publish(info_msg);
    return true;
}
bool EDIE::EDIE_reinforcement_env::PathServiceCallback(std::shared_ptr<edie_msgs::srv::FilePath::Request> req, 
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

  rclcpp::WallRate r(HZ);
  while (rclcpp::ok()) {
    //---------------------------------------------------------
    
    //---------------------------------------------------------
    rclcpp::spin_some(EDIE_reinforcement_env);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}