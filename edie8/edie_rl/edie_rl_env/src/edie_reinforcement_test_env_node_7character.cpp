#include <edie_rl_env/edie_reinforcement_test_env_node.h>
template<typename dataType>
//######################################################################################
double softmax(std::vector<dataType>& arr, dataType sj){
	if(!std::any_of(arr.begin(), arr.end(), [&sj](dataType& j){ return j == sj; })) throw std::runtime_error("Invalid value");
	
	dataType maxElement = *std::max_element(arr.begin(), arr.end());
	double sum = 0.0;
	for(auto const& i : arr) sum += std::exp(i - maxElement);
	
	return (std::exp(sj - maxElement) / sum);
}
template<typename dataType>
double SumIsOne(std::vector<dataType>& arr, dataType sj){
	if(!std::any_of(arr.begin(), arr.end(), [&sj](dataType& j){ return j == sj; })) throw std::runtime_error("Invalid value");
	
	double sum = 0.0;
	for(auto const& i : arr) sum += i;
	
	return ((sj) / (sum));
}
//######################################################################################
Dummy::Dummy(int state_number_, int action_number_, int state_sector_number_, int seed)
{
    state_number = state_number_;
    action_number = action_number_;
    state_sector_number = state_sector_number_;
    sector_interval = (1/(double)state_sector_number)*100;
    std::vector<std::vector<int>> pos_value(state_number, std::vector<int>(action_number, 0));
    std::vector<int> must_pose;
    must_pose.resize(action_number);
    srand((unsigned)time(NULL)+seed);

    std::cout <<  "Dummy personality table was created." << std::endl;
    //action에 따라 어느 감정을 증가시킬지 경정할 table 생성
    for(int k = 0; k < action_number; k++){
        must_pose[k]= (double)(rand()%(state_number-2));
        std::cout <<  must_pose[k] << "   " << std::endl; 
    }

    for(int i = 0; i < (state_number-2)*state_sector_number; i++)
    {
        std::vector<double> tmp_vector;
        personality_table.push_back(tmp_vector);
        if(i%state_sector_number == 0)  std::cout << "----------------------------------------------" << std::endl;
        for(int j = 0; j < action_number; j++)
        {
            double random_value, tmp_value;
            if(i%state_sector_number == 0) {
                if(i/state_sector_number == must_pose[j]) pos_value[i/state_sector_number][j] = 1;
                else pos_value[i/state_sector_number][j] = 0;
            }
            if(pos_value[i/state_sector_number][j] == 1)  
                random_value = ((double)(rand()%sector_interval) + ((state_sector_number-1-(i%state_sector_number))*sector_interval));
            else                
                random_value = -((double)(rand()%sector_interval) + ((i%state_sector_number))*sector_interval);
            tmp_value = (random_value / 100);
            personality_table.at(i).push_back(tmp_value);
            std::cout << personality_table[i][j]<< " ";
        }
        std::cout << std::endl;
    }
    std::cout << "----------------------------------------------" << std::endl;
}
Dummy::~Dummy(){}
//######################################################################################
EDIE::EDIE_reinforcement_env::EDIE_reinforcement_env() : Node("edie_reinforcement_test_env_node"){
  state_number            =   9;//9;
  action_number           =   15;//15;
  state_sector_number     =   10;//10;
  max_step                =   30;
  reward                  =   0;
  episode                 =   0;
  main_input_type         =   0;
  table_value_chanegr     =   1/(double)max_step;
  dummy_instance          =   new Dummy(state_number, action_number, state_sector_number, 0);
  dummy_instance_1        =   new Dummy(state_number, action_number, state_sector_number, 100);
  dummy_instance_2        =   new Dummy(state_number, action_number, state_sector_number, 200);
  dummy_instance_3        =   new Dummy(state_number, action_number, state_sector_number, 300);
  dummy_instance_4        =   new Dummy(state_number, action_number, state_sector_number, 400);
  dummy_instance_5        =   new Dummy(state_number, action_number, state_sector_number, 500);
  dummy_instance_6        =   new Dummy(state_number, action_number, state_sector_number, 600);
  dummy_instance_7        =   new Dummy(state_number, action_number, state_sector_number, 700);
  info_msg.data.resize(4);
  input_msg.data.resize(9); 
  reaction_msg.data.resize(9);
  pub_output              =   this->create_publisher<std_msgs::msg::String>("/edie8/rl/output", 10);
  pub_info                =   this->create_publisher<std_msgs::msg::Int16MultiArray>("/edie8/rl/info", 10);
  pub_input               =   this->create_publisher<std_msgs::msg::Float32MultiArray>("/edie8/rl/input", 10);
  pub_reaction            =   this->create_publisher<std_msgs::msg::Float32MultiArray>("/edie8/rl/reaction", 10);
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
    //std::cout <<  "action: " << num << std::endl;
    //std::cout <<  "emotion: " << num_emotion << std::endl;
    //std::cout <<  "character: " << num_character << std::endl;
    //std::cout <<  "motion: " << edie_emotion_pool[num_emotion+1] + edie_character_pool[num_character+1] << std::endl;
    return edie_emotion_pool[num_emotion+1] + edie_character_pool[num_character];
}
//****************************************************************************************************
//****************************************************************************************************
bool EDIE::EDIE_reinforcement_env::ResetServiceCallback([[maybe_unused]]std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                                                          [[maybe_unused]]std::shared_ptr<std_srvs::srv::Empty::Response> res){
    //RCLCPP_INFO(this->get_logger(),"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
    //RCLCPP_INFO(this->get_logger(),"State Initialize.");
    step = 0;
    srand((unsigned)time(NULL));
    double tmp_value;

    origin_state.clear();
    state.clear();
    input_state.clear();
    std::vector<double> state_emotion = { 0, 0, 0, 0, 0, 0, 0};
    
    for(int i = 0; i < state_number-2; i++)
    {
        //state_emotion[i] = ((double)(rand()%100) / 100);
        state_emotion[i] = ((double)pow(2.71,(rand()%20)));
    }
    for(int i = 0; i < state_number-2; i++)
    {
        tmp_value = SumIsOne<double>(state_emotion, state_emotion[i]);
        origin_state.push_back(tmp_value);
        state.push_back(tmp_value);
        input_state.push_back(tmp_value);
    }
    for(int i = state_number-2; i < state_number; i++)
    {
        tmp_value = (((double)(rand()%1800) / 10) - 90) * (3.14/180);
        origin_state.push_back(tmp_value);
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

    //next state 계산 (normal 제외)
    std::vector<double> state_emotion = { 0, 0, 0, 0, 0, 0, 0};  //state중 감정만 저장할 배열
    double *tmp;
    tmp = new double [state_number];
    double sector_interval = 1/(double)state_sector_number;
    //std::cout <<  "sector_interval: " << sector_interval << std::endl; 
    switch(max_state){
        case 0:
        personality_table_copy = dummy_instance_1->personality_table;
        break;
        case 1:
        personality_table_copy = dummy_instance_2->personality_table;
        break;
        case 2:
        personality_table_copy = dummy_instance_3->personality_table;
        break;
        case 3:
        personality_table_copy = dummy_instance_4->personality_table;
        break;
        case 4:
        personality_table_copy = dummy_instance_5->personality_table;
        break;
        case 5:
        personality_table_copy = dummy_instance_6->personality_table;
        break;
        case 6:
        personality_table_copy = dummy_instance_7->personality_table;
        break;
    }

    for(int i = 0; i<state_number-2; i++)
    {
        for(int j = 0; j<state_sector_number; j++){
            if(input_state.at(i) >= sector_interval*j && input_state.at(i) <= sector_interval*(j+1)){
                state_emotion[i] = input_state.at(i) + personality_table_copy[state_sector_number*i+j][max_action_index]*table_value_chanegr ;
                if(state_emotion[i]<0) state_emotion[i]=0;
                std::cout <<  "state at " << i <<": " << input_state.at(i) << " + " << personality_table_copy[state_sector_number*i+j][max_action_index]*table_value_chanegr  << " = " << state_emotion[i] << std::endl; 
            }
        }
    }
    for(int i = 0; i < state_number-2; i++){
        //tmp[i] = softmax<double>(state_emotion, state_emotion[i]);
        tmp[i] = SumIsOne<double>(state_emotion, state_emotion[i]);
    }
    state.clear();
    reaction_state.clear();
    for(int i = 0; i < state_number-2; i++)
    {
        state.push_back(tmp[i]);
        reaction_state.push_back(tmp[i]);
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

    //reward 계산
    reward = 0;
    //std::cout <<  "----------reward----------" << std::endl;
    int max_next_state_index = max_element(reaction_state.begin(), reaction_state.end()) - reaction_state.begin();
    int max_state_index = max_element(input_state.begin(), input_state.end()) - input_state.begin();
    if(max_state_index == 1 || max_state_index == 6)
        if(input_state[max_state_index] == reaction_state[max_next_state_index])
            reward = -0.1;
        else if(input_state[max_state_index] > reaction_state[max_next_state_index])
            reward = -(reaction_state[max_next_state_index] - input_state[max_state_index]);
        else if(input_state[max_state_index] < reaction_state[max_next_state_index])
            reward = (reaction_state[max_next_state_index] - input_state[max_state_index]);
    else if(max_state_index == 0 || max_state_index == 2 || max_state_index == 3 || max_state_index == 4 || max_state_index == 5)
        if(input_state[max_state_index] == reaction_state[max_next_state_index])
            reward = -0.1;
        else if(input_state[max_state_index] > reaction_state[max_next_state_index])
            reward = (reaction_state[max_next_state_index] - input_state[max_state_index]);
        else if(input_state[max_state_index] < reaction_state[max_next_state_index])
            reward = -(reaction_state[max_next_state_index] - input_state[max_state_index]);
    reward = reward*10;
    std::cout <<  "final reward: " << reward << std::endl;

    //pub state msgs
    for(int i = 0; i < state_number-2; i++)
    {
        input_msg.data[i] = input_state.at(i);
        reaction_msg.data[i] = reaction_state.at(i);
    }
    for(int i = state_number-2; i < state_number; i++)
    {
        input_msg.data[i] = origin_state.at(i)* (180/3.14);
        reaction_msg.data[i] = origin_state.at(i) * (180/3.14);
    }
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
    if(step == max_step-1){
        episode = episode + 1;
        res->done = true;
        std::cout << step << " step done---------------------------------------------------------------------------------------" << std::endl;
        step = 0;
    }
    else{
        std::cout << step << " step done---------------------------------------------------------------------------------------" << std::endl;
        step = step+1;
        res->done = false;
    }
    input_state.clear();
    for(int i = 0; i < state_number-2; i++)
    {
        input_state.push_back(reaction_state.at(i));
    }
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