#include <edie_rl_env/edie_rl_bt_node.h>

//######################################################################################
EDIE::EDIE_rl_bt::EDIE_rl_bt() : Node("edie_rl_bt_node"){
  pub_rl_step_info        =   this->create_publisher<std_msgs::msg::Int8>("/edie8/rl/step_info", 10);
  sub_episode_done        =   this->create_subscription<std_msgs::msg::Bool>("/edie8/rl/episode_done", 
                              10, std::bind(&EDIE::EDIE_rl_bt::EpisodeDoneCallback, this, 
                              std::placeholders::_1));
  sub_person_emotion      =   this->create_subscription<std_msgs::msg::Float64MultiArray>("/emotion_publisher", 
                              10, std::bind(&EDIE::EDIE_rl_bt::PersonEmotionCallback, this, 
                              std::placeholders::_1));
  rl_bt_client            =   this->create_client<edie_msgs::srv::BtRl>("/edie8/bt/rl_action");
  //----------------------------------------------------------------------------------------
  episode_done = true;
  bt_sequence_step = 1;
  while (!rl_bt_client->wait_for_service(1s))
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
}
EDIE::EDIE_rl_bt::~EDIE_rl_bt(){ 
}
//****************************************************************************************************
void EDIE::EDIE_rl_bt::EpisodeDoneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if(msg->data){
    episode_done = true;
  }
}
void EDIE::EDIE_rl_bt::PersonEmotionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
    if(msg->data[0] == 0 && msg->data[1] == 0 && msg->data[2] == 0 &&
       msg->data[3] == 0 && msg->data[4] == 0 && msg->data[5] == 0 && msg->data[6] == 0)
    {
    }
    else{
    }
}
void EDIE::EDIE_rl_bt::BtRlCallback(rclcpp::Client<edie_msgs::srv::BtRl>::SharedFuture future)
{
  auto status = future.wait_for(1s);
  if (status == std::future_status::ready)
  {
    edie_action_emotion = future.get()->emotion;
  }
  else
  {
   RCLCPP_INFO(rclcpp::get_logger("Bt_RL"), "BT RL Service In-Progress...");
  }
}
//****************************************************************************************************
void EDIE::EDIE_rl_bt::BtService(){
  //std::cout << "hi 1" << std::endl;
  auto rl_bt_request = std::make_shared<edie_msgs::srv::BtRl::Request>();
  rl_bt_request->launch = true;
  auto btl_rl_result_future = rl_bt_client->async_send_request(rl_bt_request, std::bind(&EDIE::EDIE_rl_bt::BtRlCallback, this, std::placeholders::_1));
  step_info_msg.data = 3;
  pub_rl_step_info->publish(step_info_msg);
}
void EDIE::EDIE_rl_bt::BtGetResult(){
  //std::cout << "hi 2" << std::endl;
  if(edie_action_emotion > 0){
    std::cout << edie_action_emotion << std::endl;
    episode_done = false;
    bt_sequence_step++;
  }
}
void EDIE::EDIE_rl_bt::BtWait(int tick){
  countdown = tick;
  step_info_msg.data = 0;
  pub_rl_step_info->publish(step_info_msg);
}
//****************************************************************************************************
void EDIE::EDIE_rl_bt::BtSequence(){
  switch(bt_sequence_step){
    case 1: BtService(); bt_sequence_step++; break;
    case 2: BtGetResult(); break;
    case 3: if(episode_done){BtWait(300); bt_sequence_step++;} break;
		case 4: if(!countdown){bt_sequence_step=1;} break;
  }
  if(countdown) 
	{
		//RCLCPP_INFO(this->get_logger(), "countdown : %d", countdown);
		countdown--;
	}
}
//######################################################################################

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto EDIE_rl_bt = std::make_shared<EDIE::EDIE_rl_bt>();

  rclcpp::WallRate r(HZ);
  while (rclcpp::ok()) {
    //---------------------------------------------------------
    EDIE_rl_bt->BtSequence();
    //std::cout << EDIE_rl_bt->episode_done << std::endl;
    //---------------------------------------------------------
    rclcpp::spin_some(EDIE_rl_bt);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}