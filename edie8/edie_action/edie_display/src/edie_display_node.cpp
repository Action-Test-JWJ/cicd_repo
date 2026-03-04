#include "edie_display/edie_display_node.hpp"
#include "edie_display/parser.hpp"

using namespace std;

namespace aeirobot
{

  EdieDisplayNode::EdieDisplayNode()
      : Node("edie_display_node")
  {
    pub_display_done_ = this->create_publisher<std_msgs::msg::Bool>("/edie8/emotion/display_done", 10);
    sub_display_idx_ = this->create_subscription<std_msgs::msg::UInt8>("/edie8/emotion/display_index", 10, std::bind(&EdieDisplayNode::DisplayIndexCallback, this, std::placeholders::_1));
    main_loop_timer = this->create_wall_timer(std::chrono::milliseconds(1000 / loop_rate_hz), std::bind(&EdieDisplayNode::MainLoop, this));

    InitWindow();
    InitEmotionSequences();
  }

  EdieDisplayNode::~EdieDisplayNode()
  {
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
  }

  // Check display id has changed
  bool EdieDisplayNode::IsDisplayIDChanged()
  {
    if (display_id != last_display_id)
    {
      last_display_id = display_id;
      return true;
    }
    return false;
  }

  void EdieDisplayNode::PublishDoneMsg(const bool is_done)
  {
    std_msgs::msg::Bool msg;
    msg.data = is_done;
    pub_display_done_->publish(msg);
  }

  int EdieDisplayNode::GetDisplayId() const
  {
    return display_id;
  }

  void EdieDisplayNode::SetDisplayId(int val)
  {
    display_id = val;
  }

  void EdieDisplayNode::MainLoop()
  {
    // 렌더링 루프
    if (quit)
    {
      rclcpp::shutdown();
      return;
    }

    SDL_Event e;
    while (SDL_PollEvent(&e))
    {
      if (e.type == SDL_QUIT ||
          (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE))
      {
        quit = true;
        break;
      }
    }

    // 초기화
    SDL_RenderClear(renderer);
    // 검정 배경
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    // 현재 감정 시퀀스의 현재 프레임 가져오기
    auto cur_seq = emotion_sequences.at(cur_emotion).GetCurFrame();
    auto nxt_seq = emotion_sequences.at(cur_emotion).GoNextFrame();
    // RCLCPP_INFO_STREAM(this->get_logger(), "Emotion: "<< ToString(cur_emotion)<<"["<< ToInt(cur_emotion) <<"], frame: "<< cur_seq);
    // 현재 감정 이미지 렌더링
    SDL_RenderCopy(renderer, emotions[ToInt(cur_emotion)].GetImage(cur_seq), NULL, &img_frame); //&img_frame);
    // 렌더러에 그리기
    SDL_RenderPresent(renderer);

    if(nxt_seq == 0)
    {
      PublishDoneMsg(true);
    }
  }

  void EdieDisplayNode::DisplayIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if(msg->data >= ToInt(EmotionState::COUNT))
    {
      RCLCPP_WARN_STREAM(this->get_logger(), "Received invalid emotion index: "<< msg->data);
      return;
    }

    cur_emotion = static_cast<EmotionState>(msg->data);
  }

  int EdieDisplayNode::InitWindow()
  {
    cur_emotion = EmotionState::BLINK;
    img_frame.x = 0;
    img_frame.y = 0;
    img_frame.w = 1920;
    img_frame.h = 1080;

    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
      std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
      return 1;
    }

    // JPG 로드를 위한 SDL_image 초기화
    int imgFlags = IMG_INIT_JPG | IMG_INIT_PNG;
    if (!(IMG_Init(imgFlags) & imgFlags))
    {
      std::cerr << "IMG_Init Error: " << IMG_GetError() << std::endl;
      SDL_Quit();
      return 1;
    }

    SDL_CreateWindowAndRenderer(1920, 1080, SDL_WINDOW_FULLSCREEN_DESKTOP, &window, &renderer);
    SDL_SetWindowTitle(window, "Edie Display");
    // window = SDL_CreateWindow("Edie Display",
    //                           SDL_WINDOWPOS_CENTERED,
    //                           SDL_WINDOWPOS_CENTERED,
    //                           1080, 1080,
    //                           // SDL_WINDOW_OPENGL | 
    //                           SDL_WINDOW_RESIZABLE);//SDL_WINDOW_FULLSCREEN_DESKTOP);
    // renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED); // | SDL_RENDERER_PRESENTVSYNC);
    // 랜더러 초기화(안해도 무방할듯)
    SDL_RenderClear(renderer);
    // 배경 검정색으로 설정
    SDL_SetRenderDrawColor(renderer, 100, 0, 0, 255);
    // 렌더러에 그리기
    SDL_RenderPresent(renderer);

    resource_path = ament_index_cpp::get_package_share_directory("edie_display") + "/images/emotions/";
    emotions.resize(ToInt(EmotionState::COUNT));
    for (int i = 0; i < emotions.size(); i++)
    {
      auto emo_enum = static_cast<EmotionState>(i);
      auto emo_path = resource_path + ToString(emo_enum) + "/";
      int num_files = CountFiles(emo_path, ".jpg");

      std::cout << "Loading images for emotion " << ToString(emo_enum) << "\n";
      for (int j = 0; j < num_files; j++)
      {
        // JPG 파일 불러오기
        auto file_path = emo_path + "emotions_" + std::to_string(j) + ".jpg";
        auto img_surface = IMG_Load(file_path.c_str());
        emotions[i].AddImage(SDL_CreateTextureFromSurface(renderer, img_surface));
        SDL_FreeSurface(img_surface);
      }
    }

    return 0;
  }

  int EdieDisplayNode::InitEmotionSequences()
  {
    for(int i = 0; i < ToInt(EmotionState::COUNT); i++)
    {
      auto emo_state = static_cast<EmotionState>(i);
      auto emotion_name = ToString(emo_state);
      Parser parser = Parser();
      auto seq = parser.GetSequence(emotion_name);
      emotion_sequences[emo_state].AddFrames(seq);
      RCLCPP_INFO_STREAM(this->get_logger(), "Initialized sequence for emotion: "<< emotion_name <<", frames: "<< emotion_sequences.at(emo_state).Size());
    }

    return 0;
  }

} // namespace aeirobot

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aeirobot::EdieDisplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}