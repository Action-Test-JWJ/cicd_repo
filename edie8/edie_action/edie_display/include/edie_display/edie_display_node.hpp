#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>

#include "edie_display/emotion.hpp"
#include "edie_display/sequence_manager.hpp"

using namespace std;

namespace aeirobot
{

class EdieDisplayNode : public rclcpp::Node
{
public:
  EdieDisplayNode();
  ~EdieDisplayNode();

  void PublishDoneMsg(const bool is_done);
  int GetDisplayId() const;
  void SetDisplayId(int val);
  void MainLoop();

private:
  // Check display id has changed
  bool IsDisplayIDChanged();
  // Subscribe display id from edie main node
  void DisplayIndexCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  int InitWindow();
  int InitEmotionSequences();

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_display_done_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_display_idx_;
  int display_id;
  int last_display_id;

  rclcpp::TimerBase::SharedPtr main_loop_timer;
  static constexpr int loop_rate_hz = 24;

  SDL_Window *window;
  SDL_Renderer *renderer;
  SDL_Rect img_frame;
  SDL_Surface *cur_surface;
  EmotionState cur_emotion;
  std::vector<Emotion> emotions;
  std::string resource_path;
  std::unordered_map<EmotionState, SequenceManager> emotion_sequences;

  bool quit = false;
};

// Count the total number of image(.jpg) files in directory
int CountFiles(std::string directory, std::string ext)
{
  namespace fs = boost::filesystem;
  fs::path Path(directory);
  int nb_ext = 0;
  fs::directory_iterator end_iter;

  for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
    if (iter->path().extension() == ext)
      ++nb_ext;

  return nb_ext;
}

} // namespace aeirobot
