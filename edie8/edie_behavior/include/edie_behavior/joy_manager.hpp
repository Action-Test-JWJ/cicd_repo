#ifndef JOY_MANAGER_HPP
#define JOY_MANAGER_HPP

#include <memory>
#include <unordered_map>
#include <deque>
#include <functional>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <aeirobot_toolbox/qos_profiles.hpp>

using namespace std;

namespace aeirobot
{

class Key
{
public:
  // Logitech F710
  class KeyCodeF710
  {
  public:
    enum Axes
    {
      k_l_stick_x = 0,
      k_l_stick_y = 1,
      k_lt        = 2,
      k_r_stick_x = 3,
      k_r_stick_y = 4,
      k_rt        = 5,
      k_arrow_x   = 6,
      k_arrow_y   = 7,
    };
    enum Buttons
    {
      k_a         = 0,
      k_b         = 1,
      k_x         = 2,
      k_y         = 3,
      k_lb        = 4,
      k_rb        = 5,
      k_back      = 6,
      k_start     = 7,
      k_logo      = 8,
      k_l_stick   = 9,
      k_r_stick   = 10,
    };
  };
  
  enum ButtonMap
  {
    k_arrow_up = 0,
    k_arrow_right,
    k_arrow_down,
    k_arrow_left,
    k_a,
    k_b,
    k_x,
    k_y,
    k_lb,
    k_rb,
    k_back,
    k_start,
    k_logo,
    k_l_stick,
    k_r_stick,
  };
  
  enum class EventType
  {
    k_on_key_down, 
    k_on_key_pressed, 
    k_on_key_up
  };

public:
  string name;
  float current_value;
  float default_value;
  float release_value;
  float press_value;

  bool is_key_pressed;
  bool is_press_value_higher;

  function<void()> on_key_down;
  function<void()> on_key_pressed;
  function<void()> on_key_up;
  // function<void()> on_key_released;

  Key(){}
  Key(string key_name, float default_v, float release_v, float press_v)
  : name(key_name), current_value(default_v), default_value(default_v), 
    release_value(release_v), press_value(press_v), is_key_pressed(false)
  {
    is_press_value_higher = (press_value > release_value) ? true : false;

    on_key_down = [](){return;};
    on_key_pressed = [](){return;};
    on_key_up = [](){return;};
    // on_key_released = [](){return;};
  }
  
  void SetKey(const float v)
  {
    float press_dir = (is_press_value_higher) ? 1 : -1;

    if(v * press_dir <= release_value * press_dir)
    {
      OnKeyUp();
    }
    else if(v * press_dir >= press_value * press_dir)
    {
      OnKeyDown();
    }
    OnKeyPressed();
    // OnKeyReleased();

    current_value = v;
  }
  
  void SetKey(const int v)
  {
    int press_dir = (is_press_value_higher) ? 1 : -1;

    if(v * press_dir <= release_value * press_dir)
    {
      OnKeyUp();
    }
    else if(v * press_dir >= press_value * press_dir)
    {
      OnKeyDown();
    }
    OnKeyPressed();
    // OnKeyReleased();

    current_value = (float)v;
  }
  
  void SetKey(const bool v)
  {
    if(!v)
    {
      OnKeyUp();
    }
    else if(v)
    {
      OnKeyDown();
    }
    OnKeyPressed();
    // OnKeyReleased();

    current_value = v ? 1 : 0;
  }

  variant<float, bool> GetKeyValue()
  {
    return current_value;
  }

  void Reset()
  {
    current_value = default_value;
  }

  void SetOnKeyEvent(EventType on_key_event, function<void()> func)
  {
    switch(on_key_event)
    {
    case EventType::k_on_key_up:
      on_key_up = func;
      on_key_up();
      break;
    case EventType::k_on_key_down:
      on_key_down = func;
      on_key_down();
      break;
    case EventType::k_on_key_pressed:
      on_key_pressed = func;
      on_key_pressed();
      break;
    default:
      cout << "Error : on_key_event input does not exist. Please check SetOnKeyEvent() function. \n";
      break;
    }
  }

  void OnKeyDown()
  {
    if(is_key_pressed)
      return;
    is_key_pressed = true;
    // cout <<"key " << name << " down. \n";
    on_key_down();
  }
  void OnKeyPressed()
  {
    if(!is_key_pressed)
      return;
    // cout <<"key " << name << " pressed. \n";
    on_key_pressed();
  }
  void OnKeyUp()
  {
    if(!is_key_pressed)
      return;
    is_key_pressed = false;
    // cout <<"key " << name << " up. \n";
    on_key_up();
  }
  // void OnKeyReleased()
  // {
  //   if(is_key_pressed)
  //     return;
  //   // cout <<"key " << name << " released. \n";
  //   on_key_released();
  // }
};

/*
class KeyCode
{
public:
  unordered_map<string, uint64_t> key_name_to_code;
  unordered_map<uint64_t, string> key_code_to_name;

  KeyCode(){}

  bool AddKeyCode(string name, uint64_t code)
  {
    auto key_name_it = key_code_to_name.find(code);
    if(key_name_it == key_code_to_name.end())
    {
      auto key_code_it = key_name_to_code.find(name);
      if(key_code_it == key_name_to_code.end())
      {
        SetKeyCode(name, code);
        return true;
      }
    }
    else
    {
      cout << "Warning : KeyCode [" << name << " , " << (uint64_t)code << "] already exist. \n";

      return false;
    }
  }

  void SetKeyCode(string name, uint64_t code)
  {
    key_code_to_name[code] = name;
    key_name_to_code[name] = code;
  }

  uint64_t GetKeyCode(string name)
  {
    auto key_it = key_name_to_code.find(name);
    if(key_it == key_name_to_code.end())
    {
      cout << "Warning : KeyCode name [" << name << "] does not exist. \n";
    }
    return key_name_to_code[name];
  }

  string GetKeyName(uint64_t code)
  {
    auto key_it = key_code_to_name.find(code);
    if(key_it == key_code_to_name.end())
    {
      cout << "Warning : KeyCode code [" << code << "] does not exist. \n";
    }
    return key_code_to_name[code];
  }
};
*/

class JoyManager : public rclcpp::Node
{
public:
  vector<shared_ptr<Key>> buttons;
private:
  deque<sensor_msgs::msg::Joy> joy_buf;
  const uint64_t joy_buf_size = 2;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

  rclcpp::TimerBase::SharedPtr loop_timer;
  const chrono::milliseconds loop_interval = 10ms;

  bool is_joy_enabled;
  
  const chrono::milliseconds timeout = 100ms;

public:
  JoyManager(string node_name) : Node(node_name)
  {
    // joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", qos_sensor_profile, bind(&JoyManager::JoyCallback, this, placeholders::_1));
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", qos_topic_profile, bind(&JoyManager::JoyCallback, this, placeholders::_1));
    RCLCPP_INFO_STREAM(this->get_logger(), "joy_manager is ready. ");

    buttons.resize(15);
    buttons[Key::ButtonMap::k_arrow_up]    = std::make_shared<Key>("arrow_up", 0, 0, 1);
    buttons[Key::ButtonMap::k_arrow_right] = std::make_shared<Key>("arrow_right", 0, 0, -1);
    buttons[Key::ButtonMap::k_arrow_down]  = std::make_shared<Key>("arrow_down", 0, 0, -1);
    buttons[Key::ButtonMap::k_arrow_left]  = std::make_shared<Key>("arrow_left", 0, 0, 1);
    buttons[Key::ButtonMap::k_a]           = std::make_shared<Key>("a", 0, 0, 1);
    buttons[Key::ButtonMap::k_b]           = std::make_shared<Key>("b", 0, 0, 1);
    buttons[Key::ButtonMap::k_x]           = std::make_shared<Key>("x", 0, 0, 1);
    buttons[Key::ButtonMap::k_y]           = std::make_shared<Key>("y", 0, 0, 1);
    buttons[Key::ButtonMap::k_lb]          = std::make_shared<Key>("lb", 0, 0, 1);
    buttons[Key::ButtonMap::k_rb]          = std::make_shared<Key>("rb", 0, 0, 1);
    buttons[Key::ButtonMap::k_back]        = std::make_shared<Key>("back", 0, 0, 1);
    buttons[Key::ButtonMap::k_start]       = std::make_shared<Key>("start", 0, 0, 1);
    buttons[Key::ButtonMap::k_logo]        = std::make_shared<Key>("logo", 0, 0, 1);
    buttons[Key::ButtonMap::k_l_stick]     = std::make_shared<Key>("l_stick", 0, 0, 1);
    buttons[Key::ButtonMap::k_r_stick]     = std::make_shared<Key>("r_stick", 0, 0, 1);

    InitJoy();

    loop_timer = this->create_wall_timer(
      loop_interval, 
      bind(&JoyManager::UpdateJoy, this));

    SetActive(true);
  }

  void SetOnKeyEvent(Key::ButtonMap btn, Key::EventType on_key_event, function<void()> func)
  {
    buttons[btn]->SetOnKeyEvent(on_key_event, func);
  }

  float GetAxesData(Key::KeyCodeF710::Axes key_name)
  {
    if (joy_buf.size() <= 0)// || joy_buf.front().axes.size() >= key_name)
    {
      // RCLCPP_INFO_STREAM(this->get_logger(), "joy data doesn't include requested axis data. ");
      return 0;
    }

    return joy_buf.front().axes[key_name];
  }

  int GetButtonsData(Key::KeyCodeF710::Buttons key_name)
  {
    if (joy_buf.size() <= 0)// || joy_buf.front().buttons.size() >= key_name)
    {
      // RCLCPP_INFO_STREAM(this->get_logger(), "joy data doesn't include requested button data. ");
      return 0;
    }

    return joy_buf.front().buttons[key_name];
  }

private:
  void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if(is_joy_enabled)
    {
      AddJoyData(*msg);
    }
  }

  void UpdateJoy()
  {
    if (is_joy_enabled && !joy_buf.empty())
    {
      const auto &last_joy_data = joy_buf.front();
      const auto time_stamp = rclcpp::Time(last_joy_data.header.stamp);
      if ((this->now() - time_stamp) > timeout)
      {
        InitJoy();
      }
      else
      {
        SetButtonData(last_joy_data);
      }
    }
  }

  void SetButtonData(const sensor_msgs::msg::Joy &joy_input)
  {
    buttons[Key::ButtonMap::k_arrow_up]->SetKey(joy_input.axes[Key::KeyCodeF710::Axes::k_arrow_y]);
    buttons[Key::ButtonMap::k_arrow_right]->SetKey(joy_input.axes[Key::KeyCodeF710::Axes::k_arrow_x]);
    buttons[Key::ButtonMap::k_arrow_down]->SetKey(joy_input.axes[Key::KeyCodeF710::Axes::k_arrow_y]);
    buttons[Key::ButtonMap::k_arrow_left]->SetKey(joy_input.axes[Key::KeyCodeF710::Axes::k_arrow_x]);
    buttons[Key::ButtonMap::k_a]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_a]);
    buttons[Key::ButtonMap::k_b]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_b]);
    buttons[Key::ButtonMap::k_x]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_x]);
    buttons[Key::ButtonMap::k_y]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_y]);
    buttons[Key::ButtonMap::k_lb]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_lb]);
    buttons[Key::ButtonMap::k_rb]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_rb]);
    buttons[Key::ButtonMap::k_back]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_back]);
    buttons[Key::ButtonMap::k_start]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_start]);
    buttons[Key::ButtonMap::k_logo]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_logo]);
    buttons[Key::ButtonMap::k_l_stick]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_l_stick]);
    buttons[Key::ButtonMap::k_r_stick]->SetKey(joy_input.buttons[Key::KeyCodeF710::Buttons::k_r_stick]);
  }

  void InitJoy()
  {
    // 모든 버튼의 값을 기본값으로 초기화함
    for(auto &b : buttons)
    {
      b->Reset();
    }
  }

  void AddJoyData(const sensor_msgs::msg::Joy &input_data)
  {
    joy_buf.push_front(input_data);

    if(joy_buf.size() > joy_buf_size)
      joy_buf.pop_back();

    // RCLCPP_INFO_STREAM(this->get_logger(), "joy_buf.size() : " << joy_buf.size());
  }

  void SetActive(bool enable)
  {
    is_joy_enabled = enable;

    if(!is_joy_enabled)
    {
      InitJoy();
      joy_buf.clear();
    }
  }

  bool IsActive()
  {
    return is_joy_enabled;
  }
};

} // namespace aeirobot

#endif // JOY_MANAGER_HPP