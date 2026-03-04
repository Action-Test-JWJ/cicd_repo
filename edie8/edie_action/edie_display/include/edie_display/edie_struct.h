#ifndef EDIE_STRUCT_H
#define EDIE_STRUCT_H

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace aeirobot
{

enum class State : int
{
  STANBY = 0, // 대기모드
  TOPPAT,     // 머리쓰담
  REARPAT,    // 엉덩이쓰담
  LSIDEPAT,   // 왼쪽옆구리쓰담
  RSIDEPAT,   // 오른옆구리쓰담
  TOPHIT,     // 머리때리기
  REARHIT,    // 엉덩이때리기
  LSIDEHIT,   // 왼쪽옆구리때리기
  RSIDEHIT,   // 오른옆구리때리기
};

enum class EmotionState : int
{
  BLINK = 0,
  CURIOUS,
  SLEEPY,
  AMUSED,
  HOPEFUL,
  CRYING,
  SURPRISED,
  SURPRISED_2,
  DISAPPOINTED,
  LOVE,
  DIZZY,
  DIZZY_2,
  LOW_BATTERY,
  COUNT
};

inline int ToInt(EmotionState state)
{
  return static_cast<int>(state);
}

inline std::string ToString(EmotionState state)
{
  switch(state)
  {
    case EmotionState::BLINK        : return "BLINK";
    case EmotionState::CURIOUS      : return "CURIOUS";
    case EmotionState::SLEEPY       : return "SLEEPY";
    case EmotionState::AMUSED       : return "AMUSED";
    case EmotionState::HOPEFUL      : return "HOPEFUL";
    case EmotionState::CRYING       : return "CRYING";
    case EmotionState::SURPRISED    : return "SURPRISED";
    case EmotionState::SURPRISED_2  : return "SURPRISED_2";
    case EmotionState::DISAPPOINTED : return "DISAPPOINTED";
    case EmotionState::LOVE         : return "LOVE";
    case EmotionState::DIZZY        : return "DIZZY";
    case EmotionState::DIZZY_2      : return "DIZZY_2";
    case EmotionState::LOW_BATTERY  : return "LOW_BATTERY";
    case EmotionState::COUNT        : return "COUNT";
  }
  return "ERROR : " + static_cast<int>(state);
}

inline std::ostream &operator<<(std::ostream &os, EmotionState state)
{
  return os << ToString(state);
}

} // namespace aeirobot

#endif // EDIE_STRUCT_H