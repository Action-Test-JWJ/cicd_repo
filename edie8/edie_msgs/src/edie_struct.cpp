#include "edie_msgs/edie_struct.h"

std::ostream& operator<<(std::ostream& os, EmotionState state)
{
  switch(state)
  {
    case EmotionState::BLINK        : os << "BLINK"; break;
    case EmotionState::CURIOUS      : os << "CURIOUS"; break;
    case EmotionState::SLEEPY       : os << "SLEEPY"; break;
    case EmotionState::AMUSED       : os << "AMUSED"; break;
    case EmotionState::HOPEFUL      : os << "HOPEFUL"; break;
    case EmotionState::CRYING       : os << "CRYING"; break;
    case EmotionState::SURPRISED    : os << "SURPRISED"; break;
    case EmotionState::SURPRISED_2  : os << "SURPRISED_2"; break;
    case EmotionState::DISAPPOINTED : os << "DISAPPOINTED"; break;
    case EmotionState::LOVE         : os << "LOVE"; break;
    case EmotionState::DIZZY        : os << "DIZZY"; break;
    case EmotionState::DIZZY_2      : os << "DIZZY_2"; break;
    default                         : os << "ERROR : " << static_cast<int>(state); break;
  }
  return os; 
}
