// /home/tenu/ros2_ws/src/edie8/edie_action/edie_display/include/edie_display/sequence_manager.hpp
#ifndef EDIE_DISPLAY_SEQUENCE_MANAGER_HPP
#define EDIE_DISPLAY_SEQUENCE_MANAGER_HPP

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <optional>
#include <functional>
#include <chrono>

#include "edie_display/parser.hpp"

namespace aeirobot
{

  class SequenceManager
  {
  public:
    SequenceManager() = default;

    // Sequence manipulation
    void AddFrame(const int f)
    {
      frames.push_back(f);
    }

    void AddFrames(const std::vector<int> &fs)
    {
      frames.insert(frames.end(), fs.begin(), fs.end());
    }

    void Clear()
    {
      frames.clear();
      index = 0;
    }

    std::size_t Size() const
    {
      return frames.size();
    }

    bool Empty() const
    {
      return Size() == 0;
    }

    void Reset()
    {
      index = 0;
    }

    int GetCurFrame()
    {
      if (Empty())
        return -1;

      return frames[index];
      index = 0;
    }

    int GoNextFrame()
    {
      if (Empty())
        return -1;

      index++;
      if (index >= frames.size())
      {
        index = 0;
      }
      return frames[index];
    }

  private:
    std::vector<int> frames;
    std::size_t index{0};
  };

} // namespace edie_display

#endif // EDIE_DISPLAY_SEQUENCE_MANAGER_HPP