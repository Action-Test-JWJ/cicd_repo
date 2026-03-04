#ifndef SOUND_HPP
#define SOUND_HPP

#include <signal.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <string.h>
#include <sstream>
#include <pwd.h>

#endif

namespace Sound
{
  class SoundFile
  {
    public:
      SoundFile(std::string path);
      ~SoundFile();

      void Play();

    private:
      std::string path_;
  };

  class SoundManager
  {
    public:
      SoundManager(std::string path);
      ~SoundManager();

      void Add(std::string name);
      void Play(int index);
   
      bool ValidCheck(int index);

      pid_t GetPid();

    private:
      std::string path_;
      pid_t g_play_pid_; 
      std::vector<SoundFile> files_;
  };
};
