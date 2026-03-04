#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <vector>
#include <string.h>
#include <sstream>
#include <pwd.h>
#include <stdlib.h>

#include <edie_sound/sound.hpp>

using namespace std;

namespace Sound
{
  //class SoundFile
  SoundFile::SoundFile(string path)
  {
    path_ = path;
  }

  SoundFile::~SoundFile(){}

  void SoundFile::Play()
  {
    std::string instruction = "/usr/bin/aplay " + path_;
    //system(instruction.c_str());
    execl("/usr/bin/aplay", "aplay", path_.c_str(), NULL);
  }

  //class SoundManager
  SoundManager::SoundManager(string path)
  {
    g_play_pid_ = -1;
    path_ = path;
  }

  SoundManager::~SoundManager(){}

  void SoundManager::Add(string name)
  {
    if(name[0] != '/')
    {
      name = "/" + name;
    }
    files_.push_back(SoundFile(path_+name));
  }

  void SoundManager::Play(int index)
  {
    if(g_play_pid_ != -1)
    {
      kill(g_play_pid_, SIGKILL);
    }
     
    g_play_pid_ = fork();
    
    switch(g_play_pid_)
    {
      case -1:
        fprintf(stderr, "Fork Failed!! \n");
        break;
      case 0:
        files_[index].Play();
        break;
      default:
        break;
    }
  }

  bool SoundManager::ValidCheck(int index) {
      return (index >= 0 && static_cast<std::vector<SoundFile>::size_type>(index) < files_.size());
  }

  pid_t SoundManager::GetPid()
  {
    return g_play_pid_;
  }
};
