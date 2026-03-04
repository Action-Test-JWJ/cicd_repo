/*
 * Emotion.hpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25
 */

#ifndef EMOTION_HPP
#define EMOTION_HPP

#include <vector>
#include <memory>
#include <string>
#include <pwd.h>

#include <boost/filesystem.hpp>
#include <SDL2/SDL.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "edie_display/edie_struct.h"

namespace aeirobot
{

class Emotion
{
  private:
    std::string name;
    std::vector<SDL_Texture*> images;

  public:
    Emotion();
    ~Emotion();

    void AddImage(SDL_Texture *img);
    int GetSize();
    std::string GetName() const;
    SDL_Texture *GetImage(int idx);
};

} // namespace aeirobot

#endif
