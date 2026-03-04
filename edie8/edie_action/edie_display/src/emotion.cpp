/*
 * Emotion.cpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25 EDIE6 version
 *  Refactoring: jwj6488 , 2023.09.19 EDIE8 version
 *  Refactoring: cheonyu1 , 2025.11.02 EDIE8.4 version
 */

#include "edie_display/emotion.hpp"
#include <iostream>

namespace aeirobot
{

// Initialize variable
Emotion::Emotion()
{
  name = "";
  images = std::vector<SDL_Texture*>();
}

Emotion::~Emotion()
{
  for (auto img : images)
  {
    // SDL_FreeSurface(img);
    SDL_DestroyTexture(img);
  }
}

// Add image to images vector 
void Emotion::AddImage(SDL_Texture *img)
{
  if (!img)
    std::cerr << "AddImage() Error. " << std::endl;
  else
    images.push_back(img);
}

// Get total image file size
int Emotion::GetSize()
{
  return images.size();
}

// Get current emotion name
std::string Emotion::GetName() const
{
  return name;
}

// Get current image
SDL_Texture *Emotion::GetImage(int idx)
{
  return images[idx];
}

} // namespace aeirobot