#ifndef SINGLETON_HPP
#define SINGLETON_HPP

#include <memory>
#include <mutex>

using namespace std;

template<typename T>
class Singleton
{
public:
  static T* GetInstance()
  {
    call_once(singleton_flag, [] {
      instance.reset();
      instance = make_shared<T>();
    });

    return instance.get();
  }

private:
  static shared_ptr<T> instance;
  static once_flag singleton_flag;
};

template<typename T> shared_ptr<T> Singleton<T>::instance = NULL;
template<typename T> once_flag Singleton<T>::singleton_flag;

#endif // SINGLETON_HPP