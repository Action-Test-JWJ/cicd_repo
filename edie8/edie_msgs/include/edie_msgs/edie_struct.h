#ifndef EDIE_STRUCT_H
#define EDIE_STRUCT_H

#include <stdint.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>

#define PI 3.14159294

class Flag
{
private:
    bool flag;

public:
    Flag():flag(false) {}

    bool Check()
    {
        if(flag)
        {
            flag = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool State()
    {
        return flag;
    }

    void On()
    {
        flag = true;
    }

    void Set(bool val)
    {
        flag = val;
    }
};

class Timer
{
private:
    rclcpp::Duration rate;
    rclcpp::Time     last_check;

public:
    Timer() : rate(rclcpp::Duration(std::chrono::nanoseconds(0))) {}  // Explicitly initializing rate with 0 nanoseconds
    Timer(double d) : rate(rclcpp::Duration::from_seconds(d)) 
    {
        last_check = rclcpp::Clock(RCL_ROS_TIME).now();
    }
    Timer(rclcpp::Duration d) : rate(d) 
    {
        last_check = rclcpp::Clock(RCL_ROS_TIME).now();
    }

    double State()
    {
        rclcpp::Time t = rclcpp::Clock().now();
        return (t - last_check).seconds();
    }

    bool Check()
    {
        rclcpp::Time t = rclcpp::Clock().now();
        if(last_check + rate <= t)
        {
            if(last_check + (rate * 2) <= t)
                last_check = t;
            else
                last_check += rate;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool Check(rclcpp::Time t)
    {
        if(last_check + rate <= t)
        {
            if(last_check + (rate * 2) <= t)
                last_check = t;
            else
                last_check += rate;
            return true;
        }
        else
        {
            return false;
        }
    }
};

enum class Mode
{
    MOVING,
    STOP,
};

enum class State
{
    STANBY,     // 대기모드
    TOPPAT,     // 머리쓰담
    REARPAT,    // 엉덩이쓰담 
    LSIDEPAT,   // 왼쪽옆구리쓰담
    RSIDEPAT,   // 오른옆구리쓰담
    TOPHIT,     // 머리때리기
    REARHIT,    // 엉덩이때리기
    LSIDEHIT,   // 왼쪽옆구리때리기
    RSIDEHIT,   // 오른옆구리때리기
};

enum class Action
{
    BLINK,
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
    COUNT = 12
};

enum class EmotionState
{
    BLINK,
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
    COUNT = 12
};

std::ostream& operator<<(std::ostream& os, EmotionState state);

#endif // EDIE_STRUCT_H