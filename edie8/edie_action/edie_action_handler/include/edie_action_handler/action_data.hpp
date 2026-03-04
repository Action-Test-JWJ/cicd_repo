#ifndef ACTION_DATA_HPP
#define ACTION_DATA_HPP

#include <cstdint>
#include <vector>
 
struct ActionData {
    int display_index;
    int sound_index;
    int motion_index;
    
    // 옵션이 제공되면 우선 사용하여 무작위 선택
    std::vector<int> display_options;
    std::vector<int> sound_options;
};

#endif // ACTION_DATA_HPP 