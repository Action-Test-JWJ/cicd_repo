# robot_llm_state/msg/RobotLlmState.msg

string input_past
string input_current
string input_agent
string output_agent

string thought
string emotion

string goal
string[] plan
string[] step

string observation
string position
string thinking
string[] action
string evaluation
string answer

string is_successful_goal
int32 fail_count_for_current_step
int32 planner_count
int32 binary_score

string pastmode
string mode

bool command_completed
bool fsr_touch
int32 command_case