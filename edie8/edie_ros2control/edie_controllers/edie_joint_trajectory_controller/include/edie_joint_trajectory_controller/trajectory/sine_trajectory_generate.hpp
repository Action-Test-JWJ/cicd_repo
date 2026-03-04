#ifndef SINE_TRAJECTORY_GENERATOR_HPP
#define SINE_TRAJECTORY_GENERATOR_HPP

class SineTrajectoryGenerator
{
public:
    SineTrajectoryGenerator();
    ~SineTrajectoryGenerator();

    double GenerateSineTrajectory(double initial_value_, double final_value_,
                                                                            double initial_velocity_, double final_velocity_,
                                                                            double initial_acc, double final_acc,
                                                                            double initial_time_, double final_time_);

    bool DetectChangeFinalValue(double pose, double velocity_, double time_);

    double control_time;
    double trajectory_final_value;

    double initial_time;
    double initial_pose;
    double initial_velocity;
    double initial_acc;

    double current_time;
    double current_pose;
    double current_velocity;
    double current_acc;

    double final_time;
    double final_pose;
    double final_velocity;
    double final_acc;

private:
};

#endif // SINE_TRAJECTORY_GENERATOR_HPP