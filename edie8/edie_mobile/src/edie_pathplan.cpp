#include "edie_mobile/edie_mobile_main.hpp"

void EdieMobileNode::MakePath(Pose2D start, Pose2D end)
{
    Path2D path;
    for (int i = 0; i < bezier_resolution + 1; ++i)
    {
        double ratio = static_cast<double>(i) / (bezier_resolution + 1 - 1);

        path.x.push_back(start.x + ratio * (end.x - start.x));
        path.y.push_back(start.y + ratio * (end.y - start.y));
        path.theta.push_back(std::atan2(end.y - start.y, end.x - start.x));
    }
    line_path = path;
}
// local path-----------------------------------------------------------------------------------
std::pair<double, double> EdieMobileNode::CalculatePoint(double x, double y, double theta, double distance, double offset)
{
    double newX = x + (distance * cos(theta)) - (offset * sin(theta));
    double newY = y + (distance * sin(theta)) + (offset * cos(theta));
    return std::make_pair(newX, newY);
}

void EdieMobileNode::PubBezierCurve(Path2D bezier_curve)
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->get_clock()->now();

    path.header.frame_id = "map"; // or the appropriate frame
    for (std::size_t i = 0; i < bezier_curve.x.size(); i++)
    {
        geometry_msgs::msg::PoseStamped pose;
        // pose.header.stamp = path.header.stamp;
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map"; // or the appropriate frame

        pose.pose.position.x = bezier_curve.x[i];
        pose.pose.position.y = bezier_curve.y[i];
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    pub_path->publish(path);
}

void EdieMobileNode::PlanningPoint()
{
    double err_dis_Point1 = CalculateDistance(Point1, cur_pose_2D);
    double err_dis_Point2 = CalculateDistance(Point2, cur_pose_2D);
    double err_dis_Point3 = CalculateDistance(Point3, cur_pose_2D);
    double err_dis_Point4 = CalculateDistance(Point4, cur_pose_2D);

    // double err_dis_Point1_theta = NormalizeAngle(Point1.theta - cur_pose_2D.theta);
    // double err_dis_Point2_theta = NormalizeAngle(Point2.theta - cur_pose_2D.theta);
    // double err_dis_Point3_theta = NormalizeAngle(Point3.theta - cur_pose_2D.theta);
    // double err_dis_Point4_theta = NormalizeAngle(Point4.theta - cur_pose_2D.theta);

    if (point_move_flag)
    {
        point_queue = std::queue<int>();
        point_move_flag = false;

        if ((goal_point == 0 && err_dis_Point1 > dist_stop_condition) ||
            (goal_point == 1 && err_dis_Point2 > dist_stop_condition) ||
            (goal_point == 2 && err_dis_Point3 > dist_stop_condition) ||
            (goal_point == 3 && err_dis_Point4 > dist_stop_condition))
        {
            int closestPointId = 0;
            double minDistance = err_dis_Point1;

            //if (err_dis_Point2 < minDistance)
            //{
            //    closestPointId = 1;
            //    minDistance = err_dis_Point2;
            //}
            if (err_dis_Point3 < minDistance)
            {
                closestPointId = 2;
                minDistance = err_dis_Point3;
            }
            if (err_dis_Point4 < minDistance)
            {
                closestPointId = 3;
                minDistance = err_dis_Point4;
            }
            point_queue.push(closestPointId);

            if (goal_point > closestPointId)
            {
                for (int i = closestPointId + 1; i <= goal_point; i++)
                {
                    point_queue.push(i);
                    // std::cout<<" + "<<i<<std::endl;
                }
            }
            else if (goal_point < closestPointId)
            {
                for (int i = closestPointId - 1; i >= goal_point; i--)
                {
                    point_queue.push(i);
                    // std::cout<<" - "<<i<<std::endl;
                }
            }
        }
        else if ((goal_point == 0 && err_dis_Point1 < dist_stop_condition) ||
                 (goal_point == 1 && err_dis_Point2 < dist_stop_condition) ||
                 (goal_point == 2 && err_dis_Point3 < dist_stop_condition) ||
                 (goal_point == 3 && err_dis_Point4 < dist_stop_condition))
        {
            point_queue.push(goal_point);
        }
    }

    if (!point_queue.empty())
    {
        if (point_queue.front() == 0)
        {
            goal_pose_2D.x = Point1.x;
            goal_pose_2D.y = Point1.y;
            goal_pose_2D.theta = Point1.theta;
        }
        else if (point_queue.front() == 1)
        {
            goal_pose_2D.x = Point2.x;
            goal_pose_2D.y = Point2.y;
            goal_pose_2D.theta = Point2.theta;
        }
        else if (point_queue.front() == 2)
        {
            goal_pose_2D.x = Point3.x;
            goal_pose_2D.y = Point3.y;
            goal_pose_2D.theta = Point3.theta;
        }
        else if (point_queue.front() == 3)
        {
            goal_pose_2D.x = Point4.x;
            goal_pose_2D.y = Point4.y;
            goal_pose_2D.theta = Point4.theta;
        }
        if ((point_queue.front() == 0 && err_dis_Point1 < dist_stop_condition) ||
            (point_queue.front() == 1 && err_dis_Point2 < dist_stop_condition) ||
            (point_queue.front() == 2 && err_dis_Point3 < dist_stop_condition) ||
            (point_queue.front() == 3 && err_dis_Point4 < dist_stop_condition))
        {
            // std_msgs::Int32 current_point;
            // current_point.data = point_queue.front();
            // current_point_pub.publish(current_point);
            point_queue.pop();
        }
    }
}
