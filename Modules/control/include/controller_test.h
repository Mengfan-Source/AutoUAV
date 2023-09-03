/***************************************************************************************************************************
* controller_test.h
*
* Author: Qyp
*
* Update Time: 2020.1.10
*               Controller_Test类用于控制器测试，提供圆形、8字、阶跃响应三种参考输入
*                   1、圆形需设置圆心、半径、线速度、旋转方向等参数
*                   2、8字需设置圆心、半径、角速度等参数
*                   3、阶跃响应需设置响应大小及响应间隔
***************************************************************************************************************************/
#ifndef CONTROLLER_TEST_H
#define CONTROLLER_TEST_H

#include <Eigen/Eigen>
#include <math.h>
#include <math_utils.h>
#include <prometheus_control_utils.h>
#include <prometheus_msgs/PositionReference.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


using namespace std;
double target_x,target_y ,target_yaw= 0;
class Controller_Test
{
    public:
        //构造函数
        Controller_Test(void):
            Controller_Test_nh("~")
        {
            Controller_Test_nh.param<float>("Controller_Test/Circle/Center_x", circle_center[0], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/Center_y", circle_center[1], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/Center_z", circle_center[2], 1.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/circle_radius", circle_radius, 2.0);
            Controller_Test_nh.param<float>("Controller_Test/Circle/linear_vel", linear_vel, 0.5);
            Controller_Test_nh.param<float>("Controller_Test/Circle/direction", direction, 1.0);

            Controller_Test_nh.param<float>("Controller_Test/Eight/Center_x", eight_origin_[0], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Eight/Center_y", eight_origin_[1], 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Eight/Center_z", eight_origin_[2], 1.0);
            Controller_Test_nh.param<float>("Controller_Test/Eight/omega", eight_omega_, 0.5);
            Controller_Test_nh.param<float>("Controller_Test/Eight/radial", radial, 2.0);

            Controller_Test_nh.param<float>("Controller_Test/Step/step_length", step_length, 0.0);
            Controller_Test_nh.param<float>("Controller_Test/Step/step_interval", step_interval, 0.0);

        }

        //Printf the Controller_Test parameter
        void printf_param();

        //Controller_Test Calculation [Input: time_from_start; Output: Circle_trajectory;]
        prometheus_msgs::PositionReference Circle_trajectory_generation(float time_from_start);

        prometheus_msgs::PositionReference Eight_trajectory_generation(float time_from_start);

        prometheus_msgs::PositionReference Step_trajectory_generation(float time_from_start);

        prometheus_msgs::PositionReference my_self_line_generation(float total_time,float time_from_start,float x0,float y0,float z0,float x1,float y1,float z1);

    private:

        ros::NodeHandle Controller_Test_nh;

        // Circle Parameter
        Eigen::Vector3f circle_center;
        float circle_radius;
        float linear_vel;
        float direction;         //direction = 1 for CCW 逆时针, direction = -1 for CW 顺时针

        // Eight Shape Parameter
        Eigen::Vector3f eight_origin_;
        float radial;
        float eight_omega_;

        // Step
        float step_length;
        float step_interval;
        
};


prometheus_msgs::PositionReference Controller_Test::Circle_trajectory_generation(float time_from_start)
{
    /*prometheus_msgs::PositionReference Circle_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    // cout << "omega : " << omega  * 180/M_PI <<" [deg/s] " <<endl;
    // cout << "angle : " << angle  * 180/M_PI <<" [deg] " <<endl;

    Circle_trajectory.header.stamp = ros::Time::now();

    Circle_trajectory.time_from_start = time_from_start;

    Circle_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

    Circle_trajectory.position_ref[0] = circle_radius * cos_angle + circle_center[0];
    Circle_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
    Circle_trajectory.position_ref[2] = circle_center[2];

    Circle_trajectory.velocity_ref[0] = - circle_radius * omega * sin_angle;
    Circle_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle;
    Circle_trajectory.velocity_ref[2] = 0;

    Circle_trajectory.acceleration_ref[0] = - circle_radius * pow(omega, 2.0) * cos_angle;
    Circle_trajectory.acceleration_ref[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Circle_trajectory.acceleration_ref[2] = 0;

    // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Circle_trajectory.jerk_ref[2] = 0;

    // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Circle_trajectory.snap_ref[2] = 0;

    Circle_trajectory.yaw_ref = 0;
    // Circle_trajectory.yaw_rate_ref = 0;
    // Circle_trajectory.yaw_acceleration_ref = 0;

    return Circle_trajectory;*/
prometheus_msgs::PositionReference Circle_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    // cout << "omega : " << omega  * 180/M_PI <<" [deg/s] " <<endl;
    // cout << "angle : " << angle  * 180/M_PI <<" [deg] " <<endl;

    Circle_trajectory.header.stamp = ros::Time::now();

    Circle_trajectory.time_from_start = time_from_start;

    Circle_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

    Circle_trajectory.position_ref[0] = circle_radius * cos_angle + circle_center[0];
    Circle_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
    Circle_trajectory.position_ref[2] = circle_center[2]+0.01*time_from_start;

    Circle_trajectory.velocity_ref[0] = - circle_radius * omega * sin_angle;
    Circle_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle;
    Circle_trajectory.velocity_ref[2] = 0.01;

    Circle_trajectory.acceleration_ref[0] = - circle_radius * pow(omega, 2.0) * cos_angle;
    Circle_trajectory.acceleration_ref[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Circle_trajectory.acceleration_ref[2] = 0;

    // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Circle_trajectory.jerk_ref[2] = 0;

    // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Circle_trajectory.snap_ref[2] = 0;

    Circle_trajectory.yaw_ref = 0;
    // Circle_trajectory.yaw_rate_ref = 0;
    // Circle_trajectory.yaw_acceleration_ref = 0;

    return Circle_trajectory;
}

prometheus_msgs::PositionReference Controller_Test::Eight_trajectory_generation(float time_from_start)
{
   /* Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    
    float angle = eight_omega_* time_from_start;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);
    
    Eigen::Vector3f eight_radial_ ;
    Eigen::Vector3f eight_axis_ ;
    eight_radial_ << radial, 0.0, 0.0;
    eight_axis_ << 0.0, 0.0, 2.0;

    position = cos_angle * eight_radial_ + sin_angle * cos_angle * eight_axis_.cross(eight_radial_)
                 + (1 - cos_angle) * eight_axis_.dot(eight_radial_) * eight_axis_ + eight_origin_;

    velocity = eight_omega_ * (-sin_angle * eight_radial_ + (pow(cos_angle, 2) - pow(sin_angle, 2)) * eight_axis_.cross(eight_radial_)
                 + (sin_angle) * eight_axis_.dot(eight_radial_) * eight_axis_);

    acceleration << 0.0, 0.0, 0.0;

    prometheus_msgs::PositionReference Eight_trajectory;

    Eight_trajectory.header.stamp = ros::Time::now();

    Eight_trajectory.time_from_start = time_from_start;

    Eight_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

    Eight_trajectory.position_ref[0] = position[0];
    Eight_trajectory.position_ref[1] = position[1];
    Eight_trajectory.position_ref[2] = position[2];

    Eight_trajectory.velocity_ref[0] = velocity[0];
    Eight_trajectory.velocity_ref[1] = velocity[1];
    Eight_trajectory.velocity_ref[2] = velocity[2];

    Eight_trajectory.acceleration_ref[0] = 0;
    Eight_trajectory.acceleration_ref[1] = 0;
    Eight_trajectory.acceleration_ref[2] = 0;

    Eight_trajectory.yaw_ref = 0;

    // to be continued...

    return Eight_trajectory;*/
prometheus_msgs::PositionReference Circle_trajectory;
    float omega;
    if( circle_radius != 0)
    {
        omega = direction * fabs(linear_vel / circle_radius);
    }else
    {
        omega = 0.0;
    }
    const float angle = time_from_start * omega;
    const float cos_angle = cos(angle);
    const float sin_angle = sin(angle);

    // cout << "omega : " << omega  * 180/M_PI <<" [deg/s] " <<endl;
    // cout << "angle : " << angle  * 180/M_PI <<" [deg] " <<endl;

    Circle_trajectory.header.stamp = ros::Time::now();

    Circle_trajectory.time_from_start = time_from_start;

    Circle_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

    Circle_trajectory.position_ref[0] = circle_radius * cos_angle + circle_center[0];
    Circle_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
    Circle_trajectory.position_ref[2] = circle_center[2]+0.0*time_from_start;

    Circle_trajectory.velocity_ref[0] = - circle_radius * omega * sin_angle;
    Circle_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle;
    Circle_trajectory.velocity_ref[2] = 0.0;

    Circle_trajectory.acceleration_ref[0] = - circle_radius * pow(omega, 2.0) * cos_angle;
    Circle_trajectory.acceleration_ref[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
    Circle_trajectory.acceleration_ref[2] = 0;

    // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
    // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
    // Circle_trajectory.jerk_ref[2] = 0;

    // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
    // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
    // Circle_trajectory.snap_ref[2] = 0;

    Circle_trajectory.yaw_ref = angle;
    // Circle_trajectory.yaw_rate_ref = 0;
    // Circle_trajectory.yaw_acceleration_ref = 0;

    return Circle_trajectory;
}


prometheus_msgs::PositionReference Controller_Test::Step_trajectory_generation(float time_from_start)
{
    /*prometheus_msgs::PositionReference Step_trajectory;

    Step_trajectory.header.stamp = ros::Time::now();

    Step_trajectory.time_from_start = time_from_start;

    Step_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;

    int i = time_from_start / step_interval;

    if( i%2 == 0)
    {
        Step_trajectory.position_ref[0] = step_length;
    }else 
    {
        Step_trajectory.position_ref[0] = - step_length;
    }

    Step_trajectory.position_ref[1] = 0;
    Step_trajectory.position_ref[2] = 1.0;

    Step_trajectory.velocity_ref[0] = 0;
    Step_trajectory.velocity_ref[1] = 0;
    Step_trajectory.velocity_ref[2] = 0;

    Step_trajectory.acceleration_ref[0] = 0;
    Step_trajectory.acceleration_ref[1] = 0;
    Step_trajectory.acceleration_ref[2] = 0;

    Step_trajectory.yaw_ref = 0;

    return Step_trajectory;*/
prometheus_msgs::PositionReference Circle_trajectory;
    float omega;
                        if( circle_radius != 0)//self 根据半径和线速度计算角速度
                        {
                            omega = direction * fabs(linear_vel / circle_radius);
                        }else
                        {
                            omega = 0.0;
                        }
                         float angle = time_from_start * omega; //根据时间和角速度计算角度
                        // const float cos_angle = cos(angle);
                        // const float sin_angle = sin(angle);

                        // cout << "omega : " << omega  * 180/M_PI <<" [deg/s] " <<endl;
                        // cout << "angle : " << angle  * 180/M_PI <<" [deg] " <<endl;

                        Circle_trajectory.header.stamp = ros::Time::now();

                        Circle_trajectory.time_from_start = time_from_start;
                        
                        double h = circle_center[2]+((int)floor(angle)/8)*0.25;
                        //double target_x,target_y ,target_yaw= 0;
                        Circle_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;
                       ROS_INFO("angle1=%lf",angle);
                            angle = angle-((int)floor(angle)/8)*8.0 ;
                            ROS_INFO("angle2=%lf",angle);
                            if(angle<=6.28)
                            {
                                const float cos_angle = cos(angle);
                                const float sin_angle = sin(angle);
                                Circle_trajectory.position_ref[0] = circle_radius * cos_angle + circle_center[0];
                                Circle_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
                                Circle_trajectory.position_ref[2] = h;

                                Circle_trajectory.velocity_ref[0] = - circle_radius * omega * sin_angle;
                                Circle_trajectory.velocity_ref[1] = circle_radius * omega * cos_angle;
                                Circle_trajectory.velocity_ref[2] = 0;

                                Circle_trajectory.acceleration_ref[0] = - circle_radius * pow(omega, 2.0) * cos_angle;
                                Circle_trajectory.acceleration_ref[1] = - circle_radius * pow(omega, 2.0) * sin_angle;
                                Circle_trajectory.acceleration_ref[2] = 0;

                                // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
                                // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
                                // Circle_trajectory.jerk_ref[2] = 0;

                                // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
                                // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
                                // Circle_trajectory.snap_ref[2] = 0;

                                Circle_trajectory.yaw_ref = angle;
                                // Circle_trajectory.yaw_rate_ref = 0;
                                // Circle_trajectory.yaw_acceleration_ref = 0;
                                target_x = Circle_trajectory.position_ref[0];
                                target_y =  Circle_trajectory.position_ref[1];
                                target_yaw = Circle_trajectory.yaw_ref;
                            }
                            else 
                            {
                                const float cos_angle = cos(angle);
                                const float sin_angle = sin(angle);
                                Circle_trajectory.position_ref[0] = target_x ;
                                Circle_trajectory.position_ref[1] = target_y;
                                Circle_trajectory.position_ref[2] = h+0.25;

                                Circle_trajectory.velocity_ref[0] = 0;
                                Circle_trajectory.velocity_ref[1] = 0;
                                Circle_trajectory.velocity_ref[2] = 0;

                                Circle_trajectory.acceleration_ref[0] = -0;
                                Circle_trajectory.acceleration_ref[1] = 0;
                                Circle_trajectory.acceleration_ref[2] = 0;

                                // Circle_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
                                // Circle_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
                                // Circle_trajectory.jerk_ref[2] = 0;

                                // Circle_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
                                // Circle_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
                                // Circle_trajectory.snap_ref[2] = 0;

                                Circle_trajectory.yaw_ref = target_yaw;
                                // Circle_trajectory.yaw_rate_ref = 0;
                                // Circle_trajectory.yaw_acceleration_ref = 0;
                            }
                        
                        

                        return Circle_trajectory;
}
prometheus_msgs::PositionReference Controller_Test::my_self_line_generation(float total_time,float time_from_start,float x0,float y0,float z0,float x1,float y1,float z1)
{
                    prometheus_msgs::PositionReference my_trajectory;
                    // float omega;
                    // if( circle_radius != 0)
                    // {
                    //     omega = direction * fabs(linear_vel / circle_radius);
                    // }else
                    // {
                    //     omega = 0.0;
                    // }
                    // const float angle = time_from_start * omega;
                    // const float cos_angle = cos(angle);
                    // const float sin_angle = sin(angle);

                    // float max_dis=max(x1-x0,y1-y0,z1-z0);

                    my_trajectory.header.stamp = ros::Time::now();

                    my_trajectory.time_from_start = time_from_start;

                    my_trajectory.Move_mode = prometheus_msgs::PositionReference::TRAJECTORY;





                    if(time_from_start <= abs((x1-x0))/0.1)
                    {
                        //my_trajectory.position_ref[0] = x0+(time_from_start/((x1-x0)/0.1))*(x1-x0);
                        if(x1-x0 >= 0)
                        {
                            my_trajectory.position_ref[0] = x0+(time_from_start*0.1);
                        }
                        else
                        {
                             my_trajectory.position_ref[0] = x0-(time_from_start*0.1);
                        }

                    }
                    else
                    {
                        my_trajectory.position_ref[0] = x1;
                    }
                    
                    if(time_from_start <= abs((y1-y0))/0.1)
                    {
                       // my_trajectory.position_ref[1] = y0+(time_from_start/((y1-y0)/0.1))*(y1-y0);
                       if(y1-y0 >= 0)
                       {
                        my_trajectory.position_ref[1] = y0+(time_from_start*0.1);
                       }
                       else
                       {
                        my_trajectory.position_ref[1] = y0-(time_from_start*0.1);
                       }
                       
                    }
                    else
                    {
                        my_trajectory.position_ref[1] = y1;
                    }
                    if(time_from_start <= abs((z1-z0))/0.1)
                    {
                        //my_trajectory.position_ref[2] = z0+(time_from_start/((z1-z0)/0.1))*(z1-z0);
                        if(z1-z0 >= 0)
                        {
                            my_trajectory.position_ref[2] = z0+(time_from_start*0.1);
                        }
                        else
                        {
                            my_trajectory.position_ref[2] = z0-(time_from_start*0.1);
                        }
                    }
                    else
                    {
                        my_trajectory.position_ref[2] = z1;
                    }





                    
                    
                    // my_trajectory.position_ref[0] = x0+(time_from_start/total_time)*max_dis;
                    // my_trajectory.position_ref[1] = circle_radius * sin_angle + circle_center[1];
                    // my_trajectory.position_ref[2] = circle_center[2];

                    my_trajectory.velocity_ref[0] = 0;
                    my_trajectory.velocity_ref[1] = 0;
                    my_trajectory.velocity_ref[2] = 0;

                    my_trajectory.acceleration_ref[0] = 0;
                    my_trajectory.acceleration_ref[1] = 0;
                    my_trajectory.acceleration_ref[2] = 0;

                    // Line_trajectory.jerk_ref[0] = circle_radius * pow(omega, 3.0) * sin_angle;
                    // Line_trajectory.jerk_ref[1] = - circle_radius * pow(omega, 3.0) * cos_angle;
                    // Line_trajectory.jerk_ref[2] = 0;

                    // Line_trajectory.snap_ref[0] = circle_radius * pow(omega, 4.0) * cos_angle;
                    // Line_trajectory.snap_ref[1] = circle_radius * pow(omega, 4.0) * sin_angle;
                    // Line_trajectory.snap_ref[2] = 0;

                    my_trajectory.yaw_ref = 0;
                    // Line_trajectory.yaw_rate_ref = 0;
                    // Line_trajectory.yaw_acceleration_ref = 0;

                    return my_trajectory;
}
// 【打印参数函数】
void Controller_Test::printf_param()
{
    cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Controller_Test Parameter <<<<<<<<<<<<<<<<<<<<<<" <<endl;
    cout <<"Circle Shape:  " <<endl;
    cout <<"circle_center :  " << circle_center[0] <<" [m] "<< circle_center[1] <<" [m] "<< circle_center[2] <<" [m] "<<endl;
    cout <<"circle_radius :  "<< circle_radius <<" [m] " <<"linear_vel : "<< linear_vel <<" [m/s] "<<"direction : "<< direction << endl;

    cout <<"Eight Shape:  " <<endl;
    cout <<"eight_origin_ :  "<< eight_origin_[0] <<" [m] "<< eight_origin_[1] <<" [m] "<< eight_origin_[2] <<" [m] "<<endl;
    cout <<"eight_omega_ :  "<< eight_omega_  <<" [rad/s] " << "radial : "<< radial << endl;

    cout <<"Step:  " <<endl;
    cout <<"step_length :  "<< step_length << " [m] step_interval : "<< step_interval << " [s] "<<endl;
}


#endif
