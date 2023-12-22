
#include "cad_uav_controller.hpp"

//직접 계산에 사용하는 변수는 이름을 축약해서
//퍼를리쉬 하기 위해 선언한 변수는 길게
void publisherSet();

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cad_uav");
  ros::NodeHandle nh;

  
  //integratior(PID) limitation
                integ_limit=nh.param<double>("attitude_integ_limit",10);
                integ_yaw_limit=nh.param<double>("attitude_y_integ_limit",10);
                z_integ_limit=nh.param<double>("altitude_integ_limit",100);
                position_integ_limit=nh.param<double>("position_integ_limit",10);

                CoM_hat.x = nh.param<double>("x_center_of_mass",1.0);
                CoM_hat.y = nh.param<double>("y_center_of_mass",1.0);
                CoM_hat.z = nh.param<double>("z_center_of_mass",1.0);

                        //Roll, Pitch PID gains

                        tilt_Par=nh.param<double>("tilt_attitude_r_P_gain",3.5);
                        tilt_Iar=nh.param<double>("tilt_attitude_r_I_gain",3.5);
                        tilt_Dar=nh.param<double>("tilt_attitude_r_D_gain",3.5);

                        tilt_Pap=nh.param<double>("tilt_attitude_p_P_gain",3.5);
                        tilt_Iap=nh.param<double>("tilt_attitude_p_I_gain",3.5);
                        tilt_Dap=nh.param<double>("tilt_attitude_p_D_gain",3.5);

                        //Yaw PID gains
                        tilt_Py=nh.param<double>("tilt_attitude_y_P_gain",5.0);
                        tilt_Iy=nh.param<double>("tilt_attitude_y_I_gain",5.0);
                        tilt_Dy=nh.param<double>("tilt_attitude_y_D_gain",0.3);

                        //Altitude PID gains
                        tilt_Pz=nh.param<double>("tilt_altitude_P_gain",15.0);
                        tilt_Iz=nh.param<double>("tilt_altitude_I_gain",5.0);
                        tilt_Dz=nh.param<double>("tilt_altitude_D_gain",10.0);

                        //Velocity PID gains
                        tilt_Pv=nh.param<double>("tilt_velocity_P_gain",5.0);
                        tilt_Iv=nh.param<double>("tilt_velocity_I_gain",0.1);
                        tilt_Dv=nh.param<double>("tilt_velocity_D_gain",5.0);

                        //Position PID gains
                        tilt_Pp=nh.param<double>("tilt_position_P_gain",3.0);
                        tilt_Ip=nh.param<double>("tilt_position_I_gain",0.1);
                        tilt_Dp=nh.param<double>("tilt_position_D_gain",5.0);

  //initialize ros node//
  //initSubscriber();

    /////////////////////////////////////////////////SUBSCFRIBER START//////////////////////////////////////////////////////
    dynamixel_state = nh.subscribe("joint_states",1,jointstate_Callback, ros::TransportHints().tcpNoDelay()); // servo angle data from dynamixel
    att = nh.subscribe("/imu/data",1,imu_Callback,ros::TransportHints().tcpNoDelay()); // angle data from IMU
    rc_in = nh.subscribe("/sbus",1,sbus_Callback,ros::TransportHints().tcpNoDelay()); // sbus data from arduino
    battery_checker = nh.subscribe("/battery",1,battery_Callback,ros::TransportHints().tcpNoDelay()); // battery sensor data from arduino
    Switch_checker = nh.subscribe("switch_onoff",1,switch_Callback,ros::TransportHints().tcpNoDelay()); // switch interrupt from arduino

//    t265_position=nh.subscribe("/t265_pos",1,t265_position_Callback,ros::TransportHints().tcpNoDelay()); // position data from t265
//    t265_rotation=nh.subscribe("/t265_rot",1,t265_rotation_Callback,ros::TransportHints().tcpNoDelay()); // angle data from t265

//    t265_odom=nh.subscribe("/rs_t265/odom/sample",1,t265_Odom_Callback,ros::TransportHints().tcpNoDelay()); // odometry data from t265

    main2sub_data = nh.subscribe("read_serial_magnetic",1,main2sub_data_Callback,ros::TransportHints().tcpNoDelay()); // wrench data subscribe

    main_pose_data = nh.subscribe("opti_MAIN_pose",1,main_pose_data_Callback,ros::TransportHints().tcpNoDelay());
    sub_pose_data = nh.subscribe("opti_SUB_pose",1,sub_pose_data_Callback,ros::TransportHints().tcpNoDelay());
    sub_zigbee_command = nh.subscribe("GUI_command",1,zigbee_command_Callback,ros::TransportHints().tcpNoDelay());
    /////////////////////////////////////////////////PUBLISHER START//////////////////////////////////////////////////////
    PWMs = nh.advertise<std_msgs::Int16MultiArray>("PWMs", 1); // generated PWM data for logging
    PWM_generator = nh.advertise<std_msgs::Int32MultiArray>("command",1);  // generated PWM data for publish to pca9685
    desired_motor_thrust = nh.advertise<std_msgs::Float32MultiArray>("Forces",1); // generated desired motor force F1,2,3,4

    goal_dynamixel_position = nh.advertise<sensor_msgs::JointState>("goal_dynamixel_position",1); // desired servo angle command ::theta 1234

    euler = nh.advertise<geometry_msgs::Vector3>("angle",1); // euler angle
    desired_angle = nh.advertise<geometry_msgs::Vector3>("desired_angle",1); // desired euler angle


    desired_torque = nh.advertise<geometry_msgs::Vector3>("torque_d",1);
    torque_dhat_pub = nh.advertise<geometry_msgs::Vector3>("torque_dhat",1); //23.10.09
    desired_splited_yaw_torque = nh.advertise<std_msgs::Float32MultiArray>("splited_yaw_torque_d",1); // 23.10.05

    linear_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel",1);
    desired_velocity = nh.advertise<geometry_msgs::Vector3>("lin_vel_d",1);
    linear_velocity_opti = nh.advertise<geometry_msgs::Vector3>("lin_vel_opti",1);//23.12.22

    angular_velocity = nh.advertise<geometry_msgs::Vector3>("ang_vel",1);

    desired_position = nh.advertise<geometry_msgs::Vector3>("position_d",1);
    position = nh.advertise<geometry_msgs::Vector3>("position",1);

    desired_force = nh.advertise<geometry_msgs::Vector3>("force_d",1);

    battery_voltage = nh.advertise<std_msgs::Float32>("battery_voltage",1);

    Force_allocation_factor = nh.advertise<std_msgs::Float32MultiArray>("force_allocation_factor",1);

    delta_time = nh.advertise<std_msgs::Float32>("delta_t",1);
    ToSubAgent = nh.advertise<std_msgs::String>("ToSubData",1);

    
    ros::Timer timerPublish = nh.createTimer(ros::Duration(1.0/200.0),std::bind(publisherSet));
    ros::spin();
    return 0;
}
 void publisherSet(){
    Clock();


    shape_detector(); 
    // receiving data from arduino. (switch on off, connector servo rotation)
    // (swiching safety function +++)
    UpdateParameter(module_num); // setMoI,pid_Gain_Setting, etc. W.R.T. Combined or NOT
    //Switching_safety(); //23.11.16
    if(main_agent)
    {
      if(!kill_mode) // kill_mode toggle position
      {

      Command_Generator();
      //여기에 결합 sequnce mode fucntion선언되면됌
      
      setCM_Xc_p2();
      attitude_controller();
      if(DOB_mode){torque_DOB();} //나중에는 배터리 교환할때만 ON
      position_controller();

      altitude_controller();

      Accelerometer_LPF();
      velocity_controller();
      K_matrix();
      wrench_allocation(); //contain data_2_sub fucntion
      yaw_torque_distribute();

      PWM_signal_Generator(); //contain :: setSA, etc
      
      }
      else
      {
        
	      wrench_allocation();  //contain data_2_sub fucntion
        reset_data();
        pwm_Kill();
        
      }
    }
    
    if(!main_agent)
    {
      if(!kill_mode){
        setCM_Xc_p2();
        yaw_torque_distribute(); //23.10.05
        PWM_signal_Generator();
        }   
      else{
        reset_data();
        pwm_Kill();
        } 
    }



    PublishData();
 }
