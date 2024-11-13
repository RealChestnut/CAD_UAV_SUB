/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <string.h>
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <algorithm>
#include <iterator>
//#include <bits/stdc++.h>
#include <sstream>
#include <std_msgs/UInt16.h>
#include "serial_module/serial_safety_msg.h"
#include <chrono>
#include <std_msgs/Float32.h>

using namespace std;
serial::Serial ser;

static char sSTX() { return static_cast<char>(0x02);}
static char sETX() { return static_cast<char>(0x03);}
static char sACK() { return static_cast<char>(0x06);}
static char sNAK() { return static_cast<char>(0x15);}

int topic_num = 7;
string temp_result;
bool temp_result_on=false;
string buffer="";
string temp_1;
bool temp_1_on=false;
string temp_2;
bool temp_2_on=false;
string temp_3;
bool temp_3_on=false;

string dot = ",";
string startMarker = "<";
string endMarker = ">";
string blink ="";
string receTemp;


string::size_type Start;
string::size_type Tp_end;
string::size_type Tt_end;
string::size_type Tk_end;
string::size_type Fx_end;
string::size_type Fy_end;
string::size_type Fz_end;
string::size_type End;

std::chrono::high_resolution_clock::time_point end_T=std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point start_T=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t;
std_msgs::Float32 dt;

vector<double> DoubleVec;
vector<string> last_dump(7);
void receive_data(const string& str);
void parseData(const string& msg,vector<string>& values,string& delimiter);
void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}
int switch_data;
int x_s_dot = 0;
int x_s = 0;
int switch_CoF = 0.01;
void switch_data_callback(const std_msgs::UInt16& msg){
    switch_data = msg.data;
    /*
    x_s_dot=-switch_CoF*x_s+static_cast<char>(msg.data);
  x_s+=x_s_dot*delta_t.count();
  switch_data =switch_CoF*x_s;
  if(switch_data==1){ROS_INFO_STREAM(switch_data);}
   */
}

bool is_Appr=false;
bool is_Dock=false;
bool is_Mani=false;
void zigbee_command_Callback(const std_msgs::Float32MultiArray& msg){

        is_Appr=msg.data[0]; // approching process
        is_Dock=msg.data[1]; // docking process
        is_Mani=msg.data[2]; // battery switching process
                             // if all false? --> sbus command flight
                             // if Dock mode && after commbined :: flight w.r.t. main drone
}


int open_cnt=0;
int cnt =0;
bool init_serial=true;
bool safety_msg=false;

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Subscriber switch_data_sub = nh.subscribe("switch_onoff",1,switch_data_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_zigbee_command = nh.subscribe("GUI_command",1,zigbee_command_Callback,ros::TransportHints().tcpNoDelay());

    ros::Publisher read_pub_from_main = nh.advertise<std_msgs::Float32MultiArray>("read_serial_magnetic", 1);
    ros::Publisher serial_safety_from_main = nh.advertise<std_msgs::Bool>("serial_safety_from_main", 1);

    ros::ServiceClient serial_safety_client = nh.serviceClient<serial_module::serial_safety_msg>("serial_safety_msg");

    serial_module::serial_safety_msg srv;


    try
    {
        ser.setPort("/dev/ttySERIAL");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");

   
    }else{
        return -1;
    }
    std_msgs::Bool safety_msg_; 
    int safety_cnt = 0;
    int safety_cnt_pass=7;
    int START_cnt =0;
    ros::Rate loop_rate(200);
    while(ros::ok()){
        ros::spinOnce();

	end_T=std::chrono::high_resolution_clock::now();
        delta_t=end_T-start_T;
        start_T=std::chrono::high_resolution_clock::now();
       
        if(switch_data){ser.flush();}	
	//if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
            
 	    std_msgs::Float32MultiArray result;

	    result.data.resize(topic_num);

	    buffer= ser.read(ser.available());
	    if(!switch_data){   
	    if(buffer.find("ABCD")!=string::npos){init_serial=true; ser.write("ABCD"); ROS_INFO_STREAM(buffer);}

	    if(switch_data || !is_Dock){
                    safety_msg_.data=false;
		    srv.request.safety_on = false;
		    serial_safety_client.call(srv);

                    safety_cnt=0;
                    init_serial=false;}

	    
	    if((buffer.find("START")!=string::npos) && init_serial)
	    {
		    ROS_INFO("START");
                    safety_msg_.data = true;
	 	    srv.request.safety_on = true;
		    serial_safety_client.call(srv);

		    START_cnt ++;
		    //ROS_INFO_STREAM(START_cnt);
		    init_serial=false;

	    }

	    //if(buffer.empty()){safety_msg_.data = false;}
	}
	
	   
	    serial_safety_from_main.publish(safety_msg_);
	
	
	    //ROS_INFO_STREAM(buffer);
	
	    //safety_msg_.data = true;
	    //serial_safety_from_main.publish(safety_msg_);
	    receive_data(buffer);
		
	    	if((last_dump.size()%7==0) && (last_dump.size() !=0) && !last_dump.empty()){
		
			string dumi;
			for(int i =0;i<last_dump.size();i+=topic_num)
			{
				for(int j=i; j<i+topic_num; j++){
				//ROS_INFO_STREAM(last_dump[j]);
				result.data[j-i]=strtof((last_dump.at(j)).c_str(),nullptr);
				}
				read_pub_from_main.publish(result);
			}
			
	    	}
		last_dump.clear();
	   // }

	loop_rate.sleep();
	}
}


void parseData(const string& str, vector<string>& values,string& delimiter){
	string msg;
	msg.assign(str);
	
	if((msg.find('<')!=string::npos) && (msg.find('>')!=string::npos))
	{
		msg.erase(std::find(msg.begin(),msg.end(),'<'));
		msg.erase(std::find(msg.begin(),msg.end(),'>'));
	}

		
	string::size_type Fpos = msg.find_first_not_of(delimiter,0);
	string::size_type Lpos = msg.find_first_of(delimiter, Fpos);
	while (string::npos != Fpos || string::npos != Lpos)
	{
		
	
		values.push_back(msg.substr(Fpos, Lpos - Fpos));
		
		

		Fpos = msg.find_first_not_of(delimiter, Lpos);
		Lpos = msg.find_first_of(delimiter, Fpos);
		
	}

	/*
	Start=msg.find('<');
	Tp_end=msg.find("PE");
	Tt_end=msg.find("TE");
	Tk_end=msg.find("KE");
	Fx_end=msg.find("XE");
	Fy_end=msg.find("YE");
	Fz_end=msg.find("ZE");
	End=msg.find('>');
	if(Start!=string::npos && Tp_end !=string::npos)
	{values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Tp_end !=string::npos && Tt_end !=string::npos)
	{values.push_back(msg.substr(Tp_end, Tt_end - Tp_end));}
	if(Tt_end !=string::npos && Tt_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Tk_end !=string::npos && Fx_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Fx_end !=string::npos && Fy_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Fy_end !=string::npos && Fz_end !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}
	if(Fz_end !=string::npos && End !=string::npos)
        {values.push_back(msg.substr(Start, Tp_end - Start));}

	


	
	ROS_INFO_STREAM(" CHECK : "<< start << " | " << Tp_end);
	*/
	}

void receive_data(const string& str){



	    //1. receive data from buffer 
	    // we dont know per buffer has markers ensurely 
	    //2. check start marker end, end marker
	    //first check that buffer has end marker
	    //if has end marker, put data from end marker to string head position
	    //if has start marker, put data from < to the rest
	    //3. and take a loop again until buffer has > end marker
	    //if has end marker, 

	    int i = 0;
	    string st;
	    bool start_flag=false;
	    while(i<str.size())
	    { 
		    
		    if((str[i] == '>')&& temp_2.empty())
		    {
			    temp_1=st;
			    temp_1.clear();
		    }
		    if(str[i]=='<')
		    {
			    start_flag=true;
			    if(temp_2.empty() && !temp_2_on){temp_2_on=true;}
		    }

		    if(temp_2_on)
		    {
			    temp_2.push_back(str[i]);
			    if(str[i]=='>')
			    {
				    parseData(temp_2,last_dump ,dot);
				    //ROS_INFO_STREAM(temp_2);
				    temp_2.clear();
				    temp_2_on=false;


			    }
		    }
		    //str[i]='0';
		    i++;
	    }
		


}

