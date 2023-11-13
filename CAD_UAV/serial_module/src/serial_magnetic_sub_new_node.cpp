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
#include <string.h>
#include <string>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <algorithm>
#include <iterator>
//#include <bits/stdc++.h>
#include <sstream>

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
bool reconfig_port_flag=false;

string dot = ",";
string startMarker = "<";
string endMarker = ">";
string blink ="";
string receTemp;
string message_safety="0";

string::size_type Start;
string::size_type Tp_end;
string::size_type Tt_end;
string::size_type Tk_end;
string::size_type Fx_end;
string::size_type Fy_end;
string::size_type Fz_end;
string::size_type End;


vector<double> DoubleVec;
vector<string> last_dump(7);
void receive_data(const string& str);
void receive_data_test(const string& str);
void reconfigure_port();
void parseData(const string& msg,vector<string>& values,string& delimiter);
void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}


void try_set_port(){
	ROS_INFO("try_set_port");
	/*
	ser.setPort("/dev/ttyS0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);

        ser.setTimeout(to);
	*/

        ser.open();
	reconfig_port_flag=false;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub_from_main = nh.advertise<std_msgs::Float32MultiArray>("read_serial_magnetic", 1);

    try
    {
        ser.setPort("/dev/ttyS0");
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

    ros::Rate loop_rate(200);
    while(ros::ok()){

        ros::spinOnce();

	buffer.clear();
	
	
	
	//if(ser.available()){
            //ROS_INFO_STREAM("Reading from serial port");
	    
	
	    if(ser.available()){ser.write("1");}

	    //To avoid undefined data for software blocking safety
            reconfigure_port();
	    receive_data_test(buffer);

	    if(ser.available()){buffer= ser.read(ser.available());}
	    

	    ROS_INFO_STREAM(buffer);

	    std_msgs::Float32MultiArray result;

            result.data.resize(topic_num);
	    
	    receive_data(buffer);
	     
	 ////////////// parsing from complete data to std_msgs   /////////////////////
	    	if(last_dump.size()==topic_num){

			string dumi;
			for(int i =0;i<last_dump.size();i++)
			{
				dumi = last_dump.at(i);
				result.data[i]=strtof(dumi.c_str(),nullptr);
			}
			read_pub_from_main.publish(result);
		}
	////////////////////////////////////////////////////////////////////////
			
	    	
		last_dump.clear();
	    //}
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

}
int error_cnt = 0;
int e_middle_cnt=0;
bool e_middle_flag=false;
void receive_data_test(const string& str){
	int i = 0;
	int check_ascii;
	while(i<str.size()){
                
		check_ascii = static_cast<int>(str[i]);
		//ROS_INFO_STREAM(check_ascii);
		if(check_ascii<0){error_cnt++;}
		if(error_cnt>3){
			if(ser.available()){
				ser.write("0");}}
		if(error_cnt>50){
			ser.close();
			error_cnt=0;}

		i++;
	}

	if(!ser.isOpen()){
		ROS_INFO("close");
		reconfig_port_flag = true;
	}
}
int wait_cnt=0;
void reconfigure_port(){
	if(!ser.isOpen() && reconfig_port_flag){
	wait_cnt++;}
	if(wait_cnt>1000){
	try_set_port();
	ROS_INFO("try_set_open");
	if(ser.isOpen()){
	ROS_INFO("REOPEN_PORT");
	wait_cnt=0;
	/*reconfig_port_flag=false;*/}}
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
	    string st= "";
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
				    ROS_INFO_STREAM(temp_2);
				    temp_2.clear();
				    temp_2_on=false;


			    }
		    }

		    st[i]=str[i];
		    i++;
	    }

}


