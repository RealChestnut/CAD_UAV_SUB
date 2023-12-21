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
#include <sys/ioctl.h>
#include <sstream>



using namespace std;
serial::Serial ser;

int topic_num = 3;
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
int serial_open_cnt=0;


vector<double> DoubleVec;
vector<string> last_dump(3);
void receive_data(const string& str);
void receive_data_test(const string& str);
void reconfigure_port();
void parseData(const string& msg,vector<string>& values,string& delimiter);
void serial_open_safety();

void try_set_port(){
	ROS_INFO("try_set_port");
	
	ser.setPort("/dev/ttyZIGBEE");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);

        ser.setTimeout(to);
        ser.open();
}

int main (int argc, char** argv){
    ros::init(argc, argv, "receive_command_node");
    ros::NodeHandle nh;

    ros::Publisher read_command_from_PC = nh.advertise<std_msgs::Float32MultiArray>("GUI_command", 1);


    try
    {
        ser.setPort("/dev/ttyZIGBEE");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		serial::flowcontrol_t flow_state = serial::flowcontrol_t::flowcontrol_none;
		ser.setFlowcontrol(flow_state);


		ser.setTimeout(to);
		ser.open();
		ser.flush(); //데이터가 먼저 cpu queue에 maximum으로 들어와있어서 포트가 터지는거 방지용
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

	//if(!ser.available()){buffer.clear();
	//ser.flush();}


	    //To avoid undefined data for software blocking safety
            //reconfigure_port();
	    if(ser.available()){buffer= ser.read(ser.available());
		
	    //ROS_INFO_STREAM(buffer);

	    //receive_data_test(buffer);

	    std_msgs::Float32MultiArray result;

    	result.data.resize(topic_num);
	    
	    receive_data(buffer);
	    //ROS_INFO_STREAM(last_dump.size());
	 ////////////// parsing from complete data to std_msgs   /////////////////////
	    
		if(last_dump.size()==topic_num){
		
			string dumi="";
			//ROS_INFO_STREAM(last_dump.size());
			for(int i =0;i<last_dump.size();i++)
			{
				dumi = last_dump.at(i);
				//ROS_INFO_STREAM("IM_DUMI :: " << dumi);
				result.data[i]=strtof(dumi.c_str(),nullptr); // error code last dump의 크기보다 receive_data의 크기가 작아 발생한 애러
				
			}
			read_command_from_PC.publish(result);
		}
	/////////////////////////////////////////////////////////////////////// 		
	    last_dump.clear();
		}

	loop_rate.sleep();
	}
}

void serial_open_safety(){
	
	if(ser.available()){serial_open_cnt++;}
	



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
		if(check_ascii<0){
			ROS_INFO_STREAM(check_ascii);
			ser.flush();
			buffer.clear();
			//error_cnt++;
		}
		
		/*if(error_cnt>3){
			if(ser.available()){
				ser.write("0");}}
		if(error_cnt>50){
			ser.close();
			error_cnt=0;}

		i++;*/
	}
	/*
	if(!ser.isOpen()){
		ROS_INFO("close");
		reconfig_port_flag = true;
	}*/
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

int cnt_start=0;
int cnt_end=0;
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
	    bool start_flag=false;
	    while(i<str.size())
	    {
		    if((str[i] == '>' ) && temp_2.empty())
		    {
			    temp_1=str;
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
				    cnt_start++;
				    parseData(temp_2,last_dump ,dot);
			   	    cnt_end++;
				    ROS_INFO_STREAM(temp_2);
				    temp_2.clear();
				    temp_2_on=false;


			    }
		    }

		    i++;
	    }


}

