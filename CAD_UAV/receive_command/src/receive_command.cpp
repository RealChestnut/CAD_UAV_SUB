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
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <chrono>
#include <regex>


using namespace std;
serial::Serial ser;

std::chrono::high_resolution_clock::time_point end_T=std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point start_T=std::chrono::high_resolution_clock::now();
std::chrono::duration<double> delta_t;


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

bool is_Dock=false;
bool is_Appr=false;
bool is_Mani=false;
bool dock_safety_msg_=false;
bool final_dock_safety_msg_=false;
void serial_safety_Callback(const std_msgs::Bool& msg){
  dock_safety_msg_=msg.data;
  //ROS_INFO_STREAM(dock_safety_msg_);
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
   */

}


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
    ros::Subscriber switch_data_sub = nh.subscribe("switch_onoff",1,switch_data_callback,ros::TransportHints().tcpNoDelay());
    ros::Subscriber serial_safety_sub = nh.subscribe("serial_safety_from_main",1,serial_safety_Callback,ros::TransportHints().tcpNoDelay());


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
    
    string ForToString = "";
    while(ros::ok()){

    ros::spinOnce();

    end_T=std::chrono::high_resolution_clock::now();
    delta_t=end_T-start_T;
    start_T=std::chrono::high_resolution_clock::now();


	//if(!ser.available()){buffer.clear();
	//ser.flush();}


	    //To avoid undefined data for software blocking safety
            //reconfigure_port();
	    if(ser.available()){buffer= ser.read(ser.available());

	    if( is_Dock  && !switch_data && (dock_safety_msg_==false))
           {
                final_dock_safety_msg_ = true;
                //ROS_INFO("DOCK_SAFETY_ON");
           }
           else
           {
                final_dock_safety_msg_ = false;
                //ROS_INFO("DOCK_SAFETY_OFF");
           }
           ForToString = to_string(final_dock_safety_msg_);

           //ser.write(ForToString); // For dock safety
	    
		
	    //ROS_INFO_STREAM(buffer);

	    //receive_data_test(buffer);

	    std_msgs::Float32MultiArray result;

    	result.data.resize(topic_num);
	    
	    receive_data(buffer);
	    
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
			//if(result.data[1]==0){ROS_INFO("zigbee_ting");}

		}
	///////////////////////////////////////////////////////////////////////////// 		
	    last_dump.clear();
		}

	loop_rate.sleep();
	}
}



string A__="";
string B__="";
string C__="";
string Fin__="";

int solution(string msg) {
    int answer = 0;
    char cstr[3];
    strcpy(cstr,msg.c_str());

    for (int i = 0; i<strlen(cstr);i++) {
            if (cstr[i] > 47 && cstr[i] < 58) { // 숫자 판별
            answer = (answer * 10) + (cstr[i] - 48); // 자릿값 올리면서 갱신
        }
    }

    return answer;
}

void parseData(const string& str, vector<string>& values,string& delimiter){
        string msg;
        msg.assign(str);
        /*
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

        }*/

        string A_ = "";
        if(msg.find("<0A") !=string::npos){

                A_ = msg.substr(msg.find("<0A"),3);

        }

        if(msg.find("<1A") !=string::npos){

                A_ = msg.substr(msg.find("<1A"),3);

        }
        if(!A_.empty()){
                //A__ = regex_replace( A_ , regex("[^0-9]"), "" );
                A__ = to_string(solution(A_));



        }


        //////////////////////////////////////////////////////////////

        string B_ = "";
        if(msg.find("B0C") !=string::npos){

                B_ = msg.substr(msg.find("B0C"),3);

        }

        if(msg.find("B1C") !=string::npos){

                B_ = msg.substr(msg.find("B1C"),3);

        }

        if(!B_.empty()){
               // B__ = regex_replace( B_ , regex("[^0-9]"), "" );
               B__ = to_string(solution(B_));

        }


        //////////////////////////////////////////////////////////////

        string C_ = "";
        if(msg.find("D0>") !=string::npos){

                C_ = msg.substr(msg.find("D0>"),3);

        }

        if(msg.find("D1>") !=string::npos){

                C_ = msg.substr(msg.find("D1>"),3);

        }

        if(!C_.empty()){
                //C__ = regex_replace( C_ , regex("[^0-9]"), "" ); //regex 이부분 다른 코드로 교체
                C__ = to_string(solution(C_));

        }
        //////////////////////////////////////////////////////////////
	/*
	ROS_INFO("AAA");
  	ROS_INFO_STREAM(A__);
	ROS_INFO("BBB");
	ROS_INFO_STREAM(B__);
        ROS_INFO("CCC");
	ROS_INFO_STREAM(C__);
        ROS_INFO("---");
	*/
	Fin__ = A__ +','+B__+','+C__;
	
	ROS_INFO_STREAM(Fin__);
	ROS_INFO("___");

	string::size_type Fpos = Fin__.find_first_not_of(delimiter,0);
        string::size_type Lpos = Fin__.find_first_of(delimiter, Fpos);
        while (string::npos != Fpos || string::npos != Lpos)
        {


                values.push_back(Fin__.substr(Fpos, Lpos - Fpos));



                Fpos = Fin__.find_first_not_of(delimiter, Lpos);
                Lpos = Fin__.find_first_of(delimiter, Fpos);

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
string first_str = "";


void receive_data(const string& str){
	    //1. receive data from buffer 
	    // we dont know per buffer has markers ensurely 
	    //2. check start marker end, end marker
	    //first check that buffer has end marker
	    //if has end marker, put data from end marker to string head position
	    //if has start marker, put data from < to the rest
	    //3. and take a loop again until buffer has > end marker
	    //if has end marker,
	    //  <0PE0TE0> 를 완성하고 순차적으로 데이터를 체크하는게 포인트
	    //  문자가 들어올때 <~PE까지 PE~TE까지 TE~>까지 플래그를 두고 데이터를 읽는다
	    //  str.find_first_not_of() :: address반환
	    //  str.find() :: 위치값 반환
	    //  str.substr(시작위치,위치로부터 얼마만큼의 크기)

	    int i = 0;
	    while(i<str.size())
	    {
		
		    if(str[i]=='<')
		    {
			    if(temp_2.empty() && !temp_2_on){temp_2_on=true;}
		    }

		    if(temp_2_on)
		    {
			    temp_2.push_back(str[i]);
			    if(str[i]=='>')
			    {
				    parseData(temp_2, last_dump, dot);
				    ROS_INFO_STREAM(temp_2);
				    temp_2.clear();
				    temp_2_on=false;

			    }
		    }

		    i++;
	    }


}

