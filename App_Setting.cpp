/*
 * App_Setting.cpp
 *
 *  Created on: 2014-9-17
 *      Author: alvin
 */

#include "App_Setting.h"

App_Setting::App_Setting() {
	// TODO Auto-generated constructor stub
	if (0!=this->Init_Setting())
	{
		throw logic_error("load ini file failed!");
	}
}

App_Setting::~App_Setting() {
	// TODO Auto-generated destructor stub
}
void App_Setting::Set_Vel_Level(int level){
	if(level<1||level>3)
		return;
	m_param.vel_level=level;
	m_param.car_max_vel=m_param.org_car_max_vel*m_param.vel_level/3.0f;
	m_param.car_max_acc=m_param.org_car_max_acc*m_param.vel_level/3.0f;
	m_param.car_tra_max_acc=m_param.org_car_tra_max_acc*m_param.vel_level/3.0f;
	m_param.car_tra_max_vel=m_param.org_car_tra_max_vel*m_param.vel_level/3.0f;

	m_param.rigidhand_tra_max_vel=m_param.org_rigidhand_tra_max_vel*m_param.vel_level/3.0f;
	m_param.rigidhand_tra_max_acc=m_param.org_rigidhand_tra_max_acc*m_param.vel_level/3.0f;
	m_param.rigidhand_key_step=m_param.org_rigidhand_key_step*m_param.vel_level/3.0f;

}
int App_Setting::Refresh_Param(){
	return this->Init_Setting();
}
int App_Setting::Check_Setting(){
	int flag=0;
	if(m_param.car_max_vel>=2.9){
		m_param.car_max_vel=2.9;
		flag++;
	}
	if(m_param.car_tra_max_acc>=2.9){
		m_param.car_tra_max_acc=2.9;
		flag++;
	}
	if(m_param.car_tra_max_vel>=m_param.car_max_vel){
		m_param.car_tra_max_vel=m_param.car_max_vel;
		flag++;
	}
	return flag;
}
int App_Setting::Init_Setting(){
	using property_tree::ptree;
	ptree pt;
	try
	{
		read_ini("config.ini", pt);
		m_param.udp_port=pt.get<int>("project.udp_port");
		m_param.tcp_port=pt.get<int>("project.tcp_port");
		m_param.server_ip=pt.get<string>("project.server_ip");

		m_param.log_path=pt.get<string>("project.log_path");
		m_param.log_level=pt.get<string>("project.log_level");

		m_param.channel_car=pt.get<string>("project.channel_car");
		m_param.channel_rigidhand=pt.get<string>("project.channel_rigidhand");
		m_param.channel_softhand=pt.get<string>("project.channel_softhand");


		m_param.vel_level=pt.get<int>("project.vel_level");
		m_param.org_car_max_vel=pt.get<double>("project.car_max_vel");
		m_param.org_car_max_acc=pt.get<double>("project.car_max_acc");
		m_param.org_car_tra_max_acc=pt.get<double>("project.car_tra_max_acc");
		m_param.org_car_tra_max_vel=pt.get<double>("project.car_tra_max_vel");

		m_param.org_rigidhand_tra_max_vel=pt.get<double>("project.rigidhand_tra_max_vel");
		m_param.org_rigidhand_tra_max_acc=pt.get<double>("project.rigidhand_tra_max_acc");
		m_param.org_rigidhand_key_step=pt.get<double>("project.rigidhand_key_step");


		m_param.car_max_vel=m_param.org_car_max_vel*m_param.vel_level/3.0f;
		m_param.car_max_acc=m_param.org_car_max_acc*m_param.vel_level/3.0f;
		m_param.car_tra_max_acc=m_param.org_car_tra_max_acc*m_param.vel_level/3.0f;
		m_param.car_tra_max_vel=m_param.org_car_tra_max_vel*m_param.vel_level/3.0f;

		m_param.rigidhand_tra_max_vel=m_param.org_rigidhand_tra_max_vel*m_param.vel_level/3.0f;
		m_param.rigidhand_tra_max_acc=m_param.org_rigidhand_tra_max_acc*m_param.vel_level/3.0f;
		m_param.rigidhand_key_step=m_param.org_rigidhand_key_step*m_param.vel_level/3.0f;


		m_param.rigidhand_ready_st[0]=pt.get<double>("project.rigidhand_ready_st1");
		m_param.rigidhand_ready_st[1]=pt.get<double>("project.rigidhand_ready_st2");
		m_param.rigidhand_ready_st[2]=pt.get<double>("project.rigidhand_ready_st3");
		m_param.rigidhand_ready_st[3]=pt.get<double>("project.rigidhand_ready_st4");
		
		m_param.rigidhand_reset_st[0]=pt.get<double>("project.rigidhand_reset_st1");
		m_param.rigidhand_reset_st[1]=pt.get<double>("project.rigidhand_reset_st2");
		m_param.rigidhand_reset_st[2]=pt.get<double>("project.rigidhand_reset_st3");
		m_param.rigidhand_reset_st[3]=pt.get<double>("project.rigidhand_reset_st4");
		
		m_param.softhand_ready_st[0]=pt.get<double>("project.softhand_ready_st1");
		m_param.softhand_ready_st[1]=pt.get<double>("project.softhand_ready_st2");
		m_param.softhand_ready_st[2]=pt.get<double>("project.softhand_ready_st3");
		m_param.softhand_ready_st[3]=pt.get<double>("project.softhand_ready_st4");
		
		m_param.softhand_reset_st[0]=pt.get<double>("project.softhand_reset_st1");
		m_param.softhand_reset_st[1]=pt.get<double>("project.softhand_reset_st2");
		m_param.softhand_reset_st[2]=pt.get<double>("project.softhand_reset_st3");
		m_param.softhand_reset_st[3]=pt.get<double>("project.softhand_reset_st4");
	}
	catch(...)
	{
	return -1;
	}
	int flag=this->Check_Setting();
	if(flag>0){
		throw logic_error("The setting param has  error !");
	}
	
	
/*

	char tm_buf[CHAR_MAX] = {0};
	time_t t = time(NULL);
	strftime(tm_buf, CHAR_MAX - 1, "%Y_%m_%d_%H%M%S\\", localtime(&t));

	m_param.logpath += tm_buf;
	if (_mkdir(m_param.logpath.c_str()) < 0)
	return false;
*/
	return 0;
}
const setting_param& App_Setting::get_setting_param() const
{
	return m_param;
}





