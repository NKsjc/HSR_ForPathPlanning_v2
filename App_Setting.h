/*
 * App_Setting.h
 *
 *  Created on: 2014-9-17
 *      Author: alvin
 */

#ifndef APP_SETTING_H_
#define APP_SETTING_H_
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/ini_parser.hpp"
using namespace boost;
#include <string>
#include <iostream>
using namespace std;
typedef struct _setting_param
{
	int udp_port;
	int tcp_port;
	string server_ip;
	string log_path;
	string log_level;
	string channel_car;
	string channel_rigidhand;
	string channel_softhand;
	double org_car_max_vel;
	double org_car_max_acc;
	double org_car_tra_max_vel;
	double org_car_tra_max_acc;
	double org_rigidhand_tra_max_vel;
	double org_rigidhand_tra_max_acc;
	double org_rigidhand_key_step;
	int vel_level;

	double car_max_vel;
	double car_max_acc;
	double car_tra_max_vel;
	double car_tra_max_acc;
	double rigidhand_tra_max_vel;
	double rigidhand_tra_max_acc;
	double rigidhand_key_step;
	double rigidhand_ready_st[4];
	double rigidhand_reset_st[4];
	double softhand_ready_st[4];
	double softhand_reset_st[4];
}setting_param;

class App_Setting {
private:
	setting_param m_param;
	int Check_Setting();
public:
	App_Setting();
	virtual ~App_Setting();
	void Set_Vel_Level(int level);
	int Init_Setting();
	int Refresh_Param();
	const setting_param& get_setting_param() const;
};

#endif /* APP_SETTING_H_ */
