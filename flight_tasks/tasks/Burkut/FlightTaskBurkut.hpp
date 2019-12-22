#pragma once

#include "FlightTask.hpp"
#include <cmath>

//obstacleAvoidance example
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module_params.h>
#include <commander/px4_custom_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/PublicationQueued.hpp>
#include <uORB/Publication.hpp>
//#include <lib/hysteresis/hysteresis.h>



#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/position_setpoint_triplet.h>


class FlightTaskBurkut : public FlightTask
{

public:
	FlightTaskBurkut() = default;
	virtual ~FlightTaskBurkut() = default;

	bool update() override;
	bool activate(vehicle_local_position_setpoint_s last_setpoint) override;

private:
	int _stage = 0;
	float _origin_z = 0.0f;
	float _origin_y = 0.0f;
	float _origin_x = 0.0f;
	float _origin_yaw = 0.0f;
	float _counter = 0.0f;
	float _radian_of_degree = 0.0f;
	float _radius_of_chamber = 3.0f;
	float _yaw_speed = 300.0f;
	float _counter_speed = 0.01f;

	int32_t _default_mpc_auto_mode = 1;
protected:
	DEFINE_PARAMETERS(
					(ParamInt<px4::params::MPC_AUTO_MODE>) _param_mpc_auto_mode //mode
				       )
	uORB::PublicationQueued<vehicle_command_s> _pub_vehicle_command{ORB_ID(vehicle_command)};	/**< vehicle command do publication */

	/**
	 * Publishes vehicle command.
	 */
	void _publishVehicleCmdDoLand();
};
