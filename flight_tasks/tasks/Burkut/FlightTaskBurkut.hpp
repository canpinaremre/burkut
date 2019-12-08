#pragma once

#include "FlightTask.hpp"

/*
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/orbit_status.h>
#include <StraightLine.hpp>
#include "FlightTaskManualAltitudeSmooth.hpp"
*/
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

};
