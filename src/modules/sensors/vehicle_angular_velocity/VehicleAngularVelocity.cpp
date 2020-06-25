/****************************************************************************
 *
 *   Copyright (c) 2019-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "VehicleAngularVelocity.hpp"

#include <px4_platform_common/log.h>

using namespace matrix;
using namespace time_literals;

namespace sensors
{

VehicleAngularVelocity::VehicleAngularVelocity() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// gyro low pass
	for (auto &lp : _lp_filter_velocity) {
		lp.set_cutoff_frequency(kInitialRateHz, _param_imu_gyro_cutoff.get());
	}

	// notch filter
	for (auto &nf : _notch_filter_velocity) {
		nf.setParameters(kInitialRateHz, _param_imu_gyro_nf_freq.get(), _param_imu_gyro_nf_bw.get());
	}

	// angular acceleration low pass
	for (auto &lp : _lp_filter_acceleration) {
		lp.set_cutoff_frequency(kInitialRateHz, _param_imu_dgyro_cutoff.get());
	}
}

VehicleAngularVelocity::~VehicleAngularVelocity()
{
	Stop();
}

bool VehicleAngularVelocity::Start()
{
	// force initial updates
	ParametersUpdate(true);

	// sensor_selection needed to change the active sensor if the primary stops updating
	if (!_sensor_selection_sub.registerCallback()) {
		PX4_ERR("sensor_selection callback registration failed");
		return false;
	}

	if (!SensorSelectionUpdate(true)) {
		_selected_sensor_sub_index = 0;
		_sensor_sub.registerCallback();
	}

	return true;
}

void VehicleAngularVelocity::Stop()
{
	// clear all registered callbacks
	_sensor_sub.unregisterCallback();
	_sensor_selection_sub.unregisterCallback();

	Deinit();
}

void VehicleAngularVelocity::CheckFilters()
{
	bool reset_filters = false;

	// calculate sensor update rate
	const float sample_interval_avg = _interval_sum / _interval_count;

	if (PX4_ISFINITE(sample_interval_avg) && (sample_interval_avg > 0.0f)) {

		_update_rate_hz = 1.e6f / sample_interval_avg;

		// check if sample rate error is greater than 1%
		if ((fabsf(_update_rate_hz - _filter_sample_rate) / _filter_sample_rate) > 0.01f) {
			reset_filters = true;
		}

		if (reset_filters || (_required_sample_updates == 0)) {
			if (_param_imu_gyro_rate_max.get() > 0) {
				// determine number of sensor samples that will get closest to the desired rate
				const float configured_interval_us = 1e6f / _param_imu_gyro_rate_max.get();
				const uint8_t samples = math::constrain(roundf(configured_interval_us / sample_interval_avg), 1.f,
									(float)sensor_gyro_s::ORB_QUEUE_LENGTH);

				_sensor_sub.set_required_updates(samples);
				_required_sample_updates = samples;

			} else {
				_sensor_sub.set_required_updates(1);
				_required_sample_updates = 1;
			}
		}

		// publish interval
		if (_param_imu_gyro_rate_max.get() > 0) {
			const uint64_t imu_gyro_interval = 1e6f / _param_imu_gyro_rate_max.get();
			_publish_interval_min_us = imu_gyro_interval - (sample_interval_avg / 2);

		} else {
			_publish_interval_min_us = 0;
		}
	}

	if (!reset_filters) {
		// gyro low pass cutoff frequency changed
		for (auto &lp : _lp_filter_velocity) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_gyro_cutoff.get()) > 0.01f) {
				reset_filters = true;
				break;
			}
		}

		// gyro notch filter frequency or bandwidth changed
		for (auto &nf : _notch_filter_velocity) {
			if ((fabsf(nf.getNotchFreq() - _param_imu_gyro_nf_freq.get()) > 0.01f)
			    || (fabsf(nf.getBandwidth() - _param_imu_gyro_nf_bw.get()) > 0.01f)) {

				reset_filters = true;
				break;
			}
		}

		// gyro derivative low pass cutoff changed
		for (auto &lp : _lp_filter_acceleration) {
			if (fabsf(lp.get_cutoff_freq() - _param_imu_dgyro_cutoff.get()) > 0.01f) {
				reset_filters = true;
				break;
			}
		}
	}

	if (reset_filters) {
		PX4_DEBUG("resetting filters, sample rate: %.3f Hz -> %.3f Hz", (double)_filter_sample_rate, (double)_update_rate_hz);
		_filter_sample_rate = _update_rate_hz;

		// update software low pass filters
		for (int axis = 0; axis < 3; axis++) {
			_lp_filter_velocity[axis].set_cutoff_frequency(_filter_sample_rate, _param_imu_gyro_cutoff.get());
			_lp_filter_velocity[axis].reset(_angular_velocity_filtered_last(axis));
		}


		for (int axis = 0; axis < 3; axis++) {
			_notch_filter_velocity[axis].setParameters(_filter_sample_rate, _param_imu_gyro_nf_freq.get(),
					_param_imu_gyro_nf_bw.get());
			_notch_filter_velocity[axis].reset(_angular_velocity_filtered_last(axis));
		}


		for (int axis = 0; axis < 3; axis++) {
			_lp_filter_acceleration[axis].set_cutoff_frequency(_filter_sample_rate, _param_imu_dgyro_cutoff.get());
			_lp_filter_acceleration[axis].reset(_angular_velocity_filtered_last(axis));
		}


		_filters_configured = true;
	}

	// reset sample interval accumulator
	_timestamp_interval_last = 0;
}

void VehicleAngularVelocity::SensorBiasUpdate(bool force)
{
	// find corresponding estimated sensor bias
	if (_estimator_selector_status_sub.updated()) {
		estimator_selector_status_s estimator_selector_status;

		if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
			_estimator_sensor_bias_sub.ChangeInstance(estimator_selector_status.primary_instance);
		}
	}

	if (_estimator_sensor_bias_sub.updated() || force) {
		estimator_sensor_bias_s bias;

		if (_estimator_sensor_bias_sub.copy(&bias)) {
			if (bias.gyro_device_id == _selected_sensor_device_id) {
				_bias = Vector3f{bias.gyro_bias};

			} else {
				_bias.zero();
			}
		}
	}
}

bool VehicleAngularVelocity::SensorSelectionUpdate(bool force)
{
	if (_sensor_selection_sub.updated() || (_selected_sensor_device_id == 0) || force) {
		sensor_selection_s sensor_selection{};
		_sensor_selection_sub.copy(&sensor_selection);

		if (_selected_sensor_device_id != sensor_selection.gyro_device_id) {

			// see if the selected sensor publishes sensor_gyro_fifo
			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_fifo_s> sensor_gyro_fifo_sub{ORB_ID(sensor_gyro_fifo), i};

				if ((sensor_gyro_fifo_sub.get().device_id != 0)
				    && (sensor_gyro_fifo_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_fifo_sub.ChangeInstance(i) && _sensor_fifo_sub.registerCallback()) {
						// unregister
						_sensor_sub.unregisterCallback();

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(sensor_gyro_fifo_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						IntervalAverageReset();
						_filters_configured = false;
						_required_sample_updates = 0;

						_fifo_available = true;

						return true;
					}
				}
			}

			for (uint8_t i = 0; i < MAX_SENSOR_COUNT; i++) {
				uORB::SubscriptionData<sensor_gyro_s> sensor_gyro_sub{ORB_ID(sensor_gyro), i};

				if ((sensor_gyro_sub.get().device_id != 0) && (sensor_gyro_sub.get().device_id == sensor_selection.gyro_device_id)) {

					if (_sensor_sub.ChangeInstance(i) && _sensor_sub.registerCallback()) {
						// unregister
						_sensor_fifo_sub.unregisterCallback();

						// record selected sensor (array index)
						_selected_sensor_sub_index = i;
						_selected_sensor_device_id = sensor_selection.gyro_device_id;

						// clear bias and corrections
						_bias.zero();

						_calibration.set_device_id(sensor_gyro_sub.get().device_id);

						// reset sample interval accumulator on sensor change
						IntervalAverageReset();
						_filters_configured = false;
						_required_sample_updates = 0;

						_fifo_available = false;

						return true;
					}
				}
			}

			PX4_ERR("unable to find or subscribe to selected sensor (%d)", sensor_selection.gyro_device_id);
			_selected_sensor_device_id = 0;
			_selected_sensor_sub_index = 0;
		}
	}

	return false;
}

void VehicleAngularVelocity::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_params_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_params_sub.copy(&param_update);

		updateParams();

		_calibration.ParametersUpdate();
	}
}

void VehicleAngularVelocity::IntervalAverageReset()
{
	_timestamp_interval_last = 0;
	_interval_sum = 0.f;
	_interval_count = 0.f;
}

void VehicleAngularVelocity::IntervalAverageUpdate(const hrt_abstime &timestamp, int count)
{
	// collect sample interval average for filters
	if ((_timestamp_interval_last != 0) && (timestamp > _timestamp_interval_last)) {
		_interval_sum += (timestamp - _timestamp_interval_last);
		_interval_count += count;

		if (!_filters_configured && (_interval_sum > 500_ms)) {
			CheckFilters();

		} else if (_interval_sum > 10_s) {
			IntervalAverageReset();
		}

	} else {
		IntervalAverageReset();
	}

	_timestamp_interval_last = timestamp;
}

void VehicleAngularVelocity::Run()
{
	// backup schedule
	ScheduleDelayed(10_ms);

	// update corrections first to set _selected_sensor
	const bool selection_updated = SensorSelectionUpdate();

	_calibration.SensorCorrectionsUpdate(selection_updated);
	SensorBiasUpdate(selection_updated);
	ParametersUpdate();

	bool publish = false;

	if (_fifo_available) {
		// process all outstanding fifo messages
		sensor_gyro_fifo_s sensor_fifo_data;

		while (_sensor_fifo_sub.update(&sensor_fifo_data)) {

			if (_sensor_fifo_sub.get_last_generation() == _sensor_last_generation + 1) {
				// update interval average if there's no gap in data
				IntervalAverageUpdate(sensor_fifo_data.timestamp_sample, sensor_fifo_data.samples);

			} else {
				// gap in sensor data, reset
				IntervalAverageReset();
			}

			_sensor_last_generation = _sensor_fifo_sub.get_last_generation();

			_timestamp_sample_last = sensor_fifo_data.timestamp_sample;

			if (sensor_fifo_data.samples > 0 && sensor_fifo_data.samples <= 32) {
				const int N = sensor_fifo_data.samples;
				const float dt = sensor_fifo_data.dt;
				const enum Rotation fifo_rotation = static_cast<enum Rotation>(sensor_fifo_data.rotation);

				Vector3f gyro_filtered;
				Vector3f delta_angular_velocity;

				for (int axis = 0; axis < 3; axis++) {
					// copy raw int16 sensor samples to float array for filtering

					int16_t *raw_data = nullptr;

					switch (axis) {
					case 0:
						raw_data = sensor_fifo_data.x;
						break;

					case 1:
						raw_data = sensor_fifo_data.y;
						break;

					case 2:
						raw_data = sensor_fifo_data.z;
						break;
					}

					float data_array[N];

					for (int n = 0; n < N; n++) {
						data_array[n] = raw_data[n];
					}

					// Apply general notch filter (IMU_GYRO_NF_FREQ)
					if (_notch_filter_velocity[axis].getNotchFreq() > 0.f) {
						_notch_filter_velocity[axis].apply(data_array, N);
					}

					// Apply general low-pass filter (IMU_GYRO_CUTOFF)
					_lp_filter_velocity[axis].apply(data_array, N);

					gyro_filtered(axis) = data_array[N - 1];

					// angular acceleration: Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
					for (int n = 0; n < N; n++) {
						float accel = (raw_data[n] - _fifo_data_last[axis]) / dt;
						delta_angular_velocity(axis) = _lp_filter_acceleration[axis].apply(accel);
						_fifo_data_last[axis] = raw_data[n];
					}
				}

				// Angular velocity: rotate sensor frame to board, scale raw data to SI, apply calibration, and remove in-run estimated bias
				rotate_3f(fifo_rotation, gyro_filtered(0), gyro_filtered(1), gyro_filtered(2));
				_angular_velocity_filtered_last = _calibration.Correct(gyro_filtered * sensor_fifo_data.scale) - _bias;

				// Angular acceleration: rotate sensor frame to board, scale raw data to SI, apply any additional configured rotation
				rotate_3f(fifo_rotation, delta_angular_velocity(0), delta_angular_velocity(1), delta_angular_velocity(2));
				delta_angular_velocity = delta_angular_velocity * sensor_fifo_data.scale;
				_angular_acceleration_filtered_last = _calibration.rotation() * delta_angular_velocity;

				publish = true;
			}
		}

	} else {
		// process all outstanding messages
		sensor_gyro_s sensor_data;

		while (_sensor_sub.update(&sensor_data)) {
			if (_sensor_sub.get_last_generation() == _sensor_last_generation + 1) {
				// update interval average if there's no gap in data
				IntervalAverageUpdate(sensor_data.timestamp_sample);

			} else {
				// gap in sensor data, reset
				IntervalAverageReset();
			}

			_sensor_last_generation = _sensor_sub.get_last_generation();

			// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
			const float dt = math::constrain(((sensor_data.timestamp_sample - _timestamp_sample_last) / 1e6f), 0.0002f, 0.02f);
			_timestamp_sample_last = sensor_data.timestamp_sample;

			// Apply calibration, rotation, and correct for in-run bias errors
			Vector3f angular_velocity{_calibration.Correct(Vector3f{sensor_data.x, sensor_data.y, sensor_data.z}) - _bias};

			for (int axis = 0; axis < 3; axis++) {
				// Apply general notch filter (IMU_GYRO_NF_FREQ)
				_notch_filter_velocity[axis].apply(&angular_velocity(axis), 1);

				// Apply general low-pass filter (IMU_GYRO_CUTOFF)
				_lp_filter_velocity[axis].apply(&angular_velocity(axis), 1);

				// Differentiate & apply specific angular acceleration (D-term) low-pass (IMU_DGYRO_CUTOFF)
				float angular_acceleration = (angular_velocity(axis) - _angular_velocity_filtered_last(axis)) / dt;
				_angular_acceleration_filtered_last(axis) = _lp_filter_acceleration[axis].apply(angular_acceleration);
			}

			// save last filtered angular velocity
			_angular_velocity_filtered_last = angular_velocity;

			publish = true;
		}
	}

	if (_param_imu_gyro_rate_max.get() > 0) {
		if (hrt_elapsed_time(&_last_publish) < _publish_interval_min_us) {
			publish = false;
		}
	}

	if (publish) {
		// Publish vehicle_angular_acceleration
		vehicle_angular_acceleration_s v_angular_acceleration;
		v_angular_acceleration.timestamp_sample = _timestamp_sample_last;
		_angular_acceleration_filtered_last.copyTo(v_angular_acceleration.xyz);
		v_angular_acceleration.timestamp = hrt_absolute_time();
		_vehicle_angular_acceleration_pub.publish(v_angular_acceleration);

		// Publish vehicle_angular_velocity
		vehicle_angular_velocity_s v_angular_velocity;
		v_angular_velocity.timestamp_sample = _timestamp_sample_last;
		_angular_velocity_filtered_last.copyTo(v_angular_velocity.xyz);
		v_angular_velocity.timestamp = hrt_absolute_time();
		_vehicle_angular_velocity_pub.publish(v_angular_velocity);

		_last_publish = _timestamp_sample_last;
	}
}

void VehicleAngularVelocity::PrintStatus()
{
	PX4_INFO("selected sensor: %d (%d), rate: %.1f Hz %s",
		 _selected_sensor_device_id, _selected_sensor_sub_index, (double)_update_rate_hz, _fifo_available ? "FIFO" : "");
	PX4_INFO("estimated bias: [%.4f %.4f %.4f]", (double)_bias(0), (double)_bias(1), (double)_bias(2));

	_calibration.PrintStatus();
}

} // namespace sensors
