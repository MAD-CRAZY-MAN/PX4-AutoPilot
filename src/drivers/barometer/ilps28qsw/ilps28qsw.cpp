/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "ilps28qsw.hpp"

using namespace ST_ILPS28QSW;
using namespace time_literals;

template<typename T>
static void getTwosComplement(T &raw, uint8_t length)
{
	if (raw & ((T)1 << (length - 1))) {
		raw -= (T)1 << length;
	}
}

ILPS28QSW::ILPS28QSW(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm errors")),
	_keep_retrying(config.keep_running)
{
}

ILPS28QSW::~ILPS28QSW()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int ILPS28QSW::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	uint8_t who_am_i;
	ret = RegisterRead(Register::WHO_AM_I, who_am_i);

	if (ret != 0) {
		PX4_DEBUG("read failed (%i)", ret);

		if (_keep_retrying) {
			PX4_INFO("no sensor found, but will keep retrying");
			ScheduleNow();
			return 0;
		}

		return ret;
	}

	if (who_am_i != WHO_AM_I_VALUE) {
		PX4_DEBUG("WHO_AM_I mismatch (%i)", who_am_i);
		return PX4_ERROR;
	}

	_state = State::Reset;
	ScheduleNow();
	return PX4_OK;
}

void ILPS28QSW::RunImpl()
{
	int ret;

	switch (_state) {
	case State::Detect:
		uint8_t who_am_i;
		ret = RegisterRead(Register::WHO_AM_I, who_am_i);

		if (ret != 0 || who_am_i != WHO_AM_I_VALUE) {
			ScheduleDelayed(300_ms);
			return;
		}

		ScheduleDelayed(10_ms);
		_state = State::Reset;
		break;

	case State::Reset:
		ret = RegisterWrite(Register::CTRL_REG2, SWRESET);

		if (ret != OK) {
			PX4_DEBUG("reset failed");
			ScheduleDelayed(100_ms);
			_state = State::Detect;
			return;
		}

		ScheduleDelayed(20_ms);
		_state = State::WaitForReset;
		break;

	case State::WaitForReset:
		uint8_t val;
		ret = RegisterRead(Register::CTRL_REG2, val);

		if (ret != 0 || (val & SWRESET) != 0) {
			ScheduleDelayed(10_ms);
			_state = State::Reset;
			return;
		}

		// Configure sampling rate
		ret = RegisterWrite(Register::CTRL_REG1, ODR_75HZ | AGV_128);
		if (ret != 0) {
			ScheduleDelayed(10_ms);
			_state = State::Detect;
			return;
		}

		ret = RegisterWrite(Register::CTRL_REG2, BDU);
		if (ret != 0) {
			ScheduleDelayed(10_ms);
			_state = State::Detect;
			return;
		}

		ScheduleDelayed(1000000 / SAMPLE_RATE);
		_state = State::Running;
		break;

	case State::Running:
		perf_begin(_sample_perf);
		uint8_t data[6]; // STATUS, PRESS_OUT_H, PRESS_OUT_L, PRESS_OUT_XL, TEMP_OUT_H, TEMP_OUT_L

		if (read((uint8_t)Register::STATUS, data, sizeof(data)) != PX4_OK) {
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			ScheduleDelayed(10_ms);
			_state = State::Reset;
			return;
		}

		uint8_t status = data[0];

		if ((status & P_DA) == 0) { // check if pressure data is available
			perf_end(_sample_perf);
			ScheduleDelayed(1000000 / SAMPLE_RATE);
			return;
		}

		hrt_abstime timestamp_sample = hrt_absolute_time();
		float temp = ((int16_t)data[4] | (data[5] << 8)) / 100.f;

		int32_t Praw = (int32_t)data[1] | (data[2] << 8) | (data[3] << 16);
		getTwosComplement(Praw, 24);
		float pressure_hPa = Praw / 4096.f;
		float pressure_pa = pressure_hPa * 100.f;

		// publish
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = timestamp_sample;
		sensor_baro.device_id = get_device_id();
		sensor_baro.pressure = pressure_pa;
		sensor_baro.temperature = temp;
		sensor_baro.error_count = perf_event_count(_comms_errors);
		sensor_baro.timestamp = hrt_absolute_time();

		_sensor_baro_pub.publish(sensor_baro);

		perf_end(_sample_perf);
		ScheduleDelayed(1000000 / SAMPLE_RATE);
		break;
	}
}

int ILPS28QSW::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int ILPS28QSW::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

int ILPS28QSW::RegisterRead(Register reg, uint8_t &val)
{
	return read((uint8_t)reg, &val, 1);
}

int ILPS28QSW::RegisterWrite(Register reg, uint8_t val)
{
	return write((uint8_t)reg, &val, 1);
}

void ILPS28QSW::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
