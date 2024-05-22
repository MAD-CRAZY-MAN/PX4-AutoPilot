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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_baro.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "ilps28qsw_registers.hpp"

#define ILPS28QSW_ADDRESS        0x5C

using ST_ILPS28QSW::Register;

class ILPS28QSW : public device::I2C, public I2CSPIDriver<ILPS28QSW>
{
public:
	ILPS28QSW(const I2CSPIDriverConfig &config);
	~ILPS28QSW() override;

	static void print_usage();

	int init() override;
//	int probe() override;

	void print_status() override;

	void RunImpl();

	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

private:

	enum class State {
		Detect = 0,
		Reset,
		WaitForReset,
		Running
	};

	int RegisterRead(Register reg, uint8_t &val);
	int RegisterWrite(Register reg, uint8_t val);

	static constexpr uint32_t SAMPLE_RATE{75};

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};

	const bool		_keep_retrying;
	State			_state{State::Detect};
};
