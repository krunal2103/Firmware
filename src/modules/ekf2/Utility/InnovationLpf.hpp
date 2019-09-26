/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "FirstOrderLpf.hpp"

class InnovationLpf final
{
public:
	InnovationLpf() = default;
	~InnovationLpf() = default;

	void reset(float val = 0.f) { _filter.reset(val); }

	float update(float val); ///< update filter, compute alpha from tau and dt
	float update(float val, float alpha); ///< update filter and provide alpha

	void setTauInv(float tau_inv) { _tau_inv = tau_inv; }
	void setDt(float dt) { _dt = dt; }
	void setSpikeLimit(float lim) { _spike_limit = lim; }
	float getAlpha() const { return _alpha; }

private:
	float computeAlpha(); ///< compute _alpha from tau and dt

	FirstOrderLpf _filter;
	float _spike_limit{-1.f}; ///< constrain the input before sending to the lpf (-1 = no constraint)

	float _dt{0.f}; ///< sampling time in seconds
	float _tau_inv{0.f}; ///< inverse of the filter's time constant
	float _alpha{0.f};
};
