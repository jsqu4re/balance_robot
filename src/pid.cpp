/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <balance_robot/pid.h>
#include <cmath>
#include <iostream>

using namespace std;

class PIDImpl {
public:
  PIDImpl(float max, float min, float Kp, float Kd, float Ki);
  ~PIDImpl();
  float calculate(float setpoint, float pv, float dt);

private:
  float _max;
  float _min;
  float _Kp;
  float _Kd;
  float _Ki;
  float _pre_error;
  float _integral;
};

PID::PID(float min, float max, float Kp, float Ki, float Kd) {
  pimpl = new PIDImpl(min, max, Kp, Ki, Kd);
}
float PID::calculate(float setpoint, float pv, float dt) {
  return pimpl->calculate(setpoint, pv, dt);
}
PID::~PID() { delete pimpl; }

/**
 * Implementation
 */
PIDImpl::PIDImpl(float min, float max, float Kp, float Ki, float Kd)
    : _max(max), _min(min), _Kp(Kp), _Kd(Kd), _Ki(Ki), _pre_error(0),
      _integral(0) {}

float PIDImpl::calculate(float setpoint, float pv, float dt) {

  // Calculate error
  float error = setpoint - pv;

  // Proportional term
  float Pout = _Kp * error;

  // Integral term
  _integral += error * dt;
  float Iout = _Ki * _integral;

  // Derivative term
  float derivative = (error - _pre_error) / dt;
  float Dout = _Kd * derivative;

  // Calculate total output
  float output = Pout + Iout + Dout;

  // Restrict to max/min
  if (output > _max)
    output = _max;
  else if (output < _min)
    output = _min;

  // Save error to previous error
  _pre_error = error;

  return output;
}

PIDImpl::~PIDImpl() {}

#endif
