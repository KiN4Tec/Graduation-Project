class PIDController
{
  public:

  double set_point = 0.0;
  double kp = 0.0, ki = 0.0, kd = 0.0;
  int out_min = 0, out_max = 255;

  double accumulator = 0.0;
  double last_err = 0.0;
  int last_result = 0;

  PIDController() {}

  PIDController(double kp, double ki, double kd)
    : kp(kp), ki(ki), kd(kd)
  {}

  PIDController(double kp, double ki, double kd, int out_min, int out_max)
    : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max)
  {}

  int control(double process_value, double t)
  {
    double err = this->set_point - process_value;
    double err_delta = err - this->last_err;

    bool stop_summation = ((last_result > out_max) && (err > 0.0)) || ((last_result < out_min) && (err < 0.0));
    if (!stop_summation) this->accumulator += err * t;

    double result = this->kp * err
                  + this->ki * this->accumulator
                  + this->kd * err_delta;

    this->last_err = err;

    return (int)result;
  }
};
