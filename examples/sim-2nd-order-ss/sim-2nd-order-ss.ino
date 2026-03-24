/*
 * Continuous-time 2nd-order system
 * y'' + 2*zeta*wn*y' + wn^2*y = K*u
 * where:
 * wn = 2.0       natural frequency [rad/s]
 * zeta = 0.2     damping ratio
 * K = 1.0        input gain
 * Ts = 0.05      sampling time [s]
 *
 * Continuous-time state-space:
 *
 * A = [ 0        1      ]
 *     [ -wn^2   -2*zeta*wn ]
 *
 * B = [ 0 ]
 *     [ K ]
 *
 * C = [ 1  0 ]
 *
 * D = [ 0 ]
 *
 * These matrices were discretized using zero-order hold (ZOH):
 * sys_c = ctrl.ss(A, B, C, D)
 * sys_d = ctrl.c2d(sys_c, Ts, method='zoh')
 */

#include <Arduino.h>
#include <yapid_ss.h>

static const float Ad[2][2] = {
  {0.99507010f, 0.04893146f},
  {-0.19572584f, 0.95592493f}
};

static const float Bd[2][1] = {
  {0.00123247f},
  {0.04893146f}
};

static const float C[1][2] = {
  {1.0f, 0.0f}
};

static const float D[1][1] = {
  {0.0f}
};

YAPID_SS<2,1,1> sys(Ad, Bd, C, D);

static const unsigned long Ts_us = 50000;

unsigned long t_prev = 0;
unsigned long t0 = 0;

bool started = false;
bool sample_ready = false;

float log_t  = 0.0f;
float log_u  = 0.0f;
float log_y  = 0.0f;
float log_x1 = 0.0f;
float log_x2 = 0.0f;

void setup()
{
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial connection
  }

  float x0[2] = {0.0f, 0.0f};
  sys.SetState(x0);

  t0 = micros();
  t_prev = t0;
  started = true;

  Serial.println("t,u,y_k,x1_k1,x2_k1");
}

void loop()
{
  if (!started) return;

  unsigned long now = micros();

  if ((unsigned long)(now - t_prev) >= Ts_us) {
    t_prev += Ts_us;

    float u[1] = {1.0f};
    sys.Compute(u);

    log_t  = (float)(t_prev - t0) * 1e-6f;
    log_u  = u[0];
    log_y  = sys.Y(0);
    log_x1 = sys.X(0);
    log_x2 = sys.X(1);

    sample_ready = true;
  }
  else {
    if (sample_ready) {
      sample_ready = false;

      Serial.print(log_t, 6);
      Serial.print(",");
      Serial.print(log_u, 6);
      Serial.print(",");
      Serial.print(log_y, 6);
      Serial.print(",");
      Serial.print(log_x1, 6);
      Serial.print(",");
      Serial.println(log_x2, 6);
    }
  }
}