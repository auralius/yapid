/*
 * Validation of YAPID_SS with two discrete state-space blocks:
 *
 *   1) PID controller block
 *   2) 2nd-order plant block
 *
 * Closed-loop structure:
 *
 *      r ----> (+) ----> [ PID ] ----> u ----> [ Plant ] ----> y
 *               ^                                          |
 *               |------------------------------------------|
 *
 * where:
 *   e = r - y
 *
 * ------------------------------------------------------------
 * 1) CONTINUOUS-TIME PLANT MODEL
 * ------------------------------------------------------------
 *
 * The plant is a standard 2nd-order system:
 *
 *   y'' + 2*zeta*wn*y' + wn^2*y = K*u
 *
 * with:
 *   wn   = 2.0      rad/s
 *   zeta = 0.2
 *   K    = 1.0
 *
 * Define plant states:
 *
 *   xp1 = y
 *   xp2 = y'
 *
 * Then:
 *
 *   d/dt [xp1]   [  0          1        ] [xp1]   [ 0 ] u
 *        [xp2] = [ -wn^2   -2*zeta*wn   ] [xp2] + [ K ]
 *
 * Output:
 *
 *   y = [1  0] [xp1 xp2]^T
 *
 * Thus, the continuous-time plant matrices are:
 *
 *   Ap = [  0        1      ]
 *        [ -4.0    -0.8     ]
 *
 *   Bp = [ 0 ]
 *        [ 1 ]
 *
 *   Cp = [ 1  0 ]
 *
 *   Dp = [ 0 ]
 *
 * ------------------------------------------------------------
 * 2) CONTINUOUS-TIME PID CONTROLLER MODEL
 * ------------------------------------------------------------
 *
 * The PID uses a filtered derivative (like in MATLAB Simulink):
 *
 *   C(s) = Kp + Ki/s + Kd * (N*s)/(s+N)
 *
 * with:
 *   Kp = 5.0
 *   Ki = 2.0
 *   Kd = 0.5
 *   N  = 20.0
 *
 * The controller input is the error:
 *
 *   e = r - y
 *
 * Define controller states:
 *
 *   xc1 = integral state
 *   xc2 = derivative filter state
 *
 * Continuous-time realization:
 *
 *   xc1_dot = e
 *   xc2_dot = -N*xc2 + N*e
 *
 * Control output:
 *
 *   u = Ki*xc1 - Kd*N*xc2 + (Kp + Kd*N)*e
 *
 * Therefore:
 *
 *   d/dt [xc1]   [ 0    0 ] [xc1]   [  1 ] e
 *        [xc2] = [ 0   -N ] [xc2] + [  N ]
 *
 *   u = [ Ki   -Kd*N ] [xc1 xc2]^T + [ Kp + Kd*N ] e
 *
 * Numerically:
 *
 *   Ac = [ 0    0 ]
 *        [ 0  -20 ]
 *
 *   Bc = [  1 ]
 *        [ 20 ]
 *
 *   Cc = [ 2  -10 ]
 *
 *   Dc = [ 15 ]
 *
 * ------------------------------------------------------------
 * 3) DISCRETIZATION
 * ------------------------------------------------------------
 *
 * Both controller and plant were discretized in Python using:
 *
 *   from scipy.signal import cont2discrete
 *
 * with:
 *
 *   Ts = 0.05 s
 *   method = 'zoh'
 *
 * The discrete matrices below are the result of that conversion.
 *
 * ------------------------------------------------------------
 * 4) EXECUTION MODEL
 * ------------------------------------------------------------
 *
 * At each sample:
 *
 *   y = plant.Y(0)
 *   e = r - y
 *   pid.Compute(e)
 *   u = pid.Y(0)
 *   plant.Compute(u)
 *
 * Notes:
 * - plant.Y(0) is the current plant output before the new plant update
 * - pid.Y(0) is the current controller output before the new controller state update
 * - this setup is for validation of YAPID_SS against Python simulation
 */

#include <Arduino.h>
#include <yapid_ss.h>

// ============================================================
// Discrete PID matrices
// ============================================================
static const float Adc[2][2] = {
  {1.00000000f, 0.00000000f},
  {0.00000000f, 0.36787944f},
};

static const float Bdc[2][1] = {
  {0.05000000f},
  {0.63212056f},
};

static const float Cdc[1][2] = {
  {2.00000000f, -10.00000000f},
};

static const float Ddc[1][1] = {
  {15.00000000f},
};

// ============================================================
// Discrete plant matrices
// ============================================================
static const float Adp[2][2] = {
  {0.99507010f, 0.04893156f},
  {-0.19572622f, 0.95592486f},
};

static const float Bdp[2][1] = {
  {0.00123247f},
  {0.04893156f},
};

static const float Cdp[1][2] = {
  {1.00000000f, 0.00000000f},
};

static const float Ddp[1][1] = {
  {0.00000000f},
};

// ============================================================
// Two YAPID_SS blocks
// ============================================================
YAPID_SS<2,1,1> pid(Adc, Bdc, Cdc, Ddc);
YAPID_SS<2,1,1> plant(Adp, Bdp, Cdp, Ddp);

static const unsigned long Ts_us = 50000UL;
static const float r = 1.0f;

unsigned long t0 = 0;
unsigned long t_prev = 0;

bool running = false;
String cmd_buffer;

void reset_system()
{
  float xc0[2] = {0.0f, 0.0f};
  float xp0[2] = {0.0f, 0.0f};

  pid.SetState(xc0);
  plant.SetState(xp0);

  t0 = micros();
  t_prev = t0;
}

void handle_command(const String& cmd)
{
  if (cmd == "START") {
    reset_system();
    running = true;
    Serial.println("ACK_START");
    Serial.println("t,r,y,e,u,xc1,xc2,xp1,xp2");
  }
  else if (cmd == "STOP") {
    running = false;
    Serial.println("ACK_STOP");
  }
  else if (cmd == "RESET") {
    reset_system();
    Serial.println("ACK_RESET");
  }
  else if (cmd == "PING") {
    Serial.println("PONG");
  }
}

void setup()
{
  Serial.begin(115200);

  while (!Serial) {
    ;
  }

  Serial.println("READY");
}

void loop()
{
  // -------------------------------
  // Read serial commands
  // -------------------------------
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmd_buffer.length() > 0) {
        handle_command(cmd_buffer);
        cmd_buffer = "";
      }
    } else {
      cmd_buffer += c;
    }
  }

  if (!running) {
    return;
  }

  unsigned long now = micros();

  if ((unsigned long)(now - t_prev) >= Ts_us) {
    t_prev += Ts_us;

    // current plant output
    float y = plant.Y(0);

    // error
    float e = r - y;

    // PID update
    float ein[1] = {e};
    pid.Compute(ein);
    float u = pid.Y(0);

    // Plant update
    float uin[1] = {u};
    plant.Compute(uin);

    float t = (float)(t_prev - t0) * 1e-6f;

    Serial.print(t, 6); Serial.print(",");
    Serial.print(r, 6); Serial.print(",");
    Serial.print(y, 6); Serial.print(",");
    Serial.print(e, 6); Serial.print(",");
    Serial.print(u, 6); Serial.print(",");
    Serial.print(pid.X(0), 6); Serial.print(",");
    Serial.print(pid.X(1), 6); Serial.print(",");
    Serial.print(plant.X(0), 6); Serial.print(",");
    Serial.println(plant.X(1), 6);
  }
}