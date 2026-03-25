#ifndef YAPID_SS_h
#define YAPID_SS_h

/*
  YAPID_SS
  ========

  A small discrete-time state-space simulator/controller block for Arduino.

  This class implements the standard discrete-time state-space equations:

      x[k+1] = Ad x[k] + Bd u[k]
      y[k]   = C  x[k] + D  u[k]

  ----------------------------------------------------------------------
  TEMPLATE PARAMETERS
  ----------------------------------------------------------------------
    N : number of states
        - dimension of the state vector x
        - x is an N x 1 vector

    M : number of inputs
        - dimension of the input vector u
        - u is an M x 1 vector

    P : number of outputs
        - dimension of the output vector y
        - y is a P x 1 vector

  ----------------------------------------------------------------------
  MATRIX / VECTOR DIMENSIONS
  ----------------------------------------------------------------------
    Ad : state transition matrix
         size = N x N

    Bd : input matrix
         size = N x M

    C  : output matrix
         size = P x N

    D  : feedthrough / direct transmission matrix
         size = P x M

    x  : state vector
         size = N x 1

    u  : input vector
         size = M x 1

    y  : output vector
         size = P x 1

  ----------------------------------------------------------------------
  INTERPRETATION
  ----------------------------------------------------------------------
    - x[k]     is the current internal state at sample k
    - u[k]     is the current input at sample k
    - y[k]     is the current output at sample k
    - x[k+1]   is the next state after one sample period

    The sample time Ts is NOT stored inside this class.
    It is assumed that Ad and Bd already come from a discretized model
    using your chosen sampling period Ts.

  ----------------------------------------------------------------------
  COMPUTATION ORDER IN Compute()
  ----------------------------------------------------------------------
    1. Output is computed first using the current state x[k]:
           y[k] = C x[k] + D u[k]

    2. Next state is computed:
           x[k+1] = Ad x[k] + Bd u[k]

    3. Internal state is updated:
           x <- x_next

    Therefore, after calling Compute(u):
      - Y(i) returns y[k]
      - X(i) returns the updated state x[k+1]

  ----------------------------------------------------------------------
  IMPORTANT NOTES
  ----------------------------------------------------------------------
    - This class is generic and can represent:
        * a plant
        * a controller
        * an observer
        * a filter
        * any discrete LTI state-space system

    - All matrices must remain valid in memory while attached to this
      object, because the class stores pointers to them.

    - Typical usage:
        1. define Ad, Bd, C, D as static const float arrays
        2. construct YAPID_SS with those matrices
        3. call Compute(u) each sample
        4. read output with Y(i)
        
     - This class assumes each sample is executed exactly within a given 
       time period. This class DOES NOT measure the actual elapsed time of 
       each sample run.

  ----------------------------------------------------------------------
  EXAMPLE
  ----------------------------------------------------------------------
    For a SISO system:
      N = 2   -> 2 states
      M = 1   -> 1 input
      P = 1   -> 1 output

    Then:
      Ad is 2x2
      Bd is 2x1
      C  is 1x2
      D  is 1x1
      x  is 2x1
      u  is 1x1
      y  is 1x1
*/

template<int N, int M, int P>
class YAPID_SS
{
public:
  YAPID_SS()
  : Ad(nullptr), Bd(nullptr), C(nullptr), D(nullptr)
  {
    Reset();
  }

  YAPID_SS(const float (*Ad_)[N],
           const float (*Bd_)[M],
           const float (*C_)[N],
           const float (*D_)[M])
  : Ad(Ad_), Bd(Bd_), C(C_), D(D_)
  {
    Reset();
  }

  void AttachMatrices(const float (*Ad_)[N],
                      const float (*Bd_)[M],
                      const float (*C_)[N],
                      const float (*D_)[M])
  {
    Ad = Ad_;
    Bd = Bd_;
    C  = C_;
    D  = D_;
  }

  void Reset()
  {
    for (int i = 0; i < N; i++) {
      x[i] = 0.0f;
      x_next[i] = 0.0f;
    }

    for (int i = 0; i < P; i++) {
      y[i] = 0.0f;
    }
  }

  void SetState(const float x0[N])
  {
    if (!x0) return;
    for (int i = 0; i < N; i++)
      x[i] = x0[i];
  }

  void SetState(int i, float value)
  {
    if (i >= 0 && i < N)
      x[i] = value;
  }

  void Compute(const float u[M])
  {
    if (!Ad || !Bd || !C || !D || !u)
      return;

    // Output equation: y[k] = C x[k] + D u[k]
    for (int i = 0; i < P; i++) {
      y[i] = 0.0f;
      for (int j = 0; j < N; j++)
        y[i] += C[i][j] * x[j];
      for (int j = 0; j < M; j++)
        y[i] += D[i][j] * u[j];
    }

    // State update equation: x[k+1] = Ad x[k] + Bd u[k]
    for (int i = 0; i < N; i++) {
      x_next[i] = 0.0f;
      for (int j = 0; j < N; j++)
        x_next[i] += Ad[i][j] * x[j];
      for (int j = 0; j < M; j++)
        x_next[i] += Bd[i][j] * u[j];
    }

    // Commit next state
    for (int i = 0; i < N; i++)
      x[i] = x_next[i];
  }

  float X(int i) const
  {
    if (i >= 0 && i < N)
      return x[i];
    return 0.0f;
  }

  float Y(int i) const
  {
    if (i >= 0 && i < P)
      return y[i];
    return 0.0f;
  }

private:
  // System matrices
  const float (*Ad)[N];  // N x N
  const float (*Bd)[M];  // N x M
  const float (*C)[N];   // P x N
  const float (*D)[M];   // P x M

  // Internal vectors
  float x[N];       // current state vector, size N x 1
  float x_next[N];  // next state vector, size N x 1
  float y[P];       // output vector, size P x 1
};

#endif