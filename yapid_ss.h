#ifndef YAPID_SS_h
#define YAPID_SS_h

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

    for (int i = 0; i < P; i++) {
        y[i] = 0.0f;
        for (int j = 0; j < N; j++)
        y[i] += C[i][j] * x[j];
        for (int j = 0; j < M; j++)
        y[i] += D[i][j] * u[j];
    }

    for (int i = 0; i < N; i++) {
        x_next[i] = 0.0f;
        for (int j = 0; j < N; j++)
        x_next[i] += Ad[i][j] * x[j];
        for (int j = 0; j < M; j++)
        x_next[i] += Bd[i][j] * u[j];
    }

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
  const float (*Ad)[N];
  const float (*Bd)[M];
  const float (*C)[N];
  const float (*D)[M];

  float x[N];
  float x_next[N];
  float y[P];
};

#endif