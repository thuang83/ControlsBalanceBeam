template <int n>
class CircBuff {
  private:
    double buff[n];
    int index = 0;
    double sum;
    bool buffer_full = false;

  public:
    CircBuff() {}

    void addSample(double val) {
      buff[index] = val;
      sum += val;

      buffer_full |= (index > 0 && (index + 1) % n == 0);
      index = (index + 1) % n;
      sum -= buff[index];
    }

    double getAverage() {
      if (buffer_full) {
        return sum / n;
      }

      return NAN;
    }
};
