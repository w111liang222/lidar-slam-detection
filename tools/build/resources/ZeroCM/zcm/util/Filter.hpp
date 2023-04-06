namespace zcm {

#include <cassert>

class Filter
{
  private:
    double obs;
    double x1;
    double x2;
    double natFreq, damping;
    double natFreq2, natFreqDamping_times2;
    bool initialized;

  public:
    Filter(double naturalFreq = 1, double dampingFactor = 1) :
        natFreq(naturalFreq), damping(dampingFactor),
        natFreq2(naturalFreq * naturalFreq),
        natFreqDamping_times2(2 * naturalFreq * dampingFactor),
        initialized(false)
    {
        assert(natFreq > 0 && "Filter must have positive natFreq");
        assert(damping > 0 && "Filter must have positive damping");
        reset();
    }

    static double convergenceTimeToNatFreq(double riseTime, double damping)
    {
        assert(damping > 0.7 &&
               "Cannot estimate natural frequency based on rise "
               "time for underdamped systems");

        /*
         * See below for explanation:
         * https://en.wikipedia.org/wiki/Rise_time#Rise_time_of_damped_second_order_systems
         */
        return (2.230 * damping * damping - 0.078 * damping + 1.12) / riseTime;
    }

    inline void initialize(double obs)
    {
        this->obs = obs;
        x1 = obs / natFreq2;
        x2 = 0;
        initialized = true;
    }

    inline void reset()
    {
        obs = x1 = x2 = 0;
        initialized = false;
    }

    inline void newObs(double obs, double dt)
    {
        if (!initialized) {
            initialize(obs);
            return;
        }

        this->obs = obs;
        double dx1 = x2;
        double dx2 = -natFreq2 * x1 - natFreqDamping_times2 * x2 + obs;
        x1 += dt * dx1;
        x2 += dt * dx2;
    }

    inline double lowPass() const
    {
        return natFreq2 * x1;
    }

    inline double bandPass() const
    {
        return natFreqDamping_times2 * x2;
    }

    inline double highPass() const
    {
        return obs - (lowPass() + bandPass());
    }

    inline Filter& operator()(double obs, double dt) { newObs(obs, dt); return *this; }

    enum FilterMode { LOW_PASS, BAND_PASS, HIGH_PASS };
    inline double operator[](enum FilterMode m) const
    {
        switch (m) {
            case LOW_PASS:
                return lowPass();
            case BAND_PASS:
                return bandPass();
            case HIGH_PASS:
                return highPass();
        }
        return 0;
    }

    friend std::ostream& operator<<(std::ostream& o, const Filter& f)
    {
        o << "Filter: " << std::endl
          << "   Low Pass: " << f.lowPass()  << std::endl
          << "  Band Pass: " << f.bandPass() << std::endl
          << "  High Pass: " << f.highPass() << std::endl;
        return o;
    }
};

}
