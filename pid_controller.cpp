#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>

// ── PID Controller
class PIDController
{
public:
    double kp, ki, kd; // tuning gains
    double integral = 0;
    double prev_err = 0;

    PIDController(double p, double i, double d) : kp(p), ki(i), kd(d) {}

    double compute(double setpoint, double measured, double dt)
    {
        double err = setpoint - measured;
        integral += err * dt;
        double deriv = (err - prev_err) / dt;
        prev_err = err;
        return kp * err + ki * integral + kd * deriv;
    }

    void reset()
    {
        integral = 0;
        prev_err = 0;
    }
};

// ── Simulated Fluid System
// Models a pump where pressure rises with control signal,
// decays naturally, and has a bit of disturbance noise.
class FluidSystem
{
public:
    double pressure = 0.0;  // current pressure (PSI)
    double max_psi = 100.0; // physical limit

    double update(double control_signal, double dt)
    {
        double drive = std::max(0.0, std::min(control_signal, 100.0));
        pressure += (drive * 0.4 - pressure * 0.15) * dt; // rise + decay
        pressure = std::max(0.0, std::min(pressure, max_psi));
        return pressure;
    }
};

// ── Main simulation
int main()
{
    const double SETPOINT = 50.0; // target pressure (PSI)
    const double DT = 0.05;       // time step (seconds)
    const double T_END = 20.0;    // total sim time

    // Tune these gains — experiment with them!
    PIDController pid(2.0, 0.5, 0.8);
    FluidSystem pump;

    std::ofstream csv;
    csv.open("pid_output.csv");
    csv << "time,setpoint,pressure,control_signal\n";

    std::cout << "Time(s)  | Setpoint | Pressure | Control\n";
    std::cout << "---------+----------+----------+--------\n";

    for (double t = 0; t <= T_END; t += DT)
    {

        // Simulate a disturbance at t=10s (e.g. valve opens)
        double effective_setpoint = (t > 10.0) ? 70.0 : SETPOINT;

        double ctrl = pid.compute(effective_setpoint, pump.pressure, DT);
        double measured = pump.update(ctrl, DT);

        csv << t << "," << effective_setpoint << ","
            << measured << "," << ctrl << "\n";

        if (std::fmod(t, 1.0) < DT)
        { // print every 1s
            printf("  %5.1f  |  %6.1f  |  %7.2f |  %6.2f\n",
                   t, effective_setpoint, measured, ctrl);
        }
    }

    csv.close();
    std::cout << "\nOutput saved to pid_output.csv\n";
    return 0;
}

/*
── To Build & Run
    g++ -o pid pid_controller.cpp && ./pid

── Python plotter (plot_pid.py)
    import pandas as pd, matplotlib.pyplot as plt
    df = pd.read_csv("pid_output.csv")
    plt.figure(figsize=(10,4))
    plt.plot(df.time, df.setpoint,  'r--', label='Setpoint')
    plt.plot(df.time, df.pressure,  'b-',  label='Pressure (PSI)')
    plt.plot(df.time, df.control_signal, 'g:', label='Control Signal')
    plt.xlabel("Time (s)"); plt.ylabel("PSI / Signal")
    plt.title("PID Fluid Pressure Control")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig("pid_plot.png"); plt.show()

── Tuning guide
    kp (Proportional): Higher = faster response, but may overshoot
    ki (Integral):     Higher = eliminates steady-state error faster
    kd (Derivative):   Higher = dampens oscillation and overshoot
*/