import tkinter as tk
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_err = 0

    def compute(self, setpoint, measured, dt):
        err = setpoint - measured
        self.integral += err * dt
        self.integral = max(-50, min(self.integral, 50))
        deriv = (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv

    def reset(self):
        self.integral = 0
        self.prev_err = 0


class FluidSystem:
    def __init__(self):
        self.pressure = 0.0

    def update(self, ctrl, dt):
        drive = max(0.0, min(ctrl, 100.0))
        self.pressure += (drive * 0.4 - self.pressure * 0.15) * dt
        self.pressure = max(0.0, min(self.pressure, 100.0))
        return self.pressure


class PIDDashboard:
    DT = 0.05
    MAX_PTS = 300
    UPDATE_MS = 50

    def __init__(self, root):
        self.root = root
        self.root.title("Fluid Pressure PID Controller")
        self.running = False
        self.t = 0.0
        self.alarm_low = 10.0
        self.alarm_high = 90.0
        self.alarm_visible = False
        self.pid = PIDController(2.0, 0.5, 0.8)
        self.pump = FluidSystem()
        self.times     = deque(maxlen=self.MAX_PTS)
        self.pressures = deque(maxlen=self.MAX_PTS)
        self.setpoints = deque(maxlen=self.MAX_PTS)
        self.controls  = deque(maxlen=self.MAX_PTS)
        self._build_ui()
        self._loop()

    def _build_ui(self):
        # ── PID controls 
        cf = tk.Frame(self.root, pady=8)
        cf.pack(fill="x", padx=16)

        tk.Label(cf, text="Setpoint (PSI)").grid(row=0, column=0, padx=4)
        self.sp_var = tk.DoubleVar(value=50)
        tk.Scale(cf, from_=0, to=100, orient="horizontal",
                 variable=self.sp_var, length=140).grid(row=0, column=1, padx=4)

        tk.Label(cf, text="Kp").grid(row=0, column=2, padx=4)
        self.kp_var = tk.DoubleVar(value=2.0)
        tk.Scale(cf, from_=0, to=10, resolution=0.1, orient="horizontal",
                 variable=self.kp_var, length=100).grid(row=0, column=3, padx=4)

        tk.Label(cf, text="Ki").grid(row=0, column=4, padx=4)
        self.ki_var = tk.DoubleVar(value=0.5)
        tk.Scale(cf, from_=0, to=5, resolution=0.05, orient="horizontal",
                 variable=self.ki_var, length=100).grid(row=0, column=5, padx=4)

        tk.Label(cf, text="Kd").grid(row=0, column=6, padx=4)
        self.kd_var = tk.DoubleVar(value=0.8)
        tk.Scale(cf, from_=0, to=5, resolution=0.05, orient="horizontal",
                 variable=self.kd_var, length=100).grid(row=0, column=7, padx=4)

        self.start_btn = tk.Button(cf, text="Start",
                                   command=self.toggle, width=8)
        self.start_btn.grid(row=0, column=8, padx=8)
        tk.Button(cf, text="Reset",
                  command=self.reset, width=8).grid(row=0, column=9, padx=4)

        # ── Alarm threshold sliders 
        af = tk.Frame(self.root, pady=4)
        af.pack(fill="x", padx=16)

        tk.Label(af, text="Alarm Low (PSI)", fg="blue").grid(row=0, column=0, padx=4)
        self.al_var = tk.DoubleVar(value=10.0)
        tk.Scale(af, from_=0, to=50, resolution=1, orient="horizontal",
                 variable=self.al_var, length=130).grid(row=0, column=1, padx=4)

        tk.Label(af, text="Alarm High (PSI)", fg="red").grid(row=0, column=2, padx=4)
        self.ah_var = tk.DoubleVar(value=90.0)
        tk.Scale(af, from_=50, to=100, resolution=1, orient="horizontal",
                 variable=self.ah_var, length=130).grid(row=0, column=3, padx=4)

        tk.Label(af, text="<-- drag to set safe range",
                 fg="gray").grid(row=0, column=4, padx=8)

        # ── Status bar 
        sf = tk.Frame(self.root, pady=4)
        sf.pack(fill="x", padx=16)
        self.lbl_time     = self._stat(sf, "Time: 0.0s")
        self.lbl_pressure = self._stat(sf, "Pressure: 0.0 PSI")
        self.lbl_error    = self._stat(sf, "Error: 0.0")
        self.lbl_ctrl     = self._stat(sf, "Control: 0.0")
        self.lbl_status   = self._stat(sf, "Status: SAFE")
        self.lbl_status.config(fg="green")

        # ── Alarm banner 
        self.alarm_frame = tk.Frame(self.root, bg="red", pady=5)
        self.alarm_label = tk.Label(
            self.alarm_frame,
            text="  PRESSURE ALARM - OUT OF SAFE RANGE  ",
            bg="red", fg="white",
            font=("Consolas", 11, "bold"))
        self.alarm_label.pack()

        # ── Plot ─────────────────────────────────────────────────
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 4))
        self.fig.tight_layout(pad=2.0)

        self.ax1.set_ylabel("Pressure (PSI)")
        self.ax1.set_title("Fluid Pressure PID - Live View")
        self.ax2.set_ylabel("Control Signal")
        self.ax2.set_xlabel("Time (s)")

        self.line_sp,   = self.ax1.plot([], [], 'r--', lw=1.2, label="Setpoint")
        self.line_pres, = self.ax1.plot([], [], 'b-',  lw=1.5, label="Pressure")
        self.line_ctrl, = self.ax2.plot([], [], 'g-',  lw=1.2, label="Control")
        self.line_alo   = self.ax1.axhline(y=10, color='blue',
                                            linestyle=':', lw=1, label="Alarm low")
        self.line_ahi   = self.ax1.axhline(y=90, color='red',
                                            linestyle=':', lw=1, label="Alarm high")
        self.ax1.legend(loc="upper left", fontsize=8)
        self.ax1.set_xlim(0, 15)
        self.ax1.set_ylim(-5, 110)
        self.ax2.set_xlim(0, 15)
        self.ax2.set_ylim(-10, 110)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=16, pady=8)
        self.canvas.draw()

    def _stat(self, parent, text):
        lbl = tk.Label(parent, text=text, padx=12)
        lbl.pack(side="left")
        return lbl

    def _loop(self):
        if self.running:
            self.pid.kp = self.kp_var.get()
            self.pid.ki = self.ki_var.get()
            self.pid.kd = self.kd_var.get()
            sp = self.sp_var.get()
            self.alarm_low  = self.al_var.get()
            self.alarm_high = self.ah_var.get()

            ctrl     = self.pid.compute(sp, self.pump.pressure, self.DT)
            measured = self.pump.update(ctrl, self.DT)
            self.t  += self.DT

            self.times.append(self.t)
            self.pressures.append(measured)
            self.setpoints.append(sp)
            self.controls.append(ctrl)

            t_list = list(self.times)
            self.line_sp.set_data(t_list, list(self.setpoints))
            self.line_pres.set_data(t_list, list(self.pressures))
            self.line_ctrl.set_data(t_list, list(self.controls))
            self.line_alo.set_ydata([self.alarm_low,  self.alarm_low])
            self.line_ahi.set_ydata([self.alarm_high, self.alarm_high])

            xmin = max(0, self.t - 15)
            self.ax1.set_xlim(xmin, self.t + 1)
            self.ax2.set_xlim(xmin, self.t + 1)

            self.lbl_time.config(text=f"Time: {self.t:.1f}s")
            self.lbl_pressure.config(text=f"Pressure: {measured:.1f} PSI")
            self.lbl_error.config(text=f"Error: {sp - measured:.2f}")
            self.lbl_ctrl.config(text=f"Control: {ctrl:.2f}")

            in_range = self.alarm_low <= measured <= self.alarm_high
            if not in_range and not self.alarm_visible:
                self.alarm_frame.pack(fill="x", padx=16,
                                      before=self.canvas.get_tk_widget())
                self.alarm_visible = True
                self.lbl_status.config(text="Status: ALARM", fg="red")
                self._flash_alarm()
            elif in_range and self.alarm_visible:
                self.alarm_frame.pack_forget()
                self.alarm_visible = False
                self.lbl_status.config(text="Status: SAFE", fg="green")

            self.canvas.draw()

        self.root.after(self.UPDATE_MS, self._loop)

    def _flash_alarm(self):
        if not self.alarm_visible:
            return
        cur = self.alarm_frame.cget("bg")
        nxt = "white" if cur == "red" else "red"
        txt = "red"   if cur == "red" else "white"
        self.alarm_frame.config(bg=nxt)
        self.alarm_label.config(bg=nxt, fg=txt)
        self.root.after(500, self._flash_alarm)

    def toggle(self):
        self.running = not self.running
        self.start_btn.config(text="Pause" if self.running else "Start")

    def reset(self):
        self.running = False
        self.t = 0.0
        self.pid.reset()
        self.pump = FluidSystem()
        for buf in (self.times, self.pressures, self.setpoints, self.controls):
            buf.clear()
        self.start_btn.config(text="Start")
        self.alarm_frame.pack_forget()
        self.alarm_visible = False
        self.lbl_status.config(text="Status: SAFE", fg="green")


if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("900x650")
    app = PIDDashboard(root)
    root.mainloop()