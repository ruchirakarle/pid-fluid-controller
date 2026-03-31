import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("pid_output.csv")
plt.figure(figsize=(10,4))
plt.plot(df.time, df.setpoint,  'r--', label='Setpoint')
plt.plot(df.time, df.pressure,  'b-',  label='Pressure (PSI)')
plt.plot(df.time, df.control_signal, 'g:', label='Control Signal')
plt.xlabel("Time (s)"); plt.ylabel("PSI")
plt.title("PID Fluid Pressure Control")
plt.legend(); plt.grid(True); plt.tight_layout()
plt.savefig("pid_plot.png"); plt.show()