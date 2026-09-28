import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# Steady-state values chosen from the long constant-PWM plateaus in the recorded run.
# Each point is the mean of the last section of a stable segment after the transient settled.
heat_x = np.array([0, 61, 128, 191], dtype=float)
heat_y = np.array([20.9, 27.9, 60.0, 67.0], dtype=float)

cool_x = np.array([0, 64, 128, 191], dtype=float)
cool_y = np.array([20.9, 18.8, 17.0, 15.0], dtype=float)

heat_slope, heat_intercept = np.polyfit(heat_x, heat_y, 1)
cool_slope, cool_intercept = np.polyfit(cool_x, cool_y, 1)

fig, ax = plt.subplots(figsize=(9, 6))

ax.plot(heat_x, heat_y, 'o', color='red', markersize=7, label='Heating')
ax.plot(cool_x, cool_y, 'o', color='blue', markersize=7, label='Cooling')

heat_fit_x = np.linspace(heat_x.min(), heat_x.max(), 200)
ax.plot(heat_fit_x, heat_slope * heat_fit_x + heat_intercept, color='red', linewidth=2, alpha=0.8)

cool_fit_x = np.linspace(cool_x.min(), cool_x.max(), 200)
ax.plot(cool_fit_x, cool_slope * cool_fit_x + cool_intercept, color='blue', linewidth=2, alpha=0.8)

ax.set_xlabel('PWM magnitude (count)')
ax.set_ylabel('Steady-state temperature (°C)')
ax.set_title('Steady-State Temperature vs. PWM Magnitude')
ax.grid(True, linestyle='--', alpha=0.4)
ax.legend()

caption = (
    'Steady state was chosen as the average temperature over the final plateau of each constant-PWM segment, '
    'after the transient response had settled.\n'
    f'Heating susceptibility ≈ {heat_slope:.3f} °C per PWM count.  '
    f'Cooling susceptibility ≈ {cool_slope:.3f} °C per PWM count.'
)
fig.text(0.5, 0.01, caption, ha='center', va='bottom', fontsize=10, wrap=True)

plt.tight_layout(rect=[0, 0.06, 1, 1])
plt.savefig('steady_state_temp_vs_pwm.png', dpi=300)
print(f'Heating slope: {heat_slope:.4f} °C/PWM count')
print(f'Cooling slope: {cool_slope:.4f} °C/PWM count')
print('Saved steady_state_temp_vs_pwm.png')
