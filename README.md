mkdir -p /home/claude/EV-BTMS-MATLAB/docs
cat > /home/claude/EV-BTMS-MATLAB/README.md << 'EOF'
# Automatic Electromechanical Battery Thermal Management System (BTMS) for Electric Vehicles

A MATLAB project that designs and simulates an automatic electromechanical control system for regulating multiple parameters — battery temperature, cooling-fan speed, and coolant-pump speed — in an Electric Vehicle (EV) battery pack, using closed-loop PID control.

## Block Diagram

![Block Diagram](docs/block_diagram.png)

## Simulation Results

![Simulation Result](results/simulation_result.png)

The battery temperature stays close to the 32°C setpoint even while the load current (a simulated EV driving cycle) keeps changing.

**Performance summary (from `main_simulation.m`):**

| Metric | Value |
|---|---|
| Max battery temperature | 34.45 °C |
| Min battery temperature | 31.65 °C |
| Max overshoot above setpoint | 2.45 °C |
| Approx. settling time | ~3203.5 s |

## PID Tuning Comparison

![PID Tuning Comparison](results/pid_tuning_comparison.png)

Three PID gain sets were compared on the same driving cycle: an under-tuned (soft) controller, a balanced controller, and an over-tuned (aggressive) controller. The balanced set gives the best trade-off between overshoot and stability.

## How It Works

- **Setpoint:** 32°C target battery temperature
- **Feedback:** battery temperature sensor
- **Controller:** PID controller
- **Actuators:** fan motor and coolant pump motor (first-order DC motor model)
- **Plant:** battery thermal model, `dT/dt = (Q_generated - Q_removed)/(m·cp)`
- **Disturbance:** simulated EV driving-cycle load current

## Repository Structure

```
├── matlab/
│   ├── main_simulation.m
│   └── pid_tuning_comparison.m
├── results/
│   ├── simulation_result.png
│   └── pid_tuning_comparison.png
├── docs/
│   ├── methodology.md
│   └── block_diagram.png
├── README.md
└── LICENSE
```

## How to Run

1. Open MATLAB.
2. Set the current folder to `matlab/`.
3. Run `main_simulation.m` for the main simulation.
4. Run `pid_tuning_comparison.m` for the PID tuning study.

## Author

<তোমার নাম>

## License

MIT License — see [LICENSE](LICENSE)
EOF
cp /home/claude/block_diagram/block_diagram.png /home/claude/EV-BTMS-MATLAB/docs/block_diagram.png
mkdir -p /mnt/user-data/outputs
cp /home/claude/EV-BTMS-MATLAB/README.md /mnt/user-data/outputs/README.md
echo done
Output

done
