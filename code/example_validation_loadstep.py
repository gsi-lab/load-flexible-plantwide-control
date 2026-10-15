"""Load-step validation: step of 10 % in the hydrogen feed setpoint at high and low load."""
from datetime import datetime

from simulation import APSSimulation
from closedloop import run_load_step, plot_load_step_results
from control_systems import CONTROL_SYSTEMS

CONTROL_SYSTEM = "basic"  # "basic" or "advanced"

system = CONTROL_SYSTEMS[CONTROL_SYSTEM]

config = {
    "sim_name": system["model"],
    "category": f"validation_loadstep_{CONTROL_SYSTEM}",
    "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
    "steady_state_time": 10,
    "dynamic_time": 20 * 60,
    "timestep": 5,
    "timestep_data": 10,
}

# Hydrogen feed setpoint: -10 % at high load, +10 % at low load
load_var = [
    {"model": "XC12", "variable": "SP", "unit": "kg/s", "multiplier": [0.9, 1.1]},
]

snapshots = [
    {"load": "high", "name": "Dyn_High"},
    {"load": "low", "name": "Dyn_Low"},
]

if __name__ == "__main__":
    sim = APSSimulation(model_name=config["sim_name"])
    if not sim.connect(timeout=600000):
        raise SystemExit("Failed to connect to AVEVA Process Simulation.")

    run_load_step(sim, snapshots, load_var, system["controllers"], system["recorded"], config)
    plot_load_step_results(config["category"], config["timestamp"], snapshots, save_plot=True, show_plot=False)

    sim.disconnect()
