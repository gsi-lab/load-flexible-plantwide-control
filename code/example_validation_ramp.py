"""Ramp validation: 100 % -> 10 % -> 100 % -> 55 % load at 3 %/min, with a hold at each load."""
from datetime import datetime

from simulation import APSSimulation
from closedloop import run_full_ramp, plot_load_step_results
from control_systems import CONTROL_SYSTEMS

CONTROL_SYSTEM = "basic"  # "basic" or "advanced"

system = CONTROL_SYSTEMS[CONTROL_SYSTEM]

config = {
    "sim_name": system["model"],
    "category": f"validation_ramp_{CONTROL_SYSTEM}",
    "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
    "steady_state_start": 5 * 60,
    "steady_state_low": 30 * 60,
    "steady_state_high": 30 * 60,
    "mid_fraction": 0.55,  # final ramp down to this fraction of the high load
    "steady_state_mid": 30 * 60,
    "ramping_rate": 7.5,  # t/d of hydrogen per minute, 3 %/min of the nominal load
    "timestep": 1,
    "timestep_data": 10,
}

load_var = [
    {"model": "XC12", "variable": "SP", "unit": "kg/s"},
]

snapshots = [
    {"load": "high", "name": "Dyn_High"},
]

if __name__ == "__main__":
    sim = APSSimulation(model_name=config["sim_name"])
    if not sim.connect(timeout=600000):
        raise SystemExit("Failed to connect to AVEVA Process Simulation.")

    run_full_ramp(sim, snapshots, load_var, system["controllers"], system["recorded"], config)
    plot_load_step_results(config["category"], config["timestamp"], snapshots, save_plot=True, show_plot=False)

    sim.disconnect()
