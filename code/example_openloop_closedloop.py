"""Open-loop step test, SIMC tuning and closed-loop setpoint step for single loops.

Shown for the steam temperature and the loop pressure of the basic control
system, as in the retuning step of the paper. All other loops stay closed.
Other loops are tested in the same way by changing the entries in `loops`.
"""
from datetime import datetime

from simulation import APSSimulation, DEFAULT_PATH_DATA
from openloop import run_open_loop_steps_retune, simc_tuning_all
from closedloop import run_closed_loop_steps

MODEL = "FlexibleHB_Basic"
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

snapshots = [
    {"load": "high", "name": "Dyn_High"},
    {"load": "low", "name": "Dyn_Low"},
]

# "multiplier" is the step applied at each snapshot: to the controller output in
# the open-loop test, to the setpoint in the closed-loop test.
loops = [
    {
        "name": "steam_temperature",
        "mv": {"model": "XC8", "variable": "OP", "unit": "fraction", "multiplier": [0.98, 0.98]},
        "cv": {"model": "XT27", "variable": "T", "unit": "K", "paired": "XC8",
               "tau_c": 10, "pv_low": 500, "pv_high": 550},
        # Time constant below 30 s: 1 s steps, 10 min per test
        "timestep": 1,
        "openloop_time": {"high": 10 * 60, "low": 10 * 60},
        "controller": {"model": "XC8", "variable": "SP", "unit": "K", "multiplier": [1.01, 1.01],
                       "Kp": 0.028, "Ti": 9.9},
    },
    {
        "name": "loop_pressure",
        "mv": {"model": "XC16", "variable": "OP", "unit": "fraction", "multiplier": [0.98, 0.99]},
        "cv": {"model": "XT15", "variable": "P", "unit": "kPa", "paired": "XC16",
               "tau_c": 20, "pv_low": 18000, "pv_high": 23000},
        # Time constant 7 min (high) and 76 min (low load); steps no larger than the 5 s valve lag
        "timestep": 5,
        "openloop_time": {"high": 1 * 60 * 60, "low": 6 * 60 * 60},
        # 0.5 % setpoint step; a larger step nearly stops the recycle compressor at low load
        "controller": {"model": "XC16", "variable": "SP", "unit": "kPa", "multiplier": [1.005, 1.005],
                       "Kp": 1.5, "Ti": 266},
    },
]

if __name__ == "__main__":
    sim = APSSimulation(model_name=MODEL)
    if not sim.connect(timeout=600000):
        raise SystemExit("Failed to connect to AVEVA Process Simulation.")

    for loop in loops:
        category = f"example_{loop['name']}"

        # Open-loop step test: the controller is set to manual and its output is stepped.
        loop_snapshots = [dict(s, dynamic_time=loop["openloop_time"][s["load"]]) for s in snapshots]
        config = {
            "sim_name": MODEL,
            "category": category,
            "timestamp": timestamp,
            "steady_state_time": 10,
            "dynamic_time": max(loop["openloop_time"].values()),
            "timestep": loop["timestep"],
        }
        run_open_loop_steps_retune(sim, loop_snapshots, [loop["mv"]], [loop["cv"]], config)

        # First-order identification and SIMC tuning from the recorded responses.
        csv_folder = DEFAULT_PATH_DATA / category / timestamp / "openloop"
        simc_tuning_all([loop["mv"]], [loop["cv"]], snapshots, str(csv_folder), final_strategy="average")

        # Closed-loop setpoint step with the tuning given above.
        config = {
            "sim_name": MODEL,
            "category": category,
            "timestamp": timestamp,
            "steady_state_time": 10,
            "dynamic_time": 15 * 60,
            "timestep": 1,
        }
        run_closed_loop_steps(sim, snapshots, [loop["controller"]], config)

    sim.disconnect()
