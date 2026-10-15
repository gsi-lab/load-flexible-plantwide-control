"""Pairing and tuning analysis of the reactor subsystem from the open-loop step tests.

Runs without the simulator. The step responses in data/basic/reactor/openloop are
copied to results/example_analysis, and the gain matrices, RGAs, aggregated RGA,
pairing metric, Niederlinski index and SIMC tunings are written next to them.
"""
import shutil
from pathlib import Path

from openloop import pairing_analysis, rga_analysis_all, simc_tuning_all

repo = Path(__file__).parent.parent
source = repo / "data" / "basic" / "reactor" / "openloop"
target = repo / "results" / "example_analysis" / "openloop"

# Stepped valve positions (controller outputs)
mvs = [
    {"model": "XC13", "variable": "OP"},
    {"model": "XC14", "variable": "OP"},
    {"model": "XC15", "variable": "OP"},
]

# Bed inlet temperatures with the selected pairing, tau_c in s, and the scaling range
cvs = [
    {"model": "XT4", "variable": "T", "unit": "K", "paired": "XC15", "tau_c": 10, "pv_low": 650, "pv_high": 750},
    {"model": "XT5", "variable": "T", "unit": "K", "paired": "XC14", "tau_c": 10, "pv_low": 650, "pv_high": 750},
    {"model": "XT6", "variable": "T", "unit": "K", "paired": "XC13", "tau_c": 45, "pv_low": 650, "pv_high": 750},
]

snapshots = [
    {"load": "high"},
    {"load": "low"},
]

if __name__ == "__main__":
    target.mkdir(parents=True, exist_ok=True)
    for mv in mvs:
        for snapshot in snapshots:
            name = f"Stepped_{mv['model']}_OP_{snapshot['load']}.csv"
            shutil.copyfile(source / name, target / name)

    rga_analysis_all(str(target), snapshots)
    pairing_analysis(str(target), snapshots, cvs)
    simc_tuning_all(mvs, cvs, snapshots, str(target), final_strategy="average")
