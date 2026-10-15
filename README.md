# Plantwide control design for a load-flexible ammonia process: data and code

Supporting material for

N. Groll, J.K. Huusom, G. Sin, "Plantwide Control Design for Highly Load-Flexible Power-to-X Processes: A Systematic Procedure Applied to Renewable Ammonia Production", manuscript submitted to *Computers & Chemical Engineering*.

Archived version of this repository: https://doi.org/10.5281/zenodo.XXXXXXX

If you use the data or code, please cite the paper and the archived version.

## Contents

| Folder | Content |
|---|---|
| `data/` | Open-loop and closed-loop responses of all control loops, the resulting gain matrices, RGAs and tunings, and the validation tests |
| `code/` | Python interface to the simulator, the test and analysis routines, and example scripts |

## Models

The dynamic models of the Haber-Bosch process were built with AVEVA Process Simulation, version 9.0.0.4196. They are not included: the model files contain library models of the simulator, and its license does not allow us to redistribute them. The files can also only be opened and run with a licensed installation of the simulator. The unit models are described in Section S6 of the Supplementary Material.

Researchers who hold a valid AVEVA Process Simulation license can contact the corresponding author of the paper. We are happy to share the models with them, subject to the license terms of AVEVA.

The scripts that run the simulator expect two models in a folder `models/` next to `code/`:

- `FlexibleHB_Basic.simx`: basic control system (single-loop PI controllers).
- `FlexibleHB_Advanced.simx`: advanced control system (basic control system with the feedforward-cascade steam temperature controller and the ratio-cascade controller). The inlet temperature setpoint of the first reactor bed is 655 K instead of 670 K.

Each model needs the snapshots `Dyn_High` (100 % load) and `Dyn_Low` (10 % load), which are the starting points of all tests, and the controller and measurement tags listed below.

## Data

`data/overview.csv` lists all tests with the figures and tables of the paper and the supplementary material they belong to. Each test folder contains a `metadata.json` with the test settings and the controller tunings.

| Folder | Test |
|---|---|
| `basic/inventory`, `obvious`, `reactor`, `compression`, `antisurge_1` to `_3` | Design of the basic control system, one folder per group of loops |
| `basic/retuning_*` | Open-loop tests for retuning the steam temperature and loop pressure controllers |
| `basic/validation_loadstep`, `basic/validation_ramp` | Validation of the basic control system |
| `advanced/steam_cascade_inner`, `steam_cascade_outer`, `ratio_cascade` | Design of the advanced control elements |
| `advanced/validation_loadstep`, `advanced/validation_ramp` | Validation of the advanced control system |

The design tests were run on earlier versions of the model, in which only the loops designed up to that step were closed. The validation tests were run on the final basic and advanced models.

Files in a design folder:

- `openloop/Stepped_<tag>_OP_<load>.csv`: response to a step in the output of controller `<tag>` at high or low load.
- `openloop/Stepped_<tag>_OP_<load>_simc.csv`: identified process parameters (gain, time constant, time delay) and the SIMC tuning for this load.
- `openloop/simc_tuning_final_average.csv`, `simc_tuning_multi_tau_c.csv`: tunings averaged over both loads, and tunings for several values of the tuning parameter.
- `openloop/gain_results_<load>.csv`, `rga_results_<load>.csv`: steady-state gain matrix and RGA at each load (reactor and compression subsystem).
- `openloop/rga_results_geometric_mean.csv`, `rga_sign_matrix.csv`, `rga_magnitude_matrix.csv`: RGA aggregated across loads, with its sign and magnitude matrices.
- `openloop/pairing_analysis.csv`: pairing metric and Niederlinski index of every possible pairing, per load and aggregated.
- `closedloop/<tuning>/Closed_<tag>_SP_<load>.csv`: response to a setpoint step of controller `<tag>`. `<tuning>` is `high` or `low` (tuned at this load only), `average`, or `best` (weighted or selected tuning, where it differs from the average).

Files in a validation folder:

- `Stepped_XC12_SP_<load>.csv`: all controller signals during the load step (starting at high or low load) or during the ramp scenario (starting at high load).

Time is in s, temperatures in K, pressures in kPa, mass flows in kg/s, volumetric flows in m3/s. Controller outputs (`.OP`) are fractions between 0 and 1. `.SP` is the setpoint and `.PV` the measured value of a controller.

In the tests of the basic control system, the output of the nitrogen flow controller is constant: both feeds are supplied at the same constant pressure through parallel valves, so their flow ratio does not change with load.

### Tags

| Controller | Controlled variable | Manipulated variable | Measurement tag in the open-loop files |
|---|---|---|---|
| XC2 | Level of the first flash tank, L1 | Outlet valve of the first flash tank | V1.Level |
| XC1 | Level of the second flash tank, L2 | Ammonia product valve | V2.Level |
| XC3 | Ammonia product purity | Purge valve | XT1.zi |
| XC9 | Nitrogen feed flow rate | Nitrogen feed valve | XT21.W |
| XC8 (basic) | Steam temperature | Steam valve | XT27.T |
| XC8 (advanced) | Water flow rate (inner loop) | Steam valve | |
| XC17 (advanced) | Steam temperature (outer loop) | Water flow setpoint | |
| XC18 (advanced) | H2/N2 ratio at the reactor inlet (outer loop) | Feed ratio setpoint | |
| XC15 | Inlet temperature of the first bed | Second intercooler valve | XT4.T |
| XC14 | Inlet temperature of the second bed | First intercooler valve | XT5.T |
| XC13 | Inlet temperature of the third bed | Bypass valve | XT6.T |
| XC10 | First-stage pressure | Speed of the first compressor | XT3.P |
| XC11 | Second-stage pressure | Speed of the second compressor | XT10.P |
| XC12 | Hydrogen feed flow rate | Speed of the third compressor | XT25.W |
| XC16 | Loop pressure | Speed of the recycle compressor | XT15.P |
| XC4, XC5, XC6 | Inlet volumetric flow of the first, second and third compressor | Anti-surge valves | XT23.Q, XT8.Q, XT13.Q |

Additional columns in the ramp tests: `S42.T`, `S47.T`, `S73.T` are the outlet temperatures of the first, second and third reactor bed; `S1.z[...]` are mole fractions in the loop gas after the make-up, which is the composition at the reactor inlet; `S129.z[ARGON]` is the argon fraction in the recycle gas before the make-up.

## Code

Requirements: Python 3 with `numpy` and `matplotlib` (see `requirements.txt`). `example_analysis.py` needs nothing else.

The scripts that run the simulator drive AVEVA Process Simulation through its Scripting Interface. They need

- a licensed installation of AVEVA Process Simulation, which must be running,
- the Scripting Interface SDK that comes with it. Its Python package `simcentralconnect` is not on PyPI and is installed from the SDK folder (`pip install -r requirements.txt` in that folder), together with `pythonnet`,
- a 64-bit Python version supported by `pythonnet`.

AVEVA Process Simulation and the Scripting Interface are products of AVEVA Group. They are not part of this repository.

| File | Purpose |
|---|---|
| `simulation.py` | Interface to the simulator |
| `openloop.py` | Open-loop step tests, first-order identification, SIMC tuning, RGA, aggregated RGA, pairing metric, Niederlinski index |
| `closedloop.py` | Closed-loop setpoint steps, load-step test, ramp test |
| `control_systems.py` | Model names and controller tunings of the two control systems |
| `example_analysis.py` | Pairing and tuning analysis of the reactor subsystem from the data in this repository; runs without the simulator |
| `example_openloop_closedloop.py` | Open-loop step test, tuning and closed-loop test of single loops |
| `example_validation_loadstep.py` | Load-step validation; `CONTROL_SYSTEM` selects the basic or advanced control system |
| `example_validation_ramp.py` | Ramp validation; `CONTROL_SYSTEM` selects the basic or advanced control system |

Run the scripts from the `code/` folder, for example

```
python example_analysis.py
```

New results are written to `results/`, next to `data/`.
