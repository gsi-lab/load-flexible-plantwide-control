
import logging
import os.path as path
from itertools import permutations
from pathlib import Path
import csv
import json
from simulation import DEFAULT_PATH_DATA
import statistics
import math
import numpy as np


logger = logging.getLogger(__name__)


def _infer_category(sim_name: str | None) -> str:
    if not sim_name:
        return "Unknown"

    parts = str(sim_name).split("_")
    return parts[-1] if parts else "Unknown"


def _column_base_name(column_name: str) -> str:
    """Return the data column name without any trailing unit annotation."""
    return column_name.split(" [", 1)[0]


def _resolve_csv_column(headers: list[str], desired_name: str) -> str | None:
    """Find the CSV header that matches a logical column name."""
    desired_base = _column_base_name(desired_name)
    for header in headers:
        if _column_base_name(header) == desired_base:
            return header
    return None


def _format_column_header(column_name: str, unit: str | None = None) -> str:
    """Format a CSV header with an optional unit suffix."""
    if unit:
        return f"{column_name} [{unit}]"
    return column_name


def _apply_snapshot_activation(simulation, activation) -> bool:
    """Apply an optional activation request before running a snapshot."""
    if not activation:
        return True

    if isinstance(activation, list):
        return all(_apply_snapshot_activation(simulation, item) for item in activation)

    if not isinstance(activation, dict):
        logger.error(f"Unsupported activation spec: {activation!r}")
        return False

    if activation.get("type") == "equation" or ("equation_name" in activation and "controller_model" not in activation):
        equation_name = activation.get("equation_name") or activation.get("name")
        if not equation_name:
            logger.error(f"Equation activation missing equation name: {activation}")
            return False
        return simulation.activate_equation(equation_name, activation.get("is_active", True))

    controller_model = activation.get("controller_model") or activation.get("model")
    if not controller_model:
        logger.error(f"Controller activation missing controller model: {activation}")
        return False

    return simulation.activate_controller(
        controller_model,
        activation.get("equation_name", ""),
        activation.get("setpoint_name", ""),
        kp=activation.get("kp"),
        ti=activation.get("ti"),
        set_params=activation.get("set_params", True),
        timeout=activation.get("timeout", 20000),
    )

def run_open_loop_steps(simulation, snapshots, mvs, cvs, openloop_config, data_path: Path = DEFAULT_PATH_DATA):
    """Run open-loop simulation for each snapshot and save results to CSV."""
    if not simulation.is_connected():
        logger.error("Simulation is not connected.")
        return

    if isinstance(snapshots, dict):
        snapshots = [snapshots]

    try:
        # Set simulation timestep
        if not simulation.set_simulation_timestep(openloop_config["timestep"]):
            logger.error("Failed to set simulation timestep. Aborting open-loop simulation.")
            return

        # Create directory for open-loop results
        category = openloop_config.get("category") or _infer_category(openloop_config.get("sim_name"))
        new_data_path = Path(data_path) / category / openloop_config["timestamp"] / "openloop"
        new_data_path.mkdir(parents=True, exist_ok=True)

        metadata = {
            "openloop_config": dict(openloop_config),
            "category": category,
            "snapshots": list(snapshots),
            "mvs": list(mvs),
            "cvs": list(cvs),
            "results_dir": str(new_data_path),
            "csv_files": [],
            "solve_status": [],
        }

        for i, snapshot in enumerate(snapshots):
            for mv in mvs:

                # Load snapshot
                print(f"Loading snapshot: {snapshot['name']}")
                if not simulation.revert_snapshot(snapshot['name']):
                    logger.error(f"Failed to load snapshot: {snapshot['name']}")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "mv": f"{mv['model']}.{mv['variable']}",
                        "success": False,
                        "reason": "Failed to revert snapshot"
                    })
                    continue

                activation = snapshot.get("activation") or snapshot.get("controller_activation")
                if activation and not _apply_snapshot_activation(simulation, activation):
                    logger.error(f"Failed to apply activation for snapshot: {snapshot.get('name')}")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "mv": f"{mv['model']}.{mv['variable']}",
                        "success": False,
                        "reason": "Failed to apply snapshot activation"
                    })
                    continue

                # Run at steady state
                print(f"Running steady state for {openloop_config['steady_state_time']}s")
                simulation.run_dynamic(openloop_config['steady_state_time'], 's')

                # Change valve position
                mv_full_name = f"{mv['model']}.{mv['variable']}"
                default_value = simulation.get_variable(mv_full_name, mv['unit'])
                print(f"Setting {mv_full_name} to {mv['multiplier'][i] if 'multiplier' in mv else default_value + mv['step'][i]}")

                if default_value is not None:
                    new_value = default_value * mv['multiplier'][i] if 'multiplier' in mv else default_value + mv['step'][i]
                    simulation.set_variable(mv_full_name, new_value, mv['unit'])
                else:
                    logger.error(f"Failed to get default value for {mv_full_name}. Skipping.")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "mv": mv_full_name,
                        "success": False,
                        "reason": "Failed to get default value"
                    })
                    continue

                # Get dynamic_time for this run:
                # snapshot-level override > MV-level override > config fallback
                dynamic_time = snapshot.get(
                    'dynamic_time',
                    mv.get('dynamic_time', openloop_config.get('dynamic_time', 60))
                )

                # Run dynamic simulation
                print(f"Running dynamic simulation for {dynamic_time}s")
                simulation.run_dynamic(dynamic_time, 's')
                solve_ok = simulation.check_simulation_status()

                # Create snapshot with history for data collection
                snapshot_name = simulation.create_snapshot_with_history()
                if not snapshot_name:
                    logger.error("Failed to create snapshot with history")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "mv": mv_full_name,
                        "success": False,
                        "reason": "Failed to create snapshot with history"
                    })
                    continue

                print(f"Created snapshot for data collection: {snapshot_name}")

                # Generate time points for data collection
                total_time = int(dynamic_time + openloop_config['steady_state_time'])
                timestamps = [i for i in range(0, total_time + 1, int(openloop_config['timestep']))]

                # Collect MV and CV histories
                mv_histories = {}
                mv_units = {}
                for m in mvs:
                    m_full_name = f"{m['model']}.{m['variable']}"
                    history = simulation.get_variable_history(m['model'], m['variable'], snapshot_name, timestamps, 2000)
                    mv_histories[m_full_name] = history
                    mv_units[m_full_name] = m.get('unit', '')

                cv_histories = {}
                cv_units = {}
                for c in cvs:
                    c_full_name = f"{c['model']}.{c['variable']}"
                    history = simulation.get_variable_history(c['model'], c['variable'], snapshot_name, timestamps, 2000)
                    cv_histories[c_full_name] = history
                    cv_units[c_full_name] = c.get('unit', '')

                # Save to CSV
                csv_path = new_data_path / f"Stepped_{mv['model']}_{mv['variable']}_{snapshot['load']}.csv"
                save_results_to_csv(mv_histories, cv_histories, csv_path, mv_units=mv_units, cv_units=cv_units)
                metadata["csv_files"].append(str(csv_path))
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "mv": mv_full_name,
                    "load": snapshot.get('load'),
                    "dynamic_time": dynamic_time,
                    "success": solve_ok,
                    "reason": "OK" if solve_ok else "Simulation status invalid"
                })
                print(f"Results saved to {csv_path}")

        # Export simulation
        export_path = new_data_path / "OpenLoopSimulation.simx"
        simulation.export_simulation(export_path)

        metadata["export_path"] = str(export_path)
        metadata_path = new_data_path / "openloop_metadata.json"
        with open(metadata_path, "w", encoding="utf-8") as fh:
            json.dump(metadata, fh, indent=2)
        logger.info(f"Open-loop metadata saved to {metadata_path}")

    except Exception as e:
        logger.error(f"Error during open-loop simulation: {e}")

def run_open_loop_steps_retune(simulation, snapshots, mvs, cvs, openloop_config, data_path: Path = DEFAULT_PATH_DATA):
    """Open-loop step tests on a model with closed loops: the stepped controllers are set to manual first."""
    if not simulation.is_connected():
        logger.error("Simulation is not connected.")
        return

    try:
        # Set simulation timestep
        if not simulation.set_simulation_timestep(openloop_config["timestep"]):
            logger.error("Failed to set simulation timestep. Aborting open-loop simulation.")
            return

        # Create directory for open-loop results
        category = openloop_config.get("category") or _infer_category(openloop_config.get("sim_name"))
        new_data_path = Path(data_path) / category / openloop_config["timestamp"] / "openloop"
        new_data_path.mkdir(parents=True, exist_ok=True)

        metadata = {
            "openloop_config": dict(openloop_config),
            "category": category,
            "snapshots": list(snapshots),
            "mvs": list(mvs),
            "cvs": list(cvs),
            "results_dir": str(new_data_path),
            "csv_files": [],
            "solve_status": [],
        }

        for i, snapshot in enumerate(snapshots):
            for mv in mvs:

                # Load snapshot
                print(f"Loading snapshot: {snapshot['name']}")
                if not simulation.revert_snapshot(snapshot['name']):
                    logger.error(f"Failed to load snapshot: {snapshot['name']}")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "mv": f"{mv['model']}.{mv['variable']}",
                        "success": False,
                        "reason": "Failed to revert snapshot"
                    })
                    continue

                # Set operations to manual
                for mv in mvs:
                    op_param = f"{mv['model']}.Operation"
                    if not simulation.update_parameter_value(op_param, "Manual"):
                        print(f"Failed to set controller '{mv['model']}' to Manual")

                # Take new snapshot after setting controllers to manual
                snap = simulation.create_snapshot()

                # Run at steady state
                print(f"Running steady state for {openloop_config['steady_state_time']}s")
                simulation.run_dynamic(openloop_config['steady_state_time'], 's')

                # Change valve position
                mv_full_name = f"{mv['model']}.{mv['variable']}"
                default_value = simulation.get_variable(mv_full_name, mv['unit'])
                print(f"Setting {mv_full_name} to {mv['multiplier'][i] if 'multiplier' in mv else default_value + mv['step'][i]}")

                if default_value is not None:
                    new_value = default_value * mv['multiplier'][i] if 'multiplier' in mv else default_value + mv['step'][i]
                    simulation.set_variable(mv_full_name, new_value, mv['unit'])
                else:
                    logger.error(f"Failed to get default value for {mv_full_name}. Skipping.")
                    metadata["solve_status"].append({
                        "snapshot": snap,
                        "mv": mv_full_name,
                        "success": False,
                        "reason": "Failed to get default value"
                    })
                    continue

                # Get dynamic_time for this run:
                # snapshot-level override > MV-level override > config fallback
                dynamic_time = snapshot.get(
                    'dynamic_time',
                    mv.get('dynamic_time', openloop_config.get('dynamic_time', 60))
                )

                # Run dynamic simulation
                print(f"Running dynamic simulation for {dynamic_time}s")
                simulation.run_dynamic(dynamic_time, 's')
                solve_ok = simulation.check_simulation_status()

                # Create snapshot with history for data collection
                snapshot_name = simulation.create_snapshot_with_history()
                if not snapshot_name:
                    logger.error("Failed to create snapshot with history")
                    metadata["solve_status"].append({
                        "snapshot": snap,
                        "mv": mv_full_name,
                        "success": False,
                        "reason": "Failed to create snapshot with history"
                    })
                    continue

                print(f"Created snapshot for data collection: {snapshot_name}")

                # Generate time points for data collection
                total_time = int(dynamic_time + openloop_config['steady_state_time'])
                timestamps = [i for i in range(0, total_time + 1, int(openloop_config['timestep']))]

                # Collect MV and CV histories
                mv_histories = {}
                mv_units = {}
                for m in mvs:
                    m_full_name = f"{m['model']}.{m['variable']}"
                    history = simulation.get_variable_history(m['model'], m['variable'], snapshot_name, timestamps, 2000)
                    mv_histories[m_full_name] = history
                    mv_units[m_full_name] = m.get('unit', '')

                cv_histories = {}
                cv_units = {}
                for c in cvs:
                    c_full_name = f"{c['model']}.{c['variable']}"
                    history = simulation.get_variable_history(c['model'], c['variable'], snapshot_name, timestamps, 2000)
                    cv_histories[c_full_name] = history
                    cv_units[c_full_name] = c.get('unit', '')

                # Save to CSV
                csv_path = new_data_path / f"Stepped_{mv['model']}_{mv['variable']}_{snapshot['load']}.csv"
                save_results_to_csv(mv_histories, cv_histories, csv_path, mv_units=mv_units, cv_units=cv_units)
                metadata["csv_files"].append(str(csv_path))
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "mv": mv_full_name,
                    "load": snapshot.get('load'),
                    "dynamic_time": dynamic_time,
                    "success": solve_ok,
                    "reason": "OK" if solve_ok else "Simulation status invalid"
                })
                print(f"Results saved to {csv_path}")

        # Export simulation
        export_path = new_data_path / "OpenLoopSimulation.simx"
        simulation.export_simulation(export_path)

        metadata["export_path"] = str(export_path)
        metadata_path = new_data_path / "openloop_metadata.json"
        with open(metadata_path, "w", encoding="utf-8") as fh:
            json.dump(metadata, fh, indent=2)
        logger.info(f"Open-loop metadata saved to {metadata_path}")

    except Exception as e:
        logger.error(f"Error during open-loop simulation: {e}")


def save_results_to_csv(
    mv_histories: dict,
    cv_histories: dict,
    output_file: str,
    mv_units: dict | None = None,
    cv_units: dict | None = None,
):

    """Save simulation results to CSV with simple time-series format."""
    try:
        # Ensure output directory exists
        out_path = Path(output_file)
        if out_path.parent:
            out_path.parent.mkdir(parents=True, exist_ok=True)

        # Build column headers in the same order the histories were provided.
        headers = ['time']
        mv_names = list(mv_histories.keys())
        cv_names = list(cv_histories.keys())
        for mv_name in mv_names:
            headers.append(_format_column_header(mv_name, (mv_units or {}).get(mv_name)))
        for cv_name in cv_names:
            headers.append(_format_column_header(cv_name, (cv_units or {}).get(cv_name)))

        # Collect all data
        time_series = {}

        # Add MV data
        for mv_name, history in mv_histories.items():
            for timestamp, value in history:
                if timestamp not in time_series:
                    time_series[timestamp] = {}
                time_series[timestamp][mv_name] = value

        # Add CV data
        for cv_name, history in cv_histories.items():
            for timestamp, value in history:
                if timestamp not in time_series:
                    time_series[timestamp] = {}
                time_series[timestamp][cv_name] = value

        # Write to CSV
        with open(output_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(headers)

            # Write data rows sorted by time
            for timestamp in sorted(time_series.keys()):
                row = [timestamp]
                for mv_name in mv_names:
                    row.append(time_series[timestamp].get(mv_name, ''))
                for cv_name in cv_names:
                    row.append(time_series[timestamp].get(cv_name, ''))
                writer.writerow(row)

        logger.info(f"Results successfully saved to {output_file}")
    except Exception as e:
        logger.error(f"Error saving results to CSV: {e}")

def plot_open_loop_csv(
    csv_file: str,
    save_plot: bool = True,
    show_plot: bool = True,
    mv_order=None,
    cv_order=None,
):
    """Plot one open-loop CSV file with two subplot columns."""
    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        logger.error(f"matplotlib is required for plotting: {e}")
        return

    file_path = Path(csv_file)
    if not file_path.is_file():
        logger.warning(f"CSV file not found: {file_path}")
        return

    try:
        with open(file_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        if not rows:
            logger.warning(f"CSV file is empty: {file_path}")
            return

        headers = reader.fieldnames or []
        if "time" not in headers:
            logger.warning(f"CSV file has no 'time' column: {file_path}")
            return

        data_columns = [name for name in headers if name != "time"]
        if not data_columns:
            logger.warning(f"No variable columns found in: {file_path}")
            return

        column_lookup = {_column_base_name(name): name for name in data_columns}

        if mv_order:
            mv_columns = [column_lookup[_column_base_name(name)] for name in mv_order if _column_base_name(name) in column_lookup]
        else:
            midpoint = len(data_columns) // 2
            mv_columns = data_columns[:midpoint] if midpoint else data_columns

        if cv_order:
            cv_columns = [column_lookup[_column_base_name(name)] for name in cv_order if _column_base_name(name) in column_lookup]
        else:
            midpoint = len(data_columns) // 2
            cv_columns = data_columns[midpoint:] if midpoint else []

        if not mv_columns and not cv_columns:
            logger.warning(f"No MV/CV columns found in: {file_path}")
            return

        times = [float(row["time"]) for row in rows if row.get("time") not in (None, "")]
        if times:
            x_min = 0.0
            x_max = max(times)
        else:
            x_min = 0.0
            x_max = 1.0
        n_rows = max(len(mv_columns), len(cv_columns), 1)
        fig, axes = plt.subplots(n_rows, 2, figsize=(12, max(4, 2.8 * n_rows)), squeeze=False)
        fig.suptitle(file_path.stem, fontsize=14)

        def plot_column(ax, column_name: str):
            values = []
            for row in rows:
                raw_value = row.get(column_name, "")
                try:
                    values.append(float(raw_value) if raw_value not in (None, "") else float("nan"))
                except (TypeError, ValueError):
                    values.append(float("nan"))

            ax.plot(times, values, linewidth=1.5)
            finite_values = [value for value in values if math.isfinite(value)]
            if finite_values:
                lower_limit = min(finite_values)
                upper_limit = max(finite_values)
                lower_pad = max(abs(lower_limit) * 0.01, 1e-9)
                upper_pad = max(abs(upper_limit) * 0.01, 1e-9)
                lower_limit = lower_limit - lower_pad
                upper_limit = upper_limit + upper_pad
                ax.set_ylim(lower_limit, upper_limit)
            ax.set_xlim(x_min, x_max)
            ax.set_title(column_name)
            ax.set_xlabel("time [s]")
            ax.grid(True, alpha=0.3)

        for row_index in range(n_rows):
            left_ax = axes[row_index][0]
            right_ax = axes[row_index][1]

            if row_index < len(mv_columns):
                plot_column(left_ax, mv_columns[row_index])
            else:
                left_ax.axis("off")

            if row_index < len(cv_columns):
                plot_column(right_ax, cv_columns[row_index])
            else:
                right_ax.axis("off")

        plt.tight_layout()

        if save_plot:
            plot_path = file_path.with_suffix(".png")
            fig.savefig(plot_path, dpi=150, bbox_inches="tight")
            logger.info(f"Plot saved to {plot_path}")

        if show_plot:
            plt.show()
        plt.close(fig)

    except Exception as e:
        logger.error(f"Error plotting {file_path}: {e}")

def plot_open_loop_results(csv_folder: str, save_plot: bool = True, show_plot: bool = True):
    """Plot every CSV file in a folder using `plot_open_loop_csv`."""
    folder_path = Path(csv_folder)
    if not folder_path.is_dir():
        logger.warning(f"CSV folder not found: {folder_path}")
        return

    mv_order = None
    cv_order = None
    metadata_path = folder_path / "openloop_metadata.json"
    if metadata_path.is_file():
        try:
            with open(metadata_path, "r", encoding="utf-8") as fh:
                metadata = json.load(fh)
            mv_order = [f"{mv['model']}.{mv['variable']}" for mv in metadata.get("mvs", []) if mv.get("model") and mv.get("variable")]
            cv_order = [f"{cv['model']}.{cv['variable']}" for cv in metadata.get("cvs", []) if cv.get("model") and cv.get("variable")]
        except Exception as e:
            logger.warning(f"Failed to read open-loop metadata from {metadata_path}: {e}")

    csv_files = sorted(folder_path.glob("*.csv"))
    if not csv_files:
        logger.warning(f"No CSV files found in {folder_path}")
        return

    for file_path in csv_files:
        plot_open_loop_csv(
            str(file_path),
            save_plot=save_plot,
            show_plot=show_plot,
            mv_order=mv_order,
            cv_order=cv_order,
        )

def simc_tuning_pair(load, mv, cv, csv_root: str = 'data'):
    """Compute SIMC tuning for one MV-CV pair and save JSON next to CSV."""
    pattern = f"Stepped_{mv['model']}_{mv['variable']}_{load}.csv"
    root = Path(csv_root)
    files = list(root.glob(f"**/{pattern}"))
    if not files:
        logger.warning(f"No CSV files found for pattern {pattern} under {csv_root}")
        return

    for csv_file in files:
        mv_full = f"{mv['model']}.{mv['variable']}"
        cv_full = f"{cv['model']}.{cv['variable']}"

        file_path = Path(csv_file)
        if not file_path.is_file():
            logger.warning(f"CSV file not found: {file_path}")
            continue

        times = []
        mv_vals = []
        cv_vals = []
        with open(file_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            headers = reader.fieldnames or []
            mv_column = _resolve_csv_column(headers, mv_full)
            cv_column = _resolve_csv_column(headers, cv_full)
            for row in reader:
                t_raw = row.get('time') or row.get('Time')
                if t_raw in (None, ''):
                    continue
                try:
                    t = float(t_raw)
                except Exception:
                    continue
                times.append(t)

                def parse_col(name):
                    v = row.get(name)
                    try:
                        return float(v) if v not in (None, '') else float('nan')
                    except Exception:
                        return float('nan')

                mv_vals.append(parse_col(mv_column) if mv_column else float('nan'))
                cv_vals.append(parse_col(cv_column) if cv_column else float('nan'))

        if not mv_column:
            logger.warning(f"MV column '{mv_full}' not found in {file_path}")
            continue
        if not cv_column:
            logger.warning(f"CV column '{cv_full}' not found in {file_path}")
            continue

        if not times or len(times) < 4:
            logger.warning(f"Not enough data in {file_path}")
            continue

        # Detect first significant MV change, then use the prior sample as t_step
        mv0 = mv_vals[0]
        if math.isnan(mv0):
            logger.warning(f"No valid MV baseline in {file_path}")
            continue

        thresh = max(1e-12, abs(mv0) * 1e-6)
        idx = 0
        for i in range(1, len(mv_vals)):
            v = mv_vals[i]
            if math.isnan(v):
                continue
            if abs(v - mv0) > thresh:
                idx = max(i - 1, 0)
                break

        t_step = times[idx]

        # --- Estimate dead time (theta) using first response method ---
        # cv0_for_theta is the CV value at the moment of the MV step.
        # noise_thresh defines a minimum CV movement required to declare the process has responded,
        # filtering out measurement noise around the baseline.
        cv0_for_theta = cv_vals[idx]
        noise_thresh = max(1e-12, abs(cv0_for_theta) * 1e-6)
        post_step_all_idx = [i for i in range(len(times)) if times[i] >= t_step]

        theta = 0.0
        if len(post_step_all_idx) >= 2:
            cv_final = cv_vals[post_step_all_idx[-1]]
            # Determine the net response direction from baseline to final value.
            # This is used to ignore any initial inverse bump and only detect
            # dead time in the direction the CV ultimately travels.
            positive_response = cv_final > cv0_for_theta

            # Skip index 0 — that is the baseline sample at t_step itself.
            for i in post_step_all_idx[1:]:
                v = cv_vals[i]
                if math.isnan(v):
                    continue
                if positive_response:
                    # Dead time ends when CV first rises above cv0, ignoring
                    # any initial inverse dip below cv0
                    if v > cv0_for_theta + noise_thresh:
                        theta = max(0.0, times[i] - t_step)
                        break
                else:
                    # Dead time ends when CV first drops below cv0, ignoring
                    # any initial inverse rise above cv0
                    if v < cv0_for_theta - noise_thresh:
                        theta = max(0.0, times[i-1] - t_step)
                        break


        # --- Classify response type as integrating or self-regulating ---
        # The process is classified as integrating if the CV is still moving significantly
        # at the end of the test window, indicated by:
        #   - tail_change_ratio: the tail window accounts for at least 1% of total CV movement
        #   - slope_retention:   the tail slope is at least 30% of the early slope
        # The 30% threshold (raised from 20%) reduces misclassification of sluggish
        # self-regulating processes that are nearly settled but not fully at steady state.
        response_type = "simple"
        post_step_idx = [i for i, t in enumerate(times) if t >= t_step]
        if len(post_step_idx) >= 6:
            post_time = [times[i] for i in post_step_idx]
            post_cv = [cv_vals[i] for i in post_step_idx]
            total_post_delta = post_cv[-1] - post_cv[0]
            window_s = 300.0
            tail_idx = [i for i, t in enumerate(post_time) if t >= (post_time[-1] - window_s)]
            early_idx = [i for i, t in enumerate(post_time) if t <= (post_time[0] + window_s)]

            if len(tail_idx) >= 2 and len(early_idx) >= 2:
                tail_delta = post_cv[tail_idx[-1]] - post_cv[tail_idx[0]]
                tail_dt = post_time[tail_idx[-1]] - post_time[tail_idx[0]]
                early_delta = post_cv[early_idx[-1]] - post_cv[early_idx[0]]
                early_dt = post_time[early_idx[-1]] - post_time[early_idx[0]]

                tail_change_ratio = abs(tail_delta / total_post_delta) if abs(total_post_delta) > 1e-12 else 0.0
                tail_slope = (tail_delta / tail_dt) if abs(tail_dt) > 1e-12 else 0.0
                early_slope = (early_delta / early_dt) if abs(early_dt) > 1e-12 else 0.0
                slope_retention = abs(tail_slope / early_slope) if abs(early_slope) > 1e-12 else 0.0

                if tail_change_ratio >= 0.01 and slope_retention >= 0.30:
                    response_type = "integrating"

        # --- Compute MV and CV deltas over the full test window ---
        mv0 = mv_vals[0]
        mv1 = mv_vals[-1]
        cv0 = cv_vals[0]
        cv1 = cv_vals[-1]

        delta_mv = mv1 - mv0
        delta_cv = cv1 - cv0
        if abs(delta_mv) < 1e-12:
            logger.warning(f"Delta MV too small in {file_path}")
            continue

        # --- Scale CV delta to engineering units fraction if PV range is available ---
        # Scaling normalises the process gain so that K is expressed as
        # (fraction of PV span) / (MV unit), making Kc dimensionally consistent
        # regardless of the CV engineering unit range.
        pv_low = cv.get('pv_low')
        pv_high = cv.get('pv_high')
        if pv_low is not None and pv_high is not None and pv_high > pv_low:
            scale = pv_high - pv_low
            if abs(scale) < 1e-12:
                scale = None
        else:
            scale = None

        scale_den = scale if scale else 1.0

        # --- Helper: interpolate the time at which a signal crosses a target value ---
        def find_cross_time(vals, target):
            for j in range(1, len(vals)):
                a = vals[j - 1]
                b = vals[j]
                if math.isnan(a) or math.isnan(b):
                    continue
                if (a <= target <= b) or (b <= target <= a):
                    t0 = times[j - 1]
                    t1 = times[j]
                    if b == a:
                        return t0
                    frac = (target - a) / (b - a)
                    return t0 + frac * (t1 - t0)
            return None

        tau_c = cv.get('tau_c', 60)

        # effective_lambda combines the desired closed-loop time constant (tau_c) and
        # the estimated dead time (theta), as required by the SIMC tuning rules.
        # Both integrating and self-regulating SIMC formulas use (tau_c + theta) as
        # the denominator term, so a single variable is computed here for reuse.
        effective_lambda = tau_c + theta

        if response_type == "integrating":
            # --- Integrating process gain estimated from tail window only ---
            # Using the tail window rather than the full post-step window avoids
            # contamination from the initial transient, giving a cleaner estimate
            # of the steady ramp rate.
            post_time = [times[i] for i in post_step_idx]
            post_cv = [cv_vals[i] for i in post_step_idx]
            window_s = 300.0
            tail_idx = [i for i, t in enumerate(post_time) if t >= (post_time[-1] - window_s)]

            tail_cv = [post_cv[i] for i in tail_idx]
            tail_time = [post_time[i] for i in tail_idx]
            tail_delta_cv = tail_cv[-1] - tail_cv[0]
            tail_dt = max(tail_time[-1] - tail_time[0], 1e-9)
            tail_delta_cv_scaled = tail_delta_cv / scale_den

            # K for an integrating process is the ramp rate per MV unit:
            # K = (delta_CV / span) / (delta_MV * delta_t)   [1 / (MV·s)]
            K = tail_delta_cv_scaled / (delta_mv * tail_dt)

            if abs(K) < 1e-12:
                logger.warning(f"Process gain too small in {file_path}")
                continue

            # SIMC tuning for integrating process (Skogestad, 2003):
            #   Kc = 1 / (K * (tau_c + theta))
            #   Ti = 4 * (tau_c + theta)
            Kc = 1.0 / (abs(K) * max(effective_lambda, 1e-6))
            Ti = 4.0 * effective_lambda
            tau = max(1e-6, times[-1] - t_step)

        else:
            # --- Self-regulating process ---
            delta_cv_scaled = delta_cv / scale_den

            # K is the steady-state process gain:
            # K = (delta_CV / span) / delta_MV   [1 / MV unit]
            K = delta_cv_scaled / delta_mv

            if abs(K) < 1e-12:
                logger.warning(f"Process gain too small in {file_path}")
                continue

            # tau is estimated as the time for CV to reach 63.2% of its total change
            # (first-order time constant approximation).
            cv_63 = cv0 + 0.63 * (cv1 - cv0)
            t63 = find_cross_time(cv_vals, cv_63)
            if t63 is None:
                tau = max(1e-6, (times[-1] - t_step) * 0.1)
            else:
                tau = max(1e-6, t63 - t_step)

            # SIMC tuning for self-regulating process (Skogestad, 2003):
            #   Kc = (1 / K) * tau / (tau_c + theta)
            #   Ti = min(tau, 4 * (tau_c + theta))
            Kc = (1.0 / abs(K)) * (tau / max(effective_lambda, 1e-6))
            Ti = min(tau, 4.0 * effective_lambda)

        tuning_meta = {
            'csv_file': str(csv_file),
            'mv': mv_full,
            'cv': cv_full,
            'load': load,
            'computed': {
                'K_process': K,
                'tau_process': tau,
                'theta_process': theta,
                'tau_c': tau_c,
                'Kc': Kc,
                'Ti': Ti,
                'controller': 'PI',
                'lambda': effective_lambda,
                'response_type': response_type,
                'delta_mv': delta_mv,
                'delta_cv': delta_cv,
            },
        }

        out_csv = file_path.parent / (file_path.stem + '_simc.csv')
        try:
            headers = ['mv', 'cv', 'load', 'response_type', 'K_process', 'tau_process', 'tau_c', 'theta_process', 'Kc', 'Ti', 'controller', 'lambda', 'delta_mv', 'delta_cv']
            comp = tuning_meta.get('computed', {})
            row = [tuning_meta.get('mv'), tuning_meta.get('cv'), tuning_meta.get('load'),
                   comp.get('response_type'), comp.get('K_process'), comp.get('tau_process'), comp.get('tau_c'), comp.get('theta_process'),
                   comp.get('Kc'), comp.get('Ti'), comp.get('controller'), comp.get('lambda'),
                   comp.get('delta_mv'), comp.get('delta_cv')]
            out_csv.parent.mkdir(parents=True, exist_ok=True)
            with open(out_csv, 'w', newline='', encoding='utf-8') as fh:
                writer = csv.writer(fh)
                writer.writerow(headers)
                writer.writerow(row)
            logger.info(f"SIMC tuning CSV saved to {out_csv}")
        except Exception as e:
            logger.error(f"Failed to save tuning CSV to {out_csv}: {e}")

def simc_tuning_all(mvs, cvs, snapshots, csv_root: str = 'data', final_strategy: str = "average"):
    """Compute SIMC tunings for all MV-CV pairs and snapshot loads."""
    cv_by_pair = {}
    for cv in cvs:
        paired_mv = cv.get('paired')
        if paired_mv is not None:
            cv_by_pair[paired_mv] = cv

    for snapshot in snapshots:
        load = snapshot.get('load')
        for i, mv in enumerate(mvs):
            cv = cv_by_pair.get(mv.get('model'))
            if cv is None:
                logger.warning(f"No paired CV provided for MV {mv.get('model')}; skipping")
                continue
            simc_tuning_pair(load, mv, cv, csv_root=csv_root)

    finalize_tuning_from_loads(csv_root, strategy=final_strategy)

def finalize_tuning_from_loads(csv_folder: str, strategy: str = "average"):
    """Create final tuning parameters across loads."""
    folder_path = Path(csv_folder)
    if not folder_path.is_dir():
        logger.warning(f"CSV folder not found: {folder_path}")
        return None

    simc_files = sorted(folder_path.glob("*_simc.csv"))
    if not simc_files:
        logger.warning(f"No SIMC CSV files found in {folder_path}")
        return None

    strategy_key = strategy.strip().lower()
    if strategy_key not in {"average", "less_aggressive", "more_aggressive"}:
        logger.warning(f"Unknown strategy '{strategy}'; using 'average'")
        strategy_key = "average"

    grouped = {}
    for simc_file in simc_files:
        with open(simc_file, "r", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                mv = row.get("mv")
                cv = row.get("cv")
                if not mv or not cv:
                    continue
                try:
                    kc = float(row.get("Kc", ""))
                    ti = float(row.get("Ti", ""))
                    k_process = float(row.get("K_process", ""))
                    tau_process = float(row.get("tau_process", ""))
                    theta_process = float(row.get("theta_process", ""))
                    tau_c = float(row.get("tau_c", "60"))
                except (TypeError, ValueError):
                    continue
                load = row.get("load", "")
                entry = {
                    "mv": mv,
                    "cv": cv,
                    "load": load,
                    "Kc": kc,
                    "Ti": ti,
                    "K_process": k_process,
                    "tau_process": tau_process,
                    "theta_process": theta_process,
                    "tau_c": tau_c,
                    "response_type": row.get("response_type", ""),
                }
                grouped.setdefault((mv, cv), []).append(entry)

    rows_out = []
    for (mv, cv), entries in grouped.items():
        if not entries:
            continue

        if strategy_key == "average":
            # Average the process parameters across loads
            k_vals = [e["K_process"] for e in entries]
            tau_vals = [e["tau_process"] for e in entries]
            theta_vals = [e["theta_process"] for e in entries]

            k_avg = sum(k_vals) / len(k_vals)
            tau_avg = sum(tau_vals) / len(tau_vals)
            theta_avg = sum(theta_vals) / len(theta_vals)

            # Use tau_c and response_type from the first entry (should be same across entries)
            tau_c = entries[0]["tau_c"]
            response_type_avg = entries[0]["response_type"]

            # Re-apply SIMC tuning formulas with averaged process parameters
            effective_lambda = tau_c + theta_avg

            if response_type_avg == "integrating":
                kc = 1.0 / (abs(k_avg) * max(effective_lambda, 1e-6))
                ti = 4.0 * effective_lambda
            else:
                # Self-regulating process
                kc = (1.0 / abs(k_avg)) * (tau_avg / max(effective_lambda, 1e-6))
                ti = min(tau_avg, 4.0 * effective_lambda)

            chosen = entries[0]
            load = "average"
        else:
            def score(e):
                return e["Kc"] / max(e["Ti"], 1e-12)

            if strategy_key == "less_aggressive":
                chosen = min(entries, key=score)
            else:
                chosen = max(entries, key=score)

            kc = chosen["Kc"]
            ti = chosen["Ti"]
            load = chosen.get("load", "")

        rows_out.append({
            "mv": mv,
            "cv": cv,
            "strategy": strategy_key,
            "load": load,
            "Kc": kc,
            "Ti": ti,
            "response_type": chosen.get("response_type", ""),
            "tau_process": chosen.get("tau_process", ""),
            "theta_process": chosen.get("theta_process", ""),
        })

    out_csv = folder_path / f"simc_tuning_final_{strategy_key}.csv"
    try:
        with open(out_csv, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow([
                "mv",
                "cv",
                "strategy",
                "load",
                "Kc",
                "Ti",
                "response_type",
                "tau_process",
                "theta_process",
            ])
            for row in rows_out:
                writer.writerow([
                    row["mv"],
                    row["cv"],
                    row["strategy"],
                    row["load"],
                    row["Kc"],
                    row["Ti"],
                    row["response_type"],
                    row["tau_process"],
                    row["theta_process"],
                ])
        logger.info(f"Final tuning CSV saved to {out_csv}")
    except Exception as e:
        logger.error(f"Failed to save final tuning CSV to {out_csv}: {e}")
        return None

    return out_csv

def rga_analysis_all(csv_folder, snapshots):
    """Run the RGA analysis for every load and aggregate the results across loads."""
    results = []
    for snapshot in snapshots:
        load = snapshot.get('load')
        if load is None:
            logger.warning(f"Snapshot missing 'load' field: {snapshot}")
            continue
        result = rga_analysis(csv_folder, load)
        if result is not None:
            results.append(result)

    if len(results) >= 2:
        rga_geometric_mean_analysis(csv_folder, results)
    elif len(results) == 1:
        logger.info("Only one load produced an RGA matrix; skipping geometric-mean RGA aggregation (need >= 2).")

    return results


def geometric_mean_rga(rga_matrices):
    """Sign-preserving geometric-mean RGA of the RGAs at several loads."""
    stacked = np.stack(rga_matrices)
    magnitude = np.prod(np.abs(stacked), axis=0) ** (1.0 / len(rga_matrices))
    sign = np.where(np.any(stacked < 0, axis=0), -1.0, 1.0)
    return sign * magnitude, sign, magnitude


def rga_geometric_mean_analysis(csv_folder, load_results):
    """Aggregate the per-load RGA results into the geometric-mean RGA and save it."""
    if not load_results:
        logger.warning("No per-load RGA results to aggregate.")
        return None

    folder_path = Path(csv_folder)
    mv_names = load_results[0]["mv_names"]
    cv_names = load_results[0]["cv_names"]
    for result in load_results[1:]:
        if result["mv_names"] != mv_names or result["cv_names"] != cv_names:
            logger.error("MV/CV names differ between loads; cannot aggregate RGA matrices.")
            return None

    loads = [result["load"] for result in load_results]
    rga_matrices = [result["rga_matrix"] for result in load_results]
    rga_gm, sign_matrix, magnitude_matrix = geometric_mean_rga(rga_matrices)

    if rga_gm.size > 0 and rga_gm.shape[0] == rga_gm.shape[1]:
        diagonal_elements = np.diag(rga_gm)
        valid_diagonal = diagonal_elements[np.isfinite(diagonal_elements)]
        if len(valid_diagonal) > 0:
            deviation_from_unity = np.mean(np.abs(valid_diagonal - 1.0))
            logger.info(f"Geometric-mean RGA (loads {loads}): diagonal-pairing deviation from unity = {deviation_from_unity:.3f}")

    def _save_matrix(matrix, filename, description):
        path = folder_path / filename
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(["cv/mv"] + mv_names)
                for cv_index, cv_name in enumerate(cv_names):
                    writer.writerow([
                        cv_name,
                        *[matrix[cv_index, mv_index] for mv_index in range(len(mv_names))],
                    ])
            logger.info(f"{description} (loads {loads}) saved to {path}")
        except Exception as e:
            logger.error(f"Failed to save {description.lower()}: {e}")
            return None
        return path

    output_path = _save_matrix(rga_gm, "rga_results_geometric_mean.csv", "Geometric-mean RGA")
    sign_output_path = _save_matrix(sign_matrix, "rga_sign_matrix.csv", "RGA sign matrix S")
    magnitude_output_path = _save_matrix(magnitude_matrix, "rga_magnitude_matrix.csv", "RGA magnitude matrix M")
    if output_path is None or sign_output_path is None or magnitude_output_path is None:
        return None

    return {
        "loads": loads,
        "rga_gm": rga_gm,
        "sign_matrix": sign_matrix,
        "magnitude_matrix": magnitude_matrix,
        "mv_names": mv_names,
        "cv_names": cv_names,
        "output_path": output_path,
        "sign_output_path": sign_output_path,
        "magnitude_output_path": magnitude_output_path,
    }


def niederlinski_index(gain_matrix, pairing):
    """Niederlinski index of a pairing: det(G) over the product of the paired gains."""
    paired_gains = gain_matrix[np.arange(len(pairing)), list(pairing)]
    return float(np.linalg.det(gain_matrix[:, list(pairing)]) / np.prod(paired_gains))


def pairing_analysis(csv_folder, snapshots, cvs=None):
    """Evaluate every MV-CV pairing of a square subsystem: pairing metric and Niederlinski index per load."""
    folder_path = Path(csv_folder)
    gain_matrices = {}
    for snapshot in snapshots:
        load = snapshot.get('load')
        gain_path = folder_path / f"gain_results_{load}.csv"
        if not gain_path.is_file():
            logger.warning(f"Gain matrix not found for pairing analysis: {gain_path}")
            return None
        with open(gain_path, newline="", encoding="utf-8") as fh:
            rows = list(csv.reader(fh))
        mv_names = rows[0][1:]
        cv_names = [row[0] for row in rows[1:]]
        gain_matrices[load] = np.array([[float(value) for value in row[1:]] for row in rows[1:]])

    if not gain_matrices or len(cv_names) != len(mv_names):
        logger.warning(f"No square gain matrix for pairing analysis in {folder_path}")
        return None

    # Same RGA rule as rga_analysis.
    rga_matrices = {load: gain * np.linalg.pinv(gain).T for load, gain in gain_matrices.items()}
    rga_matrices["aggregated"] = geometric_mean_rga(list(rga_matrices.values()))[0]

    implemented = None
    if cvs:
        implemented = {f"{cv['model']}.{cv['variable']}": f"{cv['paired']}.OP" for cv in cvs if cv.get('paired')}

    results = []
    for pairing in permutations(range(len(cv_names))):
        paired = (np.arange(len(pairing)), list(pairing))
        pairs = {cv_names[i]: mv_names[j] for i, j in enumerate(pairing)}
        result = {
            "pairing": "; ".join(f"{cv}-{mv}" for cv, mv in pairs.items()),
            "implemented": pairs == implemented if implemented else "",
        }
        for name, rga_matrix in rga_matrices.items():
            result[f"d_{name}"] = float(np.mean(np.abs(rga_matrix[paired] - 1.0)))
            result[f"negative_{name}"] = int(np.sum(rga_matrix[paired] < 0))
        for load, gain_matrix in gain_matrices.items():
            result[f"ni_{load}"] = niederlinski_index(gain_matrix, pairing)
        results.append(result)

    for name in rga_matrices:
        candidates = sorted((r for r in results if r[f"negative_{name}"] == 0), key=lambda r: r[f"d_{name}"])
        for result in results:
            result[f"rank_{name}"] = ""
        for rank, result in enumerate(candidates, start=1):
            result[f"rank_{name}"] = rank

    results.sort(key=lambda r: r["d_aggregated"])
    output_path = folder_path / "pairing_analysis.csv"
    try:
        with open(output_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=list(results[0]))
            writer.writeheader()
            writer.writerows(results)
        logger.info(f"Pairing analysis saved to {output_path}")
    except Exception as e:
        logger.error(f"Failed to save pairing analysis: {e}")
        return None

    return results


def rga_analysis(csv_folder, load):
    """Perform steady-state RGA analysis for one load."""
    folder_path = Path(csv_folder)
    if not folder_path.is_dir():
        logger.warning(f"CSV folder not found: {folder_path}")
        return None

    # Read the optional metadata first so we can keep the MV/CV order stable.
    metadata = {}
    metadata_path = folder_path / "openloop_metadata.json"
    if metadata_path.is_file():
        try:
            with open(metadata_path, "r", encoding="utf-8") as fh:
                metadata = json.load(fh)
        except Exception as e:
            logger.warning(f"Failed to read open-loop metadata from {metadata_path}: {e}")

    # Collect the step-response CSV files for this load.
    csv_files = []
    for file_path in sorted(folder_path.glob(f"**/Stepped_*_{load}.csv")):
        if file_path.is_file() and not file_path.name.endswith("_simc.csv"):
            csv_files.append(file_path)

    if not csv_files:
        logger.warning(f"No open-loop CSV files found for load '{load}' in {folder_path}")
        return None

    # Build one record per MV step file so we can estimate one gain column per MV.
    records_by_mv = {}

    for file_path in csv_files:
        stem = file_path.stem
        prefix = "Stepped_"
        suffix = f"_{load}"
        if not stem.startswith(prefix) or not stem.endswith(suffix):
            continue

        body = stem[len(prefix):-len(suffix)]
        if "_" not in body:
            logger.warning(f"Could not infer MV name from file name: {file_path.name}")
            continue

        model, variable = body.rsplit("_", 1)
        mv_name = f"{model}.{variable}"

        with open(file_path, "r", newline="") as fh:
            reader = csv.DictReader(fh)
            headers = reader.fieldnames or []
            rows = list(reader)

        if not rows:
            logger.warning(f"CSV file is empty: {file_path}")
            continue

        records_by_mv[mv_name] = {
            "path": file_path,
            "headers": headers,
            "rows": rows,
        }

    if not records_by_mv:
        logger.warning(f"No usable CSV data found for load '{load}' in {folder_path}")
        return None

    # Keep the original simulation order when possible so the saved matrix lines
    # up with the simulation configuration.
    metadata_mvs = [
        f"{mv.get('model')}.{mv.get('variable')}"
        for mv in metadata.get("mvs", [])
        if mv.get("model") and mv.get("variable")
    ]
    metadata_cvs = [
        f"{cv.get('model')}.{cv.get('variable')}"
        for cv in metadata.get("cvs", [])
        if cv.get("model") and cv.get("variable")
    ]

    if metadata_mvs:
        mv_names = [mv_name for mv_name in metadata_mvs if mv_name in records_by_mv]
    else:
        mv_names = sorted(records_by_mv.keys())

    if not mv_names:
        logger.warning(f"No MV names available for load '{load}'")
        return None

    first_headers = records_by_mv[mv_names[0]]["headers"]
    if metadata_cvs:
        cv_names = [cv_name for cv_name in metadata_cvs if cv_name in first_headers]
    else:
        cv_names = [name for name in first_headers if name != "time" and name not in mv_names]

    if not cv_names:
        logger.warning(f"No CV names available for load '{load}'")
        return None

    logger.info(f"RGA analysis for load '{load}': {len(mv_names)} MVs, {len(cv_names)} CVs")
    logger.info(f"MVs: {mv_names}")
    logger.info(f"CVs: {cv_names}")

    # Allocate one matrix entry per MV/CV pairing.
    gain_matrix = np.full((len(cv_names), len(mv_names)), np.nan, dtype=float)

    for mv_index, mv_name in enumerate(mv_names):
        record = records_by_mv[mv_name]
        rows = record["rows"]
        headers = record["headers"]

        # Pull the time series for the stepped MV and each CV from the same file.
        times = []
        mv_series = []
        for row in rows:
            raw_time = row.get("time") or row.get("Time")
            try:
                time_value = float(raw_time)
            except (TypeError, ValueError):
                time_value = float("nan")
            times.append(time_value)

            raw_mv = row.get(mv_name)
            try:
                mv_value = float(raw_mv)
            except (TypeError, ValueError):
                mv_value = float("nan")
            mv_series.append(mv_value)

        finite_mv = [value for value in mv_series if math.isfinite(value)]
        if not finite_mv:
            logger.warning(f"No valid MV values for '{mv_name}' in {record['path']}")
            continue

        # Estimate the step point from the first significant MV change.
        baseline_mv = finite_mv[0]
        threshold = max(1e-6, abs(baseline_mv) * 0.001)
        step_index = 0
        for index in range(1, len(mv_series)):
            value = mv_series[index]
            if math.isfinite(value) and abs(value - baseline_mv) > threshold:
                step_index = max(0, index - 1)
                break

        step_time = times[step_index] if step_index < len(times) and math.isfinite(times[step_index]) else times[0]
        steady_start = max(step_time, times[-1] - 300.0)

        # Average a short window before the step and a short window at the end.
        baseline_mv_values = [value for time_value, value in zip(times, mv_series) if math.isfinite(time_value) and math.isfinite(value) and time_value <= step_time]
        final_mv_values = [value for time_value, value in zip(times, mv_series) if math.isfinite(time_value) and math.isfinite(value) and time_value >= steady_start]
        if not baseline_mv_values or not final_mv_values:
            logger.warning(f"Unable to estimate MV delta for '{mv_name}' in {record['path']}")
            continue

        delta_mv = statistics.fmean(final_mv_values) - statistics.fmean(baseline_mv_values)
        if abs(delta_mv) <= 1e-12:
            logger.warning(f"Step in {record['path'].name} has too small an MV change: {delta_mv:.6e}")
            continue

        for cv_index, cv_name in enumerate(cv_names):
            if cv_name not in headers:
                logger.warning(f"CV column '{cv_name}' not found in {record['path'].name}")
                continue

            cv_series = []
            for row in rows:
                raw_cv = row.get(cv_name)
                try:
                    cv_value = float(raw_cv)
                except (TypeError, ValueError):
                    cv_value = float("nan")
                cv_series.append(cv_value)

            baseline_cv_values = [value for time_value, value in zip(times, cv_series) if math.isfinite(time_value) and math.isfinite(value) and time_value <= step_time]
            final_cv_values = [value for time_value, value in zip(times, cv_series) if math.isfinite(time_value) and math.isfinite(value) and time_value >= steady_start]
            if not baseline_cv_values or not final_cv_values:
                logger.warning(f"Unable to estimate CV delta for '{cv_name}' in {record['path']}")
                continue

            delta_cv = statistics.fmean(final_cv_values) - statistics.fmean(baseline_cv_values)
            gain_matrix[cv_index, mv_index] = delta_cv / delta_mv

            logger.debug(
                f"Load '{load}', MV '{mv_name}', CV '{cv_name}': "
                f"delta_mv={delta_mv:.6f}, delta_cv={delta_cv:.6f}, gain={gain_matrix[cv_index, mv_index]:.6f}"
            )

    if np.isnan(gain_matrix).any():
        logger.error(f"Incomplete gain matrix for load '{load}':\n{gain_matrix}")
        return None

    logger.info(f"Calculated steady-state gain matrix for load '{load}': {gain_matrix.shape}")

    try:
        # Use the same RGA rule as rga.py: G .* (pinv(G))^T.
        gain_inv = np.linalg.pinv(gain_matrix)
        rga_matrix = gain_matrix * gain_inv.T
    except Exception as e:
        logger.error(f"Failed to calculate RGA matrix for load '{load}': {e}")
        return None

    if rga_matrix.size > 0:
        diagonal_elements = np.diag(rga_matrix)
        valid_diagonal = diagonal_elements[np.isfinite(diagonal_elements)]
        if len(valid_diagonal) > 0:
            avg_diagonal = np.mean(np.abs(valid_diagonal))
            deviation_from_unity = np.mean(np.abs(valid_diagonal - 1.0))
            logger.info(f"RGA average diagonal magnitude for load '{load}': {avg_diagonal:.3f}")
            logger.info(f"RGA average deviation from unity for load '{load}': {deviation_from_unity:.3f}")

    gain_output_path = folder_path / f"gain_results_{load}.csv"
    output_path = folder_path / f"rga_results_{load}.csv"
    try:
        # Store the gain matrix in a matrix-shaped CSV so it works for any size.
        with open(gain_output_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["cv/mv"] + mv_names)
            for cv_index, cv_name in enumerate(cv_names):
                writer.writerow([
                    cv_name,
                    *[gain_matrix[cv_index, mv_index] for mv_index in range(len(mv_names))],
                ])

        # Store the RGA matrix in the same matrix-shaped layout.
        with open(output_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["cv/mv"] + mv_names)
            for cv_index, cv_name in enumerate(cv_names):
                writer.writerow([
                    cv_name,
                    *[rga_matrix[cv_index, mv_index] for mv_index in range(len(mv_names))],
                ])
        logger.info(f"Gain matrix saved to {gain_output_path}")
        logger.info(f"RGA results saved to {output_path}")
    except Exception as e:
        logger.error(f"Failed to save gain or RGA results for load '{load}': {e}")
        return None

    return {
        "load": load,
        "gain_matrix": gain_matrix,
        "rga_matrix": rga_matrix,
        "mv_names": mv_names,
        "cv_names": cv_names,
        "gain_output_path": gain_output_path,
        "output_path": output_path,
    }

def simc_tuning_multi_tau_c(mvs, cvs, snapshots, csv_root: str = 'data', tau_c_values=None):
    """Compute SIMC tunings for multiple tau_c values and save results to CSV."""
    if tau_c_values is None:
        tau_c_values = [30, 60, 120, 180, 300]

    # Build mapping of MV to CV
    cv_by_pair = {}
    for cv in cvs:
        paired_mv = cv.get('paired')
        if paired_mv is not None:
            cv_by_pair[paired_mv] = cv

    # Collect all SIMC results
    results = []

    for snapshot in snapshots:
        load = snapshot.get('load')
        if load is None:
            logger.warning(f"Snapshot missing 'load' field: {snapshot}")
            continue

        for mv in mvs:
            cv = cv_by_pair.get(mv.get('model'))
            if cv is None:
                logger.warning(f"No paired CV provided for MV {mv.get('model')}; skipping")
                continue

            # Get base CV info (process parameters from first tau_c calculation)
            pattern = f"Stepped_{mv['model']}_{mv['variable']}_{load}.csv"
            root = Path(csv_root)
            files = list(root.glob(f"**/{pattern}"))
            if not files:
                logger.warning(f"No CSV files found for pattern {pattern} under {csv_root}")
                continue

            csv_file = files[0]
            file_path = Path(csv_file)
            mv_full = f"{mv['model']}.{mv['variable']}"
            cv_full = f"{cv['model']}.{cv['variable']}"

            # Parse CSV and extract process parameters (K, tau, theta)
            times = []
            mv_vals = []
            cv_vals = []
            with open(file_path, 'r', newline='') as f:
                reader = csv.DictReader(f)
                headers = reader.fieldnames or []
                mv_column = _resolve_csv_column(headers, mv_full)
                cv_column = _resolve_csv_column(headers, cv_full)
                for row in reader:
                    t_raw = row.get('time') or row.get('Time')
                    if t_raw in (None, ''):
                        continue
                    try:
                        t = float(t_raw)
                    except Exception:
                        continue
                    times.append(t)

                    def parse_col(name):
                        v = row.get(name)
                        try:
                            return float(v) if v not in (None, '') else float('nan')
                        except Exception:
                            return float('nan')

                    mv_vals.append(parse_col(mv_column) if mv_column else float('nan'))
                    cv_vals.append(parse_col(cv_column) if cv_column else float('nan'))

            if not times or len(times) < 4:
                logger.warning(f"Not enough data in {file_path}")
                continue

            if not mv_column:
                logger.warning(f"MV column '{mv_full}' not found in {file_path}")
                continue
            if not cv_column:
                logger.warning(f"CV column '{cv_full}' not found in {file_path}")
                continue

            # Find step time
            mv0 = mv_vals[0]
            if math.isnan(mv0):
                logger.warning(f"No valid MV baseline in {file_path}")
                continue

            thresh = max(1e-12, abs(mv0) * 1e-6)
            idx = 0
            for i in range(1, len(mv_vals)):
                v = mv_vals[i]
                if math.isnan(v):
                    continue
                if abs(v - mv0) > thresh:
                    idx = max(i - 1, 0)
                    break

            t_step = times[idx]

            # Estimate dead time
            cv0_for_theta = cv_vals[idx]
            noise_thresh = max(1e-12, abs(cv0_for_theta) * 1e-6)
            post_step_all_idx = [i for i in range(len(times)) if times[i] >= t_step]

            theta = 0.0
            if len(post_step_all_idx) >= 2:
                cv_final = cv_vals[post_step_all_idx[-1]]
                positive_response = cv_final > cv0_for_theta

                for i in post_step_all_idx[1:]:
                    v = cv_vals[i]
                    if math.isnan(v):
                        continue
                    if positive_response:
                        if v > cv0_for_theta + noise_thresh:
                            theta = max(0.0, times[i] - t_step)
                            break
                    else:
                        if v < cv0_for_theta - noise_thresh:
                            theta = max(0.0, times[i-1] - t_step)
                            break

            # Classify response type
            response_type = "simple"
            post_step_idx = [i for i, t in enumerate(times) if t >= t_step]
            if len(post_step_idx) >= 6:
                post_time = [times[i] for i in post_step_idx]
                post_cv = [cv_vals[i] for i in post_step_idx]
                total_post_delta = post_cv[-1] - post_cv[0]
                window_s = 300.0
                tail_idx = [i for i, t in enumerate(post_time) if t >= (post_time[-1] - window_s)]
                early_idx = [i for i, t in enumerate(post_time) if t <= (post_time[0] + window_s)]

                if len(tail_idx) >= 2 and len(early_idx) >= 2:
                    tail_delta = post_cv[tail_idx[-1]] - post_cv[tail_idx[0]]
                    tail_dt = post_time[tail_idx[-1]] - post_time[tail_idx[0]]
                    early_delta = post_cv[early_idx[-1]] - post_cv[early_idx[0]]
                    early_dt = post_time[early_idx[-1]] - post_time[early_idx[0]]

                    tail_change_ratio = abs(tail_delta / total_post_delta) if abs(total_post_delta) > 1e-12 else 0.0
                    tail_slope = (tail_delta / tail_dt) if abs(tail_dt) > 1e-12 else 0.0
                    early_slope = (early_delta / early_dt) if abs(early_dt) > 1e-12 else 0.0
                    slope_retention = abs(tail_slope / early_slope) if abs(early_slope) > 1e-12 else 0.0

                    if tail_change_ratio >= 0.01 and slope_retention >= 0.30:
                        response_type = "integrating"

            # Compute process parameters
            mv0 = mv_vals[0]
            mv1 = mv_vals[-1]
            cv0 = cv_vals[0]
            cv1 = cv_vals[-1]

            delta_mv = mv1 - mv0
            delta_cv = cv1 - cv0
            if abs(delta_mv) < 1e-12:
                logger.warning(f"Delta MV too small in {file_path}")
                continue

            # Scale CV if PV range is available
            pv_low = cv.get('pv_low')
            pv_high = cv.get('pv_high')
            if pv_low is not None and pv_high is not None and pv_high > pv_low:
                scale = pv_high - pv_low
                if abs(scale) < 1e-12:
                    scale = None
            else:
                scale = None

            scale_den = scale if scale else 1.0

            # Helper function for crossing time
            def find_cross_time(vals, target):
                for j in range(1, len(vals)):
                    a = vals[j - 1]
                    b = vals[j]
                    if math.isnan(a) or math.isnan(b):
                        continue
                    if (a <= target <= b) or (b <= target <= a):
                        t0 = times[j - 1]
                        t1 = times[j]
                        if b == a:
                            return t0
                        frac = (target - a) / (b - a)
                        return t0 + frac * (t1 - t0)
                return None

            # Calculate tau
            if response_type == "integrating":
                post_time = [times[i] for i in post_step_idx]
                post_cv = [cv_vals[i] for i in post_step_idx]
                window_s = 300.0
                tail_idx = [i for i, t in enumerate(post_time) if t >= (post_time[-1] - window_s)]

                tail_cv = [post_cv[i] for i in tail_idx]
                tail_time = [post_time[i] for i in tail_idx]
                tail_delta_cv = tail_cv[-1] - tail_cv[0]
                tail_dt = max(tail_time[-1] - tail_time[0], 1e-9)
                tail_delta_cv_scaled = tail_delta_cv / scale_den

                K = tail_delta_cv_scaled / (delta_mv * tail_dt)
                tau = max(1e-6, times[-1] - t_step)
            else:
                delta_cv_scaled = delta_cv / scale_den
                K = delta_cv_scaled / delta_mv

                cv_63 = cv0 + 0.63 * (cv1 - cv0)
                t63 = find_cross_time(cv_vals, cv_63)
                if t63 is None:
                    tau = max(1e-6, (times[-1] - t_step) * 0.1)
                else:
                    tau = max(1e-6, t63 - t_step)

            if abs(K) < 1e-12:
                logger.warning(f"Process gain too small in {file_path}")
                continue

            # Now compute tunings for each tau_c value
            for tau_c in tau_c_values:
                effective_lambda = tau_c + theta

                if response_type == "integrating":
                    Kc = 1.0 / (abs(K) * max(effective_lambda, 1e-6))
                    Ti = 4.0 * effective_lambda
                else:
                    Kc = (1.0 / abs(K)) * (tau / max(effective_lambda, 1e-6))
                    Ti = min(tau, 4.0 * effective_lambda)

                results.append({
                    'mv': mv_full,
                    'cv': cv_full,
                    'load': load,
                    'tau_c': tau_c,
                    'response_type': response_type,
                    'K_process': K,
                    'tau_process': tau,
                    'theta_process': theta,
                    'Kc': Kc,
                    'Ti': Ti,
                })

    # Save results to CSV
    root = Path(csv_root)
    out_csv = root / "simc_tuning_multi_tau_c.csv"

    try:
        with open(out_csv, 'w', newline='', encoding='utf-8') as fh:
            writer = csv.DictWriter(fh, fieldnames=[
                'mv', 'cv', 'load', 'tau_c', 'response_type',
                'K_process', 'tau_process', 'theta_process', 'Kc', 'Ti'
            ])
            writer.writeheader()
            writer.writerows(results)
        logger.info(f"Multi tau_c SIMC tuning results saved to {out_csv}")
        return out_csv
    except Exception as e:
        logger.error(f"Failed to save multi tau_c tuning CSV to {out_csv}: {e}")
        return None