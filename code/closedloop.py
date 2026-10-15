from simulation import APSSimulation
import os.path as path
from simulation import DEFAULT_PATH_DATA
from pathlib import Path
import logging
import json
import csv
import math
from openloop import save_results_to_csv, _column_base_name

logger = logging.getLogger(__name__)


def _infer_category(sim_name: str | None) -> str:
    if not sim_name:
        return "Unknown"

    parts = str(sim_name).split("_")
    return parts[-1] if parts else "Unknown"


def _build_column_lookup(headers):
    return {_column_base_name(name): name for name in headers if name != "time"}


def _read_closedloop_metadata(folder_path: Path):
    metadata_path = folder_path / 'closedloop_metadata.json'
    if not metadata_path.is_file():
        return None

    try:
        with open(metadata_path, 'r', encoding='utf-8') as fh:
            return json.load(fh)
    except Exception as e:
        logger.warning(f"Failed to read closed-loop metadata from {metadata_path}: {e}")
        return None

def run_closed_loop_steps(
    simulation,
    snapshots,
    controllers,
    closedloop_config,
    data_path: Path = DEFAULT_PATH_DATA,
    additional_cvs=None,
):
    """Run closed-loop simulation steps for each snapshot and controller."""
    if not simulation or not simulation.is_connected():
        logger.error("Simulation is not connected.")
        return

    try:
        # Set simulation timestep
        if not simulation.set_simulation_timestep(closedloop_config["timestep"]):
            logger.error("Failed to set simulation timestep. Aborting closed-loop simulation.")
            return

        # Prepare output directory
        category = closedloop_config.get("category") or _infer_category(closedloop_config.get("sim_name"))
        # Allow tests to specify a run-specific subfolder (e.g. run_id) to avoid overwriting files
        run_id = closedloop_config.get("run_id") or closedloop_config.get("run") or closedloop_config.get("name")
        if run_id:
            new_data_path = Path(data_path) / category / closedloop_config["timestamp"] / "closedloop" / str(run_id)
        else:
            new_data_path = Path(data_path) / category / closedloop_config["timestamp"] / "closedloop"
        new_data_path.mkdir(parents=True, exist_ok=True)

        metadata = {
            "closedloop_config": dict(closedloop_config),
            "category": category,
            "run_id": run_id if run_id is not None else "",
            "snapshots": list(snapshots),
            "controllers": list(controllers),
            "additional_cvs": list(additional_cvs or []),
            "results_dir": str(new_data_path),
            "csv_files": [],
            "solve_status": [],
        }

        for i, snapshot in enumerate(snapshots):
            print(f"Loading snapshot: {snapshot.get('name')} ({snapshot.get('load')})")
            if not simulation.revert_snapshot(snapshot.get('name')):
                logger.error(f"Failed to load snapshot: {snapshot.get('name')}")
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Failed to revert snapshot"
                })
                continue

            # Put controllers into Automatic and apply tuning
            for ctrl in controllers:
                equation_name = ctrl.get('equation', '')
                setpoint_name = ctrl.get('setpoint', f"{ctrl['model']}.{ctrl['variable']}")
                if not simulation.activate_controller(
                    ctrl['model'],
                    equation_name,
                    setpoint_name,
                    ctrl.get('Kp'),
                    ctrl.get('Ti'),
                    set_params=ctrl.get('set_params', True),
                ):
                    logger.error(f"Failed to activate controller: {ctrl['model']}")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "controller": ctrl['model'],
                        "success": False,
                        "reason": "Failed to activate controller"
                    })
                    continue

            # Check if simulation is solved
            if not simulation.check_simulation_status():
                logger.error(f"Simulation status invalid after loading snapshot: {snapshot.get('name')}")
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Simulation status invalid after loading snapshot"
                })
                continue

            # Create a baseline snapshot with controllers in automatic mode
            base_snapshot = simulation.create_snapshot()
            if not base_snapshot:
                logger.error("Failed to create baseline snapshot with controllers in Automatic")
                continue
            print(f"Baseline snapshot created: {base_snapshot}")

            # For each controller, perform a step test and collect all controller signals
            for ctrl in controllers:

                # Rever to base snapshot
                simulation.revert_snapshot(base_snapshot)

                model = ctrl['model']
                sp_var = ctrl['variable']
                unit = ctrl.get('unit', "")

                # Run steady state
                print(f"Running steady state for {closedloop_config['steady_state_time']}s")
                simulation.run_dynamic(closedloop_config['steady_state_time'], 's')

                # Read default SP
                if 'setpoint' in ctrl:
                    sp_full = ctrl['setpoint']
                else:
                    sp_full = f"{model}.{sp_var}"
                default_sp = simulation.get_variable(sp_full, unit)
                if default_sp is None:
                    logger.error(f"Failed to read default SP for {sp_full}. Skipping.")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "controller": model,
                        "success": False,
                        "reason": "Failed to read default SP"
                    })
                    continue

                # Determine multiplier for this snapshot index
                multiplier = None
                try:
                    multiplier = ctrl['multiplier'][i]
                except Exception:
                    # fallback: if multiplier is scalar or missing
                    mval = ctrl.get('multiplier')
                    multiplier = mval if isinstance(mval, (int, float)) else 1.0

                new_sp = default_sp * multiplier
                print(f"Setting {sp_full} from {default_sp} to {new_sp}")
                simulation.set_variable(sp_full, new_sp, unit)

                # Run dynamic period
                print(f"Running dynamic simulation for {closedloop_config['dynamic_time']}s")
                simulation.run_dynamic(closedloop_config['dynamic_time'], 's')
                solve_ok = simulation.check_simulation_status()

                # Create snapshot with history and collect data
                snapshot_name = simulation.create_snapshot_with_history()
                if not snapshot_name:
                    logger.error("Failed to create snapshot with history for closed-loop step")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "controller": model,
                        "success": False,
                        "reason": "Failed to create snapshot with history"
                    })
                    continue

                # Generate timestamps
                total_time = int(closedloop_config['steady_state_time'] + closedloop_config['dynamic_time'])
                step = int(closedloop_config['timestep'])
                timestamps = [t for t in range(0, total_time + 1, max(1, step))]

                # Collect SP, OP, and PV histories for all controllers
                histories = {}
                for ctrl_hist in controllers:
                    ctrl_model = ctrl_hist['model']
                    histories[f"{ctrl_model}.SP"] = simulation.get_variable_history(
                        ctrl_model, 'SP', snapshot_name, timestamps, 2000
                    )
                    histories[f"{ctrl_model}.OP"] = simulation.get_variable_history(
                        ctrl_model, 'OP', snapshot_name, timestamps, 2000
                    )
                    histories[f"{ctrl_model}.PV"] = simulation.get_variable_history(
                        ctrl_model, 'PV', snapshot_name, timestamps, 2000
                    )

                additional_cv_histories = {}
                for cv in additional_cvs or []:
                    cv_model = cv['model']
                    cv_variable = cv['variable']
                    cv_unit = cv.get('unit', '')
                    cv_name = f"{cv_model}.{cv_variable}"
                    additional_cv_histories[cv_name] = simulation.get_variable_history(
                        cv_model, cv_variable, snapshot_name, timestamps, 2000
                    )

                histories_mv = {k: v for k, v in histories.items() if k.endswith('.SP') or k.endswith('.OP')}
                histories_cv = {k: v for k, v in histories.items() if k.endswith('.PV')}
                histories_cv.update(additional_cv_histories)

                histories_units = {}
                for ctrl_hist in controllers:
                    ctrl_model = ctrl_hist['model']
                    ctrl_unit = ctrl_hist.get('unit', '')
                    histories_units[f"{ctrl_model}.SP"] = ctrl_unit
                    histories_units[f"{ctrl_model}.PV"] = ctrl_unit
                    histories_units[f"{ctrl_model}.OP"] = "fraction"

                for cv in additional_cvs or []:
                    cv_name = f"{cv['model']}.{cv['variable']}"
                    histories_units[cv_name] = cv.get('unit', '')

                # Save CSV
                csv_path = new_data_path / f"Closed_{model}_{sp_var}_{snapshot.get('load')}.csv"
                mv_units = {name: histories_units.get(name, '') for name in histories_mv}
                cv_units = {name: histories_units.get(name, '') for name in histories_cv}
                save_results_to_csv(histories_mv, histories_cv, csv_path, mv_units=mv_units, cv_units=cv_units)
                metadata['csv_files'].append(str(csv_path))
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "controller": model,
                    "load": snapshot.get('load'),
                    "success": solve_ok,
                    "reason": "OK" if solve_ok else "Simulation status invalid"
                })
                print(f"Closed-loop results saved to {csv_path}")

        # Export simulation state
        export_path = new_data_path / "ClosedLoopSimulation.simx"
        simulation.export_simulation(export_path)
        metadata['export_path'] = str(export_path)

        metadata_path = new_data_path / "closedloop_metadata.json"
        with open(metadata_path, 'w', encoding='utf-8') as fh:
            json.dump(metadata, fh, indent=2)
        logger.info(f"Closed-loop metadata saved to {metadata_path}")

    except Exception as e:
        logger.error(f"Error during closed-loop simulation: {e}")

def plot_closed_loop_csv(
    csv_file: str,
    save_plot: bool = True,
    show_plot: bool = True,
    prefix_order=None,
):
    """Plot one closed-loop CSV file with rows: SP, PV, OP."""
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
        with open(file_path, 'r', newline='') as fh:
            reader = csv.DictReader(fh)
            rows = list(reader)

        if not rows:
            logger.warning(f"CSV file is empty: {file_path}")
            return

        headers = reader.fieldnames or []
        if 'time' not in headers:
            logger.warning(f"CSV file has no 'time' column: {file_path}")
            return

        column_lookup = _build_column_lookup(headers)

        # Determine model prefixes from column names (exclude 'time')
        discovered_prefixes = [c.rsplit('.', 1)[0] for c in column_lookup.keys() if '.' in c]
        if prefix_order:
            prefixes = [p for p in prefix_order if p in discovered_prefixes]
            for p in discovered_prefixes:
                if p not in prefixes:
                    prefixes.append(p)
        else:
            prefixes = list(dict.fromkeys(discovered_prefixes))
        if not prefixes:
            logger.warning(f"No dotted variable columns (e.g. Model.Var) found in: {file_path}")
            return

        # Build lists for SP, PV, OP per prefix (may be missing for some prefixes)
        sp_cols = [f"{p}.SP" for p in prefixes]
        pv_cols = [f"{p}.PV" for p in prefixes]
        op_cols = [f"{p}.OP" for p in prefixes]

        times = [float(r['time']) for r in rows if r.get('time') not in (None, '')]
        if times:
            x_min = 0.0
            x_max = max(times)
        else:
            x_min = 0.0
            x_max = 1.0
        n_cols = max(1, len(prefixes))
        n_rows = 3

        fig, axes = plt.subplots(n_rows, n_cols, figsize=(3 * n_cols + 4, 8), squeeze=False)
        fig.suptitle(file_path.stem, fontsize=14)

        def plot_series(ax, col_name: str, fixed_ylim=None):
            actual_name = column_lookup.get(col_name)
            if not actual_name:
                ax.axis('off')
                return
            vals = []
            for r in rows:
                raw = r.get(actual_name, '')
                try:
                    vals.append(float(raw) if raw not in (None, '') else float('nan'))
                except Exception:
                    vals.append(float('nan'))
            ax.plot(times, vals, linewidth=1.5)
            ax.set_xlim(x_min, x_max)
            if fixed_ylim is not None:
                ax.set_ylim(*fixed_ylim)
            else:
                # enforce y-limits with 1% padding
                finite_values = [v for v in vals if math.isfinite(v)]
                if finite_values:
                    lower_limit = min(finite_values)
                    upper_limit = max(finite_values)
                    lower_pad = max(abs(lower_limit) * 0.01, 1e-9)
                    upper_pad = max(abs(upper_limit) * 0.01, 1e-9)
                    ax.set_ylim(lower_limit - lower_pad, upper_limit + upper_pad)
                else:
                    ax.axis('off')

            ax.set_title(actual_name)
            ax.set_xlabel('time [s]')
            ax.grid(True, alpha=0.3)

        # Row 0: SP, Row1: PV, Row2: OP
        for j, p in enumerate(prefixes):
            plot_series(axes[0][j], f"{p}.SP")
            plot_series(axes[1][j], f"{p}.PV")
            plot_series(axes[2][j], f"{p}.OP", fixed_ylim=(0, 1))

        # Turn off any unused axes (if prefixes < n_cols for safety)
        for i in range(n_rows):
            for j in range(n_cols):
                ax = axes[i][j]
                if not ax.has_data():
                    # if no artists were added, hide axis
                    ax.axis('off')

        plt.tight_layout()

        if save_plot:
            plot_path = file_path.with_suffix('.png')
            fig.savefig(plot_path, dpi=150, bbox_inches='tight')
            logger.info(f"Plot saved to {plot_path}")

        if show_plot:
            plt.show()
        plt.close(fig)

    except Exception as e:
        logger.error(f"Error plotting {file_path}: {e}")

def plot_closed_loop_results(csv_folder: str, save_plot: bool = True, show_plot: bool = True):
    """Plot every CSV file in a folder using `plot_closed_loop_csv`."""
    folder_path = Path(csv_folder)
    if not folder_path.is_dir():
        logger.warning(f"CSV folder not found: {folder_path}")
        return

    prefix_order = None
    metadata_path = folder_path / 'closedloop_metadata.json'
    if metadata_path.is_file():
        try:
            with open(metadata_path, 'r', encoding='utf-8') as fh:
                metadata = json.load(fh)
            controllers = metadata.get('controllers', [])
            prefix_order = [ctrl.get('model') for ctrl in controllers if ctrl.get('model')]
        except Exception as e:
            logger.warning(f"Failed to read closed-loop metadata from {metadata_path}: {e}")

    csv_files = sorted(folder_path.glob('*.csv'))
    if not csv_files:
        logger.warning(f"No CSV files found in {folder_path}")
        return

    for file_path in csv_files:
        plot_closed_loop_csv(
            str(file_path),
            save_plot=save_plot,
            show_plot=show_plot,
            prefix_order=prefix_order,
        )

def plot_closed_loop_comparison(
    csv_folder: str,
    save_plot: bool = True,
    show_plot: bool = False,
    load_filter: str | None = None,
    step_prefix_filter: str | None = None,
):
    """Plot comparison of closed-loop responses across different controller tunings."""
    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        logger.error(f"matplotlib is required for plotting comparisons: {e}")
        return

    import re

    base = Path(csv_folder)
    if not base.is_dir():
        logger.warning(f"CSV folder not found: {base}")
        return

    # Collect all CSV files under folder and one level of subfolders
    all_csv_files = list(base.glob('*.csv'))
    for sub in [p for p in base.iterdir() if p.is_dir()]:
        all_csv_files.extend(list(sub.glob('*.csv')))

    if not all_csv_files:
        logger.warning(f"No closed-loop CSV files found under {base}")
        return

    # Parse filename: Closed_<STEP>_<VAR>_<LOAD>.csv  (e.g., Closed_XC1_SP_high.csv)
    rx = re.compile(r"^Closed_(?P<step>[^_]+)_(?P<var>[^_]+)_(?P<load>[^_]+)$")

    grouped: dict[tuple[str, str], list[Path]] = {}
    for f in all_csv_files:
        m = rx.match(f.stem)
        if not m:
            continue
        step = m.group('step')
        load = m.group('load')
        if load_filter and load != str(load_filter):
            continue
        if step_prefix_filter and step != str(step_prefix_filter):
            continue
        grouped.setdefault((step, load), []).append(f)

    if not grouped:
        logger.warning(f"No matching closed-loop CSV files found under {base} for the requested filters")
        return

    # Determine controller order: prefer metadata if present (from any run folder)
    prefix_order = None
    metadata_candidates = []
    for sub in [p for p in base.iterdir() if p.is_dir()]:
        mp = sub / 'closedloop_metadata.json'
        if mp.is_file():
            metadata_candidates.append(mp)
    for mp in metadata_candidates:
        try:
            with open(mp, 'r', encoding='utf-8') as fh:
                metadata = json.load(fh)
            controllers = metadata.get('controllers', [])
            order = [c.get('model') for c in controllers if c.get('model')]
            if order:
                prefix_order = order
                break
        except Exception:
            continue

    def read_csv(file_path: Path):
        with open(file_path, 'r', newline='') as fh:
            reader = csv.DictReader(fh)
            headers = reader.fieldnames or []
            rows_data = list(reader)
        return headers, rows_data

    # Build plots for each (step, load) combo
    for (step_prefix, load_name), files in sorted(grouped.items()):
        # Determine which controllers to plot based on headers; default to prefix_order or sorted from first file
        prefixes = []
        try:
            headers0, _rows0 = read_csv(files[0])
            for h in headers0:
                base_name = _column_base_name(h)
                if '.' in base_name:
                    p = base_name.split('.', 1)[0]
                    if p not in prefixes:
                        prefixes.append(p)
        except Exception:
            prefixes = []

        if prefix_order:
            prefixes = [p for p in prefix_order if p in prefixes] + [p for p in prefixes if p not in (prefix_order or [])]
        else:
            prefixes = sorted(prefixes)

        if not prefixes:
            logger.warning(f"No controller prefixes found in {files[0]}")
            continue

        # Create 3×n layout: 3 rows (SP/PV/OP) × n columns (one per controller)
        # Support any number of controllers (e.g., 2 for Inventory, 3 for Obvious/Reactor/Compression)
        n_cols = len(prefixes)
        n_rows = 3

        fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 9), squeeze=False)

        for j, p in enumerate(prefixes):
            axes[0][j].set_title(f"{p}.SP")
            axes[1][j].set_title(f"{p}.PV")
            axes[2][j].set_title(f"{p}.OP")
            axes[2][j].set_xlabel('time [s]')

        fig.suptitle(f"Closed-loop comparison (step in {step_prefix}, load={load_name})")

        # Color per tuning/run (folder name), consistent across all 6 subplots
        color_cycle = plt.rcParams['axes.prop_cycle'].by_key().get('color', ['C0', 'C1', 'C2', 'C3', 'C4', 'C5'])
        linestyles = ['-', '--', '-.', ':']

        legend_handles = []
        legend_labels = []

        # Group files by their parent folder label to keep legend clean
        def run_label_for_file(fp: Path) -> str:
            return fp.parent.name if fp.parent != base else fp.stem

        # Sort by run label to keep stable coloring
        files_sorted = sorted(files, key=lambda fp: run_label_for_file(fp))
        run_labels = []
        for fp in files_sorted:
            rl = run_label_for_file(fp)
            if rl not in run_labels:
                run_labels.append(rl)

        color_for_run = {rl: color_cycle[i % len(color_cycle)] for i, rl in enumerate(run_labels)}
        style_for_run = {rl: linestyles[i % len(linestyles)] for i, rl in enumerate(run_labels)}

        # Match plot_closed_loop_csv axis limits
        x_min = 0.0
        x_max = 1.0
        y_min = [[None for _ in range(n_cols)] for _ in range(n_rows)]
        y_max = [[None for _ in range(n_cols)] for _ in range(n_rows)]

        for fp in files_sorted:
            rl = run_label_for_file(fp)
            try:
                headers, rows_data = read_csv(fp)
                if not rows_data:
                    continue
                column_lookup = _build_column_lookup(headers)
                times = [float(r.get('time') or r.get('Time') or 0.0) for r in rows_data]
            except Exception:
                continue

            if times:
                try:
                    x_max = max(x_max, max(times))
                except Exception:
                    pass

            color = color_for_run[rl]
            linestyle = style_for_run[rl]

            added_to_legend = False
            for j, p in enumerate(prefixes):
                sp_col = f"{p}.SP"
                pv_col = f"{p}.PV"
                op_col = f"{p}.OP"
                sp_actual = column_lookup.get(sp_col)
                pv_actual = column_lookup.get(pv_col)
                op_actual = column_lookup.get(op_col)
                if not any((sp_actual, pv_actual, op_actual)):
                    continue

                sp_vals = [safe_float(r.get(sp_actual, '')) for r in rows_data]
                pv_vals = [safe_float(r.get(pv_actual, '')) for r in rows_data]
                op_vals = [safe_float(r.get(op_actual, '')) for r in rows_data]

                axes[0][j].plot(times, sp_vals, color=color, linestyle=linestyle, linewidth=1.2)
                h_pv, = axes[1][j].plot(times, pv_vals, color=color, linestyle=linestyle, linewidth=1.5)
                axes[2][j].plot(times, op_vals, color=color, linestyle=linestyle, linewidth=1.0)

                # Update y-limits (SP/PV) using finite values, same as plot_closed_loop_csv
                for row_idx, vals in ((0, sp_vals), (1, pv_vals)):
                    finite_values = [v for v in vals if math.isfinite(v)]
                    if not finite_values:
                        continue
                    vmin = min(finite_values)
                    vmax = max(finite_values)
                    cur_min = y_min[row_idx][j]
                    cur_max = y_max[row_idx][j]
                    y_min[row_idx][j] = vmin if cur_min is None else min(cur_min, vmin)
                    y_max[row_idx][j] = vmax if cur_max is None else max(cur_max, vmax)

                if not added_to_legend:
                    legend_handles.append(h_pv)
                    legend_labels.append(rl)
                    added_to_legend = True

        for i in range(n_rows):
            for j in range(n_cols):
                ax = axes[i][j]
                if not ax.has_data():
                    ax.axis('off')
                    continue
                ax.set_xlim(x_min, x_max)
                ax.grid(True, alpha=0.3)
                if i == 2:
                    ax.set_ylim(0, 1)
                else:
                    vmin = y_min[i][j]
                    vmax = y_max[i][j]
                    if vmin is None or vmax is None:
                        ax.axis('off')
                        continue
                    lower_pad = max(abs(vmin) * 0.01, 1e-9)
                    upper_pad = max(abs(vmax) * 0.01, 1e-9)
                    ax.set_ylim(vmin - lower_pad, vmax + upper_pad)

        if legend_handles:
            axes[1][0].legend(legend_handles, legend_labels)

        out_name = base / f"comparison_{step_prefix}_{load_name}.png"
        if save_plot:
            plt.tight_layout()
            plt.savefig(out_name, dpi=150, bbox_inches='tight')
            logger.info(f"Comparison plot saved to {out_name}")
        if show_plot:
            plt.show()
        plt.close(fig)

def safe_float(val):
    try:
        return float(val) if val not in (None, '') else float('nan')
    except Exception:
        return float('nan')


def run_load_step(
    simulation,
    snapshots,
    load_vars,
    controllers,
    additional_vars,
    closedloop_config,
    data_path: Path = DEFAULT_PATH_DATA,
):
    """Run a load-step closed-loop experiment and save controller histories."""
    if not simulation or not simulation.is_connected():
        logger.error("Simulation is not connected.")
        return

    try:
        # Keep all load-step runs on the configured simulation timestep.
        if not simulation.set_simulation_timestep(closedloop_config["timestep"]):
            logger.error("Failed to set simulation timestep. Aborting load-step simulation.")
            return

        # Write results under a run-specific folder when the caller provides one.
        category = closedloop_config.get("category") or _infer_category(closedloop_config.get("sim_name"))
        run_id = closedloop_config.get("run_id") or closedloop_config.get("run") or closedloop_config.get("name")
        if run_id:
            new_data_path = Path(data_path) / category / closedloop_config["timestamp"] / "closedloop" / str(run_id)
        else:
            new_data_path = Path(data_path) / category / closedloop_config["timestamp"] / "closedloop"
        new_data_path.mkdir(parents=True, exist_ok=True)

        metadata = {
            "closedloop_config": dict(closedloop_config),
            "category": category,
            "run_id": run_id if run_id is not None else "",
            "snapshots": list(snapshots),
            "load_vars": list(load_vars),
            "controllers": list(controllers),
            "additional_vars": list(additional_vars or []),
            "results_dir": str(new_data_path),
            "csv_files": [],
            "solve_status": [],
        }

        for i, snapshot in enumerate(snapshots):
            # Restore the starting point for each snapshot before applying controller setup.
            print(f"Loading snapshot: {snapshot.get('name')} ({snapshot.get('load')})")
            if not simulation.revert_snapshot(snapshot.get('name')):
                logger.error(f"Failed to load snapshot: {snapshot.get('name')}")
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Failed to revert snapshot",
                })
                continue

            # Put all controllers into Automatic and apply any tuning values.
            for ctrl in controllers:
                equation_name = ctrl.get('equation', '')
                setpoint_name = ctrl.get('setpoint', f"{ctrl['model']}.{ctrl.get('variable', 'SP')}")
                if not simulation.activate_controller(
                    ctrl['model'],
                    equation_name,
                    setpoint_name,
                    ctrl.get('Kp'),
                    ctrl.get('Ti'),
                    set_params=ctrl.get('set_params', True),
                ):
                    logger.error(f"Failed to activate controller: {ctrl['model']}")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "controller": ctrl['model'],
                        "success": False,
                        "reason": "Failed to activate controller",
                    })
                    continue

            # Skip runs that are not in a valid solved state after controller setup.
            if not simulation.check_simulation_status():
                logger.error(f"Simulation status invalid after loading snapshot: {snapshot.get('name')}")
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Simulation status invalid after loading snapshot",
                })
                continue

            # Capture a clean baseline before stepping the load variable.
            base_snapshot = simulation.create_snapshot()
            if not base_snapshot:
                logger.error("Failed to create baseline snapshot with controllers in Automatic")
                continue
            print(f"Baseline snapshot created: {base_snapshot}")

            # Apply each requested load step and collect the resulting histories.
            for load_var in load_vars:
                simulation.revert_snapshot(base_snapshot)

                load_model = load_var['model']
                load_name = load_var['variable']
                load_unit = load_var.get('unit', '')
                load_full_name = f"{load_model}.{load_name}"

                print(f"Running steady state for {closedloop_config['steady_state_time']}s")
                simulation.run_dynamic(closedloop_config['steady_state_time'], 's')

                default_value = simulation.get_variable(load_full_name, load_unit)
                if default_value is None:
                    logger.error(f"Failed to read default value for {load_full_name}. Skipping.")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "load_var": load_full_name,
                        "success": False,
                        "reason": "Failed to read default value",
                    })
                    continue

                multiplier = None
                try:
                    multiplier = load_var['multiplier'][i]
                except Exception:
                    mval = load_var.get('multiplier')
                    multiplier = mval if isinstance(mval, (int, float)) else 1.0

                new_value = default_value * multiplier
                print(f"Setting {load_full_name} from {default_value} to {new_value}")
                simulation.set_variable(load_full_name, new_value, load_unit)

                print(f"Running dynamic simulation for {closedloop_config['dynamic_time']}s")
                simulation.run_dynamic(closedloop_config['dynamic_time'], 's')
                solve_ok = simulation.check_simulation_status()

                snapshot_name = simulation.create_snapshot_with_history()
                if not snapshot_name:
                    logger.error("Failed to create snapshot with history for load-step run")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "load_var": load_full_name,
                        "success": False,
                        "reason": "Failed to create snapshot with history",
                    })
                    continue

                total_time = int(closedloop_config['steady_state_time'] + closedloop_config['dynamic_time'])
                step = int(closedloop_config.get('timestep_data', closedloop_config['timestep']))
                timestamps = [t for t in range(0, total_time + 1, max(1, step))]

                # Record controller SP/OP/PV histories plus any extra CVs requested by the caller.
                histories = {}
                for ctrl_hist in controllers:
                    ctrl_model = ctrl_hist['model']
                    histories[f"{ctrl_model}.SP"] = simulation.get_variable_history(
                        ctrl_model, 'SP', snapshot_name, timestamps, 2000
                    )
                    histories[f"{ctrl_model}.OP"] = simulation.get_variable_history(
                        ctrl_model, 'OP', snapshot_name, timestamps, 2000
                    )
                    histories[f"{ctrl_model}.PV"] = simulation.get_variable_history(
                        ctrl_model, 'PV', snapshot_name, timestamps, 2000
                    )

                for cv in additional_vars or []:
                    cv_model = cv['model']
                    cv_variable = cv['variable']
                    cv_name = f"{cv_model}.{cv_variable}"
                    histories[cv_name] = simulation.get_variable_history(
                        cv_model, cv_variable, snapshot_name, timestamps, 2000
                    )

                # Save the combined histories in the same MV/CV CSV shape used elsewhere.
                histories_mv = {k: v for k, v in histories.items() if k.endswith('.SP') or k.endswith('.OP')}
                histories_cv = {k: v for k, v in histories.items() if k.endswith('.PV') or k not in histories_mv}

                histories_units = {}
                for ctrl_hist in controllers:
                    ctrl_model = ctrl_hist['model']
                    ctrl_unit = ctrl_hist.get('unit', '')
                    histories_units[f"{ctrl_model}.SP"] = ctrl_unit
                    histories_units[f"{ctrl_model}.PV"] = ctrl_unit
                    histories_units[f"{ctrl_model}.OP"] = "fraction"

                for cv in additional_vars or []:
                    cv_name = f"{cv['model']}.{cv['variable']}"
                    histories_units[cv_name] = cv.get('unit', '')

                csv_path = new_data_path / f"Stepped_{load_model}_{load_name}_{snapshot.get('load')}.csv"
                mv_units = {name: histories_units.get(name, '') for name in histories_mv}
                cv_units = {name: histories_units.get(name, '') for name in histories_cv}
                save_results_to_csv(histories_mv, histories_cv, csv_path, mv_units=mv_units, cv_units=cv_units)
                metadata['csv_files'].append(str(csv_path))
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "load_var": load_full_name,
                    "load": snapshot.get('load'),
                    "success": solve_ok,
                    "reason": "OK" if solve_ok else "Simulation status invalid",
                })
                print(f"Load-step results saved to {csv_path}")

        export_path = new_data_path / "ClosedLoopSimulation.simx"
        simulation.export_simulation(export_path)
        metadata['export_path'] = str(export_path)

        metadata_path = new_data_path / "closedloop_metadata.json"
        with open(metadata_path, 'w', encoding='utf-8') as fh:
            json.dump(metadata, fh, indent=2)
        logger.info(f"Load-step metadata saved to {metadata_path}")

    except Exception as e:
        logger.error(f"Error during load-step simulation: {e}")

def plot_load_step_results(
    category,
    timestamp,
    snapshots,
    save_plot=True,
    show_plot=False,
    figure_width=28,
    figure_height=17,
):
    """Plot load-step results from run_load_step."""
    try:
        import matplotlib.pyplot as plt
    except ImportError as e:
        logger.error(f"matplotlib is required for plotting: {e}")
        return

    base_path = Path(DEFAULT_PATH_DATA) / str(category) / str(timestamp) / 'closedloop'
    if not base_path.is_dir():
        logger.warning(f"Load-step results folder not found: {base_path}")
        return

    load_filters = {snap.get('load') for snap in snapshots or [] if snap.get('load')}
    csv_files = sorted(base_path.rglob('Stepped_*.csv'))
    if load_filters:
        csv_files = [fp for fp in csv_files if any(str(fp.stem).endswith(f"_{load}") for load in load_filters)]

    if not csv_files:
        logger.warning(f"No load-step CSV files found under {base_path}")
        return

    metadata_candidates = []
    for folder in [base_path] + [p for p in base_path.rglob('*') if p.is_dir()]:
        metadata = _read_closedloop_metadata(folder)
        if metadata:
            metadata_candidates.append((folder, metadata))

    def metadata_for_csv(file_path: Path):
        for folder, metadata in metadata_candidates:
            try:
                if file_path.is_relative_to(folder):
                    return metadata
            except Exception:
                if str(folder) in str(file_path):
                    return metadata
        return None

    def get_ordered_items(metadata, key):
        items = metadata.get(key, []) if metadata else []
        return [item for item in items if isinstance(item, dict)]

    def plot_controller_subplot(ax, rows, headers, column_lookup, controller_name, controller_unit):
        sp_name = f"{controller_name}.SP"
        pv_name = f"{controller_name}.PV"
        op_name = f"{controller_name}.OP"

        sp_col = column_lookup.get(sp_name)
        pv_col = column_lookup.get(pv_name)
        op_col = column_lookup.get(op_name)

        if not any((sp_col, pv_col, op_col)):
            ax.axis('off')
            return

        times = [float(r['time']) for r in rows if r.get('time') not in (None, '')]
        if times:
            x_min = 0.0
            x_max = max(times)
        else:
            x_min = 0.0
            x_max = 1.0

        ax2 = ax.twinx()

        def series(column_name):
            if not column_name:
                return []
            values = []
            for row in rows:
                raw_value = row.get(column_name, '')
                try:
                    values.append(float(raw_value) if raw_value not in (None, '') else float('nan'))
                except Exception:
                    values.append(float('nan'))
            return values

        if sp_col:
            sp_vals = series(sp_col)
            ax.plot(times, sp_vals, label='SP', linewidth=1.5, color='tab:blue')
        if pv_col:
            pv_vals = series(pv_col)
            ax.plot(times, pv_vals, label='PV', linewidth=1.5, color='tab:orange')
        if op_col:
            op_vals = series(op_col)
            ax2.plot(times, op_vals, label='OP', linewidth=1.2, color='tab:green', linestyle='--')

        finite_primary = []
        for col in (sp_col, pv_col):
            if not col:
                continue
            for row in rows:
                raw_value = row.get(col, '')
                try:
                    value = float(raw_value) if raw_value not in (None, '') else float('nan')
                except Exception:
                    value = float('nan')
                if math.isfinite(value):
                    finite_primary.append(value)

        if finite_primary:
            lower_limit = min(finite_primary)
            upper_limit = max(finite_primary)
            lower_pad = max(abs(lower_limit) * 0.01, 1e-9)
            upper_pad = max(abs(upper_limit) * 0.01, 1e-9)
            ax.set_ylim(lower_limit - lower_pad, upper_limit + upper_pad)

        finite_secondary = []
        if op_col:
            for row in rows:
                raw_value = row.get(op_col, '')
                try:
                    value = float(raw_value) if raw_value not in (None, '') else float('nan')
                except Exception:
                    value = float('nan')
                if math.isfinite(value):
                    finite_secondary.append(value)

        if finite_secondary:
            lower_limit = min(finite_secondary)
            upper_limit = max(finite_secondary)
            lower_pad = max(abs(lower_limit) * 0.01, 1e-9)
            upper_pad = max(abs(upper_limit) * 0.01, 1e-9)
            ax2.set_ylim(lower_limit - lower_pad, upper_limit + upper_pad)

        ax.set_xlim(x_min, x_max)
        ax.set_title(f"{controller_name} [{controller_unit}]" if controller_unit else controller_name)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('SP / PV')
        ax2.set_ylabel('OP')
        ax.grid(True, alpha=0.3)

        handles1, labels1 = ax.get_legend_handles_labels()
        handles2, labels2 = ax2.get_legend_handles_labels()
        if handles1 or handles2:
            ax.legend(handles1 + handles2, labels1 + labels2, loc='best')

    def plot_additional_subplot(ax, rows, headers, column_lookup, variable_name, variable_unit):
        actual_name = column_lookup.get(variable_name)
        if not actual_name:
            ax.axis('off')
            return

        times = [float(r['time']) for r in rows if r.get('time') not in (None, '')]
        if times:
            x_min = 0.0
            x_max = max(times)
        else:
            x_min = 0.0
            x_max = 1.0

        values = []
        for row in rows:
            raw_value = row.get(actual_name, '')
            try:
                values.append(float(raw_value) if raw_value not in (None, '') else float('nan'))
            except Exception:
                values.append(float('nan'))

        ax.plot(times, values, linewidth=1.5, color='tab:blue')
        finite_values = [value for value in values if math.isfinite(value)]
        if finite_values:
            lower_limit = min(finite_values)
            upper_limit = max(finite_values)
            lower_pad = max(abs(lower_limit) * 0.01, 1e-9)
            upper_pad = max(abs(upper_limit) * 0.01, 1e-9)
            ax.set_ylim(lower_limit - lower_pad, upper_limit + upper_pad)

        ax.set_xlim(x_min, x_max)
        ax.set_title(f"{variable_name} [{variable_unit}]" if variable_unit else variable_name)
        ax.set_xlabel('time [s]')
        ax.grid(True, alpha=0.3)

    for csv_file in csv_files:
        try:
            with open(csv_file, 'r', newline='') as fh:
                reader = csv.DictReader(fh)
                rows = list(reader)

            if not rows:
                logger.warning(f"CSV file is empty: {csv_file}")
                continue

            headers = reader.fieldnames or []
            if 'time' not in headers:
                logger.warning(f"CSV file has no 'time' column: {csv_file}")
                continue

            column_lookup = _build_column_lookup(headers)
            metadata = metadata_for_csv(csv_file)

            controllers = get_ordered_items(metadata, 'controllers')
            additional_vars = get_ordered_items(metadata, 'additional_vars')

            controller_names = [ctrl.get('model') for ctrl in controllers if ctrl.get('model')]
            if not controller_names:
                controller_names = []
                for name in column_lookup.keys():
                    if name.endswith('.SP'):
                        controller_names.append(name.rsplit('.', 1)[0])
                controller_names = list(dict.fromkeys(controller_names))

            subplot_items = []
            for ctrl in controllers:
                model_name = ctrl.get('model')
                if model_name:
                    subplot_items.append({
                        'kind': 'controller',
                        'name': model_name,
                        'unit': ctrl.get('unit', ''),
                    })

            for cv in additional_vars:
                cv_name = cv.get('variable') or cv.get('name')
                if cv.get('model') and cv_name:
                    subplot_items.append({
                        'kind': 'additional',
                        'name': f"{cv['model']}.{cv_name}",
                        'unit': cv.get('unit', ''),
                    })

            if not subplot_items:
                seen_prefixes = []
                for base_name in column_lookup.keys():
                    if '.' in base_name:
                        prefix = base_name.split('.', 1)[0]
                        if prefix not in seen_prefixes:
                            seen_prefixes.append(prefix)
                subplot_items = [{'kind': 'controller', 'name': prefix, 'unit': ''} for prefix in seen_prefixes]

            if not subplot_items:
                logger.warning(f"No plottable columns found in: {csv_file}")
                continue

            n_cols = 4
            n_rows = max(1, math.ceil(len(subplot_items) / n_cols))
            fig, axes = plt.subplots(
                n_rows,
                n_cols,
                figsize=(figure_width, figure_height),
                squeeze=False,
            )
            fig.suptitle(csv_file.stem, fontsize=14)

            for index, item in enumerate(subplot_items):
                row_index, col_index = divmod(index, n_cols)
                ax = axes[row_index][col_index]
                if item['kind'] == 'controller':
                    plot_controller_subplot(ax, rows, headers, column_lookup, item['name'], item.get('unit', ''))
                else:
                    plot_additional_subplot(ax, rows, headers, column_lookup, item['name'], item.get('unit', ''))

            for index in range(len(subplot_items), n_rows * n_cols):
                row_index, col_index = divmod(index, n_cols)
                axes[row_index][col_index].axis('off')

            plt.tight_layout()

            if save_plot:
                plot_path = csv_file.with_suffix('.png')
                fig.savefig(plot_path, dpi=150, bbox_inches='tight')
                logger.info(f"Plot saved to {plot_path}")

            if show_plot:
                plt.show()
            plt.close(fig)

        except Exception as e:
            logger.error(f"Error plotting load-step results from {csv_file}: {e}")


def run_full_ramp(
    simulation,
    snapshots,
    load_vars,
    controllers,
    additional_vars,
    closedloop_config,
    data_path: Path = DEFAULT_PATH_DATA,
):
    """Run a full ramp experiment and save controller histories."""
    if not simulation or not simulation.is_connected():
        logger.error("Simulation is not connected.")
        return

    if not load_vars:
        logger.error("No load variables provided for ramp simulation.")
        return

    try:
        if not simulation.set_simulation_timestep(closedloop_config["timestep"]):
            logger.error("Failed to set simulation timestep. Aborting ramp simulation.")
            return

        category = closedloop_config.get("category") or _infer_category(closedloop_config.get("sim_name"))
        run_id = closedloop_config.get("run_id") or closedloop_config.get("run") or closedloop_config.get("name")
        if run_id:
            new_data_path = Path(data_path) / category / closedloop_config["timestamp"] / "closedloop" / str(run_id)
        else:
            new_data_path = Path(data_path) / category / closedloop_config["timestamp"] / "closedloop"
        new_data_path.mkdir(parents=True, exist_ok=True)

        metadata = {
            "mode": "full_ramp",
            "closedloop_config": dict(closedloop_config),
            "category": category,
            "run_id": run_id if run_id is not None else "",
            "snapshots": list(snapshots),
            "load_vars": list(load_vars),
            "controllers": list(controllers),
            "additional_vars": list(additional_vars or []),
            "results_dir": str(new_data_path),
            "csv_files": [],
            "solve_status": [],
        }

        ramp_rate = float(closedloop_config.get("ramping_rate", 0.0))
        if ramp_rate <= 0:
            logger.error(f"Ramping rate must be positive, got {ramp_rate}")
            return

        high_fraction = 1.0
        low_fraction = 0.1
        steady_state_start = float(closedloop_config["steady_state_start"])
        steady_state_low = float(closedloop_config["steady_state_low"])
        steady_state_high = float(closedloop_config["steady_state_high"])
        mid_fraction = closedloop_config.get("mid_fraction")
        steady_state_mid = float(closedloop_config.get("steady_state_mid", 0.0))

        for snapshot in snapshots:
            print(f"Loading snapshot: {snapshot.get('name')} ({snapshot.get('load')})")
            if not simulation.revert_snapshot(snapshot.get('name')):
                logger.error(f"Failed to load snapshot: {snapshot.get('name')}")
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Failed to revert snapshot",
                })
                continue

            # Put every controller back into automatic mode before running the ramp.
            controllers_ok = True
            for ctrl in controllers:
                equation_name = ctrl.get('equation', '')
                setpoint_name = ctrl.get('setpoint', f"{ctrl['model']}.{ctrl.get('variable', 'SP')}")
                if not simulation.activate_controller(
                    ctrl['model'],
                    equation_name,
                    setpoint_name,
                    ctrl.get('Kp'),
                    ctrl.get('Ti'),
                    set_params=ctrl.get('set_params', True),
                ):
                    logger.error(f"Failed to activate controller: {ctrl['model']}")
                    controllers_ok = False
                    break

            if not controllers_ok:
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Failed to activate controller",
                })
                continue

            if not simulation.check_simulation_status():
                logger.error(f"Simulation status invalid after loading snapshot: {snapshot.get('name')}")
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "success": False,
                    "reason": "Simulation status invalid after loading snapshot",
                })
                continue

            print(f"Preparing ramp baseline for {snapshot.get('name')}")
            base_snapshot = simulation.create_snapshot()
            if not base_snapshot:
                logger.error("Failed to create baseline snapshot with controllers in Automatic")
                continue
            print(f"Baseline snapshot created: {base_snapshot}")

            for load_var in load_vars:
                print(f"Starting ramp case for {load_var['model']}.{load_var['variable']}")
                simulation.revert_snapshot(base_snapshot)

                load_model = load_var['model']
                load_name = load_var['variable']
                load_unit = load_var.get('unit', '')
                load_full_name = f"{load_model}.{load_name}"

                high_value = simulation.get_variable(load_full_name, load_unit)
                if high_value is None:
                    logger.error(f"Failed to read current high-load value for {load_full_name}. Skipping.")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "load_var": load_full_name,
                        "success": False,
                        "reason": "Failed to read high-load value",
                    })
                    continue

                # Convert the current load range into a ramp duration using the configured ramp rate.
                low_value = high_value * low_fraction / high_fraction
                high_tpd = high_value * 86400.0 / 1000.0
                low_tpd = low_value * 86400.0 / 1000.0
                ramp_seconds = abs(high_tpd - low_tpd) / ramp_rate * 60.0
                if ramp_seconds <= 0:
                    logger.error(f"Computed ramp duration is not positive for {load_full_name}")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "load_var": load_full_name,
                        "success": False,
                        "reason": "Computed ramp duration is not positive",
                    })
                    continue

                print(f"Setting up ramp for {load_full_name}: high={high_value}, low={low_value}, ramp={ramp_seconds:.1f}s")

                # High-load steady state before the first ramp.
                print("Stage: high-load steady state")
                if not (
                    simulation.set_variable_specification(load_full_name,False) and
                    simulation.set_variable_specification("SP0", True) and
                    simulation.set_variable("SP0", high_value, load_unit)
                ):
                    logger.error(f"Failed to set initial SP0 for {load_full_name}")
                    continue
                simulation.set_variable("SwitchH2Flow", 0)
                simulation.set_variable("Slope", 0)
                simulation.set_variable("t0", 0, "s")

                print(f"Running steady state for {steady_state_start}s")
                simulation.run_dynamic(steady_state_start, 's')

                simulation.set_variable("SwitchH2Flow", -1)
                simulation.set_variable("Slope", ramp_rate)
                simulation.set_variable("t0", steady_state_start, "s")

                print(f"Running ramp down for {ramp_seconds:.1f}s")
                simulation.run_dynamic(ramp_seconds, 's')

                # Hold the low-load operating point before ramping back up.
                print("Stage: low-load hold")
                simulation.set_variable("SwitchH2Flow", 0)
                if not (
                    simulation.set_variable("SP0", low_value, load_unit)
                ):
                    logger.error(f"Failed to set low-load SP0 for {load_full_name}")
                    continue

                print(f"Running low steady state for {steady_state_low}s")
                simulation.run_dynamic(steady_state_low, 's')

                simulation.set_variable("SwitchH2Flow", 1)
                simulation.set_variable("Slope", ramp_rate)
                simulation.set_variable("t0", steady_state_start + ramp_seconds + steady_state_low, "s")

                print(f"Running ramp up for {ramp_seconds:.1f}s")
                simulation.run_dynamic(ramp_seconds, 's')

                # Restore the high-load steady state at the end of the profile.
                print("Stage: final high-load hold")
                simulation.set_variable("SwitchH2Flow", 0)
                if not (
                    simulation.set_variable("SP0", high_value, load_unit)
                ):
                    logger.error(f"Failed to restore high-load SP0 for {load_full_name}")
                    continue

                print(f"Running final high steady state for {steady_state_high}s")
                simulation.run_dynamic(steady_state_high, 's')

                # Optional final ramp down to an intermediate load, held there.
                mid_seconds = 0.0
                if mid_fraction is not None:
                    mid_value = high_value * float(mid_fraction) / high_fraction
                    mid_ramp_seconds = abs(high_tpd - mid_value * 86400.0 / 1000.0) / ramp_rate * 60.0

                    simulation.set_variable("SwitchH2Flow", -1)
                    simulation.set_variable("Slope", ramp_rate)
                    simulation.set_variable("t0", steady_state_start + 2 * ramp_seconds + steady_state_low + steady_state_high, "s")

                    print(f"Running ramp down to intermediate load for {mid_ramp_seconds:.1f}s")
                    simulation.run_dynamic(mid_ramp_seconds, 's')

                    print("Stage: intermediate-load hold")
                    simulation.set_variable("SwitchH2Flow", 0)
                    if not (
                        simulation.set_variable("SP0", mid_value, load_unit)
                    ):
                        logger.error(f"Failed to set intermediate-load SP0 for {load_full_name}")
                        continue

                    print(f"Running intermediate steady state for {steady_state_mid}s")
                    simulation.run_dynamic(steady_state_mid, 's')
                    mid_seconds = mid_ramp_seconds + steady_state_mid

                print("Stage: collecting results")
                solve_ok = simulation.check_simulation_status()

                snapshot_name = simulation.create_snapshot_with_history()
                if not snapshot_name:
                    logger.error("Failed to create snapshot with history for ramp run")
                    metadata["solve_status"].append({
                        "snapshot": snapshot.get('name'),
                        "load_var": load_full_name,
                        "success": False,
                        "reason": "Failed to create snapshot with history",
                    })
                    continue

                total_time = int(math.ceil(steady_state_start + ramp_seconds + steady_state_low + ramp_seconds + steady_state_high + mid_seconds))
                step = max(1, int(closedloop_config.get('timestep_data', closedloop_config['timestep'])))
                timestamps = [t for t in range(0, total_time + 1, step)]

                histories = {}
                for ctrl_hist in controllers:
                    ctrl_model = ctrl_hist['model']
                    histories[f"{ctrl_model}.SP"] = simulation.get_variable_history(
                        ctrl_model, 'SP', snapshot_name, timestamps, 2000
                    )
                    histories[f"{ctrl_model}.OP"] = simulation.get_variable_history(
                        ctrl_model, 'OP', snapshot_name, timestamps, 2000
                    )
                    histories[f"{ctrl_model}.PV"] = simulation.get_variable_history(
                        ctrl_model, 'PV', snapshot_name, timestamps, 2000
                    )

                for cv in additional_vars or []:
                    cv_model = cv['model']
                    cv_variable = cv['variable']
                    cv_name = f"{cv_model}.{cv_variable}"
                    histories[cv_name] = simulation.get_variable_history(
                        cv_model, cv_variable, snapshot_name, timestamps, 2000
                    )

                histories_mv = {k: v for k, v in histories.items() if k.endswith('.SP') or k.endswith('.OP')}
                histories_cv = {k: v for k, v in histories.items() if k.endswith('.PV') or k not in histories_mv}

                histories_units = {}
                for ctrl_hist in controllers:
                    ctrl_model = ctrl_hist['model']
                    ctrl_unit = ctrl_hist.get('unit', '')
                    histories_units[f"{ctrl_model}.SP"] = ctrl_unit
                    histories_units[f"{ctrl_model}.PV"] = ctrl_unit
                    histories_units[f"{ctrl_model}.OP"] = "fraction"

                for cv in additional_vars or []:
                    cv_name = f"{cv['model']}.{cv['variable']}"
                    histories_units[cv_name] = cv.get('unit', '')

                csv_path = new_data_path / f"Stepped_{load_model}_{load_name}_{snapshot.get('load')}.csv"
                mv_units = {name: histories_units.get(name, '') for name in histories_mv}
                cv_units = {name: histories_units.get(name, '') for name in histories_cv}
                save_results_to_csv(histories_mv, histories_cv, csv_path, mv_units=mv_units, cv_units=cv_units)
                metadata['csv_files'].append(str(csv_path))
                metadata["solve_status"].append({
                    "snapshot": snapshot.get('name'),
                    "load_var": load_full_name,
                    "load": snapshot.get('load'),
                    "success": solve_ok,
                    "reason": "OK" if solve_ok else "Simulation status invalid",
                })
                metadata.setdefault("ramp_summary", []).append({
                    "snapshot": snapshot.get('name'),
                    "load_var": load_full_name,
                    "high_value": high_value,
                    "low_value": low_value,
                    "ramp_seconds": ramp_seconds,
                    "mid_value": mid_value if mid_fraction is not None else None,
                    "mid_seconds": mid_seconds,
                })
                print(f"Ramp results saved to {csv_path}")

        export_path = new_data_path / "ClosedLoopSimulation.simx"
        simulation.export_simulation(export_path)
        metadata['export_path'] = str(export_path)

        metadata_path = new_data_path / "closedloop_metadata.json"
        with open(metadata_path, 'w', encoding='utf-8') as fh:
            json.dump(metadata, fh, indent=2)
        logger.info(f"Ramp metadata saved to {metadata_path}")

    except Exception as e:
        logger.error(f"Error during ramp simulation: {e}")

