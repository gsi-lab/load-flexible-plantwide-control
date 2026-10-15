"""Core simulation interface for AVEVA Process Simulation."""

try:
    import simcentralconnect
except ImportError:
    simcentralconnect = None
from pathlib import Path
from typing import List, Tuple, Optional, Any
import logging
from datetime import datetime
import csv

# Configuration defaults
DEFAULT_TIMEOUT = 60000
DEFAULT_API_LOGGING = False
DEFAULT_CHECK_TIME = 10.0
DEFAULT_PATH_SIM = Path(__file__).parent.parent / "models"
DEFAULT_PATH_DATA = Path(__file__).parent.parent / "results"


logger = logging.getLogger(__name__)

class APSSimulation:
    """Interface for AVEVA Process Simulation operations."""

    def __init__(self, model_name: str):
        """Initialize the simulation interface."""
        self.model_name = model_name
        self.vm = None  # Variable Manager
        self.sm = None  # Simulation Manager
        self.pm = None  # Parameter Manager
        self.ssm = None  # Snapshot Manager
        self.fm = None  # Flowsheet Manager
        self.sim_path = None
        self._connected = False

    def connect(self, model_path: Path = DEFAULT_PATH_SIM, timeout: int = DEFAULT_TIMEOUT, api_logging: bool = DEFAULT_API_LOGGING) -> bool:
        """Connect to AVEVA Process Simulation."""
        try:
            # Connect to AVEVA Process Simulation
            sc = simcentralconnect.connect().Result

            # Get required services
            self.vm = sc.GetService("IVariableManager")
            self.sm = sc.GetService("ISimulationManager")
            self.pm = sc.GetService("IParameterManager")
            self.ssm = sc.GetService("ISnapshotManager")
            self.fm = sc.GetService("IFlowsheetManager")

            # Set options
            options = {
                'Timeout': timeout,
                'EnableApiLogging': str(api_logging).lower()
            }
            sc.SetOptions(repr(options))

            # Import and open simulation
            self.sim_path = model_path / f"{self.model_name}.simx"
            logger.info(f"Attempting to import model: {self.model_name}")
            logger.info(f"Model file path: {self.sim_path}")
            logger.info(f"Model file exists: {self.sim_path.exists()}")

            imported_name = self.sm.ImportSim(str(self.sim_path), Timeout=1000000).Result
            logger.info(f"ImportSim returned model name: {imported_name}")

            if imported_name != self.model_name:
                logger.warning(f"Imported model name '{imported_name}' differs from requested '{self.model_name}'")

            self.model_name = imported_name
            self.sm.OpenSimulation(self.model_name).Result

            self._connected = True
            logger.info(f"Successfully connected to simulation: {self.model_name}")
            return True

        except Exception as e:
            logger.error(f"Failed to connect to simulation: {e}")
            self._connected = False
            return False

    def disconnect(self):

        """Clean disconnect from simulation."""
        if self._connected:
            try:
                # Close and delete only the specific simulation we imported
                if self.sm and self.model_name:
                    logger.info(f"Closing and deleting simulation: {self.model_name}")
                    # self.sm.CloseSimulation(self.model_name, False).Result
                    result = self.sm.DeleteSim(self.model_name).Result
                    if result:
                        logger.info(f"Successfully deleted simulation: {self.model_name}")
                    else:
                        logger.warning(f"Failed to delete simulation: {self.model_name}")
                logger.info("Disconnected from simulation")
            except Exception as e:
                # Check if it's just a "simulation doesn't exist" error, which is harmless
                error_msg = str(e)
                if "simulation doesn't exists" in error_msg or "doesn't exist" in error_msg:
                    logger.debug(f"Simulation already closed during disconnect: {self.model_name}")
                else:
                    logger.warning(f"Error during disconnect: {e}")
            finally:
                self._connected = False

    def is_connected(self) -> bool:
        """Check if simulation is connected."""
        return self._connected

    def revert_snapshot(self, snapshot_name: str) -> bool:
        """Revert to a specific snapshot."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        try:
            result = self.ssm.RevertSnapshot(self.model_name, snapshot_name).Result
            if result:
                logger.info(f"Reverted to snapshot: {snapshot_name}")
            else:
                logger.error(f"Failed to revert to snapshot: {snapshot_name}")
            return result
        except Exception as e:
            logger.error(f"Error reverting to snapshot {snapshot_name}: {e}")
            return False

    def set_simulation_timestep(self, timestep_seconds: float) -> bool:
        """Set the simulation timestep (Main.dt)."""
        if timestep_seconds <= 0:
            logger.error(f"Timestep must be positive, got {timestep_seconds}")
            return False

        return self.update_parameter_value("dt", str(timestep_seconds))

    def run_dynamic(self, time_step: float, uom: str = 's') -> bool:
        """Run dynamic simulation for specified time."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        try:
            self.sm.RunDynamics(self.model_name, time_step, uom, time_step).Result
            return self.check_simulation_status()
        except Exception as e:
            logger.error(f"Failed to run dynamics: {e}")
            return False

    def get_variable(self, var_name: str, uom: str = "", timeout: int = 2000) -> Optional[Any]:
        """Get a variable value from the simulation."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return None

        try:
            return self.vm.GetVariableValue(self.model_name, var_name, uom, timeout).Result
        except Exception as e:
            logger.error(f"Failed to get variable {var_name}: {e}")
            return None

    def set_variable(self, var_name: str, value: Any, uom: str = "", timeout: int = 20000) -> bool:
        """Set a variable value in the simulation."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        try:
            self.vm.SetVariableValue(self.model_name, var_name, value, uom, timeout).Result
            return True
        except Exception as e:
            logger.error(f"Failed to set variable {var_name}: {e}")
            return False

    def check_simulation_status(self) -> bool:
        """Check if simulation has valid status."""
        if not self._connected:
            return False
        try:
            status = self.sm.GetSimulationStatus(self.model_name).Result
            if not all(list(status)[:3]):
                logger.warning(f"Invalid simulation status: Input={status[0]}, "
                             f"Specification={status[1]}, Solve={status[2]}")
                return False
            return True
        except Exception as e:
            logger.error(f"Failed to check simulation status: {e}")
            return False

    def update_parameter_value(self, parameter: str, value: Any, timeout: int = 20000) -> bool:
        """Update a simulation parameter value."""
        try:
            result = self.pm.UpdateParameterValue(
                self.model_name,
                parameter,
                value,
                timeout
            ).Result
            if result is not None:
                logger.info(f"Successfully updated parameter {parameter} = {value}")
                return True
            else:
                logger.error(f"Failed to update parameter {parameter}")
                return False
        except Exception as e:
            logger.error(f"Error updating parameter {parameter}: {e}")
            return False

    def create_snapshot(self) -> str:
        """Create a new snapshot."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return None

        try:
            # Get list of snapshots before creating new one
            list_old_names = self.ssm.GetAllSnapshots(self.model_name).Result

            # Create snapshot - system will generate the name
            check = self.ssm.CreateSnapshot(self.model_name).Result

            # Get updated list of snapshots
            list_new_names = self.ssm.GetAllSnapshots(self.model_name).Result

            # Find the new snapshot name
            new_snapshot_name = None
            if list_new_names != list_old_names:
                new_snapshot_name = list_new_names[-1].name

            if check and new_snapshot_name:
                logger.info(f"Created snapshot: {new_snapshot_name}")
                return new_snapshot_name
            else:
                logger.error("Failed to create snapshot")
                return None

        except Exception as e:
            logger.error(f"Error creating snapshot: {e}")
            return None

    def create_snapshot_with_history(self) -> str:
        """Create a snapshot with dynamic history for data collection."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return None

        try:
            # Get list of snapshots before creating new one
            list_old_names = self.ssm.GetAllSnapshots(self.model_name).Result

            # Create snapshot with history - system will generate the name
            check = self.ssm.CreateSnapshotWithHistory(self.model_name).Result

            # Get updated list of snapshots
            list_new_names = self.ssm.GetAllSnapshots(self.model_name).Result

            # Find the new snapshot name
            new_snapshot_name = None
            if list_new_names != list_old_names:
                new_snapshot_name = list_new_names[-1].name

            if check and new_snapshot_name:
                logger.info(f"Created snapshot with history: {new_snapshot_name}")
                return new_snapshot_name
            else:
                logger.error("Failed to create snapshot with history")
                return None

        except Exception as e:
            logger.error(f"Error creating snapshot with history: {e}")
            return None

    def get_variable_history(self, model_name: str, variable_name: str,
                           snapshot_name: str, timestamps: List[float],
                           timeout: int = 2000) -> List[Tuple[float, Any]]:
        """Get variable history from a specific snapshot."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return []

        if not self.revert_snapshot(snapshot_name):
            return []

        data = []
        for t in timestamps:
            try:
                value = self.vm.GetVariableValueFromSnapshotHistory(
                    model_name, variable_name, self.model_name,
                    snapshot_name, t, timeout
                ).Result
                data.append((t, value))
            except Exception as e:
                logger.warning(f"Error getting history at t={t}: {e}")
                data.append((t, None))

        return data

    def export_simulation(self, export_path: str) -> bool:
        """Export the current simulation state."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        try:
            result = self.sm.ExportSim(self.model_name, str(export_path)).Result
            if result:
                logger.info(f"Simulation exported to: {export_path}")
            else:
                logger.error(f"Failed to export simulation to: {export_path}")
            return result
        except Exception as e:
            logger.error(f"Error exporting simulation: {e}")
            return False

    def activate_equation(self, equation_name: str, is_active: bool = True) -> bool:
        """Activate or deactivate an equation in Dynamics mode."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        if not self.fm:
            logger.error("Flowsheet Manager not available")
            return False

        try:
            result = self.fm.SetEquationActiveInMode(
                self.model_name,
                equation_name,
                "IsActiveInDynamicsMode",
                is_active,
                {}
            ).Result

            if result:
                action = "activated" if is_active else "deactivated"
                logger.info(f"Successfully {action} equation '{equation_name}'")
                return True
            else:
                action = "activate" if is_active else "deactivate"
                logger.error(f"Failed to {action} equation '{equation_name}'")
                return False

        except Exception as e:
            action = "activate" if is_active else "deactivate"
            logger.error(f"Error trying to {action} equation '{equation_name}': {e}")
            return False

    def activate_controller(
        self,
        controller_model: str,
        equation_name: str = "",
        setpoint_name: str = "",
        kp: Any = None,
        ti: Any = None,
        set_params: bool = True,
        timeout: int = 20000,
    ) -> bool:
        """Activate a controller for closed-loop operation."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        if not controller_model or not controller_model.strip():
            logger.error("Controller model cannot be empty")
            return False

        op_param = f"{controller_model}.Operation"
        if not self.update_parameter_value(op_param, "Automatic", timeout):
            logger.error(f"Failed to set controller '{controller_model}' to Automatic")
            return False

        if equation_name:
            check = self.activate_equation(equation_name)
            self.set_variable_specification(setpoint_name, False)
            if not check:
                return False

        if set_params:
            if kp is not None and not self.set_variable(f"{controller_model}.Kp", kp):
                logger.error(f"Failed to set Kp for controller '{controller_model}'")
                return False

            if ti is not None and not self.set_variable(f"{controller_model}.Ti", ti):
                logger.error(f"Failed to set Ti for controller '{controller_model}'")
                return False

        logger.info(f"Successfully activated controller '{controller_model}'")
        return True

    def set_variable_specification(self, variable_name: str, is_specified: bool,
                                  timeout: int = 20000) -> bool:
        """Set whether a variable is specified or unspecified in the simulation."""
        if not self._connected:
            logger.error("Not connected to simulation")
            return False

        try:
            # Use the Variable Manager to set variable specification
            result = self.vm.SetVariableSpecification(
                self.model_name,
                variable_name,
                is_specified,
                timeout
            ).Result

            if result:
                action = "specified" if is_specified else "unspecified"
                logger.info(f"Successfully {action} variable '{variable_name}'")
                return True
            else:
                action = "specify" if is_specified else "unspecify"
                logger.error(f"Failed to {action} variable '{variable_name}'")
                return False

        except Exception as e:
            action = "specify" if is_specified else "unspecify"
            logger.error(f"Error trying to {action} variable '{variable_name}': {e}")
            return False
