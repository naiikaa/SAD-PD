from dataclasses import dataclass, fields, asdict
from pathlib import Path
import toml

# decouple general config (like carla address) from sensor/scenario configs
@dataclass
class ExperimentConfig:
    experiment_name: str
    fps: int = 20
    host: str = "localhost"
    port: int = 2000
    bridge_passive_mode: bool = False
    ego_vehicle_role_name: str = "ego_vehicle"
    render_mode: bool = True

    # scenario config
    town: str = "Town10HD"
    spawn_point: int = 3
    generate_traffic_path: str = Path(__file__).parent.parent.parent / "carla" / "PythonAPI" / "examples" / "generate_traffic.py"
    scenario_type: str = "traffic_manager"
    duration_in_s: float = 10 # 0 for infinite
    num_vehicles: int = 60
    num_walkers: int = 10
    tm_port: int = 8000

    # sensor config
    ego_vehicle_file: str = Path(__file__).parent.parent / "vehicles" / "eight_car_lidar.json"
    save_actors_bbox_in_lidar: bool = True

    # recording config
    record: bool = True
    record_bboxes: bool = True
    data_dir: str = Path(__file__).parent.parent / "data"
    topics: list = None # optional None -> infer from sensor config

    # logs
    log_dir: str = Path(__file__).parent.parent / "logs"
    ros2bridge_log_name: str = "ros2bridge"
    egovehicle_log_name: str = "egovehicle"
    ros2bag_log_name: str = "ros2bag"
    generate_traffic_log_name: str = "generate_traffic"

    @property
    def generate_traffic_log_file(self) -> Path:
        return Path(self.log_dir / f"{self.generate_traffic_log_name}.log")
    
    @property
    def ros2bridge_log_file(self) -> Path:
        return Path(self.log_dir / f"{self.ros2bridge_log_name}.log")
    
    @property
    def egovehicle_log_file(self) -> Path:
        return Path(self.log_dir / f"{self.egovehicle_log_name}.log")
    
    @property
    def ros2bag_log_file(self) -> Path:
        return Path(self.log_dir / f"{self.ros2bag_log_name}.log")
    
    @property
    def bbox_save_file_path(self) -> Path:
        return Path(self.data_dir / self.experiment_name / "bbox.h5")

def load_experiment_config(path):
    field_names = {f.name for f in fields(ExperimentConfig)}
    path_fields = ["ego_vehicle_file", "data_dir", "log_dir"]

    data = toml.load(path)
    data = {k: v for (k, v) in data.items() if k in field_names}
    for key in path_fields:
        if key in data and isinstance(data[key], str):
            data[key] = Path(data[key])

    return ExperimentConfig(**data)

def save_experiment_config(config, path):
    data = asdict(config)

    for key, value in data.items():
        if isinstance(value, Path):
            data[key] = str(value)

    with open(path, 'w') as f:
        toml.dump(data, f)