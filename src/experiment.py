import os, atexit, time, carla, signal, subprocess, shutil, numpy as np, h5py, json
from pathlib import Path
from subprocess import Popen, PIPE, CalledProcessError
from config import ExperimentConfig, save_experiment_config, load_experiment_config
from util.coords import coords_to_ego
from datetime import datetime
semantic_lidar_tags = {
  0 : "Unlabeled",
  1 : "Roads",
  2 : "SideWalks",
  3 : "Buildings",
  4 : "Wall",
  5 : "Fence",
  6 : "Pole",
  7 : "TrafficLight",
  8 : "TrafficSign",
  9 : "Vegetation",
  10 : "Terrain",
  11 : "Sky",
  12 : "Pedestrian",
  13 : "Rider",
  14 : "Car",
  15 : "Truck",
  16 : "Bus",
  17 : "Train",
  18 : "Motorcycle",
  19 : "Bicycle",
  20 : "Static",
  21 : "Dynamic",
  22 : "Other",
  23 : "Water",
  24 : "RoadLine",
  25 : "Ground",
  26 : "Bridge",
  27 : "RailTrack",
  28 : "GuardRail", 
}

SEM_LIDAR_DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('cos_angle', np.float32),
    ('object_idx', np.uint32),
    ('object_tag', np.uint32)
])

BBOX_DTYPE = np.dtype([
    ("actor_id", np.uint32),
    ('x', np.float32),('y', np.float32),('z', np.float32),
    ('x_extent', np.float32),('y_extent', np.float32),('z_extent', np.float32),
    ('roll', np.float32),('pitch', np.float32),('yaw', np.float32),
])

def lidar_data_to_numpy(lidar_data:carla.SemanticLidarMeasurement):
    return np.frombuffer(lidar_data.raw_data, dtype=SEM_LIDAR_DTYPE)

class ExperimentRunner:
    def __init__(self, config: ExperimentConfig):
        self.config = config
        os.makedirs(self.config.log_dir, exist_ok=True)
        self.processes = {}
        self.client = carla.Client(self.config.host, self.config.port)
        self.client.set_timeout(100)
        self.world = self.client.load_world(self.config.town)
        self.world = self.client.get_world()
        self.spawn_point_coords = self.world.get_map().get_spawn_points()[self.config.spawn_point]
        self.timestep = 1 / self.config.fps
        self.world_ticks = int(self.config.duration_in_s / self.timestep)
        settings = self.world.get_settings()
        if self.config.bridge_passive_mode:
            settings.synchronous_mode = True
        settings.fixed_delta_seconds = self.timestep

        settings.no_rendering_mode = False
 
        self.world.apply_settings(settings)
        atexit.register(self.cleanup)

        if self.config.record_bboxes:
            os.makedirs(self.config.bbox_save_file_path.parent, exist_ok=True)
            self.bbox_save_file = h5py.File(self.config.bbox_save_file_path, 'w')
            atexit.register(self.bbox_save_file.close)
            self.bbox_tick = 0
            self.save_static_bboxes()

    def save_static_bboxes(self):
        print("Saving static bboxes...")
        with h5py.File(self.config.bbox_save_file_path.with_stem("bbox_static"), 'w') as f:
            env_objs = self.world.get_environment_objects()

            obj_bboxes = np.array([[
                obj.id, obj.type,
                obj.transform.location.x, obj.transform.location.y, obj.transform.location.z,
                obj.bounding_box.extent.x, obj.bounding_box.extent.y, obj.bounding_box.extent.z,
                obj.bounding_box.rotation.roll, obj.bounding_box.rotation.pitch, obj.bounding_box.rotation.yaw
            ] for obj in env_objs])

            f.create_dataset("static_bounding_boxes", data=obj_bboxes, compression="gzip")

    def _debug_mark_sensors_in_carla(self):
        sensors = self.world.get_actors().filter('sensor.*')
        ego_vehicle_transform = self.ego_vehicle.get_transform()
        for t in sensors:
            self.world.debug.draw_line(
                ego_vehicle_transform.location,
                t.get_transform().location,
                thickness=0.05,
                color=carla.Color(r=255, g=0, b=0),
                life_time=0
            )

    def _debug_generate_traffic(self):
        self.setup_generate_traffic()
        input()

    def get_topic_list(self):
        topic_list = subprocess.check_output(["ros2", "topic", "list"])
        return topic_list.decode('utf-8').split("\n")
    
    def get_ego_vehicle_actor(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == 'ego_vehicle':
                return actor

    def setup_bridge(self):
        print("Starting ros2 bridge... (see logfile)")
        ros2bridge_logfile = open(self.config.ros2bridge_log_file, "w")
        atexit.register(ros2bridge_logfile.close)
        #ego_vehicle_role_name instead of just ego_vehicle
        if self.config.bridge_passive_mode:
            passive_mode = "passive:=True"
        else:
            passive_mode = "passive:=False"

        ros2bridge_cmd = f"ros2 launch carla_ros_bridge carla_ros_bridge.launch.py {passive_mode} town:={self.config.town} fixed_delta_seconds:={self.timestep} host:={self.config.host} ego_vehicle_role_name:={self.config.ego_vehicle_role_name}".split(" ")
        self.processes["ros2bridge_process"] = Popen(ros2bridge_cmd, stdout=ros2bridge_logfile, bufsize=1, universal_newlines=True)

    def setup_generate_traffic(self):
        print("Generating Traffic...")
        generate_traffic_logfile = open(self.config.generate_traffic_log_file, "w")
        atexit.register(generate_traffic_logfile.close)
        generate_traffic_cmd = f"python3 {self.config.generate_traffic_path} -n {self.config.num_vehicles} -w {self.config.num_walkers} --host {self.config.host} --tm-port {self.config.tm_port}".split(" ")
        self.processes["generate_traffic"] = Popen(generate_traffic_cmd, stdout=generate_traffic_logfile, bufsize=1, universal_newlines=True)
        time.sleep(5)
        if self.config.bridge_passive_mode:
            self.world.tick()

    def setup_ego_vehicle(self, move_spectator=True):
        print("Spawning ego vehicle... (see logfile)")
        egovehicle_logfile = open(self.config.egovehicle_log_file, "w")
        atexit.register(egovehicle_logfile.close)

        egovehicle_cmd = f"ros2 launch carla_spawn_objects carla_spawn_objects.launch.py objects_definition_file:={self.config.ego_vehicle_file}".split(" ")
        self.processes["egovehicle_process"] = Popen(egovehicle_cmd, stdout=egovehicle_logfile, bufsize=1, universal_newlines=True)
        time.sleep(5)
        if self.config.bridge_passive_mode:
            self.world.tick()
        time.sleep(15)
        if self.config.bridge_passive_mode:
            self.world.tick()
        self.ego_vehicle = self.get_ego_vehicle_actor()
        time.sleep(15)

        
        self.ego_vehicle.set_transform(self.spawn_point_coords)
        if self.config.bridge_passive_mode:
            self.world.tick() 
        time.sleep(15)

        if move_spectator:
            print(f"moving spectator to {self.ego_vehicle}")
            self.world.get_spectator().set_transform(self.ego_vehicle.get_transform())

        if self.config.save_actors_bbox_in_lidar:
            self.lidar_sensors = self.world.get_actors().filter("sensor.lidar.ray_cast_semantic")
            self.lidar_frames = [None] * len(self.lidar_sensors)
            self.lidar_ready = [False] * len(self.lidar_sensors) 

            def make_callback(lidar_index):
                def callback(lidar_data):
                    self.lidar_frames[lidar_index] = lidar_data
                    self.lidar_ready[lidar_index] = True
                return callback

            for i, sensor in enumerate(self.lidar_sensors):
                sensor.listen(make_callback(i))

    def start_recording(self):
        print("Start Recording... (see logfile)")
        ros2bag_logfile = open(self.config.ros2bag_log_file, "w")
        atexit.register(ros2bag_logfile.close)
        # exclude vehicle info, status, control etc.
        topic_list = [topic for topic in self.get_topic_list() if 'ego_vehicle' in topic and 'ego_vehicle/vehicle' not in topic]
        ros2bag_cmd = f"ros2 bag record -o {self.config.experiment_name}/db".split(" ") + topic_list
        os.makedirs(self.config.data_dir, exist_ok=True)
        self.processes["ros2bag_process"] = Popen(ros2bag_cmd, cwd=self.config.data_dir, stdout=ros2bag_logfile, bufsize=1, universal_newlines=True)

    def cleanup(self):
        for process in self.processes.values():
            if not process:
                continue
            process.send_signal(signal.SIGINT)
            process.wait()
        
        # copy remaining sensor metadata (lidars/radars)
        if self.config.record:
            src = Path(self.config.ego_vehicle_file)
            dst = Path(self.config.data_dir) / self.config.experiment_name / src.name
            shutil.copy(src, dst)
            config_dest = (Path(self.config.data_dir) / self.config.experiment_name / self.config.experiment_name).with_suffix('.toml')
            save_experiment_config(self.config, config_dest)
        if self.config.bridge_passive_mode:
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = 0.1
            self.world.apply_settings(settings)
        self.traffic_manager.set_synchronous_mode(False)

    def save_actors_bbox_in_lidar(self):
        self.lidar_ready[:] = [False] * len(self.lidar_sensors)
        lidar_numpy = [lidar_data_to_numpy(lf) for lf in self.lidar_frames]

        # Extract object IDs for each lidar
        object_ids_per_lidar = []
        object_tags_per_lidar = []
        for arr in lidar_numpy:
            obj_ids = np.unique(arr['object_idx'])
            obj_tags = np.unique(arr['object_tag'])
            obj_ids = obj_ids[obj_ids != 0]   # remove "no object"
            obj_tags = obj_tags[obj_tags != 0]   # remove "no object"
            object_ids_per_lidar.append(obj_ids)
            object_tags_per_lidar.append(obj_tags)

        # Combine all visible actors across all sensors
        if len(object_ids_per_lidar) > 0:
            visible_actors_ids = np.unique(np.concatenate(object_ids_per_lidar))
            tags = np.unique(np.concatenate(object_tags_per_lidar))

            # Extract actor objects from actor IDs
            actor_list = self.world.get_actors()
            visible_actors = []
            for actor_id in visible_actors_ids:
                actor = actor_list.find(int(actor_id))
                if actor:
                    visible_actors.append(actor)
            
        else:
            visible_actors_ids = np.array([])
            visible_actors = np.array([])
            bounding_boxes = np.array([])
            tags = np.array([])

        if self.config.record_bboxes:
            bounding_boxes = []
            ebbox = self.ego_vehicle.bounding_box
            etf = self.ego_vehicle.get_transform()

            ex,ey,ez = etf.location.x, etf.location.y, etf.location.z
            eroll, epitch, eyaw = etf.rotation.roll, etf.rotation.pitch, etf.rotation.yaw

            for actor in visible_actors:
                
                abbox = actor.bounding_box
                atf = actor.get_transform()
                
                ax,ay,az = atf.location.x, atf.location.y, atf.location.z
                aroll,apitch,ayaw = atf.rotation.roll, atf.rotation.pitch, atf.rotation.yaw
                

                
                bounding_boxes.append(np.array([
                    actor.id,
                    ax, ay, az,
                    abbox.extent.x, abbox.extent.y, abbox.extent.z,
                    aroll, apitch, ayaw
                ]))

            

            ego_vehicle_bbox_data = np.array([
                ex, ey, ez,
                ebbox.extent.x, ebbox.extent.y, ebbox.extent.z,
                eroll, epitch, eyaw
            ])

            grp = self.bbox_save_file.create_group(f"frame_{self.bbox_tick:06d}")
            grp.create_dataset("ego", data=ego_vehicle_bbox_data)
            grp.create_dataset("actors", data=np.array(bounding_boxes))
            self.bbox_tick += 1

            visible_actors_ids = np.array([])
            visible_actors = np.array([])
            bounding_boxes = np.array([])

    def save_actor_id_and_type(self):
        actor_id_type_map = {a.id: a.type_id for a in self.world.get_actors()}
        self.bbox_save_file["actor_id_type_map"] = [json.dumps(actor_id_type_map)]

    def run_once(self):
        

        self.setup_bridge()
        time.sleep(15) #we could also be elaborate and somehow check the logs for success... but for now this should be enough
        if self.config.bridge_passive_mode:
            self.world.tick() #necessary for passive ros bridge

        self.setup_ego_vehicle()
        time.sleep(15)
        if self.config.bridge_passive_mode:
            self.world.tick()

        if self.config.record:
            self.start_recording()
            time.sleep(5)
            if self.config.bridge_passive_mode:
                self.world.tick()

        print("Start Scenario...")
        self.traffic_manager = self.client.get_trafficmanager(self.config.tm_port)
        time.sleep(5)
        print("Traffic manager: ", self.traffic_manager)
        self.traffic_manager.set_synchronous_mode(True)
        self.ego_vehicle.set_autopilot(True, self.traffic_manager.get_port())
        time.sleep(5)
        self.setup_generate_traffic()
        time.sleep(5)
        if self.config.bridge_passive_mode:
            self.world.tick()
        if self.config.record_bboxes:
            self.save_actor_id_and_type()
        print(f"world ticks: {self.world_ticks}")
        tick = 0
        while True:
            if self.config.bridge_passive_mode:
                self.world.tick()
                time.sleep(self.timestep) # need some sleep time, otherwise some sensors might not be able to keep up publishing to ROS or the recording cant keep up
            else:
                self.world.wait_for_tick()

            while not all(self.lidar_ready):
                print("Not all lidars were ready")
                time.sleep(self.timestep)

            if self.config.save_actors_bbox_in_lidar and  all(self.lidar_ready):
                self.save_actors_bbox_in_lidar()

            if self.world_ticks:
                msg = f"step {tick+1}/{self.world_ticks}..." + '\r' * (tick+1 != self.world_ticks)
                end = "" + "\n" * (tick+1 == self.world_ticks)
                #print(msg, end=end)
                tick += 1
                if tick == self.world_ticks:
                    break

        print("Experiment Finished. Stopping processes...")
        self.cleanup()


if __name__ == '__main__':
    #build name from current date+time in format YYYYMMDD_HHMMSS
    spawn_point=3
    vehicles = 250
    walker = 150
    
    print(f"Running experiment with {vehicles} vehicles, {walker} walkers, spawn point {spawn_point}")
    postfix = f"{vehicles}v_{walker}w_{spawn_point}sp"

    name = datetime.now().strftime("%Y%m%d_%H%M") + "_" + postfix
    
    test_config = ExperimentConfig(name, 
        bridge_passive_mode=True,
        record=True,
        record_bboxes=True,
        duration_in_s=10,
        num_vehicles=vehicles,
        num_walkers=walker,
        town="Town15",
        spawn_point=spawn_point,
        host="localhost", # set to "winhost" when using windows carla server
    )
    runner = ExperimentRunner(test_config)
    runner.run_once()