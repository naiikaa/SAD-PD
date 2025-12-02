import carla

def print_blueprint_sensors(host="winhost"):
    client = carla.Client(host, 2000)
    world = client.get_world()
    for sensor in world.get_blueprint_library().filter("sensor.*"):
        print(sensor)

if __name__ == '__main__':
    print_blueprint_sensors()