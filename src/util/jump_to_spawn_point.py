import carla, sys

def jump_to_spawn_point(client, continuous=True):
    spawn_point_idx = ""
    world = client.get_world()
    map = world.get_map()
    spawn_points = map.get_spawn_points()
    spectator = world.get_spectator()

    while spawn_point_idx != "exit":
        spawn_point_idx = input("Enter a spawn point index (exit to finish): ")
        spectator.set_transform(spawn_points[int(spawn_point_idx)])

if __name__ == '__main__':
    host = "winhost"

    client = carla.Client(host)
    jump_to_spawn_point(client)