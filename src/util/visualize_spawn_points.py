import carla, os, sys

def visualize_spawn_points(world, life_time=600):
    map = world.get_map()
    for i, waypoint in enumerate(map.get_spawn_points()):
        world.debug.draw_string(waypoint.location, str(i), draw_shadow=False, color=carla.Color(r=255, g=0, b=0), life_time=life_time, persistent_lines=True)

if __name__ == '__main__':
    host = "localhost"
    town = "Town15"

    client = carla.Client(host)
    world = client.get_world()
    visualize_spawn_points(world)