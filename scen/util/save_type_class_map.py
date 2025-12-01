import carla, json, time
import numpy as np


if __name__ == '__main__':
    client = carla.Client("winhost")

    client.set_timeout(1000.0)


    world = client.load_world("Town15")
    bp_library = world.get_blueprint_library()

    semantic_tag_map = {}

    landmark_list = world.get_map().get_all_landmarks()  # or relevant method
    signs = []
    for lm in landmark_list:
        sign = world.get_traffic_sign(lm)
        if sign:
            semantic_tag_map[sign.type_id] = int(sign.semantic_tags[-1])
    

    spawn_point = world.get_map().get_spawn_points()[10]
    for bp in bp_library.filter("vehicle.*"):
        actor = world.spawn_actor(bp, spawn_point)
        time.sleep(.1)
        print(actor)
        if actor:
            semantic_tag_map[bp.id] = actor.semantic_tags[0]
            actor.destroy()
    spawn_point = carla.Transform()
    spawn_point.location = world.get_random_location_from_navigation()
    for bp in bp_library.filter("walker.*"):
        actor = world.spawn_actor(bp, spawn_point)
        time.sleep(.1)
        print(actor)
        if actor:
            semantic_tag_map[bp.id] = actor.semantic_tags[0]
            actor.destroy()

    with open("semantic_tag_map.json", "w") as f:
        json.dump(semantic_tag_map, f)