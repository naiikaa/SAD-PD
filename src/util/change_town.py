import carla
if __name__ == '__main__':
    host = "winhost"
    town = "Town15"

    client = carla.Client(host)
    client.set_timeout(20)
    world = client.load_world(town)