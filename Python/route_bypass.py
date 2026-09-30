import carla

COLLISION_CENTER = carla.Location(x=347.0, y=-911.75, z=0.0)

def apply_collision_bypass(planner, route):
    print("\nROUTE BYPASS: KAPALI - V5.4 kinematic-final kullaniliyor.")
    return route

def draw_collision_bypass_debug(world, route, life_time=2.0):
    world.debug.draw_point(
        carla.Location(x=COLLISION_CENTER.x, y=COLLISION_CENTER.y, z=1.0),
        size=0.08,
        color=carla.Color(255, 0, 0),
        life_time=life_time,
    )
