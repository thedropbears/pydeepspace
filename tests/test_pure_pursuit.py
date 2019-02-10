import utilities.pure_pursuit as pp


def test_trapezoidal():
    waypoints = [pp.Waypoint(0, 0, 0, 0), pp.Waypoint(10, 10, 0, 2)]
    trap = pp.insert_trapezoidal_waypoints(waypoints, 2, -2)
    assert len(trap) == 3
    assert trap[1][2] == waypoints[1][2], (
        "Intermediate waypoint should have end speed when accelerating: %s" % trap
    )

    waypoints = [pp.Waypoint(10, 10, 0, 2), pp.Waypoint(0, 0, 0, 0)]
    trap = pp.insert_trapezoidal_waypoints(waypoints, 2, -2)
    assert len(trap) == 3
    assert trap[1][2] == waypoints[0][2], (
        "Intermediate waypoint should have beginning speed when decelerating: %s" % trap
    )
