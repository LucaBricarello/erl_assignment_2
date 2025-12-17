(define (problem assignment2_task)
(:domain assignment2)

(:objects
    r2d2 - robot
    wp_start wp1 wp2 wp3 wp4 - waypoint
)

(:init
    (robot_at r2d2 wp_start)
)

(:goal
    (and (marker_analyzed wp1) (marker_analyzed wp2) (marker_analyzed wp3) (marker_analyzed wp4))
)
)