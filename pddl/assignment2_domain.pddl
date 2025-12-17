(define (domain assignment2)
(:requirements :strips :typing :adl :durative-actions)

(:types
    robot
    waypoint
)

(:predicates
    (robot_at ?r - robot ?w - waypoint)
    (visited ?w - waypoint)
    (marker_detected ?w - waypoint)
    (marker_analyzed ?w - waypoint)
)

(:durative-action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_at ?r ?from))
    )
    :effect (and
        (at start (not (robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
        (at end (visited ?to))
    )
)

(:durative-action rotate_and_detect
    :parameters (?r - robot ?w - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?w))
    )
    :effect (and
        (at end (marker_detected ?w))
    )
)

(:durative-action analyze_marker
    :parameters (?r - robot ?w - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start (robot_at ?r ?w))
        (at start (marker_detected ?w))
    )
    :effect (and
        (at end (marker_analyzed ?w))
    )
)
)