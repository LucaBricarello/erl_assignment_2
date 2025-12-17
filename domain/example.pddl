(define (domain simple)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
    robot
    waypoint
    flag_searched
    flag_visited
)

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

    (visited ?r - robot ?w - waypoint)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:action search
    :parameters (?r - robot ?w - waypoint ?f - flag)
    :precondition (and

        )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action askcharge
    :parameters (?r - robot ?r1 ?r2 - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?r1))
        (at start(charging_point_at ?r2))
       )
    :effect (and
        (at start(not(robot_at ?r ?r1)))
        (at end(robot_at ?r ?r2))
    )
)

(:durative-action charge
    :parameters (?r - robot ?ro - room)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?ro))
        (at start(charging_point_at ?ro))
    )
    :effect (and
         (at end(not(battery_low ?r)))
         (at end(battery_full ?r))
    )
)