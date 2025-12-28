(define (domain assignment2)
    (:requirements 
        :strips 
        :typing 
        :fluents
        :universal-preconditions
        :durative-actions
    )

    (:types
        robot
        waypoint
    )

    (:predicates
        (robot_at ?r - robot ?w - waypoint)
        (marker_not_detected ?w - waypoint)
        (marker_detected ?w - waypoint)
        (markers_analyzed)
    )

    (:functions
        (wp_detected)
    )

    ;; Azione di movimento: dura 5 secondi
    (:durative-action move
        :parameters (?r - robot ?from - waypoint ?to - waypoint)
        :duration (= ?duration 5)
        :condition (and
            (at start (robot_at ?r ?from))
        )
        :effect (and
            (at start (not (robot_at ?r ?from)))
            (at end (robot_at ?r ?to))
        )
    )

    ;; Azione di rotazione: dura 3 secondi
    ;; Usiamo (over all ...) per costringere il robot a stare fermo al waypoint
    ;; durante tutta la rotazione
    (:durative-action rotate_and_detect
        :parameters (?r - robot ?w - waypoint)
        :duration (= ?duration 3)
        :condition (and
            (over all (robot_at ?r ?w))
            (at start  (marker_not_detected ?w))
        )
        :effect (and
            (at start (not (marker_not_detected ?w)))
            (at start  (marker_detected ?w))
            (at end (increase (wp_detected) 1))
        )
    )

    ;; Azione di analisi finale: dura 2 secondi
    (:durative-action analyze_marker
        :parameters (?r - robot ?w - waypoint)
        :duration (= ?duration 2)
        :condition (and
            (at start (> (wp_detected) 3.5))
            ;(at start (forall (?w - waypoint) (marker_detected ?w)))
        )
        :effect (and
            (at end (markers_analyzed))
        )
    )
)