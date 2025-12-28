(define (problem assignment2_task)
    (:domain assignment2)

    (:objects
        my_robot - robot
        wp_start wp1 wp2 wp3 wp4 - waypoint
    )

    (:init
        (robot_at my_robot wp_start)
        (= (wp_detected) 0)
        (marker_not_detected wp1) 
        (marker_not_detected wp2) 
        (marker_not_detected wp3) 
        (marker_not_detected wp4)
        (marker_detected wp_start)
    )

    (:goal
        (and 
            (markers_analyzed)
        )
    )

    (:metric minimize (total-time))
)