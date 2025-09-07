(define (problem problem_name) (:domain domain_name)
    (:objects 
        rob - robot
        start wp1 wp2 wp3 wp4 - location
        fakem m1 m2 m3 m4 - marker
    )

    (:init
        (loc rob start)
        (searched start rob)
        (found fakem rob start)
        (has_loc fakem start)

        (connected start wp1)
        (connected wp1 start)
        (connected wp1 wp2)
        (connected wp2 wp1)
        (connected wp2 wp3)
        (connected wp3 wp2)
        (connected wp3 wp4)
        (connected wp4 wp3)

        (has_loc m1 wp1)
        (has_loc m2 wp2)
        (has_loc m3 wp3)
        (has_loc m4 wp4)
    )

    (:goal (and
        (mission rob wp1 wp2 wp3 wp4 m1 m2 m3 m4)
    ))

)
