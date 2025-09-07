(define (domain domain_name)
  (:requirements :strips :typing :durative-actions)

  (:types
    robot
    location
    marker
  )

  (:predicates
    ;; robot position
    (loc ?r - robot ?l - location)

    ;; map structure
    (connected ?l1 - location ?l2 - location)

    ;; exploration state
    (searched ?l - location ?r - robot)

    ;; markers
    (has_loc ?m - marker ?l - location)
    (found ?m - marker ?r - robot ?l - location)

    ;; finished task
    (mission ?r - robot ?l1 - location ?l2 - location ?l3 - location ?l4 - location ?m1 - marker ?m2 - marker ?m3 - marker ?m4 - marker)
  )

  ;; --------------------
  ;; Actions
  ;; --------------------

  (:durative-action move
    :parameters (?r - robot ?from - location ?to - location)
    :duration (= ?duration 700)

    :condition (and
      (at start (loc ?r ?from))
      (at start (connected ?from ?to))
      (at start (searched ?from ?r)) ;; must search before leaving
    )

    :effect (and
      (at end (loc ?r ?to))
      (at end (not (loc ?r ?from)))
    )
  )

  (:durative-action search
    :parameters (?r - robot ?l - location)
    :duration (= ?duration 700)

    :condition (and
      (at start (loc ?r ?l))
    )

    :effect (and
      (at end (searched ?l ?r))
    )
  )

  (:durative-action find_marker
      :parameters (?r - robot ?l - location ?m - marker)
      :duration (= ?duration 700)

      :condition (and 
          (at start(loc ?r ?l))
          (at start(has_loc ?m ?l))
          (at start(searched ?l ?r))
      )
      :effect (and 
          (at end(found ?m ?r ?l))
      )
  )

  (:durative-action finish_mission
      :parameters (?r - robot ?l1 - location ?l2 - location ?l3 - location ?l4 - location ?m1 - marker ?m2 - marker ?m3 - marker ?m4 - marker)
      :duration (= ?duration 700)

      :condition (and 
          (at start(loc ?r ?l4))
          (at start(found ?m1 ?r ?l1))
          (at start(found ?m2 ?r ?l2))
          (at start(found ?m3 ?r ?l3))
          (at start(found ?m4 ?r ?l4))
      )
      :effect (and 
          (at end(mission ?r ?l1 ?l2 ?l3 ?l4 ?m1 ?m2 ?m3 ?m4))
      )
  )
  
)
