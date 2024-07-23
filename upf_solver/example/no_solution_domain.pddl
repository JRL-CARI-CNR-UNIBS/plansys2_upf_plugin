(define (domain very_simple)
  (:requirements :strips :durative-actions)
  (:predicates
    (free ?x)
    (occupied ?x)
  )

  (:durative-action occupy
    :parameters (?x)
    :duration (= ?duration 10)
    :condition (and (at start (free ?x)))
    :effect (and (at start (not (free ?x)))
                 (at end (occupied ?x)))
  )
)

