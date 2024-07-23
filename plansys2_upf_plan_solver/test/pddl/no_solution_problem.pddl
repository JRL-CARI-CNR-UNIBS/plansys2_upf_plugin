(define (problem very_simple_problem)
  (:domain very_simple)
  (:objects
    a b c
  )
  (:init
    (occupied a)
    (free b)
    (free c)
  )
  (:goal
    (and
      (free a)
      (free b)
      (free c)
    )
  )
)

