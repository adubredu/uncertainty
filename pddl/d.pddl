(define (domain tequila)
  (:requirements :typing :strips :equality)
  (:types
    bottle
    )
  (:functions
    (total-cost ?b - bottle)
    )
  (:predicates
    (bottle-finished ?b - bottle)
    )
(:action drink
  :parameters (?b - bottle)
  :effect (increase (total-cost ?b) -1)
  )
 (:action done-drinking
 :parameters (?b - bottle)
 :precondition (= (total-cost ?b) 0)
 :effect (bottle-finished ?b)
 )
)