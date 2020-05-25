(define (domain GROCERY)
    (:requirements :strips :typing :adl)
    (:types item)
    (:predicates (on ?x - item ?y - item)
                 (inbox ?x - item)
                 (inclutter ?x - item)
                 (beside ?x - item ?y - item)
                 (handempty)
                 (holding ?x - item)
                 (topfree ?x - item)
                 )
                 
    
    (:action pick-up
            :parameters (?x - item)
            :precondition (and (topfree ?x) (handempty))
            :effect
            (and (holding ?x)
                (not (handempty))
                (not (inbox ?x))
                (not (inclutter ?x))
                (not (topfree ?x)))
    )
    
    (:action pick-from
            :parameters (?x - item ?y - item)
            :precondition (and (on ?x ?y) (topfree ?x) (handempty))
            :effect
            (and (holding ?x)
                (not (handempty))
                (not (inbox ?x))
                (not (inclutter ?x))
                (not (topfree ?x))
                (topfree ?y))
    )
    
    (:action put-in-clutter
            :parameters (?x - item)
            :precondition (holding ?x)
            :effect
            (and (not (holding ?x))
                (handempty)
                (inclutter ?x)
                (topfree ?x))
    )
    
    (:action put-in-box
            :parameters (?x - item)
            :precondition  (holding ?x)
            :effect
            (and (not (holding ?x))
                (handempty)
                (inbox ?x)
                (topfree ?x))
    )
    
    (:action put-on 
            :parameters (?x - item ?y - item)
            :precondition (and (holding ?x) (topfree ?y) )
            :effect
            (and (not (holding ?x))
                (on ?x ?y)
                (handempty)
                (topfree ?x)
                (not (topfree ?y)))
    )
)