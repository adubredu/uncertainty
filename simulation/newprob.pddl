(define (problem PACKED-GROCERY) 
(:domain GROCERY) 
 (:objects m0 m1 - item)
(:init (handempty) (topfree m0) (inclutter m0) (topfree m1) (inclutter m1) )
(:goal (and (inbox m0) (inbox m1) )))