(define (problem PACKED-GROCERY) 
(:domain GROCERY) 
 (:objects h0 h1 h2 h3 m0 m1 m2 m3 m4 - item)
(:init (handempty) (inbox m0) (topfree m0) (inbox m1) (topfree m1) (inbox m2) (topfree m2) (inbox h0) (topfree h0) (inbox h1) (topfree h1) (inbox m3) (topfree m3) (inbox h2) (topfree h2) (inbox m4) (topfree m4) (inbox h3) (topfree h3) (boxfull))
(:goal (and (inbox h0) (inbox h1) (inbox h2) (inbox h3) (inbox m0) (inbox m1) (inbox m2) (inbox m3) (inbox m4) )))