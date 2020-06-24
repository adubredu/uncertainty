(define (problem PACKED-GROCERY) 
(:domain GROCERY) 
<<<<<<< HEAD
 (:objects h0 h1 h2 h3 h4 m5 m1 m2 m3 m4 - item)
(:init (handempty) (inbox h0) (topfree h0) (inbox m5) (topfree m5) (inbox m1) (topfree m1) (inbox h1) (topfree h1) (inbox h2) (topfree h2) (inbox m2) (topfree m2) (topfree h3) (inclutter h3) (topfree m3) (inclutter m3) (topfree h4) (inclutter h4) (topfree m4) (inclutter m4) (topfree m5) (inclutter m5) )
(:goal (and (inbox h0) (inbox h1) (inbox h2) (inbox h3) (inbox h4) (inbox m5) (inbox m1) (inbox m2) (inbox m3) (or (on m4 h0) (on m4 h1) (on m4 h2) (on m4 h3) (on m4 h4) (on m4 m5) (on m4 m1) (on m4 m2) (on m4 m3) ) (or (on m5 h0) (on m5 h1) (on m5 h2) (on m5 h3) (on m5 h4) (on m5 m5) (on m5 m1) (on m5 m2) (on m5 m3) ) )))
=======
 (:objects h0 h1 h2 h3 m0 m1 m2 m3 m4 m5 - item)
(:init (handempty) (inbox m0) (topfree m0) (inbox m1) (topfree m1) (inbox m2) (topfree m2) (inbox h0) (topfree h0) (inbox h1) (topfree h1) (inbox m3) (topfree m3) (inbox h2) (topfree h2) (inbox m4) (topfree m4) (inbox h3) (topfree h3) (topfree m5) (inclutter m5) (boxfull))
(:goal (and (inbox h0) (inbox h1) (inbox h2) (inbox h3) (inbox m0) (inbox m1) (inbox m2) (inbox m3) (inbox m4) (or (on m5 h0) (on m5 h1) (on m5 h2) (on m5 h3) (on m5 m0) (on m5 m1) (on m5 m2) (on m5 m3) (on m5 m4) ) )))
>>>>>>> 7a083f14c92c0a070ae27a81c752239e2791490c
