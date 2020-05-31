(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton ambrosia banana cereal lysol milk tangerine oreo - item) 

(:init (handempty) (topfree pepsi) (inclutter pepsi) (on lipton nutella) (inclutter nutella) (topfree coke) (inclutter coke) (topfree lipton) (inclutter lipton) (on oreo bleach) (inclutter bleach) (on cereal ambrosia) (inclutter ambrosia) (on nutella banana) (inclutter banana) (on coke cereal) (inclutter cereal) (on pepsi lysol) (inclutter lysol) (on bleach milk) (inclutter milk) (topfree oreo) (inclutter oreo) (on lysol tangerine) (inclutter tangerine))

(:goal (and (topfree coke)         (topfree lipton) (topfree nutella)         (topfree pepsi) (topfree bleach) (topfree ambrosia)        (topfree banana) (topfree cereal) (topfree lysol)        (topfree milk) (topfree oreo) (topfree tangerine)        (inclutter nutella) (inclutter pepsi) (inclutter bleach)         (inclutter coke) (inclutter lipton)        (inclutter ambrosia) (inclutter banana) (inclutter milk)        (inclutter cereal) (inclutter oreo)        (inclutter tangerine) (inclutter lysol))))