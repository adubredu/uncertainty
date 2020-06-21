(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton ambrosia banana cereal lysol milk tangerine oreo - item) 

(:init (handempty) (on cereal pepsi) (inclutter pepsi) (on coke nutella) (inclutter nutella) (on tangerine coke) (inclutter coke) (on nutella lipton) (inclutter lipton) (on pepsi bleach) (inclutter bleach) (on lipton ambrosia) (inclutter ambrosia) (on oreo banana) (inclutter banana) (on banana cereal) (inclutter cereal) (on ambrosia lysol) (inclutter lysol) (on bleach milk) (inclutter milk) (topfree oreo) (inclutter oreo) (topfree tangerine) (inclutter tangerine))

(:goal (and (topfree coke)         (topfree lipton) (topfree nutella)         (topfree pepsi) (topfree bleach) (topfree ambrosia)        (topfree banana) (topfree cereal) (topfree lysol)        (topfree milk) (topfree oreo) (topfree tangerine)        (inclutter nutella) (inclutter pepsi) (inclutter bleach)         (inclutter coke) (inclutter lipton)        (inclutter ambrosia) (inclutter banana) (inclutter milk)        (inclutter cereal) (inclutter oreo)        (inclutter tangerine) (inclutter lysol))))