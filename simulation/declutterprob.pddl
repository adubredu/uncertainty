(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton ambrosia banana cereal lysol milk tangerine oreo - item) 

(:init (handempty) (on oreo pepsi) (inclutter pepsi) (on ambrosia nutella) (inclutter nutella) (on tangerine coke) (inclutter coke) (on nutella lipton) (inclutter lipton) (on milk bleach) (inclutter bleach) (topfree ambrosia) (inclutter ambrosia) (topfree banana) (inclutter banana) (topfree cereal) (inclutter cereal) (topfree lysol) (inclutter lysol) (on lysol milk) (inclutter milk) (on cereal oreo) (inclutter oreo) (on banana tangerine) (inclutter tangerine))

(:goal (and (topfree coke)         (topfree lipton) (topfree nutella)         (topfree pepsi) (topfree bleach) (topfree ambrosia)        (topfree banana) (topfree cereal) (topfree lysol)        (topfree milk) (topfree oreo) (topfree tangerine)        (inclutter nutella) (inclutter pepsi) (inclutter bleach)         (inclutter coke) (inclutter lipton)        (inclutter ambrosia) (inclutter banana) (inclutter milk)        (inclutter cereal) (inclutter oreo)        (inclutter tangerine) (inclutter lysol))))