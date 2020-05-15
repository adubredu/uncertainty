(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft bleach pepsi) (clearright pepsi) (on nutella pepsi) (toleft coke nutella) (clearright nutella) (cleartop nutella) (clearleft coke) (toright nutella coke) (cleartop coke) (clearleft lipton) (toright bleach lipton) (cleartop lipton) (toleft lipton bleach) (toright pepsi bleach) (on coke bleach) (handempty) )

(:goal (and (cleartop coke)         (cleartop lipton) (cleartop nutella)         (cleartop pepsi) (cleartop bleach))))