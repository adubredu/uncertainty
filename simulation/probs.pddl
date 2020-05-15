(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft bleach pepsi) (clearright pepsi) (cleartop pepsi) (clearleft nutella) (clearright nutella) (cleartop nutella) (clearleft coke) (clearright coke) (cleartop coke) (clearleft lipton) (toright bleach lipton) (cleartop lipton) (toleft lipton bleach) (toright pepsi bleach) (cleartop bleach) (handempty) )

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))
