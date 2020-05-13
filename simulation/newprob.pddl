(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (clearright pepsi) (cleartop pepsi) (clearleft nutella) (toright bleach nutella) (cleartop nutella) (clearleft coke) (clearright coke) (on lipton coke) (clearleft lipton) (clearright lipton) (cleartop lipton) (toleft nutella bleach) (clearright bleach) (on coke bleach) (ontable bleach) (holding pepsi) )

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))
