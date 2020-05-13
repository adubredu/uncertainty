(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (clearright pepsi) (cleartop pepsi) (clearleft nutella) (clearright nutella) (cleartop nutella) (clearleft coke) (clearright coke) (cleartop coke) (clearleft lipton) (clearright lipton) (cleartop lipton) (clearleft bleach) (clearright bleach) (cleartop bleach) (ontable bleach) (onsomething bleach) (holding pepsi) )

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))
