(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (clearright pepsi) (on lipton pepsi) (onsomething pepsi) (toleft bleach nutella) (clearright nutella) (cleartop nutella) (ontable nutella) (onsomething nutella) (clearleft coke) (toright bleach coke) (cleartop coke) (ontable coke) (onsomething coke) (clearleft lipton) (clearright lipton) (cleartop lipton) (onsomething lipton) (toleft coke bleach) (toright nutella bleach) (on pepsi bleach) (ontable bleach) (onsomething bleach) (handempty) )

(:goal (and (on pepsi bleach) (on lipton pepsi) (toleft coke bleach) (toright nutella bleach))))
