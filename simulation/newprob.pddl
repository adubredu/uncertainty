(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft bleach pepsi) (clearright pepsi) (cleartop pepsi) (ontable pepsi) (onsomething pepsi) (clearleft nutella) (toright bleach nutella) (cleartop nutella) (ontable nutella) (onsomething nutella) (clearleft coke) (clearright coke) (cleartop coke) (onclutterortable coke) (clearleft lipton) (clearright lipton) (cleartop lipton) (toleft nutella bleach) (toright pepsi bleach) (cleartop bleach) (ontable bleach) (onsomething bleach) (holding lipton) )

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))
