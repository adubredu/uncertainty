(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (clearright pepsi) (cleartop pepsi) (toleft bleach nutella) (clearright nutella) (cleartop nutella) (ontable nutella) (onsomething nutella) (toleft lipton coke) (clearright coke) (cleartop coke) (onclutterortable coke) (clearleft lipton) (toright coke lipton) (cleartop lipton) (onclutterortable lipton) (clearleft bleach) (toright nutella bleach) (cleartop bleach) (ontable bleach) (onsomething bleach) (holding pepsi) )

(:goal (and (on pepsi bleach) (on lipton pepsi) (toleft coke bleach) (toright nutella bleach))))
