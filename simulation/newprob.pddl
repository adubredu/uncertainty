(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (toright nutella pepsi) (cleartop pepsi) (onsomething pepsi) (toleft coke nutella) (clearright nutella) (cleartop nutella) (onsomething nutella) (toleft bleach coke) (clearright coke) (on nutella coke) (onclutterortable coke) (clearleft lipton) (clearright lipton) (cleartop lipton) (onclutterortable lipton) (clearleft bleach) (toright coke bleach) (on pepsi bleach) (onclutterortable bleach) (handempty) )

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))
