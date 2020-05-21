(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (toright nutella pepsi) (cleartop pepsi) (onsomething pepsi) (toleft coke nutella) (clearright nutella) (cleartop nutella) (onsomething nutella) (toleft bleach coke) (clearright coke) (on nutella coke) (onclutterortable coke) (clearleft lipton) (toright bleach lipton) (cleartop lipton) (onclutterortable lipton) (toleft lipton bleach) (toright coke bleach) (on pepsi bleach) (onclutterortable bleach) (handempty) )

(:goal (and (cleartop coke)         (cleartop lipton) (cleartop nutella)         (cleartop pepsi) (cleartop bleach))))