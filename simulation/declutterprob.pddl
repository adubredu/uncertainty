(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft bleach pepsi) (clearright pepsi) (cleartop pepsi) (onclutterortable pepsi) (clearleft nutella) (clearright nutella) (cleartop nutella) (onsomething nutella) (clearleft coke) (toright lipton coke) (cleartop coke) (onclutterortable coke) (toleft coke lipton) (toright bleach lipton) (on nutella lipton) (onclutterortable lipton) (toleft lipton bleach) (toright pepsi bleach) (cleartop bleach) (onclutterortable bleach) (handempty) )

(:goal (and (cleartop coke)         (cleartop lipton) (cleartop nutella)         (cleartop pepsi) (cleartop bleach))))