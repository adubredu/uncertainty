(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft lipton pepsi) (clearright pepsi) (cleartop pepsi) (onclutterortable pepsi) (clearleft nutella) (toright lipton nutella) (cleartop nutella) (onclutterortable nutella) (clearleft coke) (clearright coke) (on bleach coke) (onsomething coke) (toleft nutella lipton) (toright pepsi lipton) (on coke lipton) (onclutterortable lipton) (clearleft bleach) (clearright bleach) (cleartop bleach) (onsomething bleach) (handempty) )

(:goal (and (cleartop coke)         (cleartop lipton) (cleartop nutella)         (cleartop pepsi) (cleartop bleach))))