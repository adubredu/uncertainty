(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft bleach pepsi) (clearright pepsi) (cleartop pepsi) (onclutterortable pepsi) (clearleft nutella) (clearright nutella) (cleartop nutella) (onclutterortable nutella) (clearleft coke) (toright lipton coke) (cleartop coke) (onclutterortable coke) (toleft coke lipton) (toright bleach lipton) (cleartop lipton) (onclutterortable lipton) (toleft lipton bleach) (toright pepsi bleach) (cleartop bleach) (onclutterortable bleach) (handempty) )

(:goal (and (on pepsi bleach) (on lipton pepsi) (toleft coke bleach) (toright nutella bleach))))
