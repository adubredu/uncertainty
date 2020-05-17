(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (clearleft pepsi) (clearright pepsi) (cleartop pepsi) (onclutterortable pepsi) (clearleft nutella) (clearright nutella) (cleartop nutella) (onclutterortable nutella) (toleft bleach coke) (clearright coke) (cleartop coke) (onclutterortable coke) (clearleft lipton) (toright bleach lipton) (cleartop lipton) (onclutterortable lipton) (toleft lipton bleach) (toright coke bleach) (cleartop bleach) (onclutterortable bleach) (handempty) )

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))
