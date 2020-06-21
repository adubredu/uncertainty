(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft lipton pepsi) (clearright pepsi) (cleartop pepsi) (onclutterortable pepsi) (clearleft nutella) (toright lipton nutella) (cleartop nutella) (onclutterortable nutella) (clearleft coke) (clearright coke) (cleartop coke) (onclutterortable coke) (toleft nutella lipton) (toright pepsi lipton) (cleartop lipton) (onclutterortable lipton) (clearleft bleach) (clearright bleach) (cleartop bleach) (onclutterortable bleach) (handempty) )

(:goal (and (on pepsi bleach) (on lipton pepsi) (toleft coke bleach) (toright nutella bleach))))
