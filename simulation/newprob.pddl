(define (problem PACKED-GROCERY) 
 (:domain GROCERY)                             
 (:objects bleach nutella coke pepsi lipton - item) 

(:init  (toleft lipton bleach) (clearright bleach) (cleartop bleach) (onclutterortable bleach) (clearleft nutella) (toright lipton nutella) (cleartop nutella) (onclutterortable nutella) (clearleft pepsi) (clearright pepsi) (on coke pepsi) (onsomething pepsi) (toleft nutella lipton) (toright bleach lipton) (on pepsi lipton) (onclutterortable lipton) (clearleft coke) (clearright coke) (cleartop coke) (onsomething coke) (handempty) )

(:goal (and (on pepsi bleach) (on lipton pepsi) (toleft coke bleach) (toright nutella bleach))))
