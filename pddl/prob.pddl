(define (problem PACKED-GROCERY)
(:domain GROCERY)
(:objects bleach nutella coke pepsi lipton - item)

(:init  (cleartop bleach) (cleartop coke) (handempty) 
        (cleartop lipton) (cleartop nutella) (cleartop pepsi)
        (clearleft bleach) (clearleft nutella) (clearleft pepsi)
        (clearleft lipton) (clearleft coke) (clearright bleach)
        (clearright nutella) (clearright pepsi) (clearright coke)
        (clearright lipton))

(:goal (and (on coke bleach) (on lipton coke) (toleft nutella bleach) (toright pepsi bleach))))