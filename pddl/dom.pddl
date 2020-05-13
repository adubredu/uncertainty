(define (domain GROCERY)
	(:requirements :strips :typing)
	(:types item)
	(:predicates (on ?x - item ?y - item)
		(ontable ?x - item)
		(cleartop ?x - item)
		(handempty)
		(holding ?x - item)
		(clearleft ?x - item)
		(clearright ?x - item)
		(toleft ?x - item ?y - item)
		(toright ?x - item ?y - item)
		)
		
	(:action pick-up
	        :parameters (?x - item)
	        :precondition (and (cleartop ?x) (handempty))
	        :effect
	        (and (not (ontable ?x))
	            (not (handempty))
	            (not (cleartop ?x))
	            (holding ?x)))
	            
	(:action put-on-table
	    :parameters (?x - item)
	    :precondition (holding ?x)
	    :effect
	    (and (not (holding ?x))
	    (cleartop ?x)
	    (handempty)
	    (ontable ?x)))
	    
	
	(:action put-on 
	        :parameters (?x - item ?y - item)
	        :precondition (and (holding ?x) (cleartop ?y) )
	        :effect
	        (and (not (holding ?x))
	        (not (cleartop ?y))
	        (cleartop ?x)
	        (handempty)
	        (on ?x ?y)))
	        
	        
	(:action put-left
	        :parameters (?x - item ?y - item)
	        :precondition (and (holding ?x) (clearleft ?y) (ontable ?y))
	        :effect
	        (and (not (holding ?x))
	        (not (clearleft ?y))
	        (clearleft ?x)
	        (handempty)
	        (toleft ?x ?y)))
	
	(:action put-right
	        :parameters (?x - item ?y - item)
	        :precondition (and (holding ?x) (clearright ?y) (ontable ?y))
	        :effect
	        (and (not (holding ?x))
	        (not (clearright ?y))
	        (clearright ?x)
	        (handempty)
	        (toright ?x ?y)))
	        
)
	        
	       
