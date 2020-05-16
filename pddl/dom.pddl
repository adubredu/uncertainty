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
		(onsomething ?x - item)
		(onclutterortable ?x - item)
		)
		
	(:action pick-up
	        :parameters (?x - item)
	        :precondition (and (cleartop ?x) (handempty) (onclutterortable ?x))
	        :effect
	        (and (not (ontable ?x))
	            (not (handempty))
	            (not (cleartop ?x))
	            (holding ?x)
	            (clearleft ?x)
	            (clearright ?x)
	            (not (onclutterortable ?x))
	            (not (onsomething ?x))))
	            
	(:action put-on-table
	    :parameters (?x - item)
	    :precondition (holding ?x)
	    :effect
	    (and (not (holding ?x))
	    (cleartop ?x)
	    (handempty)
	    (ontable ?x)
	    (onclutterortable ?x)
	    (onsomething ?x)))


	(:action drop-in-clutter
		:parameters (?x - item)
		:precondition (holding ?x)
		:effect
		(and (not (holding ?x))
		(cleartop ?x)
		(clearleft ?x)
		(clearright ?x)
		(onclutterortable ?x)
		(not (onsomething ?x))
		(handempty)))
		
    
    (:action pick-up-from-on
		:parameters (?x - item ?y - item)
		:precondition (and (on ?x ?y) (onsomething ?x) (cleartop ?x) (handempty))
		:effect
		(and
	            (not (handempty))
	            (not (cleartop ?x))
	            (holding ?x)
	            (not (on ?x ?y))
	            (cleartop ?y)
	            (clearleft ?x)
	            (clearright ?x)
	            (not (onsomething ?x))))
	    
	
	(:action put-on 
	        :parameters (?x - item ?y - item)
	        :precondition (and (holding ?x) (cleartop ?y) (onsomething ?y))
	        :effect
	        (and (not (holding ?x))
	        (not (cleartop ?y))
	        (cleartop ?x)
	        (handempty)
	        (onsomething ?x)
	        (on ?x ?y)))
	        
	        
	(:action put-left
	        :parameters (?x - item ?y - item)
	        :precondition (and (holding ?x) (clearleft ?y) (ontable ?y))
	        :effect
	        (and (not (holding ?x))
	        (not (clearleft ?y))
	        (clearleft ?x)
	        (handempty)
	        (onsomething ?x)
	        (toleft ?x ?y)))
	
	(:action put-right
	        :parameters (?x - item ?y - item)
	        :precondition (and (holding ?x) (clearright ?y) (ontable ?y))
	        :effect
	        (and (not (holding ?x))
	        (not (clearright ?y))
	        (clearright ?x)
	        (handempty)
	        (onsomething ?x)
	        (toright ?x ?y)))
	        
)
	        
	       
