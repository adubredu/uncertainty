(define (domain GROCERY)
	(:requirements :strips :typing)
	(:types item)
	(:predicates (on ?x - item ?y - item)
			(ontable ?x - item)
			(cleartop ?x - item)
			(clearleft ?x - item)
			(clearright ?x - item)
			(handempty)
			(holding ?x - item)
			(toleft ?x - item ?y - item)
			(toright ?x -  item ?y - item)
			)

	(:action pick-up
		:parameters (?x - item)
		:precondition (and (cleartop ?x) (handempty))
		:effect
		(and (not (ontable ?x))
		     (not (cleartop ?x))
		     (not (handempty))
		     (holding ?x)))

	(:action put-on
		:parameters (?x - item ?y - item)
		:precondition (and (cleartop ?y) (holding ?x) (ontable ?y))
		:effect
		(and (on ?x ?y)
			 (not (cleartop ?y))
			 (handempty)
			 (cleartop ?x)
			 (clearleft ?x)
			 (clearright ?x)
			 (not (holding ?x))))


	(:action put-on-table
		:parameters (?x - item)
		:precondition (and (not (ontable ?x)) (holding ?x))
		:effect
		(and (ontable ?x)
			 (not (holding ?x))
			 (cleartop ?x)
			 (clearright ?x)
			 (clearleft ?x)))


	(:action put-left
		:parameters (?x - item ?y - item)
		:precondition (and (clearleft ?y) (holding ?x) (ontable ?y))
		:effect
		(and (toleft ?x ?y)
			 (not (clearleft ?y))
			 (handempty)
			 (ontable ?x)
			 (clearleft ?x)
			 (not (clearright ?x))
			 (not (holding ?x))))

	(:action put-right
		:parameters (?x - item ?y - item)
		:precondition (and (clearright ?y) (holding ?x) (ontable ?y))
		:effect
		(and (toright ?x ?y)
			 (not (clearright ?y))
			 (handempty)
			 (ontable ?x)
			 (clearright ?x)
			 (not (clearleft ?x))
			 (not (holding ?x))))

)