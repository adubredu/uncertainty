(define (problem drink)
  (:domain tequila)
  (:objects
    casamigos-anejo - bottle
  )
  (:init
    (= (total-cost casamigos-anejo) 4)
  )
  ; drink all the tequila
  (:goal
    (bottle-finished casamigos-anejo) 
  )
)