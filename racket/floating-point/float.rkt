#lang typed/racket

(struct float
        ([sign : Integer]
         [significand : Natural]
         [exponent : Integer])
        #:transparent)

(: rat (-> float Exact-Rational))
(define (rat x)
  (match-define (float s m e) x)
  (* s m (expt 2 e)))

;; TODO: write some tests
(rat (float 1 10 3))
(rat (float -1 23 -6))

