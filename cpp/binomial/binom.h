#ifndef BINOM_H
#define BINOM_H

/** Binomal coefficient (also known as n choose k)
 *
 * Definition
 *   binom(n, k) := n! / (k! (n-k)!)
 * Follows the recurrence
 *   binom(n, k) = (n/k) * binom(n-1, k-1)
 * Or
 *   binom(n, k) = binom(n, k-1) * (n-k+1) / k
 * Which expands to
 *   binom(n, k) = (n-0/1)(n-1/2)(n-2/3)...(n-k+1/k)
 * This is a good formulation because each multiplication is a valid
 * binomial expansion, and therefore is an integer.
 */
template <typename Int>
Int binom(Int n, Int k) {
  if (k > n) { return 0; }
  if (n - k < k) { k = n - k; } // utilize symmetry

  Int result = n;
  --n;
  for (Int d = 2; d <= k; ++d) {
    if (n % d == 0) {
      result *= n / d;
    } else if (result % d == 0) {
      result /= d;
      result *= n;
    } else {
      // Can always do this, but the others prevent unnecessary overflow
      result *= n;
      result /= d; // always guaranteed to be exactly divisible
    }
    --n;
  }
  return result;
}

#endif // BINOM_H
