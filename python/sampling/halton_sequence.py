import numpy as np

small_primes = set([
    2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
   31, 37, 41, 43, 47, 53, 59, 61, 67, 71,
   73, 79, 83, 89, 97,101,103,107,109,113,
  127,131,137,139,149,151,157,163,167,173,
  179,181,191,193,197,199,211])

def uniform_samples(dim=1):
    'Return an infinite generator of uniform samples of specified dimensions'
    while True:
        yield np.random.uniform(size=dim)

def next_prime(n):
    'return the next prime bigger than n'
    if n % 2 == 0:
        n += 1
    else:
        n += 2
    while not is_prime(n):
        n += 2
    return n

def is_prime(n):
    'Returns true if n is a prime integer'
    if n < 212:
        return n in small_primes

    for prime in small_primes:
        if n % prime == 0:
            return False

    i = 211
    while i * i < n:
        i += 2
        if n % i == 0:
            return False
    return True

def halton(dim=None, bases=None):
    '''
    Returns a generator of the infinite halton series over the space of [0, 1]
    in each dimension.

    This can return high-dimensional vectors.

    @param dim: dimensionality of the vector to return.  Default = 1
    @param bases: bases to use in the halton sequence.  If None, then this will
        choose 5, 7, 11, 13, etc. -- simply increasing primes.  If not None,
        then bases must match the number of dimensions (if given).
    @return a generator of numpy.array() objects of size dim (or len(bases),
        whichever was specified).  This generator will keep generating values
        as long as they are requested, so please do not convert it to a list,
        otherwise that will cause an infinite loop.
    '''
    if dim is None and bases is None:
        dim = 1
        bases = [5]
    elif dim is None:
        dim = len(bases)
    elif bases is None:
        bases = []
        prime = 5
        bases.append(prime)
        for i in range(dim - 1):
            prime = next_prime(prime)
            bases.append(prime)
    assert dim is not None
    assert bases is not None
    assert dim == len(bases)
    generators = [halton_1d(base) for base in bases]
    gen = zip(*generators)
    for val in gen:
        yield np.array(val)

def halton_1d(base=2, i=1):
    '''
    Returns a generator of the infinite halton series over the space of [0, 1].

    @param base: the base used to generate the halton sequence
    @param i: the starting index to generate (must be 1 or bigger)
    @return a generator that will generate infinitely the halton sequence
    '''
    while True:
        n = i
        value = 0
        denominator = 1
        while n > 0:
            denominator *= base
            n, remainder = divmod(n, base)
            value += remainder / float(denominator)
        yield value
        i += 1
    
