# The identity function I
identity = lambda x: x
I = identity

# The successor function S
successor = lambda w: lambda y: lambda x: y(w(y)(x))
S = successor

# Define the function that represents zero and one in lambda calculus
zero = lambda x: lambda y: y
one = lambda x: lambda y: x(y)  # which is equal to S(zero)
two = S(one)
three = S(two)
four = S(three)
five = S(four)

# Define functions to convert numbers from lambda calculus to base 10
next = lambda x: x+1
eval = lambda x: x(next)(0)

assert 0 == eval(zero)
assert 1 == eval(one)
assert 2 == eval(two)
assert 3 == eval(three)
assert 4 == eval(four)
assert 5 == eval(five)
assert 0 == eval(I(I)(zero))
assert 1 == eval(I(I)(I)(I)(one))
assert 0 == eval(zero)
assert 1 == eval(S(zero))
assert 2 == eval(S(S(zero)))
assert 3 == eval(S(S(S(zero))))
assert 4 == eval(S(S(S(S(zero)))))

print(eval(one(S)(S(one))))

