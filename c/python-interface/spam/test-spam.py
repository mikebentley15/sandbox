#!/usr/bin/env python3
import spam

print()
status = spam.system("ls -l | wc")
print("status: ", status)

print()
print('Expect spam.SpamError')
try:
    status = spam.check_system("false")
    print("status: ", status)
except spam.SpamError as ex:
    print(' ', ex)
    print('  ignored')

print()
s = spam.Spam("Real brand of SPAM")
s.print()
print(s)

print()
n1 = spam.Noddy1()
print(n1)

print()
n2 = spam.Noddy2(first="Mike", last="Bentley")
print(n2)
print("Name: ", n2.name())

print()
n2 = spam.Noddy2(first=2)
print(n2)
print("Name: ", n2.name())

print()
n3 = spam.Noddy3(first="Mike", last="Bentley")
print(n3)
print("Name: ", n3.name())

print()
print('Expect TypeError')
try:
    spam.Noddy3(first=3)
except TypeError as ex:
    print(' ', ex)
    print('  ignored')

print()
n4 = spam.Noddy4(first="Mike", last="Bentley")
print(n4)
print("Name: ", n4.name())

print()
n4 = spam.Noddy4(first=2)
print(n4)
print("Name: ", n4.name())
