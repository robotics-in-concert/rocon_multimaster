#!/usr/bin/env python

import roslib; roslib.load_manifest('rocon_gateway_tests')
from gateway_comms.msg import Connection

a = Connection("a","b","c","d","e","f")
b = Connection("a","b","c","d","e","f")

print "Basic Operations: "
print "Verifies a and b are different objects with same data"
print "a is b fails, but the equality evalutes to true"
print "the hash is different (the base message class has different values)"
print a is b
print a == b
print hash(a)
print hash(b)

print ""
print "Sets don't work well"
print "if a==b and set s contains a, b in s will still fail"
print "both a and b can be added into the set (even though the data is same)"
print "this happens as the hash is different"
s = set()
s.add(a)
print s
print a in s
print b in s
s.add(b) #should add if different
print s

print ""
print "lists work absolutely fine"
print "if a==b and list l contains a, b in l will evaluate to true"
print "all we have to do is keep the lists unique, and sets were not working anyway"
l = list()
l.append(a)
print l
print a in l
print b in l

if b not in l: #check to maintain uniqueness
    l.append(b)
print l

print "lists yay, sets nay"

