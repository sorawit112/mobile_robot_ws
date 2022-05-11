from enum import Enum

class Unit(Enum):
    """ units instance list"""
    IDLE = 0
    NAV = 1
    DOCK = 2
    TURTLE = 3

    _dict = {'IDLE':IDLE, 'NAV':NAV, 'DOCK':DOCK, 'TURTLE':TURTLE}

dic = Unit._dict.value
print(dic.values())

x = Unit.NAV
y = 1

print (x == Unit.NAV)
print (x is Unit.NAV)
print (y is Unit.NAV)
print (x is not Unit.DOCK)