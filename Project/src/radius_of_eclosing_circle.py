#!/usr/bin/python
import sys

def radius_of_eclosing_circle(argv):

    p = argv[0]
    q = argv[1]
    r = argv[2]

    if ((p != q) & (q!=r) & (r!=p)):

        #computing distances
        PQ = sqrt(pow(p[0]-q[0],2), pow(p[1]-q[1],2))
        QR = sqrt(pow(q[0]-r[0],2), pow(q[1]-r[1],2))
        RP = sqrt(pow(r[0]-p[0],2), pow(r[1]-p[1],2))

        #semiperimeter
        s = (PQ+QR+RP) / 2.0
        #area using Heron's formula
        area = sqrt(s*(s-PQ)*(s-QR)*(s-RP))

        #radius
        radius = PQ*QR*RP/(4*area)

return radius