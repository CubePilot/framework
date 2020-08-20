from common import *

quat_n, x_n, P_n, subx = extractSubexpressions(derive_zero_rot_err(x, P), 'subx', threshold=0)

print '{ //////// Begin generated code: zero rotation error    ////////'
for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_float(subx[i][1]))

print ''

for i in range(len(x_n)):
    if x_n[i] != x[i]:
        print '    x_n(%u,0) = %s;' % (i, ccode_float(x_n[i]))

print ''

for i in range(P_n.rows):
    for j in range(P_n.cols):
        if P_n[i,j] != P[i,j]:
            print '    P_n(%u,%u) = %s;' % (i, j, ccode_float(P_n[i,j]))

print ''

for i in range(len(quat_n)):
    print '    quat_n.%s() = %s;' % ('wxyz'[i], ccode_float(quat_n[i]))
print '} //////// End generated code: zero rotation error        ////////\n'
