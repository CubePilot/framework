from common import *

z = Matrix(symbols('pos_obs((0:3)) vel_obs((0:3))'))
h = Matrix([pos,vel])
H = h.jacobian(x)
y = z-h
S = H*P*H.T

S,subx = extractSubexpressions([S], 'subx', threshold=4)
print 1
S_I = quickinv_sym(S)#copy_upper_to_lower_offdiagonals(Matrix(6, 6, symbols(r'S_I(0:6\,0:6)', real=True)))
print 2
#pprint(S_I)

NIS = y.T*S_I*y

K = P*H.T*S_I
x_n = x+K*y
P_n = (eye(n_states,n_states)-K*H)*P


P_n = copy_upper_to_lower_offdiagonals(P_n)
quat_n, x_n, P_n = derive_zero_rot_err(x_n,P_n)
P_n = copy_upper_to_lower_offdiagonals(P_n)

quat_n, x_n, P_n, subx = extractSubexpressions([quat_n, x_n, P_n], 'subx', threshold=4)


print '{ //////// Begin generated code: Prediction model      ////////'
for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_float(subx[i][1]))

print ''

for i in range(len(x_n)):
    print '    x_n(%u,0) = %s;' % (i, ccode_float(x_n[i]))

print ''

for i in range(P_n.rows):
    for j in range(P_n.cols):
        print '    P_n(%u,%u) = %s;' % (i, j, ccode_float(P_n[i,j]))

print ''

for i in range(len(quat_n)):
    print '    quat_n.%s() = %s;' % ('wxyz'[i], ccode_float(quat_n[i]))
print '} //////// End generated code: Prediction model        ////////\n'
