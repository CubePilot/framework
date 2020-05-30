from common import *

del_gibbs = Matrix(symbols('del_gibbs((0:3))', real=True))
#err_quat = gibbs_to_quat(rot_err)
rotErrNew = gibbs_multiply(-del_gibbs, rot_err)

f = Matrix([rotErrNew, gbias, abias, pos, vel])

F = f.jacobian(x)

soln = solve(rotErrNew, del_gibbs, dict=True)[0]
f = f.xreplace(soln)
F = F.xreplace(soln)
del_quat = gibbs_to_quat(del_gibbs).xreplace(soln)

new_quat = quat_multiply(quat,del_quat)
new_quat, F, subx = extractSubexpressions([new_quat, F], 'subx', threshold=4)

for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

for i in range(3):
    for j in range(3):
        print '    F(%u,%u) = %s;' % (i, j, ccode_double(F[i,j]))

for i in range(4):
    print '    quat.coeffs()(%u) = %s;' % (i, ccode_double(new_quat[i]))
