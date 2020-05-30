from common import *

pprint(P)

err_quat = gibbs_to_quat(rot_err)
att_euler = quat_to_euler(quat_multiply(est_quat, err_quat))

G = att_euler[2,:].jacobian(x)

heading_sigma = sqrt((G*P*G.T)[0])

pprint(heading_sigma)

heading_sigma, subx = extractSubexpressions([heading_sigma], 'subx', threshold=4)

for i in range(len(subx)):
    print '    double %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

print '    heading_sigma = %s;' % (ccode_double(heading_sigma),)
