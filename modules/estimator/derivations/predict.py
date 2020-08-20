from common import *

for line in state_idx_defines:
    print line

# Generate prediction model
dt = Symbol('imu_dt', real=True, nonnegative=True)
del_ang = Matrix(symbols('del_ang((0:3))', real=True))
del_vel = Matrix(symbols('del_vel((0:3))', real=True))
gbias_pnoise = Symbol('gbias_pnoise', real=True, nonnegative=True)
abias_pnoise_xy = Symbol('abias_pnoise_xy', real=True, nonnegative=True)
abias_pnoise_z = Symbol('abias_pnoise_z', real=True, nonnegative=True)
gscale_pnoise = Symbol('gscale_pnoise', real=True, nonnegative=True)
tsca_err_pnoise = Symbol('tsca_err_pnoise', real=True, nonnegative=True)
accel_sigma = Symbol('accel_sigma', real=True, nonnegative=True)
gyro_sigma = Symbol('gyro_sigma', real=True, nonnegative=True)
accel_scale_sigma = Symbol('accel_scale_sigma', real=True, nonnegative=True)
gyro_cross_sigma = Symbol('gyro_cross_sigma', real=True, nonnegative=True)
accel_cross_sigma = Symbol('accel_cross_sigma', real=True, nonnegative=True)
gravity_vec = Matrix(symbols('gravity_vec((0:3))', real=True))

earth_omega_body = Tbn.T*Matrix([0,0,7.2921159e-5])

del_ang_corrected = del_ang.multiply_elementwise(gscale+ones(3,1))-gbias*dt - earth_omega_body*dt
del_vel_corrected = del_vel-abias*dt
del_vel_coordinate_ned = Tbn*del_vel_corrected+gravity_vec*dt

rot_err_new_approx = quat_to_gibbs(quat_rotate_approx(err_quat, del_ang_corrected))
rot_err_new = quat_to_gibbs(quat_rotate(err_quat, del_ang_corrected))

#pprint(rot_err_new.xreplace(dict(zip(del_ang_corrected, symbols('del_ang_corrected((0:3))',real=True))+zip(rot_err, zeros(3,1))))[0])

# f: state-transtition model for the purpose of linearization
f = Matrix([rot_err_new_approx, gbias, gscale, abias, pos+vel*dt, vel+del_vel_coordinate_ned, magb, mage])

F = f.jacobian(x)

# u: control input vector
u = Matrix([del_ang, del_vel])

# G: control-influence matrix, AKA "B" in literature
G = f.jacobian(u)

del_ang_sigma = ones(3,1)*gyro_sigma*dt + Matrix([[0, gyro_cross_sigma, gyro_cross_sigma], [gyro_cross_sigma, 0, gyro_cross_sigma], [gyro_cross_sigma, gyro_cross_sigma, 0]])*del_ang_corrected

del_vel_sigma = ones(3,1)*accel_sigma*dt + Matrix([[accel_scale_sigma, accel_cross_sigma, accel_cross_sigma], [accel_cross_sigma, accel_scale_sigma, accel_cross_sigma], [accel_cross_sigma, accel_cross_sigma, accel_scale_sigma]])*del_vel_corrected

# w_u_sigma: additive noise on u
w_u_sigma = Matrix([del_ang_sigma, del_vel_sigma])

# Q_u: covariance of additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma))

# Q: covariance of additive noise on x
Q = G*Q_u*G.T

for sym in gbias:
    i = get_state_index(sym)
    Q[i,i] += (gbias_pnoise*dt)**2

for sym in gscale:
    i = get_state_index(sym)
    Q[i,i] += (gscale_pnoise*dt)**2

for sym in abias:
    i = get_state_index(sym)
    if sym == abias[2]:
        Q[i,i] += (abias_pnoise_z*dt)**2
    else:
        Q[i,i] += (abias_pnoise_xy*dt)**2

f[0:3,:] = rot_err_new

f = f.xreplace(dict(zip(rot_err, zeros(3,1))))
F = F.xreplace(dict(zip(rot_err, zeros(3,1))))
Q = Q.xreplace(dict(zip(rot_err, zeros(3,1))))

P_n = F*P*F.T+Q

pprint(f)
P_n = copy_upper_to_lower_offdiagonals(P_n)
quat_n, f, P_n = derive_zero_rot_err(f,P_n)
P_n = copy_upper_to_lower_offdiagonals(P_n)

pprint(f)

# Generate C code for prediction model
quat_n, x_n, P_n, subx = extractSubexpressions([quat_n, f, P_n], 'subx', threshold=4)

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

## Generate C code for prediction model
#Q, subx = extractSubexpressions([Q], 'subx', threshold=4)

#print '{ //////// Begin generated code: Process noise         ////////'
#for i in range(len(subx)):
    #print '    float %s = %s;' % (subx[i][0], ccode_float(subx[i][1]))

#print ''

#for i in range(Q.rows):
    #for j in range(Q.cols):
        #if Q[i,j] != 0:
            #print '    P(%u,%u) += %s;' % (i, j, ccode_float(Q[i,j]))
#print '} //////// End generated code: Process noise           ////////\n'
