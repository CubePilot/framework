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

del_ang_corrected = del_ang.multiply_elementwise(gscale+ones(3,1))-gbias*dt
del_vel_corrected = del_vel-abias*dt
del_vel_coordinate_ned = Tbn*del_vel_corrected+Matrix([0,0,9.80655])*dt

rot_err_new_approx = quat_to_gibbs(quat_rotate_approx(err_quat, del_ang_corrected))
rot_err_new = quat_to_gibbs(quat_rotate(err_quat, del_ang_corrected))

# f: state-transtition model for the purpose of linearization
f = Matrix([rot_err_new_approx, gbias, gscale, abias, pos+vel*dt, vel+del_vel_coordinate_ned])
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

# Generate C code for prediction model
f, subx = extractSubexpressions([f], 'subx', threshold=4)

print '{ //////// Begin generated code: Prediction model      ////////'
for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

print ''

for i in range(len(f)):
    print '    f(%u,0) = %s;' % (i, ccode_double(f[i]))

print ''

for i in range(F.rows):
    for j in range(F.cols):
        if F[i,j] != 0:
            print '    F(%u,%u) = %s;' % (i, j, ccode_double(F[i,j]))
print '} //////// End generated code: Prediction model        ////////\n'

# Generate C code for prediction model
Q, subx = extractSubexpressions([Q], 'subx', threshold=4)

print '{ //////// Begin generated code: Process noise         ////////'
for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_double(subx[i][1]))

print ''

for i in range(Q.rows):
    for j in range(Q.cols):
        if Q[i,j] != 0:
            print '    P(%u,%u) += %s;' % (i, j, ccode_double(Q[i,j]))
print '} //////// End generated code: Process noise           ////////\n'
