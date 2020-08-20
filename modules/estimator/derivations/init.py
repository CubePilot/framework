from common import *

# Generate covariance initialization
accel_vector = Matrix(symbols('init_accel_vec((0:3))', real=True))
pos_ecef = Matrix(symbols('pos_ecef((0:3))', real=True))
lat, lon = symbols('lat lon', real=True)
yaw_angle = Symbol('yaw_angle', real=True)
accel_sigma = Symbol('accel_sigma', real=True, nonnegative=True)
yaw_sigma = Symbol('yaw_sigma', real=True, nonnegative=True)
init_quat_sym = Matrix(symbols('init_quat((0:4))', real=True))
init_rot_err_sym = Matrix(symbols('init_rot_err((0:3))', real=True))

T_ecef2ned = rotation_ecef_to_ned_from_lat_lon(lat,lon)
down_ecef = -T_ecef2ned.T*Matrix([0,0,1])
gravity_vec = gravity_ecef(pos_ecef)

accel_unit_vector = accel_vector/accel_vector.norm()

subs = {accel_vector[0]: 0, accel_vector[1]: 0, accel_vector[2]: -9.81, lat: pi/6, lon: pi/6, yaw_angle: 15, accel_sigma:0.1, yaw_sigma: 1}
subs.update(dict(zip(gravity_vec, gravity_ecef(lla2ecef(lat, lon, 0)))))

roll = atan2(-accel_vector[1],-accel_vector[2])
pitch = asin(accel_unit_vector[0])

init_quat = quat_rotate_y(quat_rotate_x(quat_rotate_y(Matrix([1,0,0,0]), -pi/2), lon), -lat)
init_quat = quat_multiply(init_quat, quat_from_euler(roll, pitch, yaw_angle))

err_quat_influence = init_quat.jacobian(Matrix([accel_vector, [yaw_angle]]))

init_sigma = Matrix([accel_sigma, accel_sigma, accel_sigma, yaw_sigma])
err_quat_param_covariance = diag(*init_sigma.multiply_elementwise(init_sigma))
err_quat_cov = err_quat_influence*err_quat_param_covariance*err_quat_influence.T

truth_quat = quat_multiply(init_quat_sym, gibbs_to_quat(init_rot_err_sym))
init_rot_err_influence = truth_quat.jacobian(init_rot_err_sym).xreplace(dict(zip(init_rot_err_sym, zeros(3,1)))).pinv()
pprint(init_rot_err_influence.xreplace(dict(zip(init_quat_sym, init_quat))).xreplace(subs).evalf())
init_rot_err_cov = init_rot_err_influence*err_quat_cov*init_rot_err_influence.T
init_rot_err_cov = init_rot_err_cov.xreplace(dict(zip(init_quat_sym, init_quat)))


mag_field = Matrix(symbols('mag_meas((0:3))', real=True))

angle = acos((mag_meas-magb).dot(Tbn.T*mage))
axis = (mag_meas-magb).cross(Tbn.T*mage) / (sin(angle)*(mag_meas-magb).norm()*mage.norm())

rot_err_mag = axis*tan(angle/4)

f = Matrix([rot_err, zeros(3,1), zeros(3,1), zeros(3,1), mag_field-Tbn.T*mage, Tbn*(mag_field-magb), zeros(3,1), zeros(3,1)])

mag_meas_influence = f_mage_assumed.jacobian(Matrix([rot_err, mag_field, magb, mage]))

for _ in range(3):
    mag_meas_influence = mag_meas_influence.xreplace(zip(rot_err, zeros(3,1))))

R = diag(init_rot_err_cov, diag(*symbols("mag_meas_var((0:3))",real=True)), eye(3,3)*Symbol('magb_var',real=True), eye(3,3)*Symbol('mage_var',real=True))
P_init = mag_meas_influence*R*mag_meas_influence.T

P_init.xreplace(dict(zip(est_quat, init_quat)))

#pprint(mag_meas_influence)
#pprint(simplify(mag_meas_influence))

#pprint(mag_meas_influence)

init_quat, P_init, subx = extractSubexpressions([init_quat,P_init], 'subx', threshold=4)

#for sym,expr in subx:
    #subs[sym] = expr.xreplace(subs).evalf()

#pprint(init_quat.xreplace(subs).evalf())
#pprint(init_rot_err_cov.xreplace(subs).evalf())

# Generate C code for quaternion covariance initialization
print '{ //////// Begin generated code: Attitude covariance initialization ////////'
for i in range(len(subx)):
    print '    float %s = %s;' % (subx[i][0], ccode_float(subx[i][1]))

print ''

for i in range(init_quat.rows):
    print '    quat.%s() = %s;' % (('w', 'x', 'y', 'z')[i], ccode_float(init_quat[i]))

print ''

for i in range(P_init.rows):
    for j in range(P_init.cols):
        if P_init[i,j] != 0:
            print '    P(%u,%u) = %s;' % (i, j, ccode_float(P_init[i,j]))

print '} //////// End generated code: Attitude covariance initialization   ////////\n'
