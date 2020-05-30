from helpers import *

x = Matrix([])
state_idx_defines = []

def add_states(name, n):
    global x, state_idx_defines
    syms = []
    names = []

    for i in range(n):
        names.append('%s%u' % (name.upper(), i))
        state_idx_defines.append('#define STATE_IDX_%s %u' % (names[-1], len(state_idx_defines)))

    ret = Matrix([Symbol('x(STATE_IDX_%s)' % (name,), real=True) for name in names])
    x = Matrix([x, ret])
    return ret

def get_state_index(state_symbol):
    global x

    for i in range(len(x)):
        if x[i] == state_symbol:
            return i
    return None

est_quat = Matrix(symbols('quat.coeffs()((0:4))', real=True))
rot_err = add_states('rot_err', 3)
gbias = add_states('gbias', 3)
gscale = add_states('gscale', 3)
abias = add_states('abias', 3)
pos = add_states('pos', 3)
vel = add_states('vel', 3)

n_states = len(x)

P = copy_upper_to_lower_offdiagonals(Matrix(n_states, n_states, symbols(r'P(0:%u\,0:%u)' % (n_states,n_states), real=True)))

for i in range(n_states):
    P[i,i] = Symbol(str(P[i,i]), real=True, nonnegative=True)

err_quat = gibbs_to_quat(rot_err)
Tbn = quat_to_matrix(quat_multiply(est_quat, err_quat))
