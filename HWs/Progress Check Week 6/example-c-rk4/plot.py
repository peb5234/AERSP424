# %%
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

# %%


def ffunc(t):
    _p = np.pi / 6 + 0 * t
    _q = np.cos(t * 6 / np.pi)
    _r = 3 * np.sin(t * 30 / np.pi)
    return _p, _q, _r


def dfunc(x, dcm, v_body, t):
    # For readability, pull out Euler angles
    _phi = x[0]
    _theta = x[1]
    _psi = x[2]

    # In this case, p,q & r are given by an analytic formula
    _p, _q, _r = ffunc(t)

    # Gimbal equation
    _qdot = np.array(
        [
            [1, np.tan(_theta) * np.sin(_phi), np.tan(_theta) * np.cos(_phi)],
            [0, np.cos(_phi), -np.sin(_phi)],
            [0, np.sin(_phi) / np.cos(_theta), np.cos(_phi) / np.cos(_theta)],
        ]
    ).dot(np.array([_p, _q, _r]))

    # Strapdown Equation
    _Cdot = dcm.dot(np.array([[0, -_r, _q], [_r, 0, -_p], [-_q, _p, 0]]))

    # Get Velocity in NED
    # this is position_dot for position measured in NED
    _V_NED = dcm.dot(v_body)

    # Assemble the vector of state derivatives -- putting in zeros for the
    # derivative of velocity since it is a constant here
    _xdot = np.hstack((_qdot, [0, 0, 0], _V_NED))

    return _xdot, _Cdot


# %%

df = np.loadtxt("results.txt", delimiter="\t", skiprows=3)
x = df[:, 1:10]
xd = df[:, 10:19]
t = df[:, 0]


# %%

fig, axes = plt.subplots(5, 1, sharex=True, figsize=(16, 9))

RAD2DEG = 180 / np.pi

ps, qs, rs = ffunc(t)
axes[0].plot(t, RAD2DEG * ps, t, RAD2DEG * qs, t, RAD2DEG * rs)
axes[0].legend(["$p$", "$q$", "$r$"])
axes[0].set_ylabel("degrees/sec")

axes[1].plot(
    t[:], RAD2DEG * xd[:, 0], t[:], RAD2DEG * xd[:, 1], t[:], RAD2DEG * xd[:, 2]
)
axes[1].legend(["$\dot{\phi}$", "$\dot{\\theta}$", "$\dot{\psi}$"])
axes[1].set_ylabel("degrees/sec")

phis = (x[:, 0] + np.pi) % (2 * np.pi) - np.pi
thetas = (x[:, 1] + np.pi) % (2 * np.pi) - np.pi
psis = (x[:, 2] + np.pi) % (2 * np.pi) - np.pi
axes[2].plot(t, RAD2DEG * phis, t, RAD2DEG * thetas, t, RAD2DEG * psis)
axes[2].legend(["$\phi$", "$\\theta$", "$\psi$"])
axes[2].set_ylabel("degrees")

vt = np.sqrt(np.sum(xd[:, 6:] ** 2, axis=1))
axes[3].plot(t[:], xd[:, 6], t[:], xd[:, 7], t[:], xd[:, 8], t[:], vt)
axes[3].legend(["$V_N$", "$V_E$", "$V_D$", "$V$"])
axes[3].set_ylabel("velocity (fps)")

pt = np.sqrt(np.sum(x[:, 2:] ** 2, axis=1))
axes[4].plot(t, x[:, 3], t, x[:, 4], t, x[:, 5], t, pt)
axes[4].legend(["$P_N$", "$P_E$", "$P_D$", "$P$"])
axes[4].set_ylabel("position (ft)")

plt.xlabel("time (sec)")
plt.tight_layout()
# plt.savefig('runge-kutta.png', format='png', dpi=1200)
plt.show()

# %%
fig, ax = plt.subplots(1, 1, sharex=True, figsize=(16, 9))

ax.plot(t, RAD2DEG * phis, t, RAD2DEG * thetas, t, RAD2DEG * psis)
ax.legend(["$\phi$", "$\\theta$", "$\psi$"])
ax.set_ylabel("degrees")
plt.show()
# %%
fig = plt.figure(figsize=(16, 9))
ax = fig.add_subplot(111, projection="3d")
ax.plot(x[:, 6], x[:, 7], x[:, 8])

plt.show()


# %%
