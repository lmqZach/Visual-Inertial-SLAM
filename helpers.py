import numpy as np


def hat(x):
    assert x.shape == (3, )
    x1, x2, x3 = x
    return np.array([
        [0, -x3, x2],
        [x3, 0, -x1],
        [-x2, x1, 0]
    ])


def hat6(u):
    assert u.shape == (6, )
    v, w = u[:3], u[3:]
    return np.block([
        [hat(w), v.reshape(-1, 1)],
        [np.zeros((1, 4))]
    ])


def wedge(u):
    assert u.shape == (6, )
    v, w = u[:3], u[3:]
    return np.block([
        [hat(w), hat(v)],
        [np.zeros((3, 3)), hat(w)]
    ])


def pi(q):
    assert q.shape == (4, )
    q3 = q[2]
    return q / q3


def dpi_dq(q):
    assert q.shape == (4, )
    q1, q2, q3, q4 = q
    return np.array([
        [1, 0, -q1 / q3, 0],
        [0, 1, -q2 / q3, 0],
        [0, 0, 0, 0],
        [0, 0, -q4 / q3, 1]
    ])


def cdot(s):
    assert s.shape == (4, )
    return np.block([
        [np.eye(3), -hat(s[:3])],
        [np.zeros(6)]
    ])


def calc_Ks(K, b):
    fsu = K[0, 0]
    Ks = np.hstack((
        np.vstack((K[:2], K[:2])),
        np.reshape([0, 0, -fsu * b, 0], (-1, 1))
    ))
    return Ks
