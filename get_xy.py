import numpy as np
def rot(th):
    return np.array([
        [np.cos(th), -np.sin(th)],
        [np.sin(th),  np.cos(th)]
    ])
def get_xy(x, th1, th2, l):
    cart = np.array([[x], [0]])
    pend_1 = cart + rot(th1)@np.array([[0],[-l]])
    pend_2 = pend_1 + rot(th1+th2)@np.array([[0], [-l]])
    return np.concatenate([cart, pend_1, pend_2], axis=1)
