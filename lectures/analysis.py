import control
import numpy as np
import matplotlib.pyplot as plt


def rlocus(name, sys, kvect, k=None):
    if k is None:
        k = kvect[-1]
    sysc = control.feedback(sys*k, 1)
    closed_loop_poles = control.pole(sysc)
        
    res = control.rlocus(sys, kvect, Plot=False);
    for locus in res[0].T:
        plt.plot(np.real(locus), np.imag(locus))
    p = control.pole(sys)
    z = control.zero(sys)
    marker_props = {'markeredgecolor':'r', 'markerfacecolor':'none', 'markeredgewidth':2, 'markersize':10, 'linestyle':'none'}
    plt.plot(np.real(p), np.imag(p), marker='x', label='pole', **marker_props)
    plt.plot(np.real(z), np.imag(z), marker='o', label='zero', **marker_props)
    plt.plot(np.real(closed_loop_poles), np.imag(closed_loop_poles), marker='s', label='closed loop pole', **marker_props)
                    
    plt.legend(loc='best')
    plt.xlabel('real')
    plt.ylabel('imag')
    plt.grid(True)
    plt.title(name + ' root locus')

def step_response(name, sys, t_vect):
    t, y = control.step_response(sys, T=t_vect)
    plt.plot(t, y)
    plt.xlabel('t, sec')
    plt.grid(True)
    plt.title(name + ' step response')


def bode(name, sys, omega, margins=False, Hz=False):
    mag, phase, omega = control.bode(sys, omega, Plot=False)
    if Hz:
        omega = omega/(2*np.pi)
    mag_dB = 20*np.log10(mag)
    if margins:
        gm, pm, sm, wg, wp, ws = control.stability_margins(sys)

    plt.subplot(211)
    if margins:
        plt.hlines(0, omega[0], omega[-1], linestyle='--')
        plt.vlines([wp, wg], np.min(mag_dB), np.max(mag_dB), linestyle='--')
    plt.semilogx(omega, mag_dB)
    if Hz:
        plt.xlabel('Hz')
    else:
        plt.xlabel('rad')
    plt.ylabel('dB')
    plt.grid()
    
    if margins:
        plt.title(name  +' bode pm: {:0.2f} deg @{:0.2f} rad/s gm: {:0.2f} @{:0.2f} rad/s'.format(pm, wp, gm, wg))
    else:
        plt.title(name + ' bode')
    plt.subplot(212)
    phase_deg = np.rad2deg(phase)
    plt.semilogx(omega, phase_deg)
    if margins:
        plt.vlines([wp, wg], np.min(phase_deg), np.max(phase_deg), linestyle='--')
        plt.hlines(-180, omega[0], omega[-1], linestyle='--')
    if Hz:
        plt.xlabel('Hz')
    else:
        plt.xlabel('rad')
    plt.ylabel('deg')
    plt.grid()
    
def loop_analysis(name, sys, zeta, wd, k, t_vect, k_vect, omega_vect):
    sysc = control.feedback(sys*k, 1)
    closed_loop_poles = control.pole(sysc)
    
    plt.figure(figsize=(10, 5))
    rlocus(name, sys, k_vect, k)
    plt.axis([-16, 0, -8 ,8])
    theta = np.arccos(zeta)
    plt.plot([0, -10], [0, 10*np.tan(theta)], 'r--')
    plt.vlines(-wd, -8, 8, linestyle='--', color='r')

    plt.figure(figsize=(10, 5))
    bode(name, sys, omega_vect, margins=True)
    
    plt.figure(figsize=(10, 5))
    control.nyquist(sys, omega=omega_vect)
    plt.title(name + ' nyquist')
    plt.axis([-10, 10, -10, 10])


    plt.figure(figsize=(10, 5))
    step_reponse(name, sysc, t_vect)
    
    plt.figure(figsize=(10, 5))
    bode(name + ' closed loop ', sysc, np.logspace(-2, 2))