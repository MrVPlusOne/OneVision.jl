import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

fleet_size=3
fn_list = ["trajectory{}.json".format(i) for i in range(1, fleet_size+1)]
X, U, Z = "x̃", "ũ", "z̃"
n_list = [0, 1] #list(range(fleet_size))
Tx,Tu,Tc = 3, 18, 5
T_ctrl = 1
offset=0
def get_file():
    j_list = []
    for fn in fn_list:
        with open(fn, "r") as  f:
            data = json.load(f)
            t = [int(t) for t in data.keys()]
            t.sort()
            j_list.append(data)
    return j_list
            
"""
Get car n's idea on car ft
"""
def get_state(j_list, val, n, ft):
    print("val is {}, str is {}".format(val, str(val) in j_list[n].keys()))
    print(ft)
    x = [i[0] for i in j_list[n][str(val)][X][ft]]
    y = [i[1] for i in j_list[n][str(val)][X][ft]]
    theta = [i[2] for i in j_list[n][str(val)][X][ft]]
    vel = [i[3] for i in j_list[n][str(val)][X][ft]]
    psi = [i[4] for i in j_list[n][str(val)][X][ft]]
    return x, y, theta, vel, psi

def get_action(j_list, val, n, ft):
    vel_act = [i[0] for i in j_list[n][str(val)][U][ft]]
    psi_act = [i[1] for i in j_list[n][str(val)][U][ft]]
    return vel_act, psi_act
def plot_axis(j_list, n_list):
    fig, axs = plt.subplots(3, 2, squeeze=False)
    l0s = {n: [] for n in n_list}
    l1s = {n: [] for n in n_list}
    l2s = {n: [] for n in n_list}
    l3s = {n: [] for n in n_list}
    l4s = {n: [] for n in n_list}
    l5s = {n: [] for n in n_list}
    p0s = {n: [] for n in n_list}
    p1s = {n: [] for n in n_list}
    p2s = {n: [] for n in n_list}
    p3s = {n: [] for n in n_list}
    p4s = {n: [] for n in n_list}
    p5s = {n: [] for n in n_list}

    stimes = []
    sliders = []
    for n in n_list:
        for ft in range(fleet_size):
            plt.subplots_adjust(left=0.1, bottom=0.1, right=0.95)
            data = j_list[0]
            x, y, theta, vel, psi = get_state(j_list, 1, n, ft)
            vel_act, psi_act  = get_action(j_list, 1, n, ft)

            t = [int(t) for t in data.keys()]
            t.sort()
            print(t)
            l0, = axs[0, 0].plot(x, y, label="car {} on car {}".format(n+1, ft+1))
            l1, = axs[0, 1].plot([i for i in range(len(theta))], theta, label="car {} on car {}".format(n+1, ft+1))
            l2, = axs[1, 0].plot([i for i in range(len(theta))], vel, label="car {} on car {}".format(n+1, ft+1))
            l3, = axs[1, 1].plot([i for i in range(len(theta))], psi, label="car {} on car {}".format(n+1, ft+1))
            l4, =  axs[2, 0].plot([i for i in range(len(theta))], vel_act, label="car {} on car {}".format(n+1, ft+1))
            l5, =  axs[2, 1].plot([i for i in range(len(theta))], psi_act, label="car {} on car {}".format(n+1, ft+1))

            p0 = axs[0, 0].scatter([x[Tx+Tc+T_ctrl-1]], [y[Tx+Tc+T_ctrl-1]])
            p1 = axs[0, 1].scatter([t[Tx+Tc+T_ctrl-1]], [theta[Tx+Tc+T_ctrl-1]])
            p2 = axs[1, 0].scatter([t[Tx+Tc+T_ctrl-1]], [vel[Tx+Tc+T_ctrl-1]])
            p3 = axs[1, 1].scatter([t[Tx+Tc+T_ctrl-1]], [psi[Tx+Tc+T_ctrl-1]])
            p4 = axs[2, 0].scatter([t[Tx+Tc+T_ctrl-1]], [vel_act[Tx+Tc+T_ctrl-1]])
            p5 = axs[2, 1].scatter([t[Tx+Tc+T_ctrl-1]], [psi_act[Tx+Tc+T_ctrl-1]])

            l0s[n].append(l0)
            l1s[n].append(l1)
            l2s[n].append(l2)
            l3s[n].append(l3)
            p0s[n].append(p0)
            p1s[n].append(p1)
            p2s[n].append(p2)
            p3s[n].append(p3)

            l4s[n].append(l4)
            l5s[n].append(l5)
            p4s[n].append(p4)
            p5s[n].append(p5)

            axs[0, 0].legend()
            axs[0, 0].set_title("2d Location")
            axs[0, 1].set_title("Angle vs Time")
            axs[1, 0].set_title("Velocity vs Time")
            axs[1, 1].set_title("Steering Angle vs Time")
            axs[2, 0].set_title("Desired Velocity vs Time")
            axs[2, 1].set_title("Desired Steering Angle vs Time")

            axs[0, 0].set_xlim([0, 5])
            axs[0, 0].set_ylim([60, 78])
            axs[0, 1].set_ylim([-1.6,1.6])
            axs[1, 0].set_ylim([-0.5,2.0])
            axs[1, 1].set_ylim([-1.6,1.6])
            axs[2, 0].set_ylim([-0.5,2.0])
            axs[2, 1].set_ylim([-1.6,1.6])
    axcolor = 'lightgoldenrodyellow'      
    
    axtime_all =  plt.axes([0.1, 0.07-0.015*(fleet_size+1), 0.7, 0.01], facecolor=axcolor)
    stime = Slider(axtime_all, 'Time', t[0], t[-1], valinit=0.0, valstep=1)  
    def update_all_func(l0s, l1s, l2s, l3s, p0s, p1s, p2s, p3s, sliders):
        def update_all(val):
            for s in sliders:
                s.set_val(val)
            for n in n_list:
                for v in range(val-offset, val+offset+1, 1):
                    ft = 0
                    for l0, l1, l2, l3, p0, p1, p2, p3, p4, p5 in zip(l0s[n], l1s[n], l2s[n], l3s[n], p0s[n], p1s[n], p2s[n], p3s[n], p4s[n], p5s[n]):
                        print(v)
                        x, y, theta, vel, psi = get_state(j_list, v, n, ft)
                        vel_act, psi_act  = get_action(j_list, v, n, ft)

                        l0.set_xdata(x)
                        l0.set_ydata(y)
                        l1.set_ydata(theta)
                        l2.set_ydata(vel)
                        l3.set_ydata(psi)
                        p0.set_offsets([x[Tx+Tc+T_ctrl-1], y[Tx+Tc+T_ctrl-1]])
                        p1.set_offsets([Tx+Tc+T_ctrl-1, theta[Tx+Tc+T_ctrl-1]])
                        p2.set_offsets([Tx+Tc+T_ctrl-1, vel[Tx+Tc+T_ctrl-1]])
                        p3.set_offsets([Tx+Tc+T_ctrl-1, psi[Tx+Tc+T_ctrl-1]])

                        l4.set_ydata(vel_act)
                        l5.set_ydata(psi_act)
                        p4.set_offsets([Tx+Tc+T_ctrl-1, vel_act[Tx+Tc+T_ctrl-1]])
                        p5.set_offsets([Tx+Tc+T_ctrl-1, psi_act[Tx+Tc+T_ctrl-1]])

                        ft += 1
                        fig.canvas.draw_idle()
        return update_all
    stime.on_changed(update_all_func(l0s, l1s, l2s, l3s, p0s, p1s, p2s, p3s, stimes))

    plt.show()
"""
Video: Stop frame
Trajectory 

send numloop to c++


close wall - show trajectory
"""

def main():
    j_list = get_file()
    
    plot_axis(j_list, n_list)

if __name__== "__main__":
    main()