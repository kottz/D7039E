
import time
import math
import pandas
import numpy as np
#import matplotlib
#import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline



def ik(x, y, z):
    pass

def fk(angles):
    pass

def read_angles():
    pass

def save_file(data, file_name):
    
    #names = ["angle" + str(i) for i in range(0, len(data[0]))]
    
    #convert list to dataframe and save to a .csv file
    df = pandas.DataFrame(np.array(data), )#columns=names)
    df.to_csv(file_name, header=False, index=False)




def read_file(file_name):
    
    file_path = ""
    path_df = pandas.read_csv(file_path + file_name, header=None)
    
    #Convert dataframe to list
    q = path_df.values.tolist()
    return q



def calc_speeds(ang_now, ang_next):
    #Calculate speeds so that ang_now -> ang_next take equal amount of time for all angles


    #if len(ang_now) != len(ang_next):
    #    print("ERROR, len(ang_now) != len(ang_next)")

    diff = []
    for i in range(0, len(ang_now)):
        diff.append(abs(ang_now[i] - ang_next[i]))
    
    max_diff = max(diff)

    max_speed = 15; #rpm
    
    #scale speeds
    speeds = []
    for i in range(0, len(ang_now)):
        if max_diff == 0:
            speeds.append(0)
        else:
            speeds.append(max_speed * diff[i] / max_diff)

    return speeds



def get_traj_between_points(start, end, steps):
    x = np.arange(2)
    #Cubic interpolation, first derivative = 0.0 at start and end
    cs = CubicSpline(x, [start, end], bc_type='clamped')    #bc_type=((1, 0.0), (1, 0.0)))
    return cs(np.linspace(0, 1, steps)).tolist()

def show_figure(x, y ):

    fig, ax = plt.subplots()

    ax.plot(x, y, 'o-')

    ax.set(xlabel='X', ylabel='Y',
       title='Figure')
    ax.grid()

    fig.savefig("test.png")
    plt.show()


def generate_traj(points):
    traj = []
    

    for i in range(0, len(points) - 1):
        p = get_traj_between_points(points[i], points[i+1], 20)
        #print("ppppp " + str(p))
        for t in p:
            traj.append(t)

    # print("   ")
    #print(traj)
    return traj



"""
path = [[0, 0, 0, 0, 0], [0, 0, 0, 0, -100], [0, 0, 0, 0, 100]]
save_file(path, "test.csv)
#save_file([[0, 0, 90, 0, 0], [45, 57, 51, 2, 0], [-45, 57, 51, 40, 100], [0, 0, 90, -40, 100]], "test.csv")
d = read_file("test.csv")
print(d)




traj = generate_traj(d)
#show_figure(np.linspace(0, 1, len(traj)), traj)
#print(traj)
save_file(traj, "test.csv")
"""

"""
fig, ax = plt.subplots()

ax.plot(np.linspace(0, 1, len(traj)), traj, '.-')
ax.plot(np.linspace(0, 1, len(d)), d, 'ro')


ax.set(xlabel='X', ylabel='Y',
   title='Figure')
ax.grid()

fig.savefig("test.png")
plt.show()
"""

















#y = get_traj_between_points([0, 0], [1, 2], 10)
#print(y([0, 0.5, 1]))
#s = calc_speeds([0, 0], [1, 2])
#print(s)

#x = np.linspace([0], [1], 10)
#print(y)
#show_figure(x, y)


