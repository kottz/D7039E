import niklas

file_name = "pickup.csv"

#path = [[0, 0, 0, 0, 0], [0, 0, 0, 0, -20], [0, 0, 0, 0, 100], [-60, -40, -40, -40, 100], [0, 0, 0, 0, 100], [60, 40,40,40, 100], [0, 0, 90, 00, 100], [0, 0, 60, 00, 100], [0, 0, 120, 00, 100], [0, 0, 60, 00, 100], [0, 0, 120, 00, 100]]
#path = [[0, 0, 90, 0, 90], [0, 50, 55, 7, 90], [0, 50, 55, 7, 0], [0, 0, 55, 7, 0], [0, 0, 90, 0, 0]]
#path = [[0, 0, 90, 0, 50]]
path = [[0, 0, 90, 0, 0], [0, 50, 55, 7, 0], [0, 50, 55, 7, 90], [0, 0, 55, 7, 90], [0, 0, 90, 0, 90]]



niklas.save_file(path, file_name)
d = niklas.read_file(file_name)
print(d)




traj = niklas.generate_traj(d)
#show_figure(np.linspace(0, 1, len(traj)), traj)
#print(traj)
niklas.save_file(traj, file_name)
