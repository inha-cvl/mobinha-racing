import csv

def to_csv(file_path, trajectory_info):
    with open(file_path, 'w', newline='') as file:
        writer = csv.writer(file, delimiter=';')
        writer.writerow(['#x', 'y', 'w_right', 'w_left', 'x_normvec', 'y_normvec', 'alpha', 's', 'psj', 'kappa', 'vx', 'ax'])
        for tri in trajectory_info:
            writer.writerow(tri)
def to_txt(file, arr):
    with open(file, 'w') as file:
        for i, ar in enumerate(arr):
            file.write(f"{ar} ")
            if i+1 % 20 == 0 :
                file.write("\n")
                