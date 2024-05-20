import csv

def to_csv(file, path):
    with open(file, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'w_right', 'w_left', 'x_normvec', 'y_normvec', 'alpha', 's', 'psj', 'kappa', 'vx', 'ax'])
        for xy in path:
            writer.writerow(xy)

def to_txt(file, arr):
    with open(file, 'w') as file:
        for i, ar in enumerate(arr):
            file.write(f"{ar} ")
            if i+1 % 20 == 0 :
                file.write("\n")
                