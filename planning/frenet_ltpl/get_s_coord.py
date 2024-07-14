import numpy as np

def closest_path_index(path: np.ndarray, pos: tuple, prev_idx: int, n_closest: int=5) -> int:
    distances2 = np.power(path[:, 0] - pos[0], 2) + np.power(path[:, 1] - pos[1], 2)
    # Get indexes of all points
    valid_indices = np.where(np.arange(len(path)) >= prev_idx)[0]
    valid_distances2 = distances2[valid_indices]

    # Get the index of the closest point from the valid indices
    idx_array = np.argpartition(valid_distances2, n_closest)[:n_closest]
    closest_valid_idx = idx_array[np.argmin(valid_distances2[idx_array])]
    return closest_valid_idx
