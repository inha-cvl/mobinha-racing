import numpy as np
import math


def isclose(a, b, rel_tol=1e-9, abs_tol=0.0):
    return abs(a - b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

def diff_2d(array):
    result = np.empty((array.shape[0] - 1, array.shape[1]), dtype=array.dtype)
    for i in range(array.shape[0] - 1):
        for j in range(array.shape[1]):
            result[i, j] = array[i + 1, j] - array[i, j]
    return result

def calc_splines_optimized(path, el_lengths=None, psi_s=None, psi_e=None, use_dist_scaling=True):
    closed = np.all([isclose(path[0, i], path[-1, i]) for i in range(path.shape[1])])

    if not closed and (psi_s is None or psi_e is None):
        raise ValueError("Headings must be provided for unclosed spline calculation!")

    if el_lengths is not None and path.shape[0] != el_lengths.size + 1:
        raise ValueError("el_lengths input must be one element smaller than path input!")

    if use_dist_scaling and el_lengths is None:
        diffs = diff_2d(path)
        el_lengths = np.sqrt(np.sum(diffs**2, axis=1))
    elif el_lengths is not None:
        el_lengths = np.copy(el_lengths)

    if use_dist_scaling and closed:
        el_lengths = np.append(el_lengths, el_lengths[0])

    no_splines = path.shape[0] - 1

    if use_dist_scaling:
        scaling = el_lengths[:-1] / el_lengths[1:]
        scaling_squared = scaling**2
    else:
        scaling = np.ones(no_splines - 1)
        scaling_squared = scaling

    scaling_factors = np.vstack((scaling, scaling_squared)).T

    M = np.zeros((no_splines * 4, no_splines * 4))
    b_x = np.zeros(no_splines * 4)
    b_y = np.zeros(no_splines * 4)

    template_M = np.array([[1,  0,  0,  0,  0,  0,  0,  0],
                           [1,  1,  1,  1,  0,  0,  0,  0],
                           [0,  1,  2,  3,  0, -1,  0,  0],
                           [0,  0,  2,  6,  0,  0, -2,  0]])

    for i in range(no_splines):
        j = i * 4
        if i < no_splines - 1:
            M[j: j + 4, j: j + 8] = template_M
            M[j + 2, j + 5] *= scaling_factors[i, 0]
            M[j + 3, j + 6] *= scaling_factors[i, 1]
        else:
            M[j: j + 2, j: j + 4] = np.array([[1,  0,  0,  0],
                                              [1,  1,  1,  1]])

        b_x[j:j + 2] = path[i:i + 2, 0]
        b_y[j:j + 2] = path[i:i + 2, 1]

    if not closed:
        M[-2, 1] = 1
        el_length_s = 1.0 if el_lengths is None else el_lengths[0]
        b_x[-2] = math.cos(psi_s + math.pi / 2) * el_length_s
        b_y[-2] = math.sin(psi_s + math.pi / 2) * el_length_s

        M[-1, -4:] = np.array([0, 1, 2, 3])
        el_length_e = 1.0 if el_lengths is None else el_lengths[-1]
        b_x[-1] = math.cos(psi_e + math.pi / 2) * el_length_e
        b_y[-1] = math.sin(psi_e + math.pi / 2) * el_length_e
    else:
        M[-2, 1] = scaling[-1]
        M[-2, -3:] = np.array([-1, -2, -3])
        M[-1, 2] = 2 * scaling_squared[-1]
        M[-1, -2:] = np.array([-2, -6])

    x_les = np.linalg.solve(M, b_x)
    y_les = np.linalg.solve(M, b_y)

    coeffs_x = x_les.reshape((no_splines, 4))
    coeffs_y = y_les.reshape((no_splines, 4))

    normvec = np.column_stack((coeffs_y[:, 1], -coeffs_x[:, 1]))
    norm_factors = 1.0 / np.sqrt(np.sum(normvec**2, axis=1))
    normvec_normalized = normvec * norm_factors[:, np.newaxis]

    return coeffs_x, coeffs_y, M, normvec_normalized

def calc_linear_splines(path: np.ndarray,
                        el_lengths: np.ndarray = None,
                        psi_s: float = None,
                        psi_e: float = None,
                        use_dist_scaling: bool = True) -> tuple:
    """
    Simplified linear interpolation between given points.
    This function does not ensure smoothness but significantly reduces computation time.

    :param path:                x and y coordinates as the basis for the spline construction (closed or unclosed).
    :type path:                 np.ndarray
    :param el_lengths:          distances between path points (optional, not used in linear interpolation).
    :type el_lengths:           np.ndarray
    :param psi_s:               orientation of the start point (optional, not used in linear interpolation).
    :type psi_s:                float
    :param psi_e:               orientation of the end point (optional, not used in linear interpolation).
    :type psi_e:                float
    :param use_dist_scaling:    flag to indicate if heading and curvature scaling should be performed (not used here).
    :type use_dist_scaling:     bool

    :return x_coeff: linear interpolation coefficients of the x-component.
    :rtype x_coeff: np.ndarray
    :return y_coeff: linear interpolation coefficients of the y-component.
    :rtype y_coeff: np.ndarray
    :return M: linear equation system matrix (identity matrix for simplicity).
    :rtype M: np.ndarray
    :return normvec_normalized: normalized normal vectors [x, y].
    :rtype normvec_normalized:  np.ndarray
    """

    # Number of segments
    no_segments = path.shape[0] - 1

    # Initialize coefficients arrays
    x_coeff = np.zeros((no_segments, 4))  # Only two coefficients needed, but keeping shape for compatibility
    y_coeff = np.zeros((no_segments, 4))  # Only two coefficients needed, but keeping shape for compatibility

    for i in range(no_segments):
        # Linear interpolation coefficients (keeping a_2, a_3 as 0 for compatibility)
        x_coeff[i] = [path[i, 0], path[i + 1, 0] - path[i, 0], 0, 0]
        y_coeff[i] = [path[i, 1], path[i + 1, 1] - path[i, 1], 0, 0]

    # For compatibility, creating an identity matrix for M
    M = np.eye(no_segments * 4)

    # Normal vectors (not really applicable for linear, but we'll keep them for compatibility)
    normvec = np.stack((y_coeff[:, 1], -x_coeff[:, 1]), axis=1)
    norm_factors = 1.0 / np.sqrt(np.sum(normvec**2, axis=1))
    normvec_normalized = normvec * norm_factors[:, np.newaxis]

    return x_coeff, y_coeff, M, normvec_normalized


