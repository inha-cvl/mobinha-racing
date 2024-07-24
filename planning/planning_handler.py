import numpy as np
import math

import trajectory_planning_helpers.calc_spline_lengths


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

def interp_splines(coeffs_x: np.ndarray,
                   coeffs_y: np.ndarray,
                   spline_lengths: np.ndarray = None,
                   incl_last_point: bool = False,
                   stepsize_approx: float = None,
                   stepnum_fixed: list = None) -> tuple:
    """
    3차 스플라인에서 포인트를 보간합니다. 경로의 마지막 포인트를 포함할지 여부와 보간할 포인트의
    스텝 사이즈를 지정할 수 있습니다.
    """

    # 입력 체크 -----------------------------------------------------------------------------------
    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise ValueError("계수 행렬의 길이가 같아야 합니다!")

    if spline_lengths is not None and coeffs_x.shape[0] != spline_lengths.size:
        raise ValueError("coeffs_x/y와 spline_lengths의 길이가 같아야 합니다!")

    if not (coeffs_x.ndim == 2 and coeffs_y.ndim == 2):
        raise ValueError("계수 행렬은 2차원이여야 합니다!")

    if (stepsize_approx is None and stepnum_fixed is None) or (stepsize_approx is not None and stepnum_fixed is not None):
        raise ValueError("stepsize_approx 또는 stepnum_fixed 중 하나만 제공해야 합니다!")

    if stepnum_fixed is not None and len(stepnum_fixed) != coeffs_x.shape[0]:
        raise ValueError("stepnum_fixed 리스트는 각 스플라인에 대해 항목을 가져야 합니다!")

    # 보간 포인트 개수 및 거리 계산 ------------------------------------------------------------------
    if stepsize_approx is not None:
        if spline_lengths is None:
            spline_lengths = trajectory_planning_helpers.calc_spline_lengths.calc_spline_lengths(coeffs_x=coeffs_x,
                                                                                                 coeffs_y=coeffs_y,
                                                                                                 quickndirty=False)
        dists_cum = np.cumsum(spline_lengths)
        total_length = dists_cum[-1]
        no_interp_points = math.ceil(total_length / stepsize_approx) + 1
        dists_interp = np.linspace(0.0, total_length, no_interp_points)
    else:
        no_interp_points = sum(stepnum_fixed) - (len(stepnum_fixed) - 1)
        dists_interp = None

    # 값을 저장할 배열 생성 ----------------------------------------------------------------------------
    path_interp = np.zeros((no_interp_points, 2))
    spline_inds = np.zeros(no_interp_points, dtype=np.int32)
    t_values = np.zeros(no_interp_points)

    if stepsize_approx is not None:
        dists_interp_iter = iter(dists_interp)
        dist = next(dists_interp_iter)
        for i in range(no_interp_points - 1):
            j = np.searchsorted(dists_cum, dist, side='right')
            spline_inds[i] = j
            t_values[i] = (dist - (dists_cum[j-1] if j > 0 else 0)) / spline_lengths[j]
            t_val = t_values[i]
            path_interp[i] = np.polyval(coeffs_x[j][::-1], t_val), np.polyval(coeffs_y[j][::-1], t_val)
            dist = next(dists_interp_iter)
    else:
        index = 0
        for i, steps in enumerate(stepnum_fixed):
            t_vals = np.linspace(0, 1, steps, endpoint=(i == len(stepnum_fixed) - 1))
            size = len(t_vals)
            t_values[index:index + size] = t_vals
            spline_inds[index:index + size] = i
            index += size

        t_set = np.column_stack((np.ones(no_interp_points), t_values, t_values**2, t_values**3))
        path_interp[:, 0] = np.einsum('ij,ij->i', np.repeat(coeffs_x, stepnum_fixed, axis=0), t_set)
        path_interp[:, 1] = np.einsum('ij,ij->i', np.repeat(coeffs_y, stepnum_fixed, axis=0), t_set)

    if incl_last_point:
        path_interp[-1] = np.sum(coeffs_x[-1]), np.sum(coeffs_y[-1])
        spline_inds[-1] = coeffs_x.shape[0] - 1
        t_values[-1] = 1.0
    else:
        path_interp = path_interp[:-1]
        spline_inds = spline_inds[:-1]
        t_values = t_values[:-1]
        if dists_interp is not None:
            dists_interp = dists_interp[:-1]

    return path_interp, spline_inds, t_values, dists_interp



def interp_linear_splines(coeffs_x: np.ndarray,
                          coeffs_y: np.ndarray,
                          spline_lengths: np.ndarray = None,
                          incl_last_point: bool = False,
                          stepsize_approx: float = None,
                          stepnum_fixed: list = None) -> tuple:
    """
    선형 스플라인을 보간하여 경로 점들을 생성하는 함수입니다.
    """

    # ------------------------------------------------------------------------------------------------------------------
    # INPUT CHECKS -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if coeffs_x.shape[0] != coeffs_y.shape[0]:
        raise ValueError("Coefficient matrices must have the same length!")

    if spline_lengths is not None and coeffs_x.shape[0] != spline_lengths.size:
        raise ValueError("coeffs_x/y and spline_lengths must have the same length!")

    if not (coeffs_x.ndim == 2 and coeffs_y.ndim == 2):
        raise ValueError("Coefficient matrices do not have two dimensions!")

    if (stepsize_approx is None and stepnum_fixed is None) or (stepsize_approx is not None and stepnum_fixed is not None):
        raise ValueError("Provide one of 'stepsize_approx' and 'stepnum_fixed' and set the other to 'None'!")
    
    if stepnum_fixed is not None and len(stepnum_fixed) != coeffs_x.shape[0]:
        raise ValueError("The provided list 'stepnum_fixed' must hold an entry for every spline!")

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE NUMBER OF INTERPOLATION POINTS AND ACCORDING DISTANCES -------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if stepsize_approx is not None:
        if spline_lengths is None:
            spline_lengths = np.sqrt(np.sum((coeffs_x[:, 1] - coeffs_x[:, 0])**2 + (coeffs_y[:, 1] - coeffs_y[:, 0])**2, axis=1))

        dists_cum = np.cumsum(spline_lengths)
        no_interp_points = int(np.floor(dists_cum[-1] / stepsize_approx)) + 1
        dists_interp = np.linspace(0.0, dists_cum[-1], no_interp_points)
    else:
        no_interp_points = sum(stepnum_fixed)
        dists_interp = None

    # ------------------------------------------------------------------------------------------------------------------
    # CALCULATE INTERMEDIATE STEPS -------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    path_interp = np.zeros((no_interp_points, 2))
    spline_inds = np.zeros(no_interp_points, dtype=int)
    t_values = np.zeros(no_interp_points)

    if stepsize_approx is not None:
        for i in range(no_interp_points):
            j = np.searchsorted(dists_cum, dists_interp[i], side='right')
            spline_inds[i] = j

            if j > 0:
                t_values[i] = (dists_interp[i] - dists_cum[j - 1]) / spline_lengths[j]
            else:
                t_values[i] = dists_interp[i] / spline_lengths[0]

            path_interp[i, 0] = (1 - t_values[i]) * coeffs_x[j, 0] + t_values[i] * coeffs_x[j, 1]
            path_interp[i, 1] = (1 - t_values[i]) * coeffs_y[j, 0] + t_values[i] * coeffs_y[j, 1]
    else:
        index = 0
        for i in range(len(stepnum_fixed)):
            t_vals = np.linspace(0, 1, stepnum_fixed[i], endpoint=(i == len(stepnum_fixed) - 1))
            path_interp[index:index + len(t_vals), 0] = (1 - t_vals) * coeffs_x[i, 0] + t_vals * coeffs_x[i, 1]
            path_interp[index:index + len(t_vals), 1] = (1 - t_vals) * coeffs_y[i, 0] + t_vals * coeffs_y[i, 1]
            spline_inds[index:index + len(t_vals)] = i
            t_values[index:index + len(t_vals)] = t_vals
            index += len(t_vals)

    if not incl_last_point:
        path_interp = path_interp[:-1]
        spline_inds = spline_inds[:-1]
        t_values = t_values[:-1]

        if dists_interp is not None:
            dists_interp = dists_interp[:-1]

    return path_interp, spline_inds, t_values, dists_interp

