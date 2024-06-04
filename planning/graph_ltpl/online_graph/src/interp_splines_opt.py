import numpy as np
import math
import trajectory_planning_helpers.calc_spline_lengths

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

