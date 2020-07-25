import copy
import time
import os
import re
import numpy as np
from os.path import join
import open3d as o3d
import teaserpp_python

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

NOISE_BOUND = 0.05
N_OUTLIERS = 1700
OUTLIER_TRANSLATION_LB = 5
OUTLIER_TRANSLATION_UB = 10


def get_angular_error(R_exp, R_est):
    """
    Calculate angular error
    """
    return abs(np.arccos(min(max(((np.matmul(R_exp.T, R_est)).trace() - 1) / 2, -1.0), 1.0)))


def extract_pc(content):

    timestamp = int(re.split('\s+', content)[0])
    num_point = (len(re.split('\s+', content)) - 2) / 3

    point = []
    for i in range(int(num_point)):
        x = float(re.split('\s+', content)[i * 3 + 1])
        y = float(re.split('\s+', content)[i * 3 + 2])
        z = float(re.split('\s+', content)[i * 3 + 3])
        point.append((x, y, z))
    point = np.array(point)

    return point


if __name__ == "__main__":

    with open('/Users/manmi/Documents/data/square_data/mm_data/mm_src_sample_corres.txt') as file:
        content_src = file.readlines()

    with open('/Users/manmi/Documents/data/square_data/mm_data/mm_dts_sample_corres.txt') as file:
        content_tgt = file.readlines()

    for i in range(len(content_tgt)):
    # for i in range(1):

        src = extract_pc(content_src[i])
        tgt = extract_pc(content_tgt[i])

        assert len(src) == len(tgt)

        src = np.transpose(src)
        dst = np.transpose(tgt)

        # Populating the parameters
        solver_params = teaserpp_python.RobustRegistrationSolver.Params()
        solver_params.cbar2 = 1
        solver_params.noise_bound = NOISE_BOUND
        solver_params.estimate_scaling = False
        solver_params.rotation_estimation_algorithm = teaserpp_python.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
        solver_params.rotation_gnc_factor = 1.4
        solver_params.rotation_max_iterations = 100
        solver_params.rotation_cost_threshold = 1e-12

        solver = teaserpp_python.RobustRegistrationSolver(solver_params)
        start = time.time()
        solver.solve(src, dst)
        end = time.time()

        solution = solver.getSolution()

        T = solution.translation
        R = solution.rotation

        with open('/Users/manmi/Documents/data/square_data/mm_data/mm_T_TEASTER.txt', 'a+') as myfile:
            myfile.write(str(T[0]) + ' ' + str(T[1]) + ' ' + str(T[2]) + '\n')

        with open('/Users/manmi/Documents/data/square_data/mm_data/mm_R_matrix_TEASER.txt', 'a+') as myfile:
            myfile.write(str(R[0, 0]) + ' ' + str(R[0, 1]) + ' ' + str(R[0, 2]) + ' ' +
                         str(R[1, 0]) + ' ' + str(R[1, 1]) + ' ' + str(R[1, 2]) + ' ' +
                         str(R[2, 0]) + ' ' + str(R[2, 1]) + ' ' + str(R[2, 2]) + '\n')
