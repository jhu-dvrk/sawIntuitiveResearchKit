import numpy as np
from numpy.linalg import inv

# copied from config-generator.py
mtm_a_max_current = np.array([0.67, 0.67, 0.67, 0.67, 0.59, 0.59, 0.407])
mtm_a_gearRatio = np.array([63.41, 49.88, 59.73, 10.53, 33.16, 33.16, 16.58])
mtm_a_motorTorque = np.array([0.0438, 0.0438, 0.0438, 0.0438, 0.00495, 0.00495, 0.00339])

# coupling matrix
mtm_a2j_p = np.matrix([[  1.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                       [  0.0000,  1.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                       [  0.0000, -1.0000,  1.0000,  0.0000,  0.0000,  0.0000,  0.0000],
                       [  0.0000,  0.6697, -0.6697,  1.0000,  0.0000,  0.0000,  0.0000],
                       [  0.0000,  0.0000,  0.0000,  0.0000,  1.0000,  0.0000,  0.0000],
                       [  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  1.0000,  0.0000],
                       [  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  1.0000]])

# start computations
mtm_a_nmToAmps = 1.0 / (mtm_a_gearRatio * mtm_a_motorTorque)
mtm_a_max_torque = mtm_a_max_current / mtm_a_nmToAmps

mtm_j2a_p = inv(mtm_a2j_p)
mtm_j_max_torque = mtm_j2a_p.T.dot(mtm_a_max_torque)

print('MTM max torques')
print(mtm_j_max_torque)




# copied from config-generator.py
psm_a_max_current = np.array([1.34, 1.34, 0.67, 0.67, 0.67, 0.67, 0.670])
psm_a_gearRatio = np.array([56.50, 56.50, 336.6, 11.71, 11.71, 11.71, 11.71])
psm_a_motorTorque = np.array([0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438, 0.0438])

# coupling matrix
psm_a2j_p = np.identity(7)

# start computations
psm_a_nmToAmps = 1.0 / (psm_a_gearRatio * psm_a_motorTorque)
psm_a_max_torque = psm_a_max_current / psm_a_nmToAmps

psm_j2a_p = inv(psm_a2j_p)
psm_j_max_torque = psm_j2a_p.T.dot(psm_a_max_torque)

print('PSM max torques')
print(psm_j_max_torque)





# copied from config-generator.py
ecm_a_max_current = np.array([0.943, 0.943, 0.67, 0.59])
ecm_a_gearRatio = np.array([240, 240, 2748.55, 300.15])
ecm_a_motorTorque = np.array([0.1190, 0.1190, 0.0438, 0.00495])

# coupling matrix
ecm_a2j_p = np.identity(4)

# start computations
ecm_a_nmToAmps = 1.0 / (ecm_a_gearRatio * ecm_a_motorTorque)
ecm_a_max_torque = ecm_a_max_current / ecm_a_nmToAmps

ecm_j2a_p = inv(ecm_a2j_p)
ecm_j_max_torque = ecm_j2a_p.T.dot(ecm_a_max_torque)

print('ECM max torques')
print(ecm_j_max_torque)

