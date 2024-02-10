from __future__ import print_function
import numpy as np
from numpy.linalg import norm, solve
import pinocchio

def perform_inverse_kinematics(translation_input):
    try:
        # Parse input coordinates
        x, y, z = map(float, translation_input.split(','))
    except ValueError:
        print("Invalid input format. Please provide coordinates in the format 'x, y, z'.")
        return None
    
    # Check if the norm of the translation vector is greater than 1m
    if x>1 or y>1 or z>1:
        print("Translation coordinates must lie inside a 1m hemisphere.")
        return None

    # Load URDF file and create model
    urdf_file = "urdf/arm.urdf"
    model = pinocchio.buildModelFromUrdf(urdf_file)
    data = model.createData()

    # Define end-effector position
    oMdes = pinocchio.SE3(np.eye(3), np.array([x, y, z]))
    
    # Define parameters for IK
    q = pinocchio.neutral(model)
    eps = 1e-4
    IT_MAX = 1000
    DT = 1e-1
    damp = 1e-12
    JOINT_ID = 6  # end-effector is attached to the 7th joint
    
    i = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[JOINT_ID].actInv(oMdes)
        err = pinocchio.log(iMd).vector  # in joint frame
        
        # Check convergence
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        
        # Compute Jacobian and update joint configuration
        J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        v = - J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(model, q, v*DT)
        
        # Print error for every 10 iterations
        if not i % 10:
            print('%d: error = %s' % (i, err.T))
        i += 1
    
    # Print convergence status
    if success:
        print("Convergence achieved!")
    else:
        print("\nWarning: the iterative algorithm has not reached convergence to the desired precision")
    
    # Print joint configuration and final error
    print('\nresult: %s' % q.flatten().tolist())
    print('\nfinal error: %s' % err.T)
    
    return q.flatten().tolist()

# Example usage:
while True:
    translation_input = input("Tell end coordinate like(1.0, 1.0, 1.0):")
    joint_angles = perform_inverse_kinematics(translation_input)
    if joint_angles is not None:
        print("Joint angles:", joint_angles)
        break

