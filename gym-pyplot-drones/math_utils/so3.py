import numpy as np


def hat(v):
    x, y, z = np.asarray(v, float).reshape(3)
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def vee(M):
    M = np.asarray(M, float).reshape(3,3)
    return np.array([M[2,1],M[0,2],M[1,0]])


def project_to_so3(R):
    U, _, Vt = np.linalg.svd(np.asarray(R,float).reshape(3,3)); D=np.eye(3); D[2,2]=np.linalg.det(U@Vt)
    return U@D@Vt


def rotation_error(R_des, R):
    R_des,R=np.asarray(R_des,float).reshape(3,3),np.asarray(R,float).reshape(3,3)
    return .5*vee(R_des.T@R-R.T@R_des)


def matrix_to_euler(R):
    R=np.asarray(R,float).reshape(3,3)
    return np.array([np.arctan2(R[2,1],R[2,2]),np.arcsin(np.clip(-R[2,0],-1,1)),np.arctan2(R[1,0],R[0,0])])
