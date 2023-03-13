from typing import Tuple
import numpy as np
import random
from sklearn.neighbors import NearestNeighbors

def ransac_plane(P, threshold=0.05, iterations=1000):
  inliers=[]
  n_points=len(P)
  i=1
  while i<iterations:
    # generate random samples 
    idx_samples = random.sample(range(n_points), 3)
    pts = P[idx_samples]
    vecA = pts[1] - pts[0]
    vecB = pts[2] - pts[0]
    
    # get surface normal
    normal = np.cross(vecA, vecB)
    
    # get distance
    a,b,c = normal / np.linalg.norm(normal)
    d=-np.sum(normal*pts[1])
    distance = (a * P[:,0] + b * P[:,1] + c * P[:,2] + d) / np.sqrt(a ** 2 + b ** 2 + c ** 2)
    
    # get the inliners
    idx_candidates = np.where(np.abs(distance) <= threshold)[0]
    
    # try to find a plane containe the most points within threshold
    if len(idx_candidates) > len(inliers):
      equation = [a,b,c,d]
      inliers = idx_candidates
      
    i+=1
  return equation, inliers

def q1_a(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a least squares plane by taking the Eigen values and vectors
    of the sample covariance matrix

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    # find the centeroid of the points
    center = P.mean(axis=0)
    center = center.reshape(3,)
    # Calculate the covariance matrix of the points relative to the centroid
    cov_mat = np.cov(P,rowvar=False)
    # Calculate the smallest Eigenvector of the covariance matrix. This is the plane normal
    eigen_value, eigen_vector = np.linalg.eig(cov_mat)
    min_eval = np.argmin(eigen_value)
    normal_vector = eigen_vector[:, min_eval]
    return normal_vector, center

def q1_c(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Fit a plane using RANSAC

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space

    Returns
    -------
    normal : np.ndarray
        array of shape (3,) denoting surface normal of the fitting plane
    center : np.ndarray
        array of shape (3,) denoting center of the points
    '''
    # find the centeroid of the points
    center = P.mean(axis=0)
    center = center.reshape(3,)
    eq,idx_inliers = ransac_plane(P)
    x,y,z,d = eq
    print(eq)
    P = np.array([x,y,z])
    return P,center


def ransac_sphere(P, N, threshold=0.01, iterations=2000):
  inliers=[]
  n_points=len(P)
  i=1
  glob_center = np.array([0,0,0])
  glob_radius = 0
  while i<iterations:
    # sample a point 
    idx_sample = random.sample(range(n_points), 1)
    sample = P[idx_sample]
    # get surface normal
    sample_normal_estimated = N[idx_sample]
    # sample radius
    sample_radius = random.uniform(0.05, 0.11)
    # estimate center point
    # (radius / | surface normal | )  * surface normal
    center = sample - (sample_radius / np.linalg.norm(sample_normal_estimated))  * sample_normal_estimated
    
    a,b,c = center[0,0], center[0,1], center[0,2]
    distance = np.sqrt((P[:,0] - a) ** 2 + (P[:,1] - b) ** 2 + (P[:,2] - c) ** 2) - sample_radius
    idx_candidates = np.where(np.abs(distance) <= threshold)[0]
    
    # try to find a plane containe the most points within threshold
    if len(idx_candidates) > len(inliers):
      glob_center = center
      inliers = idx_candidates
      glob_radius = sample_radius
      
    i+=1
  print(glob_center, glob_radius, len(inliers), len(N))
  return glob_center, glob_radius

def q2(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, float]:
    '''
    Localize a sphere in the point cloud. Given a point cloud as
    input, this function should locate the position and radius
    of a sphere

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting sphere center
    radius : float
        scalar radius of sphere

    Hint
    ----
    use `utils.estimate_normals` to compute normals for point cloud
    '''
    center, radius = ransac_sphere(P,N)
    center = center.reshape(3,)
    return center, radius


def ransac_cylinder(P, N, threshold=0.004, iterations=500, inliner_threshold=10100):
  inliers=[]
  n_points=len(P)
  i=1
  glob_center = np.array([0,0,0])
  glob_radius = 0
  glob_axis = np.array([0,0,0])
  while i < iterations or len(inliers) < inliner_threshold:

    # (a) Sample a radius for the candidate cylinder between 5 and 10 cm.
    sample_radius = random.uniform(0.05, 0.10)
    # (b) Sample two points from the cloud
    idx_sample = random.sample(range(n_points), 2)
    sampleA = P[idx_sample[0]]
    sampleB = P[idx_sample[1]]
    # (c) Set the cylinder axis direction equal to the direction of the cross product between the surface normals
    # associated with the two sampled points.
    sample_normal_estimated_pointA = N[idx_sample[0]]
    sample_normal_estimated_pointB = N[idx_sample[1]]
    cylinder_axis = np.cross(sample_normal_estimated_pointA, sample_normal_estimated_pointB)
    # (d) Pick one of the sampled points from the cloud and use it to estimate a candidate center
    center = sampleA - (sample_radius / np.linalg.norm(sample_normal_estimated_pointA))  * sample_normal_estimated_pointA
    # (e) find distance by project the points onto the plane orthogonal to the axis
    projected_center = np.dot(center,(np.identity(len(cylinder_axis)) - np.outer(cylinder_axis, cylinder_axis.T)))
    projected_points = np.dot(P, (np.identity(len(cylinder_axis)) - np.outer(cylinder_axis, cylinder_axis.T)))
    
    # print(projected_points.shape, projected_center.shape)
    distance = np.sqrt((projected_points[:,0] - projected_center[0]) ** 2 + 
                       (projected_points[:,1] - projected_center[1]) ** 2 + 
                       (projected_points[:,2] - projected_center[2]) ** 2  ) - sample_radius
    
    # try to find a plane containe the most points within threshold
    idx_candidates = np.where(np.abs(distance) <= threshold)[0]
    if len(idx_candidates) > len(inliers):
      inliers = idx_candidates
      glob_center = center
    #   print(center)
      glob_radius = sample_radius
      glob_axis = cylinder_axis
      
    i+=1
  print("global center: ", glob_center, " cylinder axis: ", cylinder_axis, " estimated radius: ",glob_radius, " inliners: ",len(inliers))
  return glob_center, glob_axis, glob_radius

def q3(P: np.ndarray, N: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    '''
    Localize a cylinder in the point cloud. Given a point cloud as
    input, this function should locate the position, orientation,
    and radius of the cylinder

    Attributes
    ----------
    P : np.ndarray
        Nx3 matrix denoting 100 points in 3D space
    N : np.ndarray
        Nx3 matrix denoting normals of pointcloud

    Returns
    -------
    center : np.ndarray
        array of shape (3,) denoting cylinder center
    axis : np.ndarray
        array of shape (3,) pointing along cylinder axis
    radius : float
        scalar radius of cylinder
    '''

    center, axis, radius = ransac_cylinder(P, N)
    return center, axis, radius

def q4_a(M: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    '''
    Find transformation T such that D = T @ M. This assumes that M and D are
    corresponding (i.e. M[i] and D[i] correspond to same point)

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    use `np.linalg.svd` to perform singular value decomposition
    '''
    T,R,t = best_fit_transform(M,D)
    print("Q4: Transformation Matrix: ", T, "  distances: ", t)
    return T


def q4_c(M: np.ndarray, D: np.ndarray) -> np.ndarray:
    '''
    Solves iterative closest point (ICP) to generate transformation T to best
    align the points clouds: D = T @ M

    Attributes
    ----------
    M : np.ndarray
        Nx3 matrix of points
    D : np.ndarray
        Nx3 matrix of points

    Returns
    -------
    T : np.ndarray
        4x4 homogenous transformation matrix

    Hint
    ----
    you should make use of the function `q4_a`
    '''
    T, distance = icp(M,D)
    return T


def best_fit_transform(A, B):
    #Calculates the transform that maps corresponding points A to B in m spatial dimensions

    # get number of dimensions
    m = A.shape[1]

    # minimize the centeroid
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # get transformation matrix with SVD
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)
    
    # homogeneous transformation
    T = np.eye(m+1)
    T[:m, :m] = R
    T[:m, m] = t

    return T, R, t


def nearest_neighbor(src, dst):
    # find nearest neighbor 
    neigh = NearestNeighbors(n_neighbors=1)
    neigh.fit(dst)
    distances, indices = neigh.kneighbors(src, return_distance=True)
    return distances.ravel(), indices.ravel()


def icp(A, B, max_iterations=20, tolerance=0.0001):
    assert A.shape == B.shape
    # get number of dimensions
    m = A.shape[1]

    src = np.ones((m+1,A.shape[0]))
    dst = np.ones((m+1,B.shape[0]))
    src[:m,:] = np.copy(A.T)
    dst[:m,:] = np.copy(B.T)
    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbors between the current source and destination points
        distances, indices = nearest_neighbor(src[:m,:].T, dst[:m,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[:m,:].T, dst[:m,indices].T)

        # update the current source cloud points set
        src = np.dot(T, src)

        # check error and early stop if the difference is small enough
        mean_error = np.mean(distances)
        if np.abs(prev_error - mean_error) < tolerance:
            print("Error is small enough")
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[:m,:].T)

    return T, distances, 