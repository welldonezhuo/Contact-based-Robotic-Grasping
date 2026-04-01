import numpy as np
import itertools as it
import scipy.spatial
import utils
import time


########## Task 1: Primitive Wrenches ##########

def primitive_wrenches(mesh, grasp, mu=0.2, n_edges=8):
    """
    Find the primitive wrenches for each contact of a grasp.
    args:   mesh: The object mesh model.
                  Type: trimesh.base.Trimesh
           grasp: The indices of the mesh triangles being contacted.
                  Type: list of int
              mu: The friction coefficient of the mesh surface.
                  (default: 0.2)
         n_edges: The number of edges of the friction polyhedral cone.
                  Type: int (default: 8)
    returns:   W: The primitive wrenches.
                  Type: numpy.ndarray of shape (len(grasp) * n_edges, 6)
    """
    ########## TODO ##########

    W = np.zeros((len(grasp) * n_edges, 6))

    # Contact points are the centroids of the contacted mesh triangles.
    contact_pts = utils.get_centroid_of_triangles(mesh, grasp)

    # Use the outward face normals at the contacted triangles.
    normals = mesh.face_normals[grasp]

    for i, (p, n) in enumerate(zip(contact_pts, normals)):
        # Normalize the normal vector.
        n = n / np.linalg.norm(n)

        # Build a local tangent basis {t1, t2} orthogonal to n.
        ref = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(ref, n)) > 0.9:
            ref = np.array([0.0, 1.0, 0.0])

        t1 = np.cross(n, ref)
        t1 = t1 / np.linalg.norm(t1)
        t2 = np.cross(n, t1)
        t2 = t2 / np.linalg.norm(t2)

        for k in range(n_edges):
            # Uniformly discretize the friction cone into n_edges directions.
            theta = 2.0 * np.pi * k / n_edges
            tangent = np.cos(theta) * t1 + np.sin(theta) * t2

            # Compute the primitive contact force.
            f = n + mu * tangent

            # Compute the corresponding torque about the mesh center of mass.
            tau = np.cross(p - mesh.center_mass, f)

            # Save the 6D wrench [force, torque].
            row = i * n_edges + k
            W[row, :3] = f
            W[row, 3:] = tau

    ##########################
    return W


########## Task 2: Grasp Quality Evaluation ##########

def eval_Q(mesh, grasp, mu=0.2, n_edges=8, lmbd=0.3):
    """
    Evaluate the L1 quality of a grasp.
    args:   mesh: The object mesh model.
                  Type: trimesh.base.Trimesh
           grasp: The indices of the mesh triangles being contacted.
                  Type: list of int
              mu: The friction coefficient of the mesh surface.
                  (default: 0.2)
         n_edges: The number of edges of the friction polyhedral cone.
                  Type: int (default: 8)
            lmbd: The scale of torque magnitude.
                  (default: 1.0)
    returns:   Q: The L1 quality score of the given grasp.
    """
    ########## TODO ##########

    if not hasattr(eval_Q, "count"):
        eval_Q.count = 0
    eval_Q.count += 1

    Q = -np.inf

    # get primitive wrenches from Task 1
    W = primitive_wrenches(mesh, grasp, mu=mu, n_edges=n_edges).copy()

    # scale the torque part in the wrench space
    W[:, 3:] *= lmbd

    # construct the convex hull of primitive wrenches
    hull = scipy.spatial.ConvexHull(W)

    # each hyperplane equation is [a, b], representing a^T x + b = 0
    # the signed distance from the origin to this hyperplane is -b / ||a||
    Q = np.inf
    for eq in hull.equations:
        a = eq[:-1]
        b = eq[-1]
        dist = -b / np.linalg.norm(a)
        Q = min(Q, dist)

    ##########################
    return Q


########## Task 3: Stable Grasp Sampling ##########

def sample_stable_grasp(mesh, thresh=0.0):
    """
    Sample a stable grasp such that its L1 quality is larger than a threshold.
    args:     mesh: The object mesh model.
                    Type: trimesh.base.Trimesh
            thresh: The threshold for stable grasp.
                    (default: 0.0)
    returns: grasp: The stable grasp represented by the indices of triangles.
                    Type: list of int
                 Q: The L1 quality score of the sampled grasp, 
                    expected to be larger than thresh.
    """
    ########## TODO ##########
    n_faces = len(mesh.faces)
    max_tries = 10000

    best_grasp = None
    best_Q = -np.inf

    for i in range(max_tries):
        # randomly sample a 3-contact grasp without replacement
        grasp = np.random.choice(n_faces, size=3, replace=False).tolist()

        # evaluate the grasp quality
        Q = eval_Q(mesh, grasp)

        # keep the best sampled grasp so far
        if Q > best_Q:
            best_Q = Q
            best_grasp = grasp

        # return immediately once a stable grasp is found
        if Q > thresh:
            return grasp, Q

    # if no stable grasp is found within max_tries,
    # return the best sampled grasp
    return best_grasp, best_Q

    ##########################
    return grasp, Q


########## Task 4: Grasp Optimization ##########

def find_neighbors(mesh, tr_id, eta=1):
    """
    Find the eta-order neighbor faces (triangles) of tr_id on the mesh model.
    args:       mesh: The object mesh model.
                      Type: trimesh.base.Trimesh
               tr_id: The index of the query face (triangle).
                      Type: int
                 eta: The maximum order of the neighbor faces:
                      Type: int
    returns: nbr_ids: The list of the indices of the neighbor faces.
                      Type: list of int
    """
    ########## TODO ##########
    nbr_ids = []

    # build adjacency list once and cache it on the function
    if not hasattr(find_neighbors, "adj_map"):
        adj_map = {}
        for a, b in mesh.face_adjacency:
            if a not in adj_map:
                adj_map[a] = []
            if b not in adj_map:
                adj_map[b] = []
            adj_map[a].append(b)
            adj_map[b].append(a)
        find_neighbors.adj_map = adj_map

    adj_map = find_neighbors.adj_map

    visited = set([tr_id])
    current = set([tr_id])

    for _ in range(eta):
        next_level = set()
        for f in current:
            for nb in adj_map.get(f, []):
                if nb not in visited:
                    next_level.add(nb)
        visited.update(next_level)
        current = next_level

    visited.discard(tr_id)
    nbr_ids = list(visited)

    #####################
    return nbr_ids


def local_optimal(mesh, grasp):
    """
    Find the optimal neighbor grasp of the given grasp.
    args:     mesh: The object mesh model.
                    Type: trimesh.base.Trimesh
             grasp: The indices of the mesh triangles being contacted.
                    Type: list of int
    returns: G_opt: The optimal neighbor grasp with the highest quality.
                    Type: list of int
             Q_max: The L1 quality score of G_opt.
    """

    ########## TODO ##########
    G_opt = grasp
    Q_max = eval_Q(mesh, grasp)

    # cache for eval_Q results
    if not hasattr(local_optimal, "q_cache"):
        local_optimal.q_cache = {}

    neighbor_lists = []

    for g in grasp:
        nbrs = find_neighbors(mesh, g, eta=1)
        nbrs.append(g)
        neighbor_lists.append(nbrs)

    for candidate in it.product(*neighbor_lists):
        candidate = tuple(candidate)

        if len(set(candidate)) < len(candidate):
            continue

        # use cache
        if candidate in local_optimal.q_cache:
            Q = local_optimal.q_cache[candidate]
        else:
            Q = eval_Q(mesh, list(candidate))
            local_optimal.q_cache[candidate] = Q

        if Q > Q_max:
            Q_max = Q
            G_opt = list(candidate)

    ##########################
    return G_opt, Q_max


def optimize_grasp(mesh, grasp):
    """
    Optimize the given grasp and return the trajectory.
    args:    mesh: The object mesh model.
                   Type: trimesh.base.Trimesh
            grasp: The indices of the mesh triangles being contacted.
                   Type: list of int
    returns: traj: The trajectory of the grasp optimization.
                   Type: list of grasp (each grasp is a list of int)
    """
    traj = []
    ########## TODO ##########

    traj = [grasp]

    current_grasp = grasp
    current_Q = eval_Q(mesh, current_grasp)

    while True:
        new_grasp, new_Q = local_optimal(mesh, current_grasp)

        if new_Q <= current_Q:
            break

        traj.append(new_grasp)
        current_grasp = new_grasp
        current_Q = new_Q

    ##########################
    return traj


########## Task 5: Grasp Optimization with Reachability ##########

def optimize_reachable_grasp(mesh, r=0.5):
    """
    Sample a reachable grasp and optimize it.
    args:    mesh: The object mesh model.
                   Type: trimesh.base.Trimesh
                r: The reachability measure. (default: 0.5)
    returns: traj: The trajectory of the grasp optimization.
                   Type: list of grasp (each grasp is a list of int) 
    """
    traj = []
    ########## TODO ##########

    n_faces = len(mesh.faces)
    centroids = utils.get_centroid_of_triangles(mesh, list(range(n_faces)))

    def reachable(grasp):
        pts = centroids[list(grasp)]
        psi = np.mean(pts, axis=0)
        avg_dist = np.mean(np.linalg.norm(pts - psi, axis=1))
        # delete later if you want
        print("grasp:", grasp, "avg_dist:", avg_dist)
        return avg_dist < r

    # Step 1: sample an initial reachable grasp
    max_tries = 10000
    best_grasp = None
    best_Q = -np.inf

    for _ in range(max_tries):
        grasp = np.random.choice(n_faces, size=3, replace=False).tolist()

        if not reachable(grasp):
            continue

        Q = eval_Q(mesh, grasp)

        if Q > best_Q:
            best_grasp = grasp
            best_Q = Q

        if Q > 0.0:
            break

    # fallback: if no reachable grasp was accepted above
    if best_grasp is None:
        while True:
            grasp = np.random.choice(n_faces, size=3, replace=False).tolist()
            if reachable(grasp):
                best_grasp = grasp
                best_Q = eval_Q(mesh, grasp)
                break

    # Step 2: optimize while keeping only reachable neighbors
    traj = [best_grasp]
    current_grasp = best_grasp
    current_Q = best_Q
    q_cache = {tuple(current_grasp): current_Q}

    while True:
        neighbor_lists = []
        for g in current_grasp:
            nbrs = find_neighbors(mesh, g, eta=1)
            nbrs.append(g)
            neighbor_lists.append(nbrs)

        next_grasp = current_grasp
        next_Q = current_Q

        for candidate in it.product(*neighbor_lists):
            if len(set(candidate)) < len(candidate):
                continue

            if not reachable(candidate):
                continue

            candidate_key = tuple(candidate)
            if candidate_key in q_cache:
                Q = q_cache[candidate_key]
            else:
                Q = eval_Q(mesh, list(candidate))
                q_cache[candidate_key] = Q

            if Q > next_Q:
                next_Q = Q
                next_grasp = list(candidate)

        if next_Q <= current_Q:
            break

        traj.append(next_grasp)
        current_grasp = next_grasp
        current_Q = next_Q

    ##########################

    return traj
