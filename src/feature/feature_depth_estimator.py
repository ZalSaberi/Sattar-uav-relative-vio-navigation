import numpy as np

class FeatureDepthEstimator:
    def generate_initial_guess(self, T_c1_c2, z1, z2):
        """
        Compute the initial guess of the feature's 3d position using 
        only two views.
        """
        m = T_c1_c2.R @ np.array([*z1, 1.0])
        a = m[:2] - z2*m[2]
        b = z2*T_c1_c2.t[2] - T_c1_c2.t[:2]
        denom = a @ a
        if (not np.all(np.isfinite(m)) or
            not np.all(np.isfinite(a)) or
            not np.all(np.isfinite(b)) or
            denom <= 1e-12):
            return None

        depth = a @ b / denom
        if not np.isfinite(depth) or depth <= 1e-6:
            return None
        p = np.array([*z1, 1.0]) * depth
        if not np.all(np.isfinite(p)):
            return None
        return p
