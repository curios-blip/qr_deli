import math, cv2, numpy as np

CODE_FRACTION = 21 / 29      # QR code area as a fraction of the whole sign image (version 1 + 4-module border)

def estimate(corners, code_m, f, cx, cy):
    """corners: 4 image points TL,TR,BR,BL of the code.
    Returns (distance m, horizontal viewing angle deg, bearing deg), or None."""
    s = code_m / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], np.float32)
    pts = np.asarray(corners, np.float32).reshape(4, 2)
    for flag in (cv2.SOLVEPNP_IPPE_SQUARE, cv2.SOLVEPNP_ITERATIVE):   # IPPE can give NaN for an exactly head-on sign
        ok, rvec, tvec = cv2.solvePnP(obj, pts, K, None, flags=flag)
        if ok and np.isfinite(rvec).all() and np.isfinite(tvec).all():
            break
    else:
        return None
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.ravel()
    n = R[:, 2]                                   # sign normal in camera frame
    if n @ t > 0:                                 # make it point from the sign towards the camera
        n = -n
    dist = float(np.linalg.norm(t))
    to_cam = math.atan2(-t[0], -t[2])             # horizontal direction sign -> camera
    normal = math.atan2(n[0], n[2])               # horizontal direction of the sign normal
    d = (normal - to_cam + math.pi) % (2 * math.pi) - math.pi
    return dist, abs(math.degrees(d)), math.degrees(math.atan2(t[0], t[2]))
