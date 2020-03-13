import cv2
import numpy as np
np.set_printoptions(suppress=True)

from skimage.measure import ransac
# from skimage.transform import FundamentalMatrixTransform
from skimage.transform import EssentialMatrixTransform

# turn [[x,y]] -> [[x, y, 1]]
def add_ones(x):
    return np.concatenate([x, np.ones((x.shape[0], 1))], axis=1)

# Essential Matrix, Determining R ant t from E
# rotation & translation
def extractRt(E):
    W = np.mat([[0, -1, 0],[1, 0, 0],[0, 0, 1]],dtype=float)
    U, d, Vt = np.linalg.svd(E)
    assert np.linalg.det(U) > 0
    if np.linalg.det(Vt) < 0:
        Vt *= -1.0
    R = np.dot(np.dot(U, W), Vt)
    if np.sum(R.diagonal()) < 0:
        R = np.dot(np.dot(U, W.T), Vt)
    t = U[:, 2]
    Rt = np.concatenate([R,t.reshape(3, 1)], axis=1)
    return Rt

class Extractor(object):
    def __init__(self, K):
        # self.orb = cv2.ORB_create(nfeatures=1500, scoreType=cv2.ORB_FAST_SCORE)
        # self.orb = cv2.ORB_create(nfeatures=1500, scoreType=cv2.ORB_HARRIS_SCORE)
        self.orb = cv2.ORB_create()
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        # self.matcher = cv2.FlannBasedMatcher()
        self.last = None
        self.K = K
        self.Kinv = np.linalg.inv(self.K)

    def normalize(self, pts):
        return np.dot(self.Kinv, add_ones(pts).T).T[:, 0:2]

    def denormalize(self, pt):
        ret = np.dot(self.K, np.array([pt[0], pt[1], 1.0]))
        return int(round(ret[0])), int(round(ret[1]))
        # return np.dot(self.Kinv, [pt[0], pt[1], 1.0])

    def extract(self, img):
        # detection
        feats = cv2.goodFeaturesToTrack(np.mean(img, axis=2).astype(np.uint8), 3000, qualityLevel=0.01, minDistance=3)

        # extraction
        kps = [cv2.KeyPoint(x=f[0][0], y=f[0][1], _size=20) for f in feats]
        kps, des = self.orb.compute(img, kps)

        ret = []
        if self.last is not None:
            matches = self.bf.knnMatch(des, self.last['des'], k=2)
            for m,n in matches:
                if m.distance < 0.75*n.distance:
                    kp1 = kps[m.queryIdx].pt
                    kp2 = self.last['kps'][m.trainIdx].pt
                    ret.append((kp1, kp2))

        # filter
        Kt = None
        # govern how points correspond
        if len(ret) > 0:
            ret = np.array(ret)

            # normalize coords: subtract to move to 0
            # ret[:, :, 0] -= img.shape[0]//2
            # ret[:, :, 1] -= img.shape[1]//2
            ret[:, 0, :] = self.normalize(ret[:, 0, :])
            ret[:, 1, :] = self.normalize(ret[:, 1, :])

            model, inliers = ransac((ret[:, 0], ret[:, 1]),
                                    EssentialMatrixTransform,
                                    # FundamentalMatrixTransform,
                                    min_samples=8,
                                    # residual_threshold=1,
                                    residual_threshold=0.005,
                                    max_trials=200)

            ret = ret[inliers]
            Kt = extractRt(model.params)

            #s,v,d = np.linalg.svd(model.params)
            #print(v)

        self.last = {'kps': kps, 'des': des}
        return ret, Kt


