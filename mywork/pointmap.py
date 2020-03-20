import numpy as np
import OpenGL.GL as gl
import pypangolin as pangolin

from multiprocessing import Process, Queue


class Point(object):
    # A Point is a 3-D point in the world
    # Each Point is observed in multiple Frames

    def __init__(self, mapp, loc):
        self.pt = loc
        self.frames = []
        self.idxs = []

        self.id = len(mapp.points)
        mapp.points.append(self)

    def add_observation(self, frame, idx):
        self.frames.append(frame)
        self.idxs.append(idx)


class Map(object):
    def __init__(self):
        self.frames = []
        self.points = []
        self.state = None
        self.q = Queue()
        self.vp = Process(target=self.viewer_thread, args=(self.q,))
        self.vp.daemon = True
        self.vp.start()

    def viewer_thread(self, q):
        self.viewer_init(1024, 768)
        while 1:
            self.viewer_refresh(q)

    def viewer_init(self, w, h):
        pangolin.CreateWindowAndBind('Main', w, h)
        gl.glEnable(gl.GL_DEPTH_TEST)

        self.scam = pangolin.OpenGlRenderState(
            pangolin.ProjectionMatrix(w, h, 420, 420, w // 2, h // 2, 0.2, 1000),
            pangolin.ModelViewLookAt(0, -10, -8,
                                     0, 0, 0,
                                     0, -1, 0))
        self.handler = pangolin.Handler3D(self.scam)

        # Create Interactive View in window
        self.dcam = pangolin.CreateDisplay()
        self.dcam.SetBounds(
            pangolin.Attach(0),
            pangolin.Attach(1),
            pangolin.Attach(0),
            pangolin.Attach(1),
            -w/h
        )
        self.dcam.SetHandler(self.handler)

        # hack to avoid small Pangolin, no idea why it's *2
        # self.dcam.Resize(pangolin.Viewport(0, 0, w * 2, h * 2))
        # self.dcam.Activate()

    def viewer_refresh(self, q):
        if self.state is None or not q.empty():
        # while not q.empty():
            self.state = q.get()

        # turn state into points
        ppts = np.array([d[:3, 3] for d in self.state[0]])
        # spts = np.array(self.state[1])

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)
        self.dcam.Activate(self.scam)

        if self.state is not None:
            if self.state[0].shape[0] >= 2:
                # draw poses
                gl.glColor3f(0.0, 1.0, 0.0)
                # pangolin.DrawCameras(self.state[0][:-1])
                drawPoints(ppts[:-1])
                # drawCameras(self.state[0])

            if self.state[0].shape[0] >= 1:
                # draw current pose as yellow
                gl.glColor3f(1.0, 1.0, 0.0)
                drawPoints(ppts[-1:])
                # pangolin.DrawCameras(self.state[0][-1:])

            if self.state[1].shape[0] != 0:
                # draw keypoints
                gl.glPointSize(5)
                gl.glColor3f(1.0, 0.0, 0.0)
                # pangolin.DrawPoints(self.state[1], self.state[2])
                drawPoints(self.state[1])#, self.state[2])

        pangolin.FinishFrame()

    def display(self):
        # if self.q is None:
        #     return

        poses, pts, colors = [], [], []
        for f in self.frames:
            # invert pose for display only
            # poses.append(np.linalg.inv(f.pose))
            poses.append(f.pose)
        for p in self.points:
            pts.append(p.pt)
            # colors.append(p.color)
        self.q.put((np.array(poses), np.array(pts)))#, np.array(colors) / 256.0))


def drawCameras(ptarr):
    w = 1.0
    h_ratio = 0.75
    z_ratio = 0.6
    h = w * h_ratio;
    z = w * z_ratio;

    for i in range(ptarr.shape[0]):
        gl.glPushMatrix();
        print(ptarr[1])
        gl.glMultTransposeMatrixd(ptarr[1]);

        gl.glBegin(gl.GL_LINES);

        gl.glVertex3f(0, 0, 0);
        gl.glVertex3f(w, h, z);
        gl.glVertex3f(0, 0, 0);
        gl.glVertex3f(w, -h, z);
        gl.glVertex3f(0, 0, 0);
        gl.glVertex3f(-w, -h, z);
        gl.glVertex3f(0, 0, 0);
        gl.glVertex3f(-w, h, z);

        gl.glVertex3f(w, h, z);
        gl.glVertex3f(w, -h, z);

        gl.glVertex3f(-w, h, z);
        gl.glVertex3f(-w, -h, z);

        gl.glVertex3f(-w, h, z);
        gl.glVertex3f(w, h, z);

        gl.glVertex3f(-w, -h, z);
        gl.glVertex3f(w, -h, z);
        gl.glEnd();

        gl.glPopMatrix();


#  void DrawCameras(py::array_t < double > cameras, float
#     w = 1.0, float
#     h_ratio = 0.75, float
#     z_ratio = 0.6) {
#         auto
#     r = cameras.unchecked < 3 > ();
#
#     float
#     h = w * h_ratio;
#     float
#     z = w * z_ratio;
#
#     for (ssize_t i = 0; i < r.shape(0); ++i) {
#     glPushMatrix();
#     // glMultMatrixd(r.data(i, 0, 0));
#     glMultTransposeMatrixd(r.data(i, 0, 0));
#
#     glBegin(GL_LINES);
#     glVertex3f(0, 0, 0);
#     glVertex3f(w, h, z);
#     glVertex3f(0, 0, 0);
#     glVertex3f(w, -h, z);
#     glVertex3f(0, 0, 0);
#     glVertex3f(-w, -h, z);
#     glVertex3f(0, 0, 0);
#     glVertex3f(-w, h, z);
#
#     glVertex3f(w, h, z);
#     glVertex3f(w, -h, z);
#
#     glVertex3f(-w, h, z);
#     glVertex3f(-w, -h, z);
#
#     glVertex3f(-w, h, z);
#     glVertex3f(w, h, z);
#
#     glVertex3f(-w, -h, z);
#     glVertex3f(w, -h, z);
#     glEnd();
#
#     glPopMatrix();
#     }
# }


def drawPoints(ptarr):
    gl.glBegin(gl.GL_POINTS);

    for i in range(ptarr.shape[0]):
        gl.glVertex3d(ptarr[i][0], ptarr[i][1], ptarr[i][2])
    gl.glEnd();

# void DrawPoints(py::array_t < double > points) {
#   auto r = points.unchecked < 2 > ();
#   glBegin(GL_POINTS);
#   for (ssize_t i = 0; i < r.shape(0); ++i) {
#       glVertex3d(r(i, 0), r(i, 1), r(i, 2));
#   }
#   glEnd();
# }
