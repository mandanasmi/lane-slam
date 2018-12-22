import numpy as np
import cv2

__all__ = [
    'color_segment',
    'drawNormals',
    'drawLines',
    'drawNormals2',
]

# draw line segments
def drawLines(bgr, lines, paint, p1_color=(0,255,0), p2_color=(0,0,255)):
    if len(lines)>0:
        for x1,y1,x2,y2 in lines:
            cv2.line(bgr, (x1,y1), (x2,y2), paint, 2)
            if p1_color is not None:
                cv2.circle(bgr, (x1,y1), 2, p1_color)
            if p2_color is not None:
                cv2.circle(bgr, (x2,y2), 2, p2_color)

# draw segment normals
def drawNormals(bgr, lines, normals):
    if len(lines)>0:
        for x1,y1,x2,y2,dx,dy in np.hstack((lines,normals)):
            x3 = int((x1+x2)/2. - 4.*dx)
            y3 = int((y1+y2)/2. - 4.*dy)
            x4 = int((x1+x2)/2. + 4.*dx)
            y4 = int((y1+y2)/2. + 4.*dy)
            cv2.circle(bgr, (x3,y3), 2, (0,255,0))
            cv2.circle(bgr, (x4,y4), 2, (0,0,255))

# draw segment normals2
def drawNormals2(bgr, centers, normals, paint):
    if len(centers)>0:
        for x,y,dx,dy in np.hstack((centers,normals)):
            x3 = int(x - 2.*dx)
            y3 = int(y - 2.*dy)
            x4 = int(x + 2.*dx)
            y4 = int(y + 2.*dy)
            cv2.line(bgr, (x3,y3), (x4,y4), paint, 1)
            cv2.circle(bgr, (x3,y3), 1, (0,255,0))
            cv2.circle(bgr, (x4,y4), 1, (0,0,255))

# generate color segments
def color_segment(area_white, area_red, area_yellow):
    B, G, R = 0, 1, 2

    def white(x):
        x = cv2.cvtColor(x, cv2.COLOR_GRAY2BGR)
        return x
    def red(x):
        x = cv2.cvtColor(x, cv2.COLOR_GRAY2BGR)
        x[:,:,R] *= 1
        x[:,:,G] *= 0
        x[:,:,B] *= 0
        return x
    def yellow(x):
        x = cv2.cvtColor(x, cv2.COLOR_GRAY2BGR)
        x[:,:,R] *= 1
        x[:,:,G] *= 1
        x[:,:,B] *= 0
        return x

    h, w = area_white.shape
    orig = [area_white, area_red, area_yellow]
    masks = [white(area_white), red(area_red), yellow(area_yellow)]

    res = np.zeros((h,w,3), dtype=np.uint8)

    for i, m in enumerate(masks):
        nz = (orig[i] > 0) * 1.0
        assert nz.shape == (h, w), nz.shape

        for j in [0, 1, 2]:
            res[:,:,j] = (1-nz) * res[:,:,j].copy() + (nz) * m[:,:,j]

    return res
