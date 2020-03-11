#!/usr/bin/env python3

import sys
import cv2
from display import Display
from extractor import Extractor
import numpy as np


#actual footage: 1280 × 720
W = 1920//2
H = 1080//2

F = 1
disp = Display(W, H)
# K = np.array(([F, 0, 0], [0, F, 0], [W//2, H//2, 1]))
K = np.array(([F, 0, W//2], [0, F, H//2], [0, 0, 1]))
fe = Extractor(K)

# sdl2.ext.init()

def process_frame(img):
  img = cv2.resize(img, (W, H))
  matches = fe.extract(img)
  print("%d matches" % (len(matches)))

  for pt1, pt2 in matches:
      u1, v1 = fe.denormalize(pt1)
      u2, v2 = fe.denormalize(pt2)

      cv2.circle(img, (u1, v1), color=(0, 255, 0), radius=3)
      cv2.line(img, (u1, v1), (u2, v2), color=(255, 0, 0))

  disp.paint(img)

if __name__ == "__main__":
  cap = cv2.VideoCapture(sys.argv[1])
 

  while cap.isOpened():
    ret, frame = cap.read()
    if ret:
       process_frame(frame)
    else:
       break


