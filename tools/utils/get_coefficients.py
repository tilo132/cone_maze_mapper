import numpy
import cv2
import argparse

from rich.progress import Progress

parser = argparse.ArgumentParser(
    prog = 'Camera Coefficient Calculator',
    description = 'retrieves the camera distortion coefficients from a checkerboard camera video',
)

parser.add_argument('filename')
parser.add_argument('-s', '--sample_rate', type=int, default=1)
args = parser.parse_args()

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = numpy.zeros((6*7,3), numpy.float32)
objp[:,:2] = numpy.mgrid[0:7,0:6].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

cv2.namedWindow('img',cv2.WINDOW_NORMAL)
cv2.resizeWindow('img', 600,600)

cap = cv2.VideoCapture(args.filename)

length = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
frame = 0

progress = Progress()
task = progress.add_task("extracting frames", total=length)
progress.start()

success = cap.grab()

stats = [0,0]

while success:
    progress.advance(task)
    if (frame % args.sample_rate) != 0:
        success = cap.grab()
        frame+=1
        continue
    _, img = cap.retrieve()

    img = cv2.rotate(img, cv2.ROTATE_180)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('img', gray)
    cv2.waitKey(1)
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)

    progress.update(task, description=f"[white on green]:)[/ white on green] {stats[0]}/{stats[1]} [white on red]):[/ white on red] | frame: [cyan]{frame}/{length}[/ cyan]")

    if ret:
        stats[0]+=1
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv2.imshow('img', img)
        cv2.drawChessboardCorners(img, (7,6), corners2, ret)
        cv2.waitKey(10)
    else:
        stats[1]+=1
    success = cap.grab()
    frame += 1
cv2.destroyAllWindows()
progress.stop()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(
    "--------------"
    "RET:", ret, "\n",
    "MTX:", mtx, "\n",
    "DIST:", dist, "\n",
    "RVECS:", rvecs, "\n",
    "TVECS:", tvecs, "\n"
    "--------------"
)