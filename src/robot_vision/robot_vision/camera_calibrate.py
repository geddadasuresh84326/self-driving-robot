import cv2
import numpy as np
import glob
import os

CHECKERBOARD = (9, 6)  # (inner corners width, inner corners height)
SQUARE_SIZE = 0.0258    # Length of one square side in meters (e.g., 0.025m = 25mm)

# Termination criteria for sub-pixel corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare 3D object points (0,0,0), (1,0,0), (2,0,0) ... scaled by SQUARE_SIZE
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

images = glob.glob("capture_images/*.png")

# creating dir to save calibrated images 
save_dir = "calib_images"
img_count = 0
path = os.path.join(save_dir)
os.makedirs(save_dir,exist_ok=True)
print(f"path : {path}")
if not images:
    print("No images found in capture_images/ folder!")
    exit()

print(f"Processing {len(images)} images...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        
        # Draw and display the corners
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        file_name = os.path.join(save_dir,f"img_{img_count:02d}.png")
        cv2.imwrite(file_name,img)
        print(f"calibrated image saved to folder")
        img_count += 1
        # cv2.imshow("Detected Corners", img)
        # cv2.waitKey(200)
    else:
        print(f"Corners not found in {fname}")

# cv2.destroyAllWindows()

# Run Calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

if ret:
    print("\n--- Calibration Successful! ---")
    print("Camera Matrix:\n", camera_matrix)
    print("\nDistortion Coefficients:\n", dist_coeffs)

    # Save results to a file for ArUco script
    np.savez("camera_calibration.npz", matrix=camera_matrix, dist=dist_coeffs)
    print("\nSaved parameters to 'camera_calibration.npz'")
else:
    print("Calibration failed.")