# cam_viewer_py.py
# lists real devices then shows the camera stream via opencv

from controller import Robot
import cv2, numpy as np, sys

TIME_STEP = 32
bot = Robot()

# enumerate devices correctly
names = []
for i in range(bot.getNumberOfDevices()):
    dev = bot.getDeviceByIndex(i)
    if dev:                      # sometimes None for padding
        names.append(dev.getName())
print('devices:', names)

cam_name = 'camera'              # change if your node’s name field differs

if cam_name not in names:
    print(f'"{cam_name}" missing – cam still not a device.')
    print('double-check: camera node sits under the gripper link *inside* the robot tree,'
          ' and you saved after converting proto → base nodes.')
    sys.exit(1)

cam = bot.getDevice(cam_name)
cam.enable(TIME_STEP)
w, h = cam.getWidth(), cam.getHeight()
print(f'camera online @ {w}×{h}')

cv2.namedWindow('cam', cv2.WINDOW_AUTOSIZE)

while bot.step(TIME_STEP) != -1:
    img = cam.getImage()
    if not img:
        continue
    # bgra → bgr numpy
    frame = np.frombuffer(img, dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
    cv2.imshow('cam', frame)
    if cv2.waitKey(1) == 27:      # esc quits
        break

cv2.destroyAllWindows()
