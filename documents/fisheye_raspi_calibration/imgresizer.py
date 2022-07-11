#resizes images to make opencv detect them better.
#resizing had no positive effect on chessboard detection.
#detection issue was later fixed with new images with better lighting.
import cv2
import glob

images=glob.glob("big/*.jpg")


for filename in images:
	img=cv2.imread(filename)
	realname=filename[4:]
	resizeImg=cv2.resize(img, (1280,720))
	cv2.imwrite(realname, resizeImg)