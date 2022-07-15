import cv2

ceilbase=cv2.imread("./ceiling_gen/ceilingbase.png", cv2.IMREAD_UNCHANGED)
ceilbase=cv2.cvtColor(ceilbase, cv2.COLOR_RGB2BGRA)
ceil=cv2.copyMakeBorder(ceilbase, 0, 1024-730, 0, 2048-1070, cv2.BORDER_CONSTANT,value=[0,0,0,0])
cv2.imwrite("./ceiling_gen/ceiling.png",ceil)
cv2.imshow("yeahaw",ceil)
cv2.waitKey(0)