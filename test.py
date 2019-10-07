import numpy as np
import cv2
import image_warp

# Test 1
print(image_warp.test_add(1, 1))

# Test 2
image_warp.test_mls()

# Test 3
image = cv2.imread('./test.jpg')
ctr_src = np.array([[100, 100], [100, 400], [400, 100], [400, 400]], dtype=np.float64)
ctr_dst = np.array([[50, 50], [50, 450], [450, 50], [450, 450]], dtype=np.float64)
frame_warp = image_warp.warp_mls(image, ctr_src, ctr_dst)
cv2.imwrite('./warp_MLS.jpg', frame_warp)

print("Test passed!")