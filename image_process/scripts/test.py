import cv2
import numpy as np
import struct
def unpack_helper(fmt, data):
    size = struct.calcsize(fmt)
    return struct.unpack(fmt, data[:size]), data[size:]
a=cv2.imread("image_data_2/3.jpg")
b=np.array(a,dtype=np.uint8).flatten()
c=b.tolist()
print(type(c[0]))
print(c[0])
# while len(buf)>0:
#     print(len(buf))
#     ele, buf= unpack_helper('I',buf)
#     c.append(ele)
# c=np.array(c)
# print((c==b).all())

