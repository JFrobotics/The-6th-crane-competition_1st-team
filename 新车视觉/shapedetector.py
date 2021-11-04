import cv2
import numpy as np
class ShapeDetector:
	def __init__(self):
		pass

	def detect(self,c,cir):
		# 初始化形状名和近似的轮廓
            shape = "unidentified"
            
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            # 如果当前的轮廓含有3个顶点，则其为三角形
            if len(approx) == 3:
                shape = 3
            # 如果当前的轮廓含有4个顶点，则其可能是矩形或者正方形
            elif len(approx) == 4:
                # 获取轮廓的边界框并计算长和宽的比例
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                shape = 4 if ar >= 0.95 and ar <= 1.05 else 4
		# 如果这个轮廓含有5个顶点，则它是一个多边形
            #elif len(approx) == 5:
			#shape = "pentagon"
		# 否则的话，我们认为它是一个圆
            else:  
                circles = cv2.HoughCircles(
                cir, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=15, minRadius=20, maxRadius=500)
                if circles is not None:  # 如果识别出圆
                     shape=5
		# 返回形状的名称
            return shape



