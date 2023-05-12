#!/usr/bin/env python
import sys
import os
import darknet

import cv2
import numpy as np
from coordinate_publisher import CoordinatesPublisher

# Set the path to the darknet directory
darknet_path = "/home/mustar/darknet"
image_path = "/home/mustar/jupiter/matus/matus_showcase/images/photo.png"

# # Load the YOLOv3 model
config_path = os.path.join(darknet_path, "cfg/yolov3.cfg")
weights_path = os.path.join(darknet_path, "yolov3.weights")
data_path = os.path.join(darknet_path, "cfg/coco.data")

network, class_names, class_colors = darknet.load_network(config_path, data_path, weights_path, batch_size=1)

# Load the image and convert it to a darknet-compatible format
img = cv2.imread(image_path)
img_darknet = darknet.make_image(img.shape[1], img.shape[0], 3)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
darknet.copy_image_from_bytes(img_darknet, img_rgb.tostring(), len(img_rgb.tostring()))

# Detect the objects in the image using YOLOv3
results = darknet.detect_image(network, class_names, img_darknet, thresh=0.25)

# Filter the results to only include the cup class
cup_results = [result for result in results if result[0].decode() == "cup"]
cup_bboxes = []

# Draw bounding boxes around the cups
for result in cup_results:
    class_name = result[0].decode()
    x, y, w, h = result[2]
    x1, y1 = int(x - w / 2), int(y - h / 2)
    x2, y2 = int(x + w / 2), int(y + h / 2)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.putText(img, class_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cup_bboxes.append((x1, y1, x2, y2))

print(results)
print(cup_bboxes)

CoordinatesPublisher(cup_bboxes[0])

# Display the image with the bounding boxes
# cv2.imshow("Cup Detection", img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
