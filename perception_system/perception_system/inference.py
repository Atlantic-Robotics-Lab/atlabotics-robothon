
from ultralytics import YOLO
import torch
import numpy as np
import time
import cv2
import multiprocessing
import matplotlib.pyplot as plt


class OnlineInference:
    def __init__(self, weights, device="cpu", img_size=640):
        print("Initializing YOLOv8 model...", flush=True)
        self.model = YOLO(weights)
        self.device = device
        self.img_size = img_size
        self.model.to(device)
        self.class_names = self.model.names
        print("Model initialized.", flush=True)

    def realtime_inference(self, img, conf_thres=0.25, iou_thres=0.45, max_det=100, view_img=False):
        """Run inference on a single frame."""
        t1 = time.time()

        # YOLOv8 handles preprocessing internally
        results = self.model.predict(
            source=img,
            conf=conf_thres,
            iou=iou_thres,
            imgsz=self.img_size,
            max_det=max_det,
            device=self.device,
            verbose=False
        )
        t2 = time.time()

        inference_result = []
        img_ori = img.copy()

        # Parse detections
        for r in results:
            boxes = r.boxes  # Boxes object
            if hasattr(boxes, 'xyxy'):
                xy_coords = boxes.xyxy
            elif hasattr(boxes, 'xyn'):  # normalized polygons
                xy_coords = boxes.xyn
            else:
                continue

            for i, box in enumerate(boxes):
                cls_id = int(box.cls)
                conf = float(box.conf)
                class_name = self.class_names[cls_id]
                label = f'{class_name} {conf:.2f}'

                # Extract polygon coordinates (if available), otherwise use bounding box
                if hasattr(box, 'xyn') and box.xyn is not None:
                    polygon = box.xyn[0].cpu().numpy() * np.array([img.shape[1], img.shape[0]])
                    polygon = polygon.astype(int)
                    cv2.polylines(img_ori, [polygon], isClosed=True, color=(0, 255, 0), thickness=2)
                    poly_coords = polygon.tolist()
                else:
                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    cv2.rectangle(img_ori, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), (0, 255, 0), 2)
                    poly_coords = [
                        [xyxy[0], xyxy[1]],
                        [xyxy[2], xyxy[1]],
                        [xyxy[2], xyxy[3]],
                        [xyxy[0], xyxy[3]]
                    ]

                cv2.putText(img_ori, label, tuple(poly_coords[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                inference_result.append({
                    "class_name": class_name,
                    "polygon": poly_coords,
                    "conf": conf
                })

        if view_img:
            def plot():
                plt.figure(multiprocessing.current_process().name)
                plt.imshow(cv2.cvtColor(img_ori, cv2.COLOR_BGR2RGB))
                plt.title("YOLOv8 Inference")
                plt.axis('off')
                plt.show()
                

            p = multiprocessing.Process(target=plot)
            p.start()
        # if view_img:
        #     cv2.imshow("YOLOv8 Inference", img_ori)
        #     cv2.waitKey(1)

        # cv2.destroyAllWindows()
        inference_time = t2 - t1
        return inference_result, img_ori, inference_time


# if view_img:
#             # plt.ion()  # Turn on interactive mode

#             if not hasattr(self, '_display_initialized'):
#                 # self.fig, self.ax = plt.subplots()
#                 # self.im = self.ax.imshow(cv2.cvtColor(img_ori, cv2.COLOR_BGR2RGB))
#                 # self.ax.set_title("YOLOv8 Inference")
#                 # self.ax.axis('off')
#                 cv2.namedWindow("YOLOv8 Inference", cv2.WINDOW_NORMAL)
#                 self._display_initialized = True
#             # else:
#             #     self.im.set_data(cv2.cvtColor(img_ori, cv2.COLOR_BGR2RGB))
            
#             # self.fig.canvas.draw()
#             # self.fig.canvas.flush_events()
#             # plt.pause(0.001)  # Small pause to allow update
#         cv2.imshow("YOLOv8 Inference", img_ori)
#         cv2.waitKey(1)