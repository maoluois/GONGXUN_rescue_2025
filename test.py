import torch
import cv2
import numpy as np
from torchvision import models, transforms

# 加载预训练的 YOLOv5 模型
model = torch.load('best.pt')
model.eval()

# 定义类别名称
class_names = ['volleyball']

# 定义图像预处理函数
transform = transforms.Compose([
    transforms.ToTensor(),
])

# 定义目标检测函数
def detect_objects(image):
    image_tensor = transform(image).unsqueeze(0)
    with torch.no_grad():
        detections = model(image_tensor)[0]
    return detections

# 打开视频流
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # 转换为RGB格式
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # 进行目标检测
    detections = detect_objects(frame_rgb)

    # 解析检测结果并绘制边界框和标签
    boxes = detections['boxes'].numpy()
    scores = detections['scores'].numpy()
    labels = detections['labels'].numpy()
    confidence_threshold = 0.5

    for i in range(len(scores)):
        if scores[i] > confidence_threshold:
            box = boxes[i]
            class_id = labels[i]
            class_name = class_names[class_id]
            left, top, right, bottom = box
            cv2.rectangle(frame, (int(left), int(top)), (int(right), int(bottom)), (0, 255, 0), 2)
            label = f'{class_name}: {scores[i]:.2f}'
            cv2.putText(frame, label, (int(left), int(top) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 显示结果图像
    cv2.imshow('Object Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()