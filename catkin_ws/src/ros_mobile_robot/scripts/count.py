#!/usr/bin/env python3

from inference import get_model
import supervision as sv
import cv2

# define the image url to use for inference
image_file = "https://i.ytimg.com/vi/2nT9_j7srz8/maxresdefault.jpg"
image = cv2.imread(image_file)

export ROBOFLOW_API_KEY=<"xbBmr2AOfa3KUtzu7QXJ">
# load a pre-trained yolov8n model
model = get_model(model_id="count-utq1d/2")

# run inference on our chosen image, image can be a url, a numpy array, a PIL image, etc.
results = model.infer(image)[0]

# load the results into the supervision Detections api
detections = sv.Detections.from_inference(results)

# create supervision annotators
bounding_box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator()

# annotate the image with our inference results
annotated_image = bounding_box_annotator.annotate(
    scene=image, detections=detections)
annotated_image = label_annotator.annotate(
    scene=annotated_image, detections=detections)

# display the image
sv.plot_image(annotated_image)
