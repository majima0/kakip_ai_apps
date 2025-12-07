import pydrpai
import os
import cv2
import numpy as np
import argparse

parser = argparse.ArgumentParser()

parser.add_argument("--model-dir", type=str)
parser.add_argument("--video-path", type=str)

args = parser.parse_args()

print(args.model_dir, args.video_path)

#video_path = "/home/ubuntu/sample_1080p_h264.mp4"
video_path = args.video_path
padding_frame = np.zeros((640 - 480, 640, 3), dtype=np.uint8)
cap = cv2.VideoCapture(video_path)
ret, frame = cap.read()
g_frame = cv2.resize(frame, (640, 480))
#print(g_frame.shape)
input_img = cv2.vconcat([g_frame, padding_frame])
#cv2.imwrite('/home/ubuntu/test.jpg', input_img)
#input_img = cv2.imread("/home/ubuntu/test.jpg")

device_path = "/dev/drpai0"
#model_dir = "/home/ubuntu/kakip_ai_apps/R01_object_detection/exe/yolov3_onnx"
model_dir = args.model_dir
pre_dir = os.path.join(model_dir, "preprocess")

fd = os.open(device_path, os.O_RDWR)
if 0 > fd:
    print("ERROR fd")
    exit(2)

drpaimem_addr_start = pydrpai.init_drpai(fd)
print("drpaimem_addr_start=", hex(drpaimem_addr_start))
if 0 == drpaimem_addr_start:
    print("ERROR drpaimem_addr_start")
    os.close(fd)
    exit(2)

preruntime = pydrpai.PreRuntime()
ret = preruntime.Load(pre_dir)
if 0 < ret:
    print("ERROR preruntime ret")
    os.close(fd)
    exit(2)

runtime = pydrpai.MeraDrpRuntimeWrapper()
runtime_status = runtime.LoadModel(model_dir, drpaimem_addr_start)
if not runtime_status:
    print("ERROR runtime.LoadModel")
    os.close(fd)
    exit(2)

input_data_type = runtime.GetInputDataType(0)
print(input_data_type)

drpai_buf = pydrpai.buffer_alloc_dmabuf(640*640*3)
if not drpai_buf.mem:
    print("ERROR buffer_alloc_dmabuf")
    os.close(fd)
    exit(2)

# numpy配列はctypes.dataで配列のポインタを取得できる
pydrpai.memcpy(drpai_buf.mem, input_img.ctypes.data, drpai_buf.size)
print("drpai_buf.size", drpai_buf.size)
ret = pydrpai.buffer_flush_dmabuf(drpai_buf.idx, drpai_buf.size)
#print(input_img.tobytes())
if 0 != ret:
    print("ERROR buffer_flush_dmabuf")
    os.close(fd)
    exit(2)

in_param = pydrpai.s_preproc_param_t()
in_param.pre_in_shape_w = 640
in_param.pre_in_shape_h = 480
in_param.pre_in_addr = drpai_buf.phy_addr

output_ptr, out_size = preruntime.Pre(in_param.get_ptr())
print("output_ptr, out_size", hex(output_ptr), out_size)

runtime.SetInput(0, output_ptr)

drpai_freq = 2
runtime.Run(drpai_freq)

result = pydrpai.get_result(runtime)
print(result)

for d in result:
    print(d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h, d.c, d.prob)
    if d.prob == 0:
        continue
    x_min = d.bbox.x - d.bbox.w / 2
    y_min = d.bbox.y - d.bbox.h / 2
    x_max = d.bbox.x + d.bbox.w / 2
    y_max = d.bbox.y + d.bbox.h / 2
    cv2.rectangle(input_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 0, 255), 2)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    g_frame = cv2.resize(frame, (640, 480))
    input_img = cv2.vconcat([g_frame, padding_frame])

    pydrpai.memcpy(drpai_buf.mem, input_img.ctypes.data, drpai_buf.size)
    print("drpai_buf.size", drpai_buf.size)
    ret = pydrpai.buffer_flush_dmabuf(drpai_buf.idx, drpai_buf.size)
    if 0 != ret:
        print("ERROR buffer_flush_dmabuf")
        break

    in_param.pre_in_addr = drpai_buf.phy_addr
    output_ptr, out_size = preruntime.Pre(in_param.get_ptr())

    runtime.SetInput(0, output_ptr)
    drpai_freq = 2
    runtime.Run(drpai_freq)

    result = pydrpai.get_result(runtime)

    for d in result:
        #print(d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h, d.c, d.prob)
        if d.prob == 0:
            continue
        x_min = d.bbox.x - d.bbox.w / 2
        y_min = d.bbox.y - d.bbox.h / 2
        x_max = d.bbox.x + d.bbox.w / 2
        y_max = d.bbox.y + d.bbox.h / 2
        cv2.rectangle(input_img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), (0, 0, 255), 2)

    show_size = (640, 360)
    show_img = cv2.resize(input_img[0:480, 0:640], show_size)
    cv2.imshow("show img", show_img)
    #cv2.imwrite('/home/ubuntu/test.jpg', input_img)
    key = cv2.waitKey(1)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
os.close(fd)
