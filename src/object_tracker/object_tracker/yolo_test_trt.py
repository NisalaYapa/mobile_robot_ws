import tensorrt as trt
import cv2
import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit

class TensorRTYOLO:
    def __init__(self, engine_path):
        self.TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        self.runtime = trt.Runtime(self.TRT_LOGGER)

        # Load the TensorRT engine
        with open(engine_path, "rb") as f:
            self.engine = self.runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.inputs, self.outputs, self.bindings = self.allocate_buffers()

    def allocate_buffers(self):
        inputs, outputs, bindings = [], [], []
        for binding in self.engine:
            size = trt.volume(self.engine.get_binding_shape(binding))
            dtype = trt.nptype(self.engine.get_binding_dtype(binding))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            bindings.append(int(device_mem))
            if self.engine.binding_is_input(binding):
                inputs.append((host_mem, device_mem))
            else:
                outputs.append((host_mem, device_mem))
        return inputs, outputs, bindings

    def infer(self, image):
        # Preprocess the image
        input_image = self.preprocess_image(image)

        # Copy input to device
        cuda.memcpy_htod(self.inputs[0][1], input_image.ravel())

        # Run inference
        self.context.execute_v2(self.bindings)

        # Copy output from device
        cuda.memcpy_dtoh(self.outputs[0][0], self.outputs[0][1])

        # Post-process results
        return self.postprocess(self.outputs[0][0])

    def preprocess_image(self, image):
        image_resized = cv2.resize(image, (640, 640))  # Resize to model input
        image_rgb = cv2.cvtColor(image_resized, cv2.COLOR_BGR2RGB) / 255.0
        image_transposed = np.transpose(image_rgb, (2, 0, 1)).astype(np.float32)
        return np.expand_dims(image_transposed, axis=0)

    def postprocess(self, output):
        # Process the output (depends on the model)
        return output  # You need to decode YOLO detections here

# Load TensorRT model
trt_model = TensorRTYOLO("best.trt")

# Load the image
image_path = "human.jpg"
image = cv2.imread(image_path)

# Run inference
detections = trt_model.infer(image)

# Display results
print("Detections:", detections)

