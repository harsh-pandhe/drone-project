import tflite_runtime.interpreter as tflite
import platform

# Explicitly try to load the library from the common Pi 64-bit path
library = '/usr/lib/aarch64-linux-gnu/libedgetpu.so.1'
model_path = "mobilenet_v2_1.0_224_inat_bird_quant_edgetpu.tflite"

print(f"--- Attempting to load TPU from {library} ---")
try:
    delegate = tflite.load_delegate(library)
    interpreter = tflite.Interpreter(model_path=model_path, 
                                     experimental_delegates=[delegate])
    interpreter.allocate_tensors()
    print("SUCCESS: Edge TPU is now ACTIVE!")
except Exception as e:
    print(f"LOAD FAILED: {e}")
