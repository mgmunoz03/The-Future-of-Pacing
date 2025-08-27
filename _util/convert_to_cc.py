import os

tflite_model_file = "F:\DD Drone 24-25\lane_detect_v2\lane_model_80.tflite"
cc_file = "lane_model_80.cc"

with open(tflite_model_file, "rb") as f:
    model_data = f.read()

with open (cc_file, "w") as f:
    f.write("#include <cstdint>\n\n")
    f.write("// Generated Tensorflow Lite Model Data\n")
    f.write("alignas(8) const unsigned char simple_model_tflite[] = {\n")
    
    for i, byte in enumerate(model_data):
        f.write(f"0x{byte:02x}, ")
        if (i + 1) % 12 == 0:  # Wrap lines after 12 values
            f.write("\n")
        
    f.write("\n};\n")
    f.write(f"const unsigned int simple_model_tflite_len = {len(model_data)};\n")
    
print(f" Model successfully converted to {cc_file}")
