import tensorflow as tf

model_path = "example/simple_model"
model = tf.keras.models.load_model(model_path)

converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

tflite_model_path = model_path + "simple_model.tflite"
with open(tflite_model_path, "wb") as f:
    f.write(tflite_model)

print("Model converted to TFLite format!")
