import numpy as np
import tensorflow as tf

from make_model import make_model


def make_tflite(n_layer: int = 100) -> bytes:
    model = make_model(n_layer)
    model.predict(np.zeros([1, 320, 240, 3]))
    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    tflite_model = converter.convert()

    return tflite_model


if __name__ == "__main__":
    tflite_model = make_tflite()
    with open("model.tflite", "wb") as file:
        file.write(tflite_model)
