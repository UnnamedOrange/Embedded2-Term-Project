# Copyright (c) UnnamedOrange. Licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

import tensorflow as tf

from make_model import make_model


def make_tflite(n_layer: int = 100) -> bytes:
    model = make_model(n_layer)
    converter = tf.lite.TFLiteConverter.from_keras_model(model)

    tflite_model = converter.convert()

    return tflite_model


if __name__ == "__main__":
    tflite_model = make_tflite()
    with open("model.tflite", "wb") as file:
        file.write(tflite_model)
