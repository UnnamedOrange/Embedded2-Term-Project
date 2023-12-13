# Copyright (c) UnnamedOrange. Licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

import numpy as np


def make_model(n_layer: int = 100):
    import tensorflow as tf

    kernel = np.array([[1.0, 2.0, 1.0], [2.0, 4.0, 2.0], [1.0, 2.0, 1.0]])
    kernel = kernel / np.sum(kernel)
    weights = np.zeros((3, 3, 3, 3))
    for i in range(3):
        weights[:, :, i, i] = kernel

    model = tf.keras.Sequential(
        [
            tf.keras.layers.Conv2D(
                filters=3,
                kernel_size=3,
                strides=1,
                padding="same",
                use_bias=False,
                input_shape=(320, 240, 3),
            )
            for _ in range(n_layer)
        ]
    )
    for i in range(n_layer):
        model.layers[i].set_weights([weights])
        model.layers[i].trainable = False

    return model


def make_model_3(n_layer: int = 100):
    import keras

    kernel = np.array([[1.0, 2.0, 1.0], [2.0, 4.0, 2.0], [1.0, 2.0, 1.0]])
    kernel = kernel / np.sum(kernel)
    weights = np.zeros((3, 3, 3, 3))
    for i in range(3):
        weights[:, :, i, i] = kernel

    # `keras.Sequential` is a special case of model where the model is purely
    # a stack of single-input, single-output layers.
    model = keras.Sequential([keras.Input(shape=(None, None, 3))], trainable=False)

    for _ in range(n_layer):
        layer = keras.layers.Conv2D(
            filters=3,
            kernel_size=3,
            padding="same",
            use_bias=False,
        )
        # Weights can be set ONLY after being built.
        model.add(layer, rebuild=True)
        layer.set_weights([weights])

    return model


if __name__ == "__main__":
    model = make_model(n_layer=50)

    def test_case():
        from PIL import Image

        img = Image.open("test.png")
        img = img.convert("RGB")
        img_array = np.array(img)
        img_array = np.expand_dims(img_array, axis=0)  # n_batch is 1.
        blurred_img_array = model.predict(img_array)
        blurred_img_array = blurred_img_array.astype(np.uint8)
        blurred_img = Image.fromarray(blurred_img_array[0])  # n_batch is 1.
        blurred_img.show()

    try:
        test_case()
    except:  # noqa: E722
        pass
