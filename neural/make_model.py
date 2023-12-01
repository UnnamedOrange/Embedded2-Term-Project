import numpy as np
import keras


def make_model(n_layer: int = 100) -> keras.Model:
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
