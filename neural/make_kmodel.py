# Copyright (c) UnnamedOrange. Licensed under the MIT License.
# See the LICENSE file in the repository root for full license text.

import nncase
import numpy as np

from make_tflite import make_tflite


def make_kmodel() -> bytes:
    # compile_options
    compile_options = nncase.CompileOptions()
    compile_options.target = "k210"
    compile_options.input_type = "uint8"
    compile_options.input_range = [0, 255]
    compile_options.output_type = "float32"  # Cannot get range if output type is uint8.

    compile_options.preprocess = True
    compile_options.input_layout = "NHWC"
    compile_options.output_layout = "NHWC"
    compile_options.input_shape = [1, 320, 240, 3]

    # compiler
    compiler = nncase.Compiler(compile_options)

    # import_options
    import_options = nncase.ImportOptions()

    # PTQ
    ptq_options = nncase.PTQTensorOptions()
    sc = 5
    ptq_options.samples_count = sc
    ptq_options.set_tensor_data(
        np.random.rand(sc, 320, 240, 3).astype(np.float32).tobytes()
    )
    compiler.use_ptq(ptq_options)

    # import
    model_content = make_tflite(n_layer=71)  # Stall if n_layer>=72
    compiler.import_tflite(model_content, import_options)

    # compile
    compiler.compile()

    # kmodel
    kmodel = compiler.gencode_tobytes()

    return kmodel


if __name__ == "__main__":
    kmodel = make_kmodel()
    with open("model.kmodel", "wb") as file:
        file.write(kmodel)
