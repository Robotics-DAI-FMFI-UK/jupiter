#!/usr/bin/env python
import numpy as np
from PIL import Image
from pycoral.adapters import classify
from pycoral.adapters import common
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter


def classification():  
  model = 'clothes/clothes_model_edgetpu.tflite'
  labels_path = 'clothes/clothing_labels.txt'
  input = 'clothes/clothing_labels.txt'

  labels = read_label_file(labels_path) if labels_path else {}

  interpreter = make_interpreter(*model.split('@'))
  interpreter.allocate_tensors()

  # Model must be uint8 quantized
  if common.input_details(interpreter, 'dtype') != np.uint8:
    raise ValueError('Only support uint8 input type.')

  size = common.input_size(interpreter)
  image = Image.open(input).convert('RGB').resize(size, Image.ANTIALIAS)

  params = common.input_details(interpreter, 'quantization_parameters')
  scale = params['scales']
  zero_point = params['zero_points']
  mean = 128.0
  std = 128.0
  if abs(scale * std - 1) < 1e-5 and abs(mean - zero_point) < 1e-5:
    # Input data does not require preprocessing.
    common.set_input(interpreter, image)
  else:
    # Input data requires preprocessing
    normalized_input = (np.asarray(image) - mean) / (std * scale) + zero_point
    np.clip(normalized_input, 0, 255, out=normalized_input)
    common.set_input(interpreter, normalized_input.astype(np.uint8))

  # Run inference
  for _ in range(5):
      interpreter.invoke()
      classes = classify.get_classes(interpreter, 1, 0.0)

  print('-------RESULT--------')
  for c in classes:
    print('%s: %.5f' % (labels.get(c.id, c.id), c.score))
    
  return (labels.get(c.id, c.id))


if __name__ == '__main__':
  classification()
