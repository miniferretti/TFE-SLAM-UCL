from tensorflow.keras.layers import Layer, InputSpec
import tensorflow.keras.utils as conv_utils
import tensorflow as tf
import tensorflow.keras.backend as K


class BilinearUpSampling2D(Layer):
    def __init__(self, size=(2, 2), data_format=None, **kwargs):
        super(BilinearUpSampling2D, self).__init__(**kwargs)
        self.data_format = self.normalize_data_format(data_format)
        self.size = self.normalize_tuple(size, 2, "size")
        self.input_spec =  InputSpec(ndim=4)

    def compute_output_shape(self, input_shape):
        if self.data_format == "channels_first":
            height = self.size[0] * \
                input_shape[2] if input_shape[2] is not None else None
            width = self.size[1] * \
                input_shape[3] if input_shape[3] is not None else None
            return (input_shape[0],
                    input_shape[1],
                    height,
                    width)
        elif self.data_format == "channels_last":
            height = self.size[0] * \
                input_shape[1] if input_shape[1] is not None else None
            width = self.size[1] * \
                input_shape[2] if input_shape[2] is not None else None
            return (input_shape[0],
                    height,
                    width,
                    input_shape[3])

    def call(self, inputs):
        input_shape = K.shape(inputs)
        if self.data_format == "channels_first":
            height = self.size[0] * \
                input_shape[2] if input_shape[2] is not None else None
            width = self.size[1] * \
                input_shape[3] if input_shape[3] is not None else None
        elif self.data_format == "channels_last":
            height = self.size[0] * \
                input_shape[1] if input_shape[1] is not None else None
            width = self.size[1] * \
                input_shape[2] if input_shape[2] is not None else None

        return tf.compat.v1.image.resize(inputs, [height, width], method=tf.image.ResizeMethod.BILINEAR, align_corners=True)

    def get_config(self):
        config = {"size": self.size, "data_format": self.data_format}
        base_config = super(BilinearUpSampling2D, self).get_config()
        return dict(list(base_config.items()) + list(config.items()))

    def normalize_data_format(self, value):
        if value is None:
            value = K.image_data_format()
        data_format = value.lower()
        if data_format not in {'channels_first', 'channels_last'}:
            raise ValueError('The `data_format` argument must be one of '
                             '"channels_first", "channels_last". Received: ' +
                             str(value))
        return data_format

    def normalize_tuple(self, value, n, name):

        if isinstance(value, int):
            return (value,) * n
        else:
            try:
                value_tuple = tuple(value)
            except TypeError:
                raise ValueError('The `' + name + '` argument must be a tuple of ' +
                                 str(n) + ' integers. Received: ' + str(value))
            if len(value_tuple) != n:
                raise ValueError('The `' + name + '` argument must be a tuple of ' +
                                 str(n) + ' integers. Received: ' + str(value))
            for single_value in value_tuple:
                try:
                    int(single_value)
                except (ValueError, TypeError):
                    raise ValueError('The `' + name + '` argument must be a tuple of ' +
                                     str(n) + ' integers. Received: ' +
                                     str(value) + ' '
                                     'including element ' + str(single_value) + ' of type' +
                                     ' ' + str(type(single_value)))
        return value_tuple
