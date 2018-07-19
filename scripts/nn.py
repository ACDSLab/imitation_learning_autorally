"""Convolutional NN similar to VGG for use with KerasLearner."""
import numpy as np
from keras.models import Sequential, Model
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras.layers import Merge, Input, Convolution2D, concatenate


def keras_dnn_ws(d_img, d_ws, d_act):
    """
    d_img, d_ws are input dimensions (tuples)
    """
    # CNN with vision input
    inp_img = Input(shape=d_img, name='input_img')
    conv_1 = Convolution2D(32, (3, 3), padding='same', activation='relu',
                           name='conv_1')(inp_img)
    conv_2 = Convolution2D(32, (3, 3), padding='same', activation='relu',
                           name='conv_2')(conv_1)
    pool_1 = MaxPooling2D(pool_size=(2, 2), name='pool_1')(conv_2)

    conv_3 = Convolution2D(64, (3, 3), padding='same', activation='relu',
                           name='conv_3')(pool_1)
    conv_4 = Convolution2D(64, (3, 3), padding='same', activation='relu',
                           name='conv_4')(conv_3)
    pool_2 = MaxPooling2D(pool_size=(2, 2), name='pool_2')(conv_4)

    conv_5 = Convolution2D(128, (3, 3), padding='same', activation='relu',
                           name='conv_5')(pool_2)
    conv_6 = Convolution2D(128, (3, 3), padding='same', activation='relu',
                           name='conv_6')(conv_5)
    pool_3 = MaxPooling2D(pool_size=(2, 2), name='pool_3')(conv_6)

    flat = Flatten()(pool_3)
    fc_1 = Dense(512, activation='relu', name='fully_connected_1')(flat)
    drop_1 = Dropout(0.5, name='dropout_1')(fc_1)
    fc_2 = Dense(256, activation='relu', name='fully_connected_2')(drop_1)
    drop_2 = Dropout(0.25, name='dropout_2')(fc_2)

    # feedforward (fully-connected) network with wheel speed input
    inp_ws = Input(shape=[d_ws], name='input_ws')
    fc_ws_1 = Dense(64, activation='relu', name='fully_connected_ws')(inp_ws)

    # concatenate two models
    concat = concatenate([drop_2, fc_ws_1], name='concatenate')
    fc_3 = Dense(128, activation='relu', name='fully_connected_3')(concat)
    drop_3 = Dropout(0.25, name='dropout_3')(fc_3)
    out_u = Dense(d_act, activation='linear', name='output_action')(drop_3)

    model = Model(inputs=[inp_img, inp_ws], outputs=out_u)

    return model
