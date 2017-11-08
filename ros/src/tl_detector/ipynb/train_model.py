
# coding: utf-8

# In[38]:

import matplotlib.pyplot as plt
import tensorflow as tf
import glob
from scipy.misc import imread 
from scipy.misc import imresize
from keras.models import Sequential
from keras.layers import Dense, Dropout, Activation, Flatten, Convolution2D, MaxPooling2D, Conv2D, MaxPool2D, Lambda
from keras.layers import BatchNormalization, LeakyReLU
from keras.utils import np_utils
import numpy as np


# In[47]:

# load data
file_names_dict = dict()
for i in [0, 1, 2]:
    image_files = glob.glob("/home/michael/tl_training/{}/*.jpg".format(i))
    file_names_dict[i] = image_files

min_length = 9999999

# In[48]:

data_dict = dict()
for key in file_names_dict:
    length = len(file_names_dict.get(key))
    if length < min_length:
        min_length = length

for key in file_names_dict:
    print(len(file_names_dict.get(key)))
    fnames = file_names_dict.get(key)[0:min_length]
    images = [imread(x)  for x in fnames]
    data_dict[key] = images
    
    


# In[49]:

X = []
Y = []
for key in data_dict:
    x = np.array(data_dict.get(key))
    y = np.ones(shape=x.shape[0]) * key
    X.append(x)
    Y.append(y)


# In[50]:

X_train = np.vstack((X[0], X[1], X[2], X[3]))
Y_train = np.hstack((Y[0], Y[1], Y[2], Y[3]))


# In[51]:

Y_train = np.hstack((Y[0], Y[1], Y[2], Y[3]))


# In[52]:

print(X_train.shape)
print(Y_train.shape)


# In[53]:

del X
del Y
del data_dict
del file_names_dict


# In[54]:




# In[77]:

model = Sequential

model = Sequential([
    Lambda(lambda x: x / 255 - 0.5, input_shape=(300, 400, 3)),
    Conv2D(8, kernel_size=(5, 5), strides=(2,2)),
    LeakyReLU(alpha=0.1),
    BatchNormalization(),
    MaxPool2D(pool_size=(2,2), strides=(2,2)),
    Conv2D(16, kernel_size=(3, 3), strides=(1,1)),
    LeakyReLU(alpha=0.1),
    BatchNormalization(),
    MaxPool2D(pool_size=(2,2), strides=(2,2)),

    Conv2D(32, kernel_size=(3, 3), strides=(1, 1)),
    LeakyReLU(alpha=0.1),
    BatchNormalization(),
    Flatten(),
    Dense(55),
    Dense(4, activation='softmax')
])
model.compile(loss='categorical_crossentropy',
             optimizer='adam',
             metrics=['accuracy'])
print(model.output_shape)


# In[56]:

# training
from keras.preprocessing.image import ImageDataGenerator

datagen = ImageDataGenerator(
    rotation_range = 10,
    width_shift_range=0.2,
    height_shift_range=0.2,
    horizontal_flip=True,
    vertical_flip=True
)
datagen.fit(X_train)


# In[57]:

from sklearn.preprocessing import OneHotEncoder
enc = OneHotEncoder()
Y_train = Y_train.reshape(-1, 1)
Y_train = enc.fit_transform(Y_train).toarray()


# In[68]:

for i in range(100):
    model.fit_generator(datagen.flow(X_train, Y_train, batch_size=64), steps_per_epoch=X_train.shape[0]/64, epochs=10)

    model.save('../light_classification/models/whole_image_model_gpu2.h5')

# In[69]:



# In[70]:

# seprate model and weights
yaml_string = model.to_yaml()


# In[71]:

with open("../light_classification/models/model.yaml", 'wt') as f:
    f.write(yaml_string)
model.save_weights("../light_classification/models/model_weights.h5")


# In[ ]:



