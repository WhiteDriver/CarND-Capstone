
# coding: utf-8

# In[1]:

# Import useful packeges:
import numpy as np
import tensorflow as tf
import cv2
import os
import pickle
import glob
from matplotlib.pyplot import figure, show, cm, imshow
import matplotlib.pyplot as plt
import matplotlib.image as mpimg


# In[2]:

f = open(r'fileX.txt', 'rb')
X = pickle.load(f)
f.close()
f = open(r'fileY.txt', 'rb')
Y = pickle.load(f)
f.close()
print(X.shape)
print(Y.shape)


# In[3]:

List = [10, 340, 474, 600, 800, 950, 1000, 1792]
for j in List:
    imshow(X[j, :, :, :])
    print(Y[j])
    show()


# In[4]:

# Max-Min Normalization:
# https://www.mathworks.com/matlabcentral/answers/25759-normalizing-data-for-neural-networks
def Max_Min(Array, ra=0.9, rb=0.1):
    Max = np.max(Array)
    Min = np.min(Array)
    return ((ra - rb) * ((Array - Min) / (Max - Min)) + rb)

X_normal = Max_Min(X)
print(X_normal.shape)


# In[5]:

from sklearn.model_selection import train_test_split
Percent_of_split = 0.20
X_train, X_split, y_train, y_split = train_test_split(X_normal, Y, test_size=Percent_of_split, random_state=1024)
print(X_train.shape)
print(X_split.shape)
print(y_train.shape)
print(y_split.shape)
Percent_of_cv = 0.50
X_cv, X_test, y_cv, y_test = train_test_split(X_split, y_split, test_size=Percent_of_cv, random_state=1024)
print(X_cv.shape)
print(X_test.shape)
print(y_cv.shape)
print(y_test.shape)


# In[6]:

#import tensorflow as tf
from sklearn.utils import shuffle

EPOCHS = 50
BATCH_SIZE = 128

from tensorflow.contrib.layers import flatten

def LeNet(x):    
    # Hyperparameters
    mu = 0
    sigma = 0.1
    
    # Layer 1: Convolutional. Input = 32x32x3. Output = 28x28x6.
    conv1_W = tf.Variable(tf.truncated_normal(shape=(5, 5, 3, 3), mean = mu, stddev = sigma))
    conv1_b = tf.Variable(tf.zeros(3))
    conv1   = tf.nn.conv2d(x, conv1_W, strides=[1, 1, 1, 1], padding='VALID') + conv1_b

    # Activation.
    conv1 = tf.nn.relu(conv1)

    # Pooling. Input = 28x28x6. Output = 14x14x6.
    conv1 = tf.nn.max_pool(conv1, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID')

    # Layer 2: Convolutional. Output = 10x10x16.
    conv2_W = tf.Variable(tf.truncated_normal(shape=(5, 5, 3, 5), mean = mu, stddev = sigma))
    conv2_b = tf.Variable(tf.zeros(5))
    conv2   = tf.nn.conv2d(conv1, conv2_W, strides=[1, 1, 1, 1], padding='VALID') + conv2_b
    
    # Activation.
    conv2 = tf.nn.relu(conv2)

    # Pooling. Input = 10x10x16. Output = 5x5x16.
    conv2 = tf.nn.max_pool(conv2, ksize=[1, 2, 2, 1], strides=[1, 2, 2, 1], padding='VALID')

    # Flatten. Input = 5x5x16. Output = 400.
    fc0   = flatten(conv2)
    
    # Layer 3: Fully Connected. Input = 400. Output = 200.
    fc1_W = tf.Variable(tf.truncated_normal(shape=(2015, 50), mean = mu, stddev = sigma))
    fc1_b = tf.Variable(tf.zeros(50))
    fc1   = tf.matmul(fc0, fc1_W) + fc1_b
    
    # Activation.
    fc1    = tf.nn.relu(fc1)

    # Layer 4: Fully Connected. Input = 200. Output = 150.
    fc2_W  = tf.Variable(tf.truncated_normal(shape=(50, 25), mean = mu, stddev = sigma))
    fc2_b  = tf.Variable(tf.zeros(25))
    fc2    = tf.matmul(fc1, fc2_W) + fc2_b
    
    # Activation.
    fc2    = tf.nn.relu(fc2)

    # Layer 5: Fully Connected. Input = 150. Output = 10.
    fc3_W  = tf.Variable(tf.truncated_normal(shape=(25, 4), mean = mu, stddev = sigma))
    fc3_b  = tf.Variable(tf.zeros(4))
    logits = tf.matmul(fc2, fc3_W) + fc3_b
    return logits


# In[7]:

### Train your model here.
### Feel free to use as many code cells as needed.

# Features and Labels
x = tf.placeholder(tf.float32, (None, 65, 137, 3))
y = tf.placeholder(tf.int32, (None))
one_hot_y = tf.one_hot(y_train, 4)

# Training Pipeline
rate = 0.001
# image = tf.cast(image, tf.float32)
logits = LeNet(tf.cast(X_train, tf.float32))
cross_entropy = tf.nn.softmax_cross_entropy_with_logits(logits = logits, labels = one_hot_y)
loss_operation = tf.reduce_mean(cross_entropy)
optimizer = tf.train.AdamOptimizer(learning_rate = rate)
training_operation = optimizer.minimize(loss_operation)
 
# Model Evaluation
correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(one_hot_y, 1))
accuracy_operation = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
save_file = './save/model.ckpt'
saver = tf.train.Saver()

def evaluate(X_data, y_data):
    num_examples = len(X_data)
    total_accuracy = 0
    sess = tf.get_default_session()
    for offset in range(0, num_examples, BATCH_SIZE):
        batch_x, batch_y = X_data[offset:offset+BATCH_SIZE], y_data[offset:offset+BATCH_SIZE]
        accuracy = sess.run(accuracy_operation, feed_dict={x: batch_x, y: batch_y})
        total_accuracy += (accuracy * len(batch_x))
    return total_accuracy / num_examples


# Train the Model

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    num_examples = len(X_train)
    validation_accuracy0 = 0
    Accuracy_Array = np.zeros(EPOCHS)
    print("Training...")
    print()
    for i in range(EPOCHS):
        X_train, y_train = shuffle(X_train, y_train)
        for offset in range(0, num_examples, BATCH_SIZE):
            end = offset + BATCH_SIZE
            batch_x, batch_y = X_train[offset:end], y_train[offset:end]
            sess.run(training_operation, feed_dict={x: batch_x, y: batch_y})
            
        validation_accuracy = evaluate(X_cv, y_cv)
        Accuracy_Array[i] = validation_accuracy
        if validation_accuracy > validation_accuracy0:
            saver.save(sess, save_file)
            print("EPOCH {} ...".format(i+1))
            print("Validation Accuracy = {:.10f}".format(validation_accuracy))
            print("Model saved")
            print()
            validation_accuracy0 = validation_accuracy
        
    print("Job is done!")
    print("Best Validation Accuracy = {:.10f}".format(validation_accuracy0))
    print()
    plt.plot(Accuracy_Array)
    plt.ylabel('Validation accuracy value')
    print('Validation accuracy curve')
    plt.show()
    #saver.save(sess, 'lenet')
    #print("Model saved")


# In[ ]:




# In[ ]:



