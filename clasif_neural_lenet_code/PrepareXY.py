
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

# Create lists of image filenames:
green_images_1 = glob.glob('bag_dump_just_traffic_light/green/*.jpg')
green_images_2 = glob.glob('bag_dump_loop_with_traffic_light/green/*.jpg')
yellow_images_1 = glob.glob('bag_dump_just_traffic_light/yellow/*.jpg')
yellow_images_2 = glob.glob('bag_dump_loop_with_traffic_light/yellow/*.jpg')
red_images_1 = glob.glob('bag_dump_just_traffic_light/red/*.jpg')
red_images_2 = glob.glob('bag_dump_loop_with_traffic_light/red/*.jpg')
no_light_images_1 = glob.glob('bag_dump_just_traffic_light/nolight/*.jpg') 
no_light_images_2 = glob.glob('bag_dump_loop_with_traffic_light/nolight/*.jpg') 


# In[3]:

# Plot number of images in each list:
count1 = 0
for i in green_images_1:
    count1 += 1
print(count1)
count2 = 0
for i in green_images_2:
    count2 += 1
print(count2)
count3 = 0
for i in yellow_images_1:
    count3 += 1
print(count3)
count4 = 0
for i in yellow_images_2:
    count4 += 1
print(count4)
count5 = 0
for i in red_images_1:
    count5 += 1
print(count5)
count6 = 0
for i in red_images_2:
    count6 += 1
print(count6)
count7 = 0
for i in no_light_images_1:
    count7 += 1
print(count7)
count8 = 0
for i in no_light_images_2:
    count8 += 1
print(count8)
count = count1 + count2 + count3 + count4 + count5 + count6 + count7 + count8 
print(count)


# In[4]:

# Example of image:
number = 4
#image = mpimg.imread(str(red_images_2[number]))
image = cv2.imread(str(green_images_2[number]))
image = image[100:image.shape[0]-350, 0:image.shape[1]]
image_to_show = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
plt.title('image: {}'.format(number))
imshow(image_to_show)
show()


# In[5]:

# Example of resized image:
res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
plt.title('resized: {}'.format(number))
res_to_show = cv2.cvtColor(res, cv2.COLOR_BGR2RGB)
imshow(res_to_show)
show()


# In[6]:

# Image shape. Should be 129/274/3
print(res.shape[0])
print(res.shape[1])
print(res.shape[2])
print(res.shape)


# In[7]:

#X = np.zeros((count, res.shape[0], res.shape[1], res.shape[2]))
#for i in range(count1):
#    X[i,:,:,:] = res
#print(X.shape)
#
#plt.title('From X: {}'.format(number))
#res2 = (X[number, :, :, :])
#imshow(res2, cmap = 'winter')
#show()


# In[8]:

X = np.zeros((count, res.shape[0], res.shape[1], res.shape[2]), dtype=np.int)
Y = np.zeros(count, dtype=np.int)

count1 = 0
for i in green_images_1:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count1,:,:,:] = res
    Y[count1] = 1
    count1 += 1
print(count1)

count2 = count1
for i in green_images_2:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count2,:,:,:] = res
    Y[count2] = 1
    count2 += 1
print(count2)

count3 = count2
for i in yellow_images_1:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count3,:,:,:] = res
    Y[count3] = 2
    count3 += 1
print(count3)

count4 = count3
for i in yellow_images_2:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count4,:,:,:] = res
    Y[count4] = 2
    count4 += 1
print(count4)

count5 = count4
for i in red_images_1:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count5,:,:,:] = res
    Y[count5] = 0
    count5 += 1
print(count5)

count6 = count5
for i in red_images_2:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count6,:,:,:] = res
    Y[count6] = 0
    count6 += 1
print(count6)

count7 = count6
for i in no_light_images_1:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count7,:,:,:] = res
    Y[count7] = 3
    count7 += 1
print(count7)

count8 = count7
for i in no_light_images_2:
    image = cv2.imread(str(i))
    image = image[100:image.shape[0]-350, 0:image.shape[1]]
    res = cv2.resize(image,None,fx=0.1, fy=0.1, interpolation = cv2.INTER_CUBIC)
    X[count8,:,:,:] = res
    Y[count8] = 3
    count8 += 1
print(count8)


# In[9]:

List = [10, 340, 474, 600, 800, 950, 1000, 1792]
for j in List:
    imshow(X[j, :, :, :])
    print(Y[j])
    show()


# In[11]:

f = open(r'fileX.txt', 'wb')
pickle.dump(X, f)
f.close()
f = open(r'fileY.txt', 'wb')
pickle.dump(Y, f)
f.close()


# In[ ]:



