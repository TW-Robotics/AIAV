#!/usr/bin/env python
# coding: utf-8

# In[2]:


# load MNIST fashion dataset
# Load it into test and train varialbes and the according labels
from keras.datasets import fashion_mnist
(trainX, trainy), (testX, testy) = fashion_mnist.load_data()


# In[3]:


# reshape Train Set, so that it is n x 784, instead of n x 28 x 28
nsamples, nx, ny = trainX.shape
trainX_2d = trainX.reshape((nsamples,nx*ny))


# In[4]:


# perform PCA on the training set, first fit and then transform
# so that the set has the shape: n x 10 (components) 
from sklearn.decomposition import PCA
pca = PCA(n_components=160)
X_r = pca.fit(trainX_2d).transform(trainX_2d)


# In[ ]:


# Load SKLearn libraries, neccessary for next steps
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.pipeline import Pipeline
from sklearn.metrics import accuracy_score 


# In[ ]:


# Create a Scaler that will normalize the Data, Create LogisticRegression Model and use a Pipeline to stitch them together
scaler = StandardScaler()
lr = LogisticRegression(max_iter=10000,n_jobs=5, class_weight='balanced', solver='sag')
model = Pipeline([('standardize', scaler),
                    ('log_reg', lr)])
model.fit(X_r, trainy)


# In[ ]:


# reshape Test Set, so that it is n x 784, instead of n x 28 x 28
nsamples, nx, ny = testX.shape
testX_2d = testX.reshape((nsamples,nx*ny))


# In[ ]:


# perform PCA on the test set, transform it according to other set
X_t = pca.transform(testX_2d)


# In[ ]:


# Predict the Labels for the new test data, compare them to the known values 
# And calculate how accurate the prediction is
pred = model.predict(X_t);
train_accuracy = accuracy_score(testy, pred)*100
print('Train accuracy is: '+str(train_accuracy)+'%')

