#install.packages('keras')
library(keras)        
#install_keras()  
library(nnet)

# load mnist data set
fashion_mnist <- keras::dataset_fashion_mnist()

# unpack training and test images and labels
c(train.images, train.labels) %<-% fashion_mnist$train
c(test.images, test.labels) %<-% fashion_mnist$test

# change dimension of train images, as well as test images
dim(train.images) <- c(dim(train.images)[1], dim(train.images)[2]*dim(train.images)[3])
dim(test.images) <- c(dim(test.images)[1], dim(test.images)[2]*dim(test.images)[3])


# Carry out pca
pca <- prcomp(train.images, scale. = TRUE, center = TRUE, rank = 10)

# Fit PCA to the data
train.components <- predict(pca, newdata = train.images)
test.components  <- predict(pca, newdata = test.images)


# Create Model
model <- multinom(train.labels ~ ., family = "multinomial", data = data.frame(train.components), maxit=150);

# Create prediction
results <- predict(model, newdata=test.components, type='probs')

# Get maximum of column, and decrease prediction by one, so it is in correct format
prediction <- max.col(results)
prediction <- prediction - 1

# calculate the average of correctly labeled classes and show as accuracy
cl <- mean(prediction != test.labels)
print(paste('Accuracy', 1 - cl))
