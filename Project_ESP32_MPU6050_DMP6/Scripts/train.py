import csv
import pandas
import numpy
import tensorflow as tf
import matplotlib.pyplot as plt
import os
from mlxtend.plotting import plot_decision_regions

# https://stackoverflow.com/questions/55424906/building-svm-with-tensorflows-linearclassifier-and-pandas-dataframes
# https://aiplanet.com/notebooks/2418/binthamza/svm-classification-with-tfkeras-model
# https://stackoverflow.com/questions/55424906/building-svm-with-tensorflows-linearclassifier-and-pandas-dataframes
# https://www.kaggle.com/code/shadesh/svm-classification-using-tensorflow-and-keras

# with open("C:\\MIoT_Device\\Project\\Project_ESP32_MPU6050_DMP6_DataGather\\Scripts\\FeatureExtractDrop.csv", newline='') as file:
#     dataDrop = list(csv.reader(file))

cwd = os.getcwd()
dataDrop = pandas.read_csv(cwd + "\\Data\\FeatureExtractDrop.csv").values
dataWalk = pandas.read_csv(cwd + "\\Data\\FeatureExtractWalk.csv").values

# Features
x = dataDrop[:, 1:]
x = numpy.append(x,dataWalk[:,1:], axis=0)

# Labels
y = dataDrop[:, 0]
y = numpy.append(y,dataWalk[:,0], axis=0)

print("x.shape: " + str(x.shape))
print("y.shape: " + str(y.shape))

# plt.scatter(x[:,0], x[:,1], c=y, cmap=plt.cm.Set1)
plt.scatter(dataDrop[:,1], dataDrop[:,2], c='red', label='Fall')
plt.scatter(dataWalk[:,1], dataWalk[:,2], c='blue', label='Walk')
plt.xlabel("axyStdDevL")
plt.ylabel("axyStdDevR")
plt.legend(loc="upper right")
plt.show()

def get_model():
    #Linear SVM model
    model = tf.keras.Sequential()
    # Metric: is a function to assess model performance. Similar to loss functions, except that the results from evaluating a metric are not used when training the model. Note that you may use any loss function as a metric.
    # Optimizer: adam is a commonly used one. Learning rate: step size when adjusting paramters in model (usually 0.001 to 0.1). Larger learning rate reduces iterations but reduces accuracy, smaller rate gives higher accuracy and more iterations. https://stackoverflow.com/questions/59737875/keras-change-learning-rate
    # Dense layer of 1 means that this layer has one neuron (unit). Since only one layer (output layer) is being added with one neuron, it's linear.
    # Activation function: mathematical function applied to the neuron output in a neural layer. These are used to introduce non-linearity in a neural network. Linear (pass through): input unmodified. https://keras.io/api/layers/activations/.
    # Kernel regularizer: Typical values for the regularisation parameters are 1 to 10, 10 being default. https://keras.io/api/layers/regularizers/. L1 and L2 regularizers have default of 0.01. The L2 regularization penalty is computed as: loss = l2 * reduce_sum(square(x)).Kernel regularizer tries to reduce the weights (excluding bias). https://stats.stackexchange.com/questions/383310/what-is-the-difference-between-kernel-bias-and-activity-regulizers-and-when-t
    # Loss: Hinge loss is equivalent to slack variable. Custom loss function used for hinge loss https://keras.io/api/losses/. https://keras.io/api/losses/hinge_losses/.
    # Kernel: linear, polynomial, guassian, sigmoid, etc
    optmzr = tf.keras.optimizers.Adam(learning_rate=0.01)
    model.add(tf.keras.layers.Dense(1, activation='linear', kernel_regularizer=tf.keras.regularizers.l2(1)))
    model.compile(optimizer=optmzr, loss=tf.keras.losses.Hinge(), metrics=["accuracy"])
    return model

model = get_model()

# batch_size: Number of samples per gradient update. If unspecified, batch_size will default to 32. The batch size is the number of training examples that you use to perform one step of stochastic gradient descent (SGD). https://stackoverflow.com/questions/61029052/what-is-the-batchsize-in-tensorflows-model-fit-function
history = model.fit(x, y, epochs=100, verbose=False)
model.summary()   
print("Finished training the model")

accuracy = history.history['accuracy']
# val_accuracy = history.history['val_accuracy']
loss = history.history['loss']
# val_loss = history.history['val_loss']

plt.figure(figsize=(5, 5))
plt.xlabel('Epoch Number')
plt.ylabel("Loss Magnitude")
plt.plot(history.history['loss'])

plt.figure(figsize=(5, 5))
plt.plot(accuracy, label='Training Accuracy')
plt.xlabel('Epoch Number')
plt.ylabel('Accuracy')

plt.figure(figsize=(5, 5))
plt.plot(loss, label='Training Loss')
plt.xlabel('Epoch Number')
plt.ylabel('Loss')

# plt.figure(figsize=(5, 5))
# plt.plot(x, y) # model.layers[0].kernel.numpy(), -model.layers[0].bias.numpy()

# model.compile(optimizer="adam", loss="sparse_categorical_crossentropy", metrics=["accuracy"])
# loss, acc = model.evaluate(test_data)
# pred = np.argmax(model.predict(test_data), axis=1)
# confusion = tf.math.confusion_matrix(labels=tf.constant(test_labels), predictions=tf.constant(pred), num_classes=4)
# print(confusion)
# print("Loss {}, Accuracy {}".format(loss, acc))

# history = model.fit(X_train, y_train, validation_data=(X_val, y_val), epochs=10, batch_size=32)

# plot(X, Y, W=model.weight.t().detach().numpy(), b=-model.bias.detach().numpy())

# history = model.fit(train_features, train_labels, epochs=400, batch_size=64, validation_data=(test_features, test_labels))

# print(f' W: {model.weights}')
# w0 = model.get_weights()[0] # weight vector
# w1 = model.get_weights()[1] # bias

# print(model.layers[0].kernel.numpy()) # Weight vector
# print(-model.layers[0].bias.numpy()) # Bias

# prediction = model.predict_generator(test, verbose=1)

# m = - w0[0]/w0[1]
# b = - w1[0]

# xSep          = [x[1] for x in xDataset]
# ySep          = [m * x + b for x in xSep]

# plt.figure(figsize=(5, 5))
# plt.scatter(dataDrop[:,1], dataDrop[:,2], c='red', label='Fall')
# plt.scatter(dataWalk[:,1], dataWalk[:,2], c='blue', label='Walk')
# plt.xlabel("axyStdDevL")
# plt.ylabel("axyStdDevR")
# plt.legend(loc="upper right")

# plot_decision_regions(x, y, clf=model, legend=2)

x_min, x_max = x[:, 0].min() - 1, x[:, 0].max() + 1
y_min, y_max = x[:, 1].min() - 1, x[:, 1].max() + 1
xx, yy = numpy.meshgrid(numpy.arange(x_min, x_max, 0.01),
                         numpy.arange(y_min, y_max, 0.01))
Z = model.predict(numpy.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)
cs = plt.contourf(xx, yy, Z, cmap=plt.cm.coolwarm, alpha=0.8)
plt.scatter(x[:, 0], x[:, 1], c=y, cmap=plt.cm.Set1)
plt.show()
