import csv
import pandas
import numpy
import tensorflow as tf
import matplotlib.pyplot as plt

# https://stackoverflow.com/questions/55424906/building-svm-with-tensorflows-linearclassifier-and-pandas-dataframes

# with open("C:\\MIoT_Device\\Project\\Project_ESP32_MPU6050_DMP6_DataGather\\Scripts\\FeatureExtractDrop.csv", newline='') as file:
#     dataDrop = list(csv.reader(file))

dataDrop = pandas.read_csv("..\\Data\\FeatureExtractDrop.csv").values
dataWalk = pandas.read_csv("..\\Data\\FeatureExtractWalk.csv").values

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

print()

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
model.summary()   

# model.compile(optimizer="adam", loss="sparse_categorical_crossentropy", metrics=["accuracy"])
# loss, acc = model.evaluate(test_data) 
# pred = np.argmax(model.predict(test_data), axis=1)
# confusion = tf.math.confusion_matrix(labels=tf.constant(test_labels), predictions=tf.constant(pred), num_classes=4)
# print(confusion)
# print("Loss {}, Accuracy {}".format(loss, acc))

# history = model.fit(celsius_q, fahrenheit_a, epochs=500, verbose=False)
# print("Finished training the model")
# import matplotlib.pyplot as plt
# plt.xlabel('Epoch Number')
# plt.ylabel("Loss Magnitude")
# plt.plot(history.history['loss'])

# history = model.fit(X_train, y_train, validation_data=(X_val, y_val), epochs=10, batch_size=32)

# print(f' W: {model.weight}')
# plot(X, Y, W=model.weight.t().detach().numpy(), b=-model.bias.detach().numpy())