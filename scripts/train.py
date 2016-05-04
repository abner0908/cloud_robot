# USAGE
# python train.py --dataset data/digits.csv --model models/svm.cpickle

# import the necessary packages
from sklearn.externals import joblib
from sklearn.svm import LinearSVC
from pyimagesearch.hog import HOG
from pyimagesearch import dataset
import argparse

dataset_path = "data/digits.csv"
models_path = "models/svm.cpickle_01"

# load the dataset and initialize the data matrix
(digits, target) = dataset.load_digits(dataset_path)
data = []

# initialize the HOG descriptor
hog = HOG(orientations=18, pixelsPerCell=(10, 10),
          cellsPerBlock=(1, 1), normalize=True)

# loop over the images
for image in digits:
    # deskew the image, center it
    image = dataset.deskew(image, 20)
    image = dataset.center_extent(image, (20, 20))

    # describe the image and update the data matrix
    hist = hog.describe(image)
    data.append(hist)

# train the model
model = LinearSVC(random_state=42)
model.fit(data, target)

# dump the model to file
joblib.dump(model, models_path)
print "create model done..."
