## Dog Classifier

This dog classifier pipeline is a project from [Deep Learning Nanodegree] at Udacity and
can be used within a web or mobile app to process real-world, user-supplied images.
Given an image of a dog, your algorithm will identify an estimate of the canine’s breed.
If supplied an image of a human, the code will identify the resembling dog breed.  

![Sample Output][Image1]

This project uses state-of-the-art CNN models for classification and localization.
In here you can find a series of models designed to perform various tasks in a data processing pipeline.
Each model has its strengths and weaknesses, and engineering a real-world application often involves solving many
problems without a perfect answer. 

## Datasets

1. [Dog dataset]: Unzip the folder and place it in the repo, at location `path/to/dog-project/dog_images`.
  The `dog_images/` folder should contain 133 folders, each corresponding to a different dog breed.
2. [Human dataset]: Unzip the folder and place it in the repo, at location `path/to/dog-project/lfw`.
  If you are using a Windows machine, you are encouraged to use [7zip] to extract the folder. 

## Jupyter Notebook

Open a terminal window and navigate to the project folder.
Open the notebook and follow the instructions.

__NOTE:__ I recommend using Jupyter Lab which is much more powerful to create Jupyter projects than
Jupyter Notebook. Follow this [link][Jupyter Lab Installation] for documentation on how to install
Jupyter Lab.

[Image1]: ./images/sample_dog_output.png "Sample Output"
[Image2]: ./images/vgg16_model.png "VGG-16 Model Layers"
[Image3]: ./images/vgg16_model_draw.png "VGG16 Model Figure"

[Dog dataset]: https://s3-us-west-1.amazonaws.com/udacity-aind/dog-project/dogImages.zip
[Human dataset]: http://vis-www.cs.umass.edu/lfw/lfw.tgz
[7zip]: http://www.7-zip.org/
[Jupyter Lab Installation]: https://jupyter.org/install
[Deep Learning Nanodegree]: https://www.udacity.com/course/deep-learning-nanodegree--nd101
