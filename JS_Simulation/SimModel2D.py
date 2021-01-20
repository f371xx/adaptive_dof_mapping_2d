"""Package to handle model loading for RGB or 8-dim vector models
"""
import math
import sys
import pathlib
import cv2
import numpy as np
import tensorflow as tf
tf.config.experimental.set_memory_growth((tf.config.list_physical_devices('GPU'))[0], True)
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent/"Learning/"))
import custom_layers

MODEL_FOLDER = "namedModels"
MODEL_JSON_FILENAME    = "model.json"
MODEL_WEIGHTS_FILENAME = "model_weights.h5"
MODEL_ABOUT_FILENAME = "about.txt"


def getNamedModels():
    """Return the available models by name"""
    return [str(pi.stem) for pi in pathlib.Path(MODEL_FOLDER).glob("**/*") if pi.is_dir()]

class Model2D:
    """Main class to handle 2D models"""
    def __init__(self, chosenModel, loadSmallerModel=False):
        """Load and initialise model
         param chosenModel: Name of the model
         param loadSmallerModel: if to load the model without the two last layers (default:False)
        """
        modelDir = f"{MODEL_FOLDER}/{chosenModel}"
        with open(f"{modelDir}/{MODEL_JSON_FILENAME}") as file:
            self.model = tf.keras.models.model_from_json(file.read(),
                custom_objects={'tf': tf, 'custom_layers': custom_layers,
                                'PathNormalisation_layer' :custom_layers.PathNormalisation_layer,
                                'Covariance_layer': custom_layers.Covariance_layer})
            self.model.load_weights(f"{modelDir}/{MODEL_WEIGHTS_FILENAME}")
        self.modelUsesSigma = isinstance(self.model.layers[-1], custom_layers.Covariance_layer)
        if not self.modelUsesSigma:
            self.eigVectorLayer = custom_layers.Eigenvector_layer()
        # load about file if possible
        try:
            with open(f"{modelDir}/{MODEL_ABOUT_FILENAME}") as file:
                self.__about = file.read()
        except FileNotFoundError:
            self._about = "About file not found"
        if loadSmallerModel:
            self.smallerModel = tf.keras.Model(self.model.input, self.model.layers[-2].output)
        if len(self.model.input_shape) == 4: # handle CNNs
            self.transformEnvironment = self.transformEnvironment_to_rgb
        if '_poles' not in chosenModel: # ugly handling of poles
            self.drawPoles = lambda _,_1,_2: None

    def getAboutFile(self):
        """Return the content of the about file"""
        return self.__about

    @tf.function
    def transformEnvironment(self, environment):
        """Transforms the Environment (model input): target, box1, box2
        Transforms from local cartesian to local polar coordinate with inverse distance
        Value range angles: -1 to 1, with 0 being forwards
                    distance: 0=infinite distance, theoretical 1= same position
        """
        def getDist(pos):
            """ calulate inverse distance to position"""
            return tf.math.reciprocal_no_nan(tf.norm(pos))*10
        def getAngle(pos):
            """ calculate angle (radians/pi) to position"""
            return tf.math.atan2(pos[1], pos[0])/np.pi
        def getRotation(delta_rot):
            """ calculate relative rotation"""
            return delta_rot/np.pi
        return tf.stack([
                getDist(environment[0:2]), # distance to target
                getAngle(environment[0:2]),# angle to target
                getDist(environment[2:4]), # distance to box1
                getAngle(environment[2:4]),# angle to box1
                getRotation(environment[4]),# rotation of box 1
                getDist(environment[5:7]), # distance to box 2
                getAngle(environment[5:7]),# angle to box2
                getRotation(environment[7]) # rotation of box 2
               ])

    def drawPoles(self, img, boxPose, boxsize):
        """Draws a pole with 5x30 box
         param img: image to draw on
         param boxPose: pose (position and rotation) of box
         param boxsize: sidelength of box
        """
        angle = boxPose[2]
        localOffset = boxsize/2.+15
        a = 0.16514867741462683827912828964394 # angle in a 5x30 rectangle
        l = 15.206906325745549222499210613005 # half diagonal in a 5x30 rectangle

        si, co = math.sin(angle+a), math.cos(angle+a)
        siM, coM = math.sin(angle-a), math.cos(angle-a)
        cv2.fillConvexPoly(img, np.int32(np.array([[-co,-si],[-coM,-siM],[co,si],[coM,siM]])*l+\
            np.array(boxPose[:2])+(0,300)+(math.cos(angle)*localOffset, \
                                           math.sin(angle)*localOffset)), (0,0,1))

    def drawSquare(self, img, pose, size):
        """Draws a box on an image, potentially with pole
         param img: Image to draw on
         param pose: Pose (position and rotation) of the center of the square
         param size: size of the square
        """
        angle = pose[2] + math.pi/4
        the_size = size/math.sqrt(2)
        co, si = np.cos(angle)*the_size, np.sin(angle)*the_size
        cv2.fillConvexPoly(img, np.int32(np.array([[-si,co],[co,si],[si, -co],[-co, -si]])+\
                                         np.array(pose[:2]))+(0,300), (0,0,1))
        self.drawPoles(img, pose, size)

    def _getRGB_cv(self, feature):
        """Plot a point in the dataset"""
        img = np.zeros((600,600,3), dtype=np.float32) # create empty canvas
        cv2.circle(img, (int(feature[0]),int(feature[1]+300)), 10, (1,0,0), -1) # draw target
        self.drawSquare(img, feature[2:5], 20)
        self.drawSquare(img, feature[5:8], 30)
        return [img]
    @tf.function
    def transformEnvironment_to_rgb(self, environment):
        """ Transforms the environment to rgb images
         param environment: feature vector
        """
        return tf.reshape(tf.numpy_function(self._getRGB_cv, [environment], \
                                            tf.float32), (600,600,3))

    def updateDofs(self, state):
        """Use the model to update the DoFs
         param state: the environment as model input
        """
        state = tf.expand_dims(self.transformEnvironment(tf.convert_to_tensor(state)),0)
        pred = self.model.predict(state)
        if self.modelUsesSigma:
            eigVals, eigVects = tf.linalg.eigh(pred)
            eigVals, eigVects = tf.reverse(eigVals, [-1]), tf.reverse(eigVects, [-1])
        else:
            if tf.rank(pred) == 2:
                pred = tf.expand_dims(pred, 0)
            eigVects, eigVals = self.eigVectorLayer(pred)
        dofs = tf.transpose(eigVects[0]).numpy() # hand over
        return dofs, eigVals.numpy()[0]

    def getValue_smallerModel(self, state):
        """Returns the output of the internal smaller model
        Behaviour undefined if loadSmallerModel==False
        """
        state = tf.expand_dims(self.transformEnvironment(tf.convert_to_tensor(state)),0)
        return self.smallerModel.predict(state)
