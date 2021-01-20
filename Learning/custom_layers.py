""" Package providing multiple custom layers for neural networs
 Includes
  - a layer to generate covariance matices from output vectors
  - a layer to calculate the eigenvectors of a covariance matrix defined by output vectors
  - a layer to normalise output vectors
"""
import tensorflow as tf

class Covariance_layer(tf.keras.layers.Layer):
    """ A layer that calculates a covariance matrix
        by calculating the sum of all the inner products of each vector  """
    def build(self, input_shape):
        self.numVectors = tf.constant(input_shape[-2], dtype=tf.float32)
    @tf.function
    def call(self, pred_vects):
        sigma = tf.matmul(pred_vects, pred_vects, transpose_a=True)
        # Divide by m to normalise sigma invariant of number of input vectors
        sigma = sigma / self.numVectors
        # Add epsilon to sigma to avoid zeros
        sigma = tf.add(sigma, tf.eye(sigma.shape[-1], batch_shape=sigma.shape[:0])*\
                       tf.keras.backend.epsilon())
        return sigma

class Eigenvector_layer(tf.keras.layers.Layer):
    """ A layer that receives a set of vectors and computes the eigenvectors and -values of the covariance matrix
    The Eigenvectors are sorted by their eigenvalues in descending order.
    Each eigenvector is represented by a column in the output Tensor
    params:
      returnSingleVector=False : weather to return only the eigenvector with the highest eigenvalue
      returnEigValues=True : Weather to not return the corresponding eigenvalues as (eigVectors, eigValues)
    """
    def __init__(self, returnSingleVector=False, returnEigValues=True, **kwargs):
        super().__init__(kwargs)
        if returnSingleVector:
            self.call = self.call_single_vec
        elif returnEigValues:
            self.call = self.call_eigenValues

    @tf.function
    def getSigma(self, pred_vects):
        """Calculates Sigma (Covariance Matrix) based on the network output pred_vects
        Format pred_vects: (batch_size x m x n) , with n=number of dimensions
        Calculation: Sum of outer Products of every set of dimensions with itself and
            divide by number of sets to normalise . (and add epsilon)
        Method: for each batch: pred_vects^T    @     pred_vects
                                {shape n x m} matmul {shape n x m}
        Returns sigma of shape (batch_size x n x n)

        Adds an epsilon of tf.keras.backend.epsilon() (*Identity Matrix) to ensure Invertabillity
        """
        sigma = tf.matmul(pred_vects, pred_vects, transpose_a=True)
        # Divide by m to normalise sigma invariant of number of input vectors
        sigma = sigma / pred_vects.shape[-2]
        # Add epsilon to sigma to avoid zeros
        sigma = tf.add(sigma, tf.eye(sigma.shape[-1], batch_shape=sigma.shape[:0])*\
                       tf.keras.backend.epsilon())
        return sigma

    @tf.function
    def getEigen(self, input_value):
        """Returns Eigenvalues, Eigenvectors"""
        sigma = self.getSigma(input_value)
        return tf.linalg.eigh(sigma)

    @tf.function
    def call_single_vec(self, input_value):
        """Returns only a single (the eigenvalue-largest) eigenvector"""
        _, eigVectors = self.getEigen(input_value)
        return eigVectors[:,:,-1]

    @tf.function
    def call_eigenValues(self, input_value):
        """Returns Eigenvectors and respective eigenvalues in order of descending eigenvalues"""
        eigValues, eigVectors = self.getEigen(input_value)
        return tf.reverse(eigVectors, [-1]), tf.reverse(eigValues, [-1])

    @tf.function
    def call(self, input_value):
        _, eigVectors = self.getEigen(input_value)
        return tf.reverse(eigVectors, [-1])

class PathNormalisation_layer(tf.keras.layers.Layer):
    """ A layer that normalises a Tensor in its last axis"""
    @tf.function
    def call(self, input_value):
        return tf.linalg.normalize(input_value, axis=-1)[0]
