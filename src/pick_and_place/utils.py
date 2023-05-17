
""" https://github.com/Oongking/bin_picking_project/blob/main/bin_picking/script/utils.py """

#ROS

import rospy
import tf
from tf.transformations import *

# Utility
import numpy as np
import copy
from matplotlib import pyplot as plt

# msg & convert
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ctypes import * # convert float to uint32

# Gripper
import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg

# Image
import cv2


# convert_rgbUint32_to_tuple = lambda rgb_uint32: (
#     (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
# )
# convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
#     int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
# )

# Camera Matrix
K = np.array([[619.5831171360619, 0, 310.7132865814095],
                                       [0, 622.0241548644699, 266.682630120719],
                                       [0, 0, 1]])
# Distrotion coefficients
dist = np.array([[0.115261367003599, -0.201086925594918, -0.001387151733832904, 0.001893161766540489, 0]])

def normalvector(base, cross1, cross2):
    vector1 = np.asarray(np.subtract(cross1,base),dtype = np.float64)
    vector2 = np.asarray(np.subtract(cross2,base),dtype = np.float64)
    normalVec = np.cross(vector1,vector2)
    UNormalVec = normalVec/np.linalg.norm(normalVec)
    print("Normal : ",UNormalVec)
    return UNormalVec

# AruCo marker parameters
arucoParams = cv2.aruco.DetectorParameters_create() 
arucoDictA4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
arucoboardA4 = cv2.aruco.GridBoard_create(5, 7, 0.024, 0.01, arucoDictA4)

# A4
A4_usbcam_offset_x=0.5*((0.0315*7)+(0.0081*(7-1)))
A4_usbcam_offset_y=0.5*((0.0315*5)+(0.0081*(5-1)))

# Instant convert cv2 and sensor_msg/Image 
bridge=CvBridge()


def Rx(theta):
	theta = np.radians(theta)
	return np.matrix([[ 1, 0           , 0           ],
                   [ 0, np.cos(theta),-np.sin(theta)],
                   [ 0, np.sin(theta), np.cos(theta)]])
  
def Ry(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), 0, np.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-np.sin(theta), 0, np.cos(theta)]])
  
def Rz(theta):
	theta = np.radians(theta)
	return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find the rotation matrix that aligns vec1 to vec2
    :param vec1: A 3d "source" vector
    :param vec2: A 3d "destination" vector
    :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
    """
    a = (vec1 / np.linalg.norm(vec1)).reshape(3)
    b = (vec2 / np.linalg.norm(vec2)).reshape(3)
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    angle = np.arccos(c)
    return rotation_matrix,angle

class sphere:
    def __init__(self, center, radius=10, color=(0, 0, 255)):
        self.center = center
        self.radius = radius
        self.color = color
    def draw(self, image):
        cv2.circle(image, self.center, self.radius, self.color, -1)
    
    
    
class Usb_cam():
    def __init__(self):
        rospy.loginfo(": Starting camera_class ::")
        
        # Subscribe Image 
        rospy.Subscriber("/usb_cam/image_raw", 
                         Image, self.image_callback)
        
        self.image = None
        
    def get_image(self):
        while 1:
            if self.image is None:
                break
        image = copy.deepcopy(self.image)
        
        self.image = None
        return image
    
    def image_callback(self, received_image):
        try:
            img = bridge.imgmsg_to_cv2(received_image, 
                                   desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        
""" ######################################################################################### """
# K-mean


def euclidean_distance(one_sample,X):
    #transfer one_sample into 1D vector
    one_sample = one_sample.reshape(1,-1)
    #transfer X into 1D vector
    X = X.reshape(X.shape[0],-1)
    #this is used to make sure one_sample's dimension is same as X
    distances = np.power(np.tile(one_sample,(X.shape[0],1))-X,2).sum(axis=1)
    return distances

class Kmeans():
    #constructor
    def __init__(self,k=2,max_iterations=1500,tolerance=0.00001):
        self.k = k
        self.max_iterations = max_iterations
        self.tolerance = tolerance
    
    #randomly select k centroids
    def init_random_centroids(self,X):
        #save the shape of X
        n_samples, n_features = np.shape(X)
        #make a zero matrix to store values
        centroids = np.zeros((self.k,n_features))
        #bcs there is k centroids, so we loop k tiems
        for i in range(self.k):
            #selecting values under the range radomly
            centroid = X[np.random.choice(range(n_samples))]
            centroids[i] = centroid
        return centroids

    #find the closest centroid of a sample
    def closest_centroid(self,sample,centroids):
        distances = euclidean_distance(sample,centroids)
        #np.argmin return the indices of the minimum of distances
        closest_i = np.argmin(distances)
        return closest_i

    #determine the clusers
    def create_clusters(self,centroids,X):
        n_samples = np.shape(X)[0]
        #This is to construct the nested list for storing clusters
        clusters = [[] for _ in range(self.k)]
        for sample_i, sample in enumerate(X):
            centroid_i = self.closest_centroid(sample,centroids)
            clusters[centroid_i].append(sample_i)
        return clusters

    #update the centroids based on mean algorithm
    def update_centroids(self,clusters,X):
        n_features = np.shape(X)[1]
        centroids = np.zeros((self.k,n_features))
        for i, cluster in enumerate(clusters):
            centroid = np.mean(X[cluster],axis=0)
            centroids[i] = centroid
        return centroids

    #obtain the labels
    #same cluster, same y_pred value
    def get_cluster_labels(self,clusters,X):
        y_pred = np.zeros(np.shape(X)[0])
        for cluster_i, cluster in enumerate(clusters):
            for sample_i in cluster:
                y_pred[sample_i] = cluster_i
        return np.array(y_pred,dtype=np.int64)

    #predict the labels
    def predict(self,X):
        #selecting the centroids randomly
        centroids = self.init_random_centroids(X)

        for _ in range(self.max_iterations):
            #clustering all the data point
            clusters = self.create_clusters(centroids,X)
            former_centroids = centroids
            #calculate new cluster center
            centroids = self.update_centroids(clusters,X)
            #judge the current difference if it meets convergence  
            diff = centroids - former_centroids
            if diff.any() < self.tolerance:
                break
            
        return self.get_cluster_labels(clusters,X) 

""" ######################################################################################### """
# Function
def resize(img,scale_percent):
    # scale_percent = 50 # percent of original size
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)
    
        # resize image
    resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)
    return resized

def workspace_ar_set(image, camera = 'usbcam', show=False):
    if camera == 'usbcam':
        boardA4 = arucoboardA4
        matrix_coefficients = K
        distortion_cofficints = dist
        offset_x = A4_usbcam_offset_x
        offset_y = A4_usbcam_offset_y
    
    rvec=None
    tvec=None
    # markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(image, arucoDictA4, parameters = arucoParams)
    (corners, ids,rejected) = cv2.aruco.detectMarkers(image, arucoDictA4, parameters = arucoParams)
    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(image,corners,ids)
        _,rvec,tvec = cv2.aruco.estimatePoseBoard(corners, ids, boardA4, matrix_coefficients, distortion_cofficints, rvec, tvec)
        
    transformation_matrix = np.eye(4, dtype=float)
    transformation_matrix[:3, :3], = cv2.Rodrigues(rvec)
    q = tf.transformations.quaternion_from_matrix(transformation_matrix)
    
    vec = [offset_x, offset_y, 0, 0]
    global_offset = quaternion_multiply(quaternion_multiply(q, vec), quaternion_conjugate(q))
    tvec[0]=tvec[0]+global_offset[0]
    tvec[1]=tvec[1]+global_offset[1]
    tvec[2]=tvec[2]+global_offset[2]
    transformation_matrix[ :3, 3] = np.asarray(tvec).transpose()
    
    if show:
        if rvec is not None and tvec is not None:
            cv2.aruco.drawAxis(image, matrix_coefficients, distortion_cofficints, rvec, tvec, 0.08)
        while True:
            cv2.imshow("Original Image",image)
            if cv2.waitKey(1) & 0xFF==ord('q'):
                cv2.destroyAllWindows()
                break        

        
        return transformation_matrix, image



        
def genCommand(keyword):
        """Update the command according to the character entered by the user."""

        if keyword == 'activate':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

        if keyword == 'reset':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 0
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

        if keyword == 'close':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 255

        if keyword == 'full_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 0

        if keyword == 'quarter_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 123 # speed
            command.rFR  = 5 # Force

            command.rPR = 150

        if keyword == 'half_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 123 # speed
            command.rFR  = 5 # Force

            command.rPR = 127

        if keyword == 'release_open':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 200

        if keyword == 'grip_close':
            command = outputMsg.Robotiq2FGripper_robot_output()
            command.rACT = 1
            command.rGTO = 1
            command.rSP  = 255 # speed
            command.rFR  = 5 # Force

            command.rPR = 250


        #If the command entered is a int, assign this value to rPRA
        # try:
        #     command.rPR = int(keyword)
        #     if command.rPR > 255:
        #         command.rPR = 255
        #     if command.rPR < 0:
        #         command.rPR = 0
        # except ValueError:
        #     pass

            # Speed control
        # if keyword == 'f':
        #     command.rSP += 25
        #     if command.rSP > 255:
        #         command.rSP = 255
        # if keyword == 'l':
        #     command.rSP -= 25
        #     if command.rSP < 0:
        #         command.rSP = 0
            # Force control
        # if keyword == 'i':
        #     command.rFR += 25
        #     if command.rFR > 255:
        #         command.rFR = 255
        # if keyword == 'd':
        #     command.rFR -= 25
        #     if command.rFR < 0:
        #         command.rFR = 0

        return command
