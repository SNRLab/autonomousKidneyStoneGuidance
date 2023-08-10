# -*- coding: utf-8 -*-
#150 and 24

from __main__ import vtk, qt, ctk, slicer
import math

import os
import math

import time
import numpy as np

import random

from scipy.signal import find_peaks
from sklearn.cluster import KMeans


windowsOS = True
colorInput = True

if windowsOS: import vgamepad as vg

import cv2

stick_x = 0
stick_y = 0
overlayColor = (60,120,232)
opacity = .3
buffer_size = 5
frame_buffer = []

class kidneyStoneNavigation:
  def __init__(self, parent):
    parent.title = "Kidney Stone Navigation"
    parent.categories = ["Utilities"]
    parent.contributors = ["Nick Brady"]
    
    parent.helpText = """
    Add help text
    """
    parent.acknowledgementText = """
""" 
    # module build directory is not the current directory when running the python script, hence why the usual method of finding resources didn't work ASAP
    self.parent = parent
    
    # Set module icon from Resources/Icons/<ModuleName>.png
    moduleDir = os.path.dirname(self.parent.path)
    for iconExtension in ['.svg', '.png']:
      iconPath = os.path.join(moduleDir, 'Resources/Icons', self.__class__.__name__ + iconExtension)
      if os.path.isfile(iconPath):
        parent.icon = qt.QIcon(iconPath)
        break
    

class kidneyStoneNavigationWidget:
  def __init__(self, parent = None):
    if not parent:
      self.parent = slicer.qMRMLWidget()
      self.parent.setLayout(qt.QVBoxLayout())
      self.parent.setMRMLScene(slicer.mrmlScene)
    else:
      self.parent = parent
    self.layout = self.parent.layout()
    self.lastCommandId = 0
    self.timeoutCounter = 0
    if not parent:
      self.setup()
      self.parent.show()
  
  def setup(self):
    SnakeControlLayoutButton = ctk.ctkCollapsibleButton()
    SnakeControlLayoutButton.text = "Snake Control Software"
    self.layout.addWidget(SnakeControlLayoutButton)
    
    openIGTLinkLayout = qt.QFormLayout(SnakeControlLayoutButton)

    #Connect OpenIGTLink connection to Control Software
    self.IGTActive = False
    self.openIGTNode = None

    self.startConnectionButton = qt.QPushButton("Start Connection")
    openIGTLinkLayout.addRow(self.startConnectionButton)
    self.startConnectionButton.connect('clicked(bool)', self.startConnection)    

    self.stopConnectionButton = qt.QPushButton("Stop Connection")
    openIGTLinkLayout.addRow(self.stopConnectionButton)
    self.stopConnectionButton.connect('clicked(bool)', self.stopConnection)    

    DepthLayoutButton = ctk.ctkCollapsibleButton()
    DepthLayoutButton.text = "Depth Map Model"
    self.layout.addWidget(DepthLayoutButton)
    
    depthLayout = qt.QFormLayout(DepthLayoutButton)

    self.rgbSelector = slicer.qMRMLNodeComboBox()
    self.rgbSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
    self.rgbSelector.selectNodeUponCreation = False
    self.rgbSelector.noneEnabled = False
    self.rgbSelector.addEnabled = False
    self.rgbSelector.showHidden = False
    self.rgbSelector.setMRMLScene( slicer.mrmlScene )
    self.rgbSelector.setToolTip( "RGB Image" )
    depthLayout.addRow("RGB Image:", self.rgbSelector)

    self.overlaySelector = slicer.qMRMLNodeComboBox()
    self.overlaySelector.nodeTypes = ["vtkMRMLMarkupsLineNode"]
    self.overlaySelector.selectNodeUponCreation = True
    self.overlaySelector.noneEnabled = True
    self.overlaySelector.addEnabled = True
    self.overlaySelector.showHidden = False
    self.overlaySelector.setMRMLScene( slicer.mrmlScene )
    self.overlaySelector.setToolTip( "Vector Overlay Output" )
    depthLayout.addRow("Vector Overlay Output:", self.overlaySelector)
    
    self.circleOverlaySelector = slicer.qMRMLNodeComboBox()
    self.circleOverlaySelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
    self.circleOverlaySelector.selectNodeUponCreation = True
    self.circleOverlaySelector.noneEnabled = True
    self.circleOverlaySelector.addEnabled = True
    self.circleOverlaySelector.showHidden = False
    self.circleOverlaySelector.setMRMLScene( slicer.mrmlScene )
    self.circleOverlaySelector.setToolTip( "Circle Overlay Output" )
    depthLayout.addRow("Circle Overlay Output:", self.circleOverlaySelector)

    self.methodComboBox = qt.QComboBox()
    self.methodComboBox.addItem("Peaks and Threshold with Blob Detector")
    self.methodComboBox.addItem("Peaks")
    self.methodComboBox.addItem("Threshold and Blob Detector")
    self.methodComboBox.addItem("Threshold and Hough Transform")
    self.methodComboBox.addItem("Naive Deepest Point")
    depthLayout.addRow("Control Strategy:", self.methodComboBox)
    
    # self.runOnceButton = qt.QPushButton("Run Once")
    # depthLayout.addRow(self.runOnceButton)
    # self.runOnceButton.connect('clicked(bool)', self.generateDepthAndSendVector)

    # self.startDepthButton = qt.QPushButton("Toggle depth map") #TODO this is the original
    #"master" button; reenable if necessary TODO
    # self.startDepthButton.setCheckable(True)
    # depthLayout.addRow(self.startDepthButton)
    # self.startDepthButton.connect('clicked(bool)', self.startDepthToggle)
        
    self.dualLockButton = qt.QPushButton("Toggle automatic control")
    self.dualLockButton.setCheckable(True)
    
    self.bendingLockButton = qt.QPushButton("Toggle Bending")
    self.bendingLockButton.setCheckable(True)

    self.linearLockButton = qt.QPushButton("Toggle Linear")
    self.linearLockButton.setCheckable(True)

    self.stopLockButton = qt.QPushButton("Stop")
    self.stopLockButton.connect('clicked(bool)', self.stopToggle)

    controlLockBoxLayout = qt.QHBoxLayout()
    controlLockBoxLayout.addWidget(self.dualLockButton)
    #controlLockBoxLayout.addWidget(self.bendingLockButton)
    #controlLockBoxLayout.addWidget(self.linearLockButton)
    controlLockBoxLayout.addWidget(self.stopLockButton)
    depthLayout.addRow(controlLockBoxLayout)    

    self.reverseToggleButton = qt.QPushButton("Toggle Reverse")
    self.reverseToggleButton.setCheckable(True)
    depthLayout.addRow(self.reverseToggleButton)

    # self.depthFPSrobotBox.setSingleStep(1)
    # self.depthFPSrobotBox.setMaximum(144)
    # self.depthFPSrobotBox.setMinimum(1)
    # self.depthFPSrobotBox.setSuffix(" FPS")
    # self.depthFPSrobotBox.value = 4
    # self.depthFPSrobotBox = qt.QSpinBox()
    # depthLayout.addRow(" Rate:", self.depthFPSrobotBox)
    
    self.linearLimitBox = qt.QDoubleSpinBox()
    self.linearLimitBox.setMinimum(0.0)
    self.linearLimitBox.setMaximum(1.0)
    self.linearLimitBox.setSingleStep(0.1)
    self.linearLimitBox.setValue(0.3)
    self.linearLimitBox.setDecimals(4)
    depthLayout.addRow("Linear Stage Engagement", self.linearLimitBox) 
    
    self.bendingSpeedBox = qt.QDoubleSpinBox()
    self.bendingSpeedBox.setMinimum(0.0)
    self.bendingSpeedBox.setMaximum(3.0)
    self.bendingSpeedBox.setSingleStep(0.1)
    self.bendingSpeedBox.setValue(3.0)
    self.bendingSpeedBox.setDecimals(2)
    depthLayout.addRow("Bending Speed:", self.bendingSpeedBox) 
    
    self.linearStageSpeedBox = qt.QDoubleSpinBox()
    self.linearStageSpeedBox.setMinimum(0.0)
    self.linearStageSpeedBox.setMaximum(3.0)
    self.linearStageSpeedBox.setSingleStep(0.1)
    self.linearStageSpeedBox.setValue(0.5)
    self.linearStageSpeedBox.setDecimals(2)
    depthLayout.addRow("Linear Speed:", self.linearStageSpeedBox)      
    
    self.depthTimer = qt.QTimer()
    self.depthTimer.timeout.connect(self.mainCodeRunner)    
    
    if windowsOS: 
      self.gamepad = vg.VX360Gamepad()

    self.thresholdBox = qt.QSpinBox()
    self.thresholdBox.setSingleStep(1)
    self.thresholdBox.setMaximum(100)
    self.thresholdBox.setMinimum(0)
    self.thresholdBox.setSuffix("%")
    self.thresholdBox.value = 90
    depthLayout.addRow("Threshold:", self.thresholdBox)

    self.saveImagesBox = qt.QCheckBox()
    self.saveImagesBox.setChecked(False)
    depthLayout.addRow("Save RGB and Depth images", self.saveImagesBox)

    self.imagePathBox = qt.QLineEdit()
    self.imageBrowseButton = qt.QPushButton("...")
    self.imageBrowseButton.clicked.connect(self.select_directory)
    pathBoxLayout = qt.QHBoxLayout()
    pathBoxLayout.addWidget(self.imagePathBox)
    pathBoxLayout.addWidget(self.imageBrowseButton)
    depthLayout.addRow(pathBoxLayout)

    #------------------------------------------------------------------
    #---------------------------Image processing------------------------------

    controllerCurveButton = ctk.ctkCollapsibleButton()
    controllerCurveButton.text = "Controller Curve"
    self.layout.addWidget(controllerCurveButton)
    
    controllerCurve = qt.QFormLayout(controllerCurveButton) #other layout

    self.DeadzoneBox = qt.QDoubleSpinBox()
    self.DeadzoneBox.setSingleStep(1)
    self.DeadzoneBox.setMaximum(100)
    self.DeadzoneBox.setMinimum(1)
    self.DeadzoneBox.value = 40
    self.DeadzoneBox.setDecimals(0)
    controllerCurve.addRow("Deadzone:", self.DeadzoneBox)

    self.N_valueBox = qt.QSpinBox()
    self.N_valueBox.setSingleStep(1)
    self.N_valueBox.setMaximum(100000000)
    self.N_valueBox.setMinimum(1)
    self.N_valueBox.setSuffix(" ")
    self.N_valueBox.value = 10000000
    controllerCurve.addRow("N:", self.N_valueBox)

    self.U_valueBox = qt.QSpinBox()
    self.U_valueBox.setSingleStep(1)
    self.U_valueBox.setMaximum(50000)
    self.U_valueBox.setMinimum(1)
    self.U_valueBox.setSuffix(" ")
    self.U_valueBox.value = 12000
    controllerCurve.addRow("U:", self.U_valueBox)

    self.D_valueBox = qt.QSpinBox()
    self.D_valueBox.setSingleStep(1)
    self.D_valueBox.setMaximum(2000)
    self.D_valueBox.setMinimum(1)
    self.D_valueBox.setSuffix(" ")
    self.D_valueBox.value = 475
    controllerCurve.addRow("D:", self.D_valueBox)

    self.M_valueBox = qt.QDoubleSpinBox()
    self.M_valueBox.setSingleStep(.001)
    self.M_valueBox.setMaximum(.500)
    self.M_valueBox.setMinimum(-.500)
    self.M_valueBox.value = .010
    self.M_valueBox.setDecimals(3)
    controllerCurve.addRow("M:", self.M_valueBox)

    #------ ------- ---------

    imgProcessingButton = ctk.ctkCollapsibleButton()
    imgProcessingButton.text = "Image Processing"
    self.layout.addWidget(imgProcessingButton)
    
    imgProcessing = qt.QFormLayout(imgProcessingButton) #other layout

    self.rgbSelector = slicer.qMRMLNodeComboBox()
    self.rgbSelector.nodeTypes = ["vtkMRMLScalarVolumeNode"]
    self.rgbSelector.selectNodeUponCreation = False
    self.rgbSelector.noneEnabled = False
    self.rgbSelector.addEnabled = False
    self.rgbSelector.showHidden = False
    self.rgbSelector.setMRMLScene( slicer.mrmlScene )
    self.rgbSelector.setToolTip( "RGB Image" )
    imgProcessing.addRow("RGB Image:", self.rgbSelector)

    #----------------Main Button
    self.startDepthButton = qt.QPushButton("Toggle video")
    self.startDepthButton.setCheckable(True)
    imgProcessing.addRow(self.startDepthButton)
    self.startDepthButton.connect('clicked(bool)', self.startDepthToggle)

    self.depthFPSBox = qt.QSpinBox()
    self.depthFPSBox.setSingleStep(1)
    self.depthFPSBox.setMaximum(144)
    self.depthFPSBox.setMinimum(1)
    self.depthFPSBox.setSuffix(" ")
    self.depthFPSBox.value = 100
    imgProcessing.addRow(" FPS multiplier:", self.depthFPSBox)

    self.parameterCBox = qt.QDoubleSpinBox()
    self.parameterCBox.setMinimum(0)
    self.parameterCBox.setMaximum(1000)
    self.parameterCBox.setSingleStep(1)
    self.parameterCBox.setValue(4)
    self.parameterCBox.setDecimals(0)
    imgProcessing.addRow("frame buffer:", self.parameterCBox) 
  
    self.parameterABox = qt.QDoubleSpinBox()
    self.parameterABox.setMinimum(0.0)
    self.parameterABox.setMaximum(255)
    self.parameterABox.setSingleStep(0.1)
    self.parameterABox.setValue(125)
    self.parameterABox.setDecimals(1)
    imgProcessing.addRow("hough canny:", self.parameterABox) 

    self.parameterBBox = qt.QDoubleSpinBox()
    self.parameterBBox.setMinimum(0.0)
    self.parameterBBox.setMaximum(255)
    self.parameterBBox.setSingleStep(0.1)
    self.parameterBBox.setValue(30)
    self.parameterBBox.setDecimals(1)
    imgProcessing.addRow("hough votes:", self.parameterBBox)     

    self.parameterZBox = qt.QDoubleSpinBox()
    self.parameterZBox.setMinimum(0.0)
    self.parameterZBox.setMaximum(255)
    self.parameterZBox.setSingleStep(0.1)
    self.parameterZBox.setValue(40)
    self.parameterZBox.setDecimals(1)
    imgProcessing.addRow("max circles:", self.parameterZBox)   

    self.parameterYBox = qt.QDoubleSpinBox()
    self.parameterYBox.setMinimum(0)
    self.parameterYBox.setMaximum(1000)
    self.parameterYBox.setSingleStep(1)
    self.parameterYBox.setValue(60)
    self.parameterYBox.setDecimals(0)
    imgProcessing.addRow("contrast threshold:", self.parameterYBox)      
    #------------------------------------------------------------------
    self.layout.addStretch(1)
    #------------------------------------------------------------------
    #------------------------------------------------------------------
    #------------------------------------------------------------------

  def mainCodeRunner(self):

    contrastSkip = False
    image = self.extractImage()

    vector = [100,100]

    # cv2.imshow('image', image)

    image = cv2.resize(image, (200,200))

    #-------------initial image processing

    # image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    overlay = np.zeros_like(image)
    overlay[:] = overlayColor
    cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)

    # Apply blurring to the image
    blurred = cv2.blur(image, (10, 10))

    # Threshold the blurred image to find the lightest parts (white areas)
    lower_threshold = np.array([int(self.parameterABox.value), int(self.parameterABox.value), int(self.parameterABox.value)])  # Adjust the threshold values based on your requirements
    upper_threshold = np.array([255, 255, 255])
    thresholded = cv2.inRange(blurred, lower_threshold, upper_threshold)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the contour with the largest area
    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
          center_x = int(M["m10"] / M["m00"])
          center_y = int(M["m01"] / M["m00"])

          # Draw a circle at the center of the largest contour
          cv2.circle(image, (center_x, center_y), 5, (0, 255, 0), -1)

          vector = [center_x-100, center_y-100]

        else: 
          center_x = 100
          center_y = 100

          vector = [center_x, center_y]

    # Display the original frame with the center of the largest contour
    cv2.imshow("Processed Image", image)

    vector = self.calcVector(vector, avoidance=False)

    # cv2.imshow("Contoured", image)
    # print(vector)   
    self.generateDepthAndSendVector(vector) 
  
  def calcVector(self, vector, avoidance):
        
    vector[0] = vector[0]/100
    vector[1] = vector[1]/100

    vector[0] = vector[0]*32768
    vector[1] = vector[1]*32768

    x = vector[0]
    y = vector[1]

    distance = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)

    if avoidance: 
      theta = theta + (math.pi)


    #curving the magniude of the controller output
    n = self.N_valueBox.value
    u = self.U_valueBox.value
    d = self.D_valueBox.value
    m = self.M_valueBox.value
    
    if distance > 0: distance = distance + n/(m*distance+d)-u

    if distance < ((self.DeadzoneBox.value)/100)*32768: distance = 0

    x = distance * math.cos(theta)
    y = distance * math.sin(theta)

    x = int(x)
    y = int(y)

    if x > 32768: x = 32768
    if y > 32768: y = 32768


    vector[0] = x
    vector[1] = y

    print(vector)
    return vector

  def extractImage(self):
    volumeNode = slicer.util.getNode('SnakeImage') #self.rgbselector value whaterver
    a = slicer.util.arrayFromVolume(volumeNode)

    if colorInput:
      image = np.squeeze(a)
      image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
      
    else:

      b0 = a[0]
      b1 = a[1]
      b2 = a[2]
    
      bgr_image = np.zeros((b0.shape[0], b0.shape[1], 3), dtype=np.uint8)
      bgr_image[:, :, 0] = b0  # Blue channel
      bgr_image[:, :, 1] = b1  # Green channel
      bgr_image[:, :, 2] = b2  # Red channel
      image = bgr_image

    # flipped = cv2.flip(image, 0)
    rotated = cv2.rotate(image, cv2.ROTATE_180)

    return rotated
  
  def temporal_averaging(self, frame_buffer):
    # Compute the average of the frames in the buffer
    averaged_frame = sum(frame_buffer) / len(frame_buffer)
    
    # Convert the averaged frame to an unsigned 8-bit integer
    averaged_frame = averaged_frame.astype('uint8')
    
    return averaged_frame  
  
  def limit_random_circles(self, circles, max_circles):
      if circles is None:
          return None

      if len(circles) <= max_circles:
          return circles

      random_indices = random.sample(range(len(circles)), max_circles)
      return circles[random_indices]  
  
  def houghCircleFit(self, image, min_radius, max_radius):
    #minDist is distance between circles; param1 is canny upper value; param2 is number of votes

    average = False
    # Append the frame to the buffer
    frame_buffer.append(image)

    buffer_size = self.parameterCBox.value

    # If the buffer is full, perform temporal averaging
    if len(frame_buffer) == buffer_size:
        # Apply temporal averaging
        image = self.temporal_averaging(frame_buffer)
        average = True
        # Clear the buffer
        frame_buffer.clear()

    circles = cv2.HoughCircles(
        image, cv2.HOUGH_GRADIENT, dp=1, minDist=1,
        param1=self.parameterABox.value, param2=self.parameterBBox.value, 
        minRadius=min_radius, maxRadius=max_radius
    )

    # edges = cv2.Canny(image, 0, 125)

    if not average: return False, False, image, 0, 0, 0 

    x_coords_avg = []
    y_coords_avg = []

    if circles is not None:

        circles = np.round(circles[0, :]).astype("int")
        circles = self.limit_random_circles(circles, max_circles=int(self.parameterZBox.value))

        centers = circles[:, :2]

        # Draw the circles on the original image
        image_with_circles = np.copy(image)
        for (x, y, radius) in circles:
            cv2.circle(image_with_circles, (x, y), radius, 255, 1)
            x_coords_avg.append(x)
            y_coords_avg.append(y)

            array_sum = sum(x_coords_avg)
            x = int(array_sum / len(x_coords_avg))

            array_sum = sum(y_coords_avg)
            y = int(array_sum / len(y_coords_avg))

            if len(circles) > 2: 

                #clustering---------
                num_clusters = 3

                # Perform K-means clustering to group the circle centers
                kmeans = KMeans(n_clusters=num_clusters, n_init=1, random_state=42)
                kmeans.fit(centers)

                # Get the cluster centers
                cluster_centers = kmeans.cluster_centers_

                            # Get the cluster labels assigned by K-means
                labels = kmeans.labels_

                # Count the number of circles in each cluster
                unique_labels, counts = np.unique(labels, return_counts=True)

                # Find the index of the cluster with the highest number of circles
                highest_count_idx = np.argmax(counts)

                # Get the center of the cluster with the highest number of circles
                center_of_highest_cluster = cluster_centers[highest_count_idx]

                x, y = center_of_highest_cluster
                x = int(x)
                y = int(y)
                #clustering end--------

        return True, True, image_with_circles, x, y, 10
    else:
        return True, False, image, 0, 0, 0
  
  def normalize_image(self, image):
      # Find the minimum and maximum pixel values in the image
      min_pixel_value = np.min(image)
      max_pixel_value = np.max(image)

      # Scale the pixel values to the range [0, 1]
      normalized_image = (image - min_pixel_value) / (max_pixel_value - min_pixel_value)

      grayscale_image = (normalized_image * 255).astype(np.uint8)

      return grayscale_image  
  
  def showComposite(self, imageA, imageB, imageC):
    # Define the dimensions of the video streams
    stream_width = 200
    stream_height = 200

    # Create a blank composite image to hold the video streams and captions
    composite_image_width = stream_width * 3  # Width of three video streams
    composite_image_height = stream_height + 40  # Height of a video stream plus caption
    composite_image = np.zeros((composite_image_height, composite_image_width, 3), dtype=np.uint8)

    # Copy video frames onto the composite image
    composite_image[30:stream_height+30, 0:stream_width] = imageA
    composite_image[30:stream_height+30, stream_width:2 * stream_width] = imageB
    composite_image[30:stream_height+30, 2 * stream_width:3 * stream_width] = imageC

    # Add captions above each stream
    caption_offset = 10
    caption_height = 20
    cv2.putText(composite_image, 'Live Stream', (caption_offset, caption_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(composite_image, 'Thresholded', (stream_width + caption_offset, caption_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(composite_image, 'Composite', (2 * stream_width + caption_offset, caption_height), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
      
    cv2.imshow('Composite Streams', composite_image)

  def select_directory(self):
    directory = qt.QFileDialog.getExistingDirectory(self.parent, "Select Directory")
    if directory:
      self.imagePathBox.setText(directory)

  def startConnection(self):
    if (self.openIGTNode == None):
      self.openIGTNode = slicer.vtkMRMLIGTLConnectorNode()
      slicer.mrmlScene.AddNode(self.openIGTNode)
      self.openIGTNode.SetTypeServer(18934)
    self.IGTActive = True
    self.openIGTNode.Start()

  def stopConnection(self):
    self.IGTActive = False
    self.openIGTNode.Stop()

  def startDepthToggle(self):
    if self.startDepthButton.isChecked():
      self.depthTimer.start(int(1000/int(self.depthFPSBox.value)))
    else:
      self.joystickControl([0,0], 0) #--------------------TODO IMPORTANT TODO 
      self.depthTimer.stop()
      cv2.destroyAllWindows()

  def stopToggle(self):
    self.dualLockButton.setChecked(False)
    self.bendingLockButton.setChecked(False)
    self.linearLockButton.setChecked(False)
  
  def generateDepthAndSendVector(self, vector): #-------------------------------------------------------------------

    if self.IGTActive:
      linearStage = 0

      self.joystickControl(vector, linearStage)
        
  def clamp(self, n, minn, maxn):
    return max(min(maxn, n), minn)
  
  def joystickControl(self, vector, linearStage):
    # print(vector)
    rightLinear = 0.0

    if not self.dualLockButton.isChecked(): #automatic control toggle
      vector = [0,0]
      linearStage = 0

    if self.reverseToggleButton.isChecked():
      vector = [0,0]
      linearStage = 0 #LEFT AND FORWARD
      rightLinear = 0.3 #RIGHT AND BACKWARD

    if windowsOS:
      self.gamepad.left_joystick(x_value=vector[0], y_value=-vector[1])
      self.gamepad.left_trigger_float(value_float=linearStage)
      self.gamepad.right_trigger_float(value_float=rightLinear)
      self.gamepad.update()

    image = np.ones((200, 200, 3), dtype=np.uint8) * 255

    stick_x = ((vector[0]/32768)*100)
    stick_y = ((vector[1]/32768)*100)
    trigger_r = linearStage
    trigger_l = rightLinear

    # Calculate the circle position based on stick_x and stick_y
    circle_x = int(stick_x + 100)
    circle_y = int(stick_y + 100)

    # Draw the circle at the calculated position, along with a center point circle
    cv2.circle(image, (circle_x, circle_y), 10, (0, 0, 0), -1)
    cv2.circle(image, (100, 100), 2, (0, 0, 255), -1)
    cv2.circle(image, (100,100), 100, (0,0,255), 1, cv2.LINE_AA)
    cv2.circle(image, (100,100), int(self.DeadzoneBox.value), (0,0,255), 1, cv2.LINE_AA)
    cv2.circle(image, (circle_x, circle_y), 2, (255, 255, 255), -1)

    # Draw the rectangles on the right and left sides
    right_rect_height = int(trigger_r * 200)
    left_rect_height = int(trigger_l * 200)

    cv2.rectangle(image, (190, 0), (199, right_rect_height), (0, 0, 0), -1)
    cv2.rectangle(image, (0, 0), (9, left_rect_height), (0, 0, 0), -1)

    # Invert the colors of the image

    # Display the image
    cv2.imshow("controls", image)   

class kidneyStoneNavigationLogic:
  def __init__(self):
    pass
 