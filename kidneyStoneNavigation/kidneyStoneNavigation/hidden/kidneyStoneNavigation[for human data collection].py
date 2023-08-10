# -*- coding: utf-8 -*-
#set zero to 220

from __main__ import vtk, qt, ctk, slicer
import math

import os
import math

import datetime

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
opacity = .6
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
  joystickCounter = 0
  stoneCenteredBool = False
  printStoneCenteredBool = True
  timeCountingBool = True
  timeCountingBool2 = True
  timeCountingBool3 = True
  timeCountingBool4 = True
  countingDistanceOutsideBool = False
  start_time = 0
  end_time = 0
  start_time2 = 0
  end_time2 = 0  
  start_time_counter = 0
  end_time_counter = 0
  
  stoneError = []

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

    self.bendingSpeedBox = qt.QDoubleSpinBox()
    self.bendingSpeedBox.setSingleStep(.001)
    self.bendingSpeedBox.setMaximum(5)
    self.bendingSpeedBox.setMinimum(0)
    self.bendingSpeedBox.value = .8
    self.bendingSpeedBox.setDecimals(3)
    depthLayout.addRow("Bending speed:", self.bendingSpeedBox)      
    
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
    self.DeadzoneBox.value = 1
    self.DeadzoneBox.setDecimals(0)
    controllerCurve.addRow("Deadzone:", self.DeadzoneBox)

    self.DeadzoneFineBox = qt.QDoubleSpinBox()
    self.DeadzoneFineBox.setSingleStep(1)
    self.DeadzoneFineBox.setMaximum(100)
    self.DeadzoneFineBox.setMinimum(0)
    self.DeadzoneFineBox.value = 1
    self.DeadzoneFineBox.setDecimals(3)
    controllerCurve.addRow("Deadzone Fine:", self.DeadzoneFineBox)    

    self.brightSpotDistanceBox = qt.QDoubleSpinBox()
    self.brightSpotDistanceBox.setSingleStep(1)
    self.brightSpotDistanceBox.setMaximum(200)
    self.brightSpotDistanceBox.setMinimum(1)
    self.brightSpotDistanceBox.value = 35
    self.brightSpotDistanceBox.setDecimals(0)
    controllerCurve.addRow("Bright spot distance:", self.brightSpotDistanceBox)        

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
    self.U_valueBox.value = 20200
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
    self.M_valueBox.value = 0.00050
    self.M_valueBox.setDecimals(5)
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
  
    self.forwardAdvanceThresholdBox = qt.QDoubleSpinBox()
    self.forwardAdvanceThresholdBox.setMinimum(0)
    self.forwardAdvanceThresholdBox.setMaximum(1000)
    self.forwardAdvanceThresholdBox.setSingleStep(1)
    self.forwardAdvanceThresholdBox.setValue(14)
    self.forwardAdvanceThresholdBox.setDecimals(0)
    imgProcessing.addRow("Forward Advance threshold:", self.forwardAdvanceThresholdBox) 

    self.forwardAdvanceTopBox = qt.QDoubleSpinBox()
    self.forwardAdvanceTopBox.setMinimum(0)
    self.forwardAdvanceTopBox.setMaximum(1000)
    self.forwardAdvanceTopBox.setSingleStep(1)
    self.forwardAdvanceTopBox.setValue(15)
    self.forwardAdvanceTopBox.setDecimals(0)
    imgProcessing.addRow("Forward Advance top:", self.forwardAdvanceTopBox)

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
    self.parameterYBox.setValue(18)
    self.parameterYBox.setDecimals(0)
    imgProcessing.addRow("contrast threshold:", self.parameterYBox)    

    self.huePeakThresholdBox = qt.QDoubleSpinBox()
    self.huePeakThresholdBox.setMinimum(0)
    self.huePeakThresholdBox.setMaximum(1)
    self.huePeakThresholdBox.setSingleStep(.00001)
    self.huePeakThresholdBox.setValue(.001)
    self.huePeakThresholdBox.setDecimals(5)
    imgProcessing.addRow("hue peak threshold:", self.huePeakThresholdBox)     

    self.parameterXBox = qt.QDoubleSpinBox()
    self.parameterXBox.setMinimum(0.0)
    self.parameterXBox.setMaximum(1)
    self.parameterXBox.setSingleStep(0.0001)
    self.parameterXBox.setValue(.03)
    self.parameterXBox.setDecimals(10)
    imgProcessing.addRow("forward speed:", self.parameterXBox)  

    self.parameterWBox = qt.QDoubleSpinBox()
    self.parameterWBox.setMinimum(0.0)
    self.parameterWBox.setMaximum(32768)
    self.parameterWBox.setSingleStep(1)
    self.parameterWBox.setValue(1000)
    self.parameterWBox.setDecimals(2)
    imgProcessing.addRow("Fine control magnitude:", self.parameterWBox)     

    self.regionGrowingThresholdBox = qt.QDoubleSpinBox()
    self.regionGrowingThresholdBox.setMinimum(0.0)
    self.regionGrowingThresholdBox.setMaximum(32768)
    self.regionGrowingThresholdBox.setSingleStep(1)
    self.regionGrowingThresholdBox.setValue(85)
    self.regionGrowingThresholdBox.setDecimals(2)
    imgProcessing.addRow("Region growing threshold:", self.regionGrowingThresholdBox)    

    self.stoneSeedBrightnessThreshold = qt.QDoubleSpinBox()
    self.stoneSeedBrightnessThreshold.setMinimum(0.0)
    self.stoneSeedBrightnessThreshold.setMaximum(32768)
    self.stoneSeedBrightnessThreshold.setSingleStep(1)
    self.stoneSeedBrightnessThreshold.setValue(160)
    self.stoneSeedBrightnessThreshold.setDecimals(2)
    imgProcessing.addRow("Stone seed brightness threshold:", self.stoneSeedBrightnessThreshold)    

         
    #------------------------------------------------------------------
    self.layout.addStretch(1)
    #------------------------------------------------------------------
    #------------------------------------------------------------------
    #------------------------------------------------------------------

  def mainCodeRunner(self):

    contrastSkip = False
    image = self.extractImage()

    # cv2.imshow('image', image)

    image = cv2.resize(image, (200,200))
    image_for_seed = image.copy()
    imageCopy5 = image.copy()

    #-------------initial image processing

    # image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    overlay = np.zeros_like(image)
    overlay[:] = overlayColor
    cv2.addWeighted(overlay, opacity, image, 1 - opacity, 0, image)
    height, width, _ = image.shape

    image_copy = image.copy()   
    contrastImage = image.copy()

    #-----------wall / close stone detection
    contrast_frame = cv2.cvtColor(contrastImage, cv2.COLOR_BGR2GRAY)
    
    contrast = np.std(contrast_frame)
    contrast_threshold = int(self.parameterYBox.value)

    img_center_x, img_center_y = contrastImage.shape[1] // 2, contrastImage.shape[0] // 2

    # cv2.rectangle(contrastImage, top_left, bottom_right, (255, 255, 255), 2)
    cv2.circle(contrastImage, (100, 100), int(self.brightSpotDistanceBox.value), (255, 255, 255), 2)

    if contrast >= contrast_threshold:

      #-----
      image_for_seed, blank_for_seed = self.regionGrowing(image_for_seed)
      contourFound, contrastImage, cX, cY = self.findContourAndCenter(blank_for_seed, contrastImage, imageCopy5)
      # print("seed prop done")

      if not contourFound: return 

      distance_to_center = np.sqrt((cX - 100) ** 2 + (cY - 100) ** 2)

      if distance_to_center < self.brightSpotDistanceBox.value:

        # Draw an arrow from the center of the contour to the center of the image
        cv2.arrowedLine(contrastImage, (cX, cY), (img_center_x, img_center_y), (255, 0, 0), 2)

        cv2.imshow("contrastImage", contrastImage)

        vector = [cX-100, cY-100]
        vector = self.calcVector(vector, avoidance=False, controlFine=True, controlFineDistance=distance_to_center)
        self.generateDepthAndSendVector(vector, linearStage=0) 

        contrastSkip = True

        if self.timeCountingBool3:
          self.start_time2 = datetime.datetime.now()
          self.timeCountingBool3 = False


    # else:
        # print("Low contrast")    
        # cv2.destroyAllWindows()

    if contrastSkip: return

    # --------------hue equalization 
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hsv_image = cv2.blur(hsv_image, (3,3))

    hue_channel = hsv_image[:, :, 0]
    equalized_hue_channel_hist = cv2.equalizeHist(hue_channel)

    equalized_hue_channel = cv2.addWeighted(hue_channel, 0.4, equalized_hue_channel_hist, 0.6, 0)
    hsv_image[:, :, 0] = equalized_hue_channel

    #---------------plot histogram
    hue_hist = cv2.calcHist([hsv_image], [0], None, [360], [0, 360])
    # hue_hist_smoothed = cv2.GaussianBlur(hue_hist, (1, 1), 0)
    threshold = self.huePeakThresholdBox.value * np.max(hue_hist)
    peaks, _ = find_peaks(hue_hist.flatten(), height=threshold)

    hue_groups = []
    region_lower_bounds = []
    region_upper_bounds = []

    j = 0
    for peak in peaks:

        if hue_hist[peak] > threshold:

            lower_bound = int(max(peak-0, 0))
            upper_bound = int(min(peak+0, 179))

            mask = cv2.inRange(hsv_image, np.array([lower_bound, 0, 0]), np.array([upper_bound, 255, 255]))

            # print(peak)
            # cleaned_image = remove_noise_threshold(mask, kernel_size=2, iterations=7)
            # mask = cleaned_image

            # cv2.imshow("Segmented Image", mask) #show the mask of each threshold range
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w >= 100 or h >= 100: continue  #size filtering
                if j < 4: continue # TODO not efficient; doesn't consider the first two peaks
                hue_groups.append(contour)

            region_lower_bounds.append(lower_bound)
            region_upper_bounds.append(upper_bound)

            j+=1

    # #------------show hue histogram 
    # plt.plot(hue_hist, color='b')
    # for peak in peaks:
    #     if hue_hist[peak] > threshold:
    #         plt.axvline(x=peak, color='r')

    # for lower_bound, upper_bound in zip(region_lower_bounds, region_upper_bounds):
    #     plt.axvline(x=lower_bound, color='g', linestyle='--')
    #     plt.axvline(x=upper_bound, color='g', linestyle='--')

    # plt.title("Hue Histogram")
    # plt.xlabel("Hue")
    # plt.ylabel("Magnitude")
    # plt.show()

    labeled_image = image.copy()

    blankContours = np.zeros((200, 200, 1), dtype=np.uint8)

    for group in hue_groups:
        cv2.drawContours(labeled_image, [group], -1, (0,0,255), 2)
        cv2.drawContours(blankContours, [group], -1, (255,255,255), 1)

    # cv2.imshow("Segmented Image", blankContours)

    # print("blankContours: {}".format(blankContours.shape))

    blur1 = 10
    blur2 = blur1
    blurred = cv2.blur(blankContours, (blur1,blur1)) 
    blurred2 = cv2.blur(blurred, (blur2,blur2)) 

    blurred2 = self.normalize_image(blurred2)

    average, circleFound, circled, center_x, center_y, radius = self.houghCircleFit(blurred2, 2, 100)

    if not average: return  #if the frame buffer (for temporal sampling) is not full

    if circleFound:
        cv2.circle(image, (center_x, center_y), radius, 255, 2)
    else: 
        center_x = 100
        center_y = 100

        # Print the center coordinates
    # print("Center coordinates: ({}, {})".format(center_x, center_y))

    # Draw red contour around the largest contour
    # cv2.drawContours(image, [largest_contour], -1, (0, 0, 255), 2)

    image_arrowed = image.copy()
    cv2.arrowedLine(image_arrowed, (100,100), (center_x,center_y), (255,255,255), 3)
    vector = [center_x-100, center_y-100]
    vector = self.calcVector(vector, avoidance=False, controlFine = False, controlFineDistance=0)

    #-----------Image Processing--------------- 

    blankContours = cv2.cvtColor(blankContours, cv2.COLOR_GRAY2BGR)
    circled = cv2.cvtColor(circled, cv2.COLOR_GRAY2BGR)
    # binary_mask = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
    # blue_mask = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)

    # image_copy = cv2.resize(image_copy, (200, 200))
    # blankContours = circled[0:200, 0:200]
    # image = cv2.resize(image, (200, 200))

    self.showComposite(image_copy, circled, image)
    # self.showComposite(image_copy, circled, image_arrowed)

    # cv2.imshow("Contoured", image)
    # print(vector)   
    self.generateDepthAndSendVector(vector, linearStage=self.parameterXBox.value) 
  
  def calcVector(self, vector, avoidance, controlFine, controlFineDistance):

    # print(controlFineDistance)
        
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
    distance = int(self.bendingSpeedBox.value*distance) #make the controller adjustments softer

    if distance < ((self.DeadzoneBox.value)/100)*32768: distance = 0

    if controlFine: 
      distance = self.parameterWBox.value

      if self.countingDistanceOutsideBool:
            
            print(controlFineDistance)

            # print("Average Error over 10 seconds:")
            # print(array_average)
            # print(len(self.stoneError))
            
            self.printStoneCenteredBool = False
            # print(controlFineDistance)

      if controlFineDistance < (((30)/1000)/100)*32768: 

        if self.timeCountingBool4:
          # print("Time taken (far): ")
          # print((datetime.datetime.now() - self.start_time).total_seconds())
          self.timeCountingBool4 = False

        if self.timeCountingBool2:
          # print("Time taken (close): ")
          # print(controlFineDistance)
          self.timeCountingBool2 = False
        
        distance = 0 
        if not self.stoneCenteredBool:
          self.start_time_counter = datetime.datetime.now()
          self.start_time = datetime.datetime.now()
          self.stoneCenteredBool = True
          self.countingDistanceOutsideBool = True


    x = distance * math.cos(theta)
    y = distance * math.sin(theta)

    x = int(x)
    y = int(y)

    if x > 32768: x = 32768
    if y > 32768: y = 32768

    vector[0] = x
    vector[1] = y

    # print(vector)
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
    
  def regionGrowing(self, image):

      height, width, channels = image.shape

      contour_image = image.copy()
      
      blank = np.zeros((height, width, 3), dtype=np.uint8)

      gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      blurred = cv2.GaussianBlur(gray, (5, 5), 0)
      _, thresholded = cv2.threshold(blurred, int(self.stoneSeedBrightnessThreshold.value), 255, cv2.THRESH_BINARY)

      contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
      largest_contour = max(contours, key=cv2.contourArea)

      # center of the largest contour
      M = cv2.moments(largest_contour)
      center_x = int(M["m10"] / M["m00"])
      center_y = int(M["m01"] / M["m00"])

      mask = np.zeros(image.shape[:2], dtype=np.uint8)

      # Draw the contour filled with white color on the mask
      cv2.drawContours(mask, [largest_contour], 0, (255), -1)

      # Calculate the average color within the contour using the mask
      seed_color = cv2.mean(image, mask=mask)
      seed_color = np.array(seed_color, dtype=np.uint8)
      seed_color = np.resize(seed_color, len(seed_color) - 1)
      # print(seed_color)
      # print(len(seed_color))
      # print(type(seed_color))

      seed_colorRead = tuple(map(int, seed_color))

      seed_coordinates = (center_x, center_y)
      x, y = seed_coordinates

      # Draw the contour and center pixel on the original image

      cv2.drawContours(contour_image, [largest_contour], -1, (0, 0, 255), 2)
      cv2.circle(contour_image, (center_x, center_y), 3, (seed_colorRead), -1)
      cv2.circle(contour_image, (x, y), 3, (255,255,0), 1)

      # # Display the contour image
      # cv2.imshow("input", image)
      # cv2.waitKey(0)
      # cv2.destroyAllWindows()

      # cv2.imshow("Contour", contour_image)
      # cv2.waitKey(0)
      # cv2.destroyAllWindows()
      
      # Convert the seed color to HSV
      seed_color_hsv = cv2.cvtColor(np.array([[seed_color]]), cv2.COLOR_BGR2HSV)[0, 0]
      seed_color_hsv = tuple(map(int, seed_color_hsv))

      bgr_color = cv2.cvtColor(np.array([[seed_color_hsv]], dtype=np.uint8), cv2.COLOR_HSV2BGR)[0, 0]
      bgr_color = tuple(map(int, bgr_color))

      hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      hsv_image = cv2.blur(hsv_image, (5, 5), 0)
      x, y = seed_coordinates

      # Plant a seed at the most similar pixel (drawn in green)
      cv2.circle(image, (x, y), 2, (bgr_color), -1)
      cv2.circle(image, (x, y), 3, (255,255,0), 1)

      # Region growing (drawing the propagation outline)
      visited = np.zeros(image.shape[:2], dtype=np.uint8)
      stack = [(y, x)]

      while stack:
          current_x, current_y = stack.pop()
          visited[current_x, current_y] = 255

          # Add neighboring pixels if they are within a color tolerance range and not visited yet
          neighbors = [(current_x - 1, current_y), (current_x + 1, current_y),
                      (current_x, current_y - 1), (current_x, current_y + 1)]
          for neighbor_x, neighbor_y in neighbors:
              if (0 <= neighbor_x < image.shape[0] and 0 <= neighbor_y < image.shape[1] and
                      visited[neighbor_x, neighbor_y] == 0):
                  neighbor_color = hsv_image[neighbor_x, neighbor_y]
                  # neighbor_color = tuple(map(int, neighbor_color)) #not sure about this line

                  distance = np.linalg.norm(neighbor_color - seed_color_hsv)
                  if distance < int(self.regionGrowingThresholdBox.value):  # Adjust the color tolerance as needed
                      stack.append((neighbor_x, neighbor_y))
                  else:
                      # Draw the propagation outline (drawn in red)
                      image[neighbor_x, neighbor_y] = (0, 0, 255)
                      blank[neighbor_x, neighbor_y] = (0, 0, 255)
                      
      return image, blank
  
  def findContourAndCenter(self, image, contrastImage, imageCopy5):

    height, width, channels = image.shape
    blank2 = np.zeros((height, width, 3), dtype=np.uint8)

    contourCenterX = 100
    contourCenterY = 100

    

    # Convert the image to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_image2 = cv2.cvtColor(imageCopy5, cv2.COLOR_BGR2GRAY)
    _, threshold_image = cv2.threshold(gray_image, 10, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(threshold_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the center of the largest contour
        M = cv2.moments(largest_contour)
        contourCenterX = int(M['m10'] / M['m00'])
        contourCenterY = int(M['m01'] / M['m00'])

        cv2.drawContours(contrastImage, [largest_contour], -1, (0, 255, 0), 2)
        cv2.circle(contrastImage, (contourCenterX, contourCenterY), 2, (0, 255, 0), -1)

        _, thresholded = cv2.threshold(gray_image2, int(self.stoneSeedBrightnessThreshold.value), 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:  
          largest_contour = max(contours, key=cv2.contourArea)
          cv2.drawContours(contrastImage, [largest_contour], -1, (0, 0, 255), 1)


        return True, contrastImage, contourCenterX, contourCenterY
    return False, contrastImage, contourCenterX, contourCenterY
  
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
      self.joystickCounter = 0

      self.stoneCenteredBool = False
      self.printStoneCenteredBool = True
      self.timeCountingBool = True
      self.timeCountingBool2 = True
      self.timeCountingBool3 = True
      self.timeCountingBool4 = True
      self.countingDistanceOutsideBool = False

      self.end_time = 0 
      self.start_time2 = 0
      self.end_time2 = 0
      self.start_time_counter = 0
      self.end_time_counter = 0
      self.stoneError=[]

      time_difference = (datetime.datetime.now() - self.start_time).total_seconds()
      print(time_difference)

      print("-------- all values reset --------")

      self.start_time = 0

  def stopToggle(self):
    self.dualLockButton.setChecked(False)
    self.bendingLockButton.setChecked(False)
    self.linearLockButton.setChecked(False)
  
  def generateDepthAndSendVector(self, vector, linearStage): #-------------------------------------------------------------------

    if self.IGTActive:
      self.joystickControl(vector, linearStage)
        
  def clamp(self, n, minn, maxn):
    return max(min(maxn, n), minn)
  
  def joystickControl(self, vector, linearStage):

    self.joystickCounter += 1
    if self.joystickCounter == int(self.forwardAdvanceTopBox.value): self.joystickCounter = 0;

    if self.joystickCounter > self.forwardAdvanceThresholdBox.value:
      linearStage = 0
    # print(vector)
    rightLinear = 0.0

    if not self.dualLockButton.isChecked(): #automatic control toggle
      vector = [0,0]
      linearStage = 0
      # self.start_time = datetime.datetime.now()

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
    trigger_l = linearStage
    trigger_r = rightLinear

    # Calculate the circle position based on stick_x and stick_y
    circle_x = int(stick_x + 100)
    circle_y = int(stick_y + 100)

    # Draw the circle at the calculated position, along with a center point circle
    cv2.circle(image, (circle_x, circle_y), 10, (0, 0, 0), -1)
    cv2.circle(image, (100, 100), 2, (0, 0, 255), -1)
    cv2.circle(image, (100,100), 100, (0,0,255), 1, cv2.LINE_AA)
    cv2.circle(image, (100,100), int(self.DeadzoneBox.value), (0,0,255), 1, cv2.LINE_AA)
    cv2.circle(image, (100,100), int(self.DeadzoneFineBox.value), (0,255,0), 1, cv2.LINE_AA)
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
 