 #!/usr/bin/env python

from urllib.response import addinfo
import cv2
import gi
import numpy as np
import threading 
import rospy
import csv
import time
from dsor_msgs.msg import Measurement
from std_msgs.msg import Bool
from auv_msgs.msg import NavigationStatus
from itertools import zip_longest

from std_msgs.msg import Float64
import torch
import mediapipe as mp
import numpy as np
from scipy.interpolate import RectBivariateSpline
import pandas as pd
import matplotlib
matplotlib.rcParams['backend'] = 'TkAgg'
import matplotlib.pyplot as plt 
import cv2
from PIL import Image
import time
from skimage import data
from skimage import io,filters,feature
import datetime
import math
from skimage.filters import roberts,sobel,scharr , prewitt,farid
from skimage.feature import canny
from skimage.feature import corner_harris
from skimage.feature import corner_peaks  
torch.cuda.empty_cache()
midas = torch.hub.load('intel-isl/MiDaS','MiDaS_small')
midas.to('cpu')
midas.eval()
transforms = torch.hub.load('intel-isl/MiDaS','transforms')
transform = transforms.small_transform

df =[{'altitude':0},{'timestamp':0},{'area':0}]
df = pd.DataFrame(df)



gi.require_version('Gst', '1.0')
from gi.repository import Gst
init = 0
area= 0 
val =0 
curr_orientation = 0
rospy.init_node('nodesubmerged','listner','orient')
 

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """


   



    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) Iv)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', 
                 '! videorate ! video/x-raw,framerate=10/1 ! videoconvert',
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videorate ! video/x-raw,framerate=10/1 ! videoconvert',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array
    
    
    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])
    
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
    
if __name__ == '__main__':
    
    # model = torch.hub.load("ultralytics/yolov5", "yolov5s")
    model = torch.hub.load('/home/nio/romir/Test/yolov5', 'custom', path='/home/nio/romir/Test/yolov5/runs/train/exp13/weights/best.pt', force_reload=True)


    mp_pose = mp.solutions.pose
    pose = mp_pose.Pose(static_image_mode=False)
    rate = rospy.Rate(10)


    pub = rospy.Publisher('/bluerov_heavy0/ref/yaw', Float64,queue_size=4)
    pub_sway = rospy.Publisher('/bluerov_heavy0/ref/sway', Float64,queue_size=4)
    video = Video()
    ang =0
    frame_rate = 100
    prev = 0
    initial_time = time.time()
    to_time = time.time()
    set_fps = 8  
    prev_frame_time = 0  
    new_frame_time = 0
    img2 = video.frame()

    while True:
 
        while_running = time.time()  
        new_time = while_running - initial_time

        if not video.frame_available():
            continue
        if new_time >= 1 / set_fps:
            img2 = video.frame()
            im12 = img2
            if video.frame_available():
 
                new_frame_time = time.time()
                fps = 1 / (new_frame_time - prev_frame_time)
                prev_frame_time = new_frame_time
                fps = int(fps)
                fps = str(fps)
                initial_time = while_running  

            else:
                total_time_of_video = while_running - to_time  
                print(total_time_of_video)
                break
            img = cv2.cvtColor(img2, cv2.COLOR_BGR2RGB)
            cv2.imshow("frame",img)
            # print(np.shape(img))
            # print(np.max(img))
            # print(np.min(img))
            imgbatch =  transform(img).to('cpu')

            with torch.no_grad():
                prediction = midas(imgbatch )

                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=img.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()

            output = prediction.cpu().numpy()
     
            plt.imshow(output)
            plt.pause(0.00001)
            
            output = cv2.normalize(src=output, dst=None, alpha=0.0, beta=1.0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            cv2.imshow('CV2Frame', output)
            # cv2.imshow('CV2Frame', output)
            print(np.shape(output))
            print(np.max(output))
            print(np.min(output))


        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break


rospy.spin()
