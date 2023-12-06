#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Digital IO Interface
-
Contains object and method definitions required for interacting with the
Digital IO topics (digital buttons and sensors) of Baxter.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from . import (
    # - typing
    Any,
    cast,
    Dict,
    List,
    Optional,
    TYPE_CHECKING,

    # - numpy
    numpy,

    # - array
    array,

    # - opencv
    cv2,

    # - rclpy
    Publisher,
    spin_until_future_complete,
    Timer,

    # - sensor_msgs
    msgImage,

    # - baxter_int_ros2_support
    msgCameraData,

    # - .dataflow
    df_wait,
    Signal,

    # - .
    _DATA,
    _TIMER,
    ROS2_Node,
    Topics,

    # - .msgs
    MSG_Image,
    MSG_CameraData,
)


# =============================================================================
# Baxter Camera Object
# =============================================================================
class Camera(ROS2_Node):
    '''
    Baxter Camera Object
    -
    Baxter interface object which is able to connect to a single Camera on the
    Baxter robot.

    Attributes
    -
    None

    Methods
    -
    None
    '''

    # =========
    # Constants
    ERR_DIR: str = 'baxter_int_ros2.camera.Camera'

    # ===========
    # Constructor
    def __init__(
            self,
            topic: str,
            verbose: int = -1,
            image_display_rt: bool = False,
            print_image_timer: float = 0.0
    ) -> None:
        '''
        Baxter Camera Constructor
        -
        Initializes an instance of the Baxter interface object which is able to
        connect to a single camera on the baxter robot.

        Parameters
        -
        - topic : `str`
            - Camera topic to subscribe to. Must be in `Topics.Camera.ALL`.
        - verbose : `int`
            - Defaults to `-1` which means no verbosity. Any value 0 or greater
                represents the number of tabs to indent logs by.
        - image_display_rt : `bool`
            - Whether or not to open up the image with OpenCV and display it
                real-time (updating every time the subscriber gets new data).
                Defaults to `False`.
        - print_image_timer : `float`
            - Defaults to `0.0`, which means the `print_image_data` method will
                never be timer activated. If this number is non-zero, the
                number will represent the number of seconds used for activating
                the timer to activate this function.

        Returns
        -
        None
        '''

        # validate the topic
        if not topic in Topics.Camera.ALL:
            raise ValueError(
                f'{self.ERR_DIR}.__init__ unable to construct object with ' \
                    + f'invalid topic: topic = {topic}, type = {type(topic)}'
            )
        
        # initialize node
        super().__init__(
            f'Baxter_Camera_{topic}',
            verbose
        )
        _t = '\t' * self._verbose
        if self._V: print(f'{_t}Constructing Camera - topic={topic}')
        
        # create main attributes
        if self._V: print(f'{_t}| - Creating Main Attributes')
        self._data_img: Optional[MSG_Image] = None
        self._image_display_rt: bool = image_display_rt
        self._number_updates_since_last_print: int = 0
        self._topic: str = topic
        self.update_img = Signal()

        # create subscribers
        if self._V: print(f'{_t}| - Creating Subscribers')
        if self._V: print(f'{_t}\t| - Creating Image Subscriber')
        self.create_sub(
            msgImage,
            self.topic_img,
            self.sub_img_callback,
            10
        )

        # create publishers
        if self._V: print(f'{_t}| - Creating Publishers')
        if self._V: print(f'{_t}\t| - Image Data - topic={self.topic_img_pub}')
        self._pub_img = self.create_pub(
            msgCameraData,
            self.topic_img_pub,
            10
        )

        # create timers
        if self._V: print(f'{_t}| - Creating Timers')
        if print_image_timer > 0:
            if self._V:
                print(
                    f'{_t}| - Creating Print Image Timer: Time = ' \
                        + f'{print_image_timer}'
                )
            self.create_tim(print_image_timer, self.print_image_data)

        if self._V:
            print(
                f'{_t}| - Finished Creating:\n{_t}' \
                    + repr(self).replace('\n', f'\n\t{_t}')
            )
        else:
            print(f'Created {self}')

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = True) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'node_name': self.node_name,
                    'topic': self.topic,
                },
                False: {
                    'node_name': self.node_name,
                    'topic': self.topic,
                    'topic_img': self.topic_img,
                    'data_img': self.data_img,
                    '_image_display_rt': self._image_display_rt,
                },
            }[short]
        )
    
    # ==========
    # Image Data
    @property
    def data_img(self) -> Optional[MSG_Image]:
        ''' Image data from the `Camera`. '''
        return self._data_img
    
    # =========
    # Node Name
    @property
    def node_name(self) -> str:
        ''' Name of the `Camera` node. '''
        return self._node_name

    # ==========
    # Main Topic
    @property
    def topic(self) -> str:
        ''' Base topic folder for all topics for the `Camera`. '''
        return f'/cameras/{self._topic}'

    # ===========
    # Image Topic
    @property
    def topic_img(self) -> str:
        ''' Topic name of the Image for the `Camera`. '''
        return f'{self.topic}/image'
    
    # =====================
    # Image Publisher Topic
    @property
    def topic_img_pub(self) -> str:
        ''' Topic name of the Image Publisher for the `Camera`. '''
        return f'{self.topic}/{Topics.Camera.IMAGE_DATA}'
    
    # =========================
    # Image Subscriber Callback
    def sub_img_callback(
            self,
            _msg: msgImage
    ) -> None:
        '''
        Camera Image Subscriber Callback
        -
        Runs whenever the `Camera` topic subscriber receives data from the
        topic it is subscribed to.

        Parameters
        -
        - _msg : `msgImage`
            - Contains the data from the `Camera` image.

        Returns
        -
        None
        '''

        # update data
        self._data_img = MSG_Image.from_msg(_msg)

        # notify data update
        self.update_img(self.data_img)
        self._number_updates_since_last_print += 1

        # display image in real-time if required
        if self._image_display_rt:
            self.opencv_image_display()

        # publish image data
        self._pub_img.publish(
            MSG_CameraData(
                image = self._data_img.data,
                width = self._data_img.width,
                height = self._data_img.height,
                channels = self._data_img.channels,
                skip_validation = True
            ).create_msg()
        )

    # ===================================
    # Show current camera image in OpenCV
    def opencv_image_display(self) -> None:
        '''
        Show current camera image in OpenCV
        -----------------------------------
        Opens up the current camera image in OpenCV and displays it in a frame
        to be seen.

        Parameters
        -
        None
        
        Returns
        -
        None
        '''

        # verbosity
        _t: str = '\t' * self._verbose_sub
        if self._V: print(f'{_t}Displaying Image Data for {self.topic}')

        # makes sure data exists
        if self.data_img is None:
            raise RuntimeError(
                f'Camera {self} tried to use opencv_image_display with no' \
                    + ' valid data'
            )

        # convert image list to numpy array
        if self._V: print(f'{_t}| - Converting Image to NumPy ({self.topic})')
        _img = numpy.array(self.data_img.data, dtype = numpy.uint8)
        if self._V: print(f'{_t}| - {self.topic} Image 1D: Shape = {_img.shape}')
        if self._V: print(f'{_t}| - Converting NumPy Array 1D to 3D')
        _img = numpy.reshape(
            _img, 
            (
                self.data_img.height,
                self.data_img.width,
                self.data_img.channels,
            )
        )
        if self._V: print(f'{_t}| - {self.topic} Image 3D: Shape = {_img.shape}')

        # convert image colour
        if self._V: print(f'{_t}| - Converting Image Colour ({self.topic})')
        _img = cv2.cvtColor(_img, cv2.COLOR_BGRA2BGR)

        # display image
        if self._V: print(f'{_t}| - Displaying Image ({self.topic})')
        cv2.imshow(f'Camera {self.topic} Image', _img)
        if (cv2.waitKey(1) & 0xff) == 27: # 27 = Escape key ASCII code
            raise ConnectionAbortedError

    # =================
    # Print Image Data
    def print_image_data(self) -> None:
        '''
        Print Image Data
        -----------------
        Prints all of the image data from the camera.

        Parameters
        ----------
        None

        Returns
        -------
        None
        '''

        _t: str = '\t' * self.verbose
        print(f'{_t}Printing Image Data for {self.topic}')
        print(f'{_t}| - Camera: {self}')
        print(
            f'{_t}| - Number of Updates since last print: ' \
                + f'{self._number_updates_since_last_print}'
        )
        if self.verbose >= 0:
            _img_str: str = repr(self.data_img).replace('\n', '\n\t\t')
            print(f'{_t}| - Image Data:\n{_t}\t{_img_str}')
        else:
            print(f'{_t}| - Image Data: {self.data_img}')


# =============================================================================
# Image Processor
# =============================================================================
class Image_Processor(ROS2_Node):
    '''
    Image Processor
    -
    Processes an image from 1 topic and publishes the result of the image
    processing in a different topic.

    Attributes
    -
    None

    Methods
    -
    None
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            topic: str,
            process_table: bool = False,
            verbose: int = -1
    ) -> None:
        '''
        Image Process Constructor
        -
        Initializes an instance of the Image Processor object which is able to
        connect to a single camera image feed and process the data as required.

        Parameters
        -
        - topic : `str`
            - Topic to subscribe the image processor to. Must be in
                `Topics.Camera.ALL`.
        - process_table : `bool`
            - Whether or not to process the image and identify where the table
                is in the image, as well as drawing a box around it with points
                at each corner and in the centre.
        - verbose : `int`
            - Defaults to `-1` which means no verbosity. Any value 0 or greater

        Returns
        -
        None
        '''

        # validate the topic
        if not topic in Topics.Camera.ALL:
            raise ValueError(
                'Image_Processor unable to construct object with invalid' \
                    + f' topic: topic = {repr(topic)}'
            )
        
        # initialize node
        super().__init__(
            f'ROS2_ImageProcessor_{topic.replace("/","__")}',
            verbose
        )
        _t = '\t' * self._verbose
        if self._V: print(f'{_t}Constructing Image Processor - topic={topic}')

        # create main attributes
        if self._V: print(f'{_t}| - Creating Main Attributes')
        self._data: Optional[MSG_CameraData] = None
        self._process_table: bool = process_table
        self._topic = topic
        self.state_changed = Signal()

        # create subscribers
        if self._V: print(f'{_t}| - Creating Subscribers')
        if self._V: print(f'{_t}\t| - Image Processing')
        self.create_sub(
            msgCameraData,
            self.topic_sub,
            self.sub_callback_process
        )

        # create publishers
        if self._V: print(f'{_t}| - Creating Publishers')
        if self._V: print(f'{_t}\t| - Table Processor')
        self._pub_table = self.create_pub(
            msgCameraData,
            self.topic_table
        )
        self._pub_table_gray = self.create_pub(
            msgCameraData,
            f'{self.topic_table}/gray'
        )
        self._pub_table_blur = self.create_pub(
            msgCameraData,
            f'{self.topic_table}/blur'
        )
        self._pub_table_edge = self.create_pub(
            msgCameraData,
            f'{self.topic_table}/edge'
        )

        if self._V:
            print(
                f'{_t}| - Finished Created:\n{_t}\t' \
                    + repr(self).replace('\n', f'\n\t{_t}')
            )
        else:
            print(f'Created {self}')

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'node_name': self._node_name,
                    'topic': self.topic,
                },
                False: {
                    'node_name': self._node_name,
                    'topic': self.topic,
                    'data': self._data
                },
            }[short]
        )

    # ==========
    # Main Topic
    @property
    def topic(self) -> str:
        ''' Base topic for getting the image from. '''
        return f'/cameras/{self._topic}'
    
    # ================
    # Subscriber Topic
    @property
    def topic_sub(self) -> str:
        ''' Subscriber Topic. '''
        return f'{self.topic}/{Topics.Camera.IMAGE_DATA}'
    
    # ===============================
    # Publisher Topic - Table Process
    @property
    def topic_table(self) -> str:
        ''' Publisher Topic - Used for publishing processed table image. '''
        return f'{self.topic}/{Topics.Camera.PROCESS_DATA_TABLE}'

    # ==========================
    # Image Processor Subscriber
    def sub_callback_process(
            self,
            msg: msgCameraData
    ) -> None:
        '''
        Image Processor Subscriber
        -
        Runs whenever data is published to the topic that this 
        `Image_Processor` is subscribed to.

        Parameters
        -
        - msg : `msgCameraData`
            - Contains the data from the image passed through the topic.

        Returns
        -
        None
        '''

        # update data
        self._data = MSG_CameraData.from_msg(msg)
        if self._process_table:
            self.process_table()

    # =================
    # Process the Table
    def process_table(self) -> None:
        '''
        Process the Table
        -
        Process the image data and identify the table.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # make sure data exists
        if self._data is None:
            raise RuntimeWarning(
                f'Image_Processor {self} tried to process_table with no' \
                    + ' valid data.'
            )
        
        # convert image array to numpy array
        _i = numpy.array(self._data.image, dtype = numpy.uint8)
        _i = _i.reshape(
            self._data.height,
            self._data.width,
            self._data.channels
        )
        _i = cv2.cvtColor(_i, cv2.COLOR_BGRA2BGR)

        '''
        Processing from:
        https://medium.com/mlearning-ai/document-scanner-using-opencv-with-
        source-code-easiest-way-e0543e1f3a72
        '''

        # get gray image
        _i_gray = cv2.cvtColor(_i.copy(), cv2.COLOR_BGR2GRAY)
        _i_blur = cv2.GaussianBlur(_i_gray, (3, 3), 0) # (5,5)
        _i_edge = cv2.Canny(_i_blur, 20, 75) # 75, 200

        _contours, _ = cv2.findContours(
            _i_edge.copy(),
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )
        _contours = sorted(_contours, key=cv2.contourArea, reverse = True)[:5]
        doc = []
        for c in _contours:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02*peri, True)
            if len(approx) == 4:
                doc = approx
                break
        p = []
        for d in doc:
            tuple_point = tuple(d[0])
            cv2.circle(_i, tuple_point, 3, (0, 0, 255), 4)
            p.append(tuple_point)
        
        self._pub_table_gray.publish(
            MSG_CameraData(
                image = _i_gray.flatten().tolist(), # type: ignore
                width = self._data.width,
                height = self._data.height,
                channels = 1,
                skip_validation = True
            ).create_msg()
        )
        self._pub_table_blur.publish(
            MSG_CameraData(
                image = _i_blur.flatten().tolist(), # type: ignore
                width = self._data.width,
                height = self._data.height,
                channels = 1,
                skip_validation = True
            ).create_msg()
        )
        self._pub_table_edge.publish(
            MSG_CameraData(
                image = _i_edge.flatten().tolist(), # type: ignore
                width = self._data.width,
                height = self._data.height,
                channels = 1,
                skip_validation = True
            ).create_msg()
        )
        self._pub_table.publish(
            MSG_CameraData(
                image = _i.flatten().tolist(), # type: ignore
                width = self._data.width,
                height = self._data.height,
                channels = 3,
                skip_validation = True
            ).create_msg()
        )


# =============================================================================
# Image Viewer
# =============================================================================
class Image_Viewer(ROS2_Node):
    '''
    Image Viewer
    -
    Streams the image data from a given ROS2 topic.
    '''

    # ===========
    # Constructor
    def __init__(
            self,
            topic: str,
            verbose: int = -1
    ) -> None:
        '''
        Image Viewer Constructor
        -
        Initializes an instance of the Image Viewer which streams the data
        from an image in OpenCV.

        Parameters
        -
        - topic : `str`
            - Image topic to subscribe to. Must be in
                `Topics.Camera.ALL_IMAGE_VIEWS`.
        - verbose : `int`
            - Defaults to `-1` which means no verbosity. Any value 0 or greater
                represents the number of tabs to indent logs by.

        Returns
        -
        None
        '''

        # validate the topic
        # if not topic in Topics.Camera.ALL_IMAGE_VIEWS:
        #     raise ValueError(
        #         f'Image_Viewer unable to construct object with invalid topic' \
        #             + f': topic = {repr(topic)}'
        #     )
        
        # initialize node
        super().__init__(
            f'ROS2_ImageViewer_{topic.replace("/","__")}',
            verbose
        )
        _t = '\t' * self._verbose
        if self._V: print(f'{_t}Constructing Image Viewer - topic={topic}')

        # create main attributes
        if self._V: print(f'{_t}| - Creating Main Attributes')
        self._data: Optional[MSG_CameraData] = None
        self._topic = topic
        self.state_changed = Signal()

        # create subscribers
        if self._V: print(f'{_t}| - Creating Subscribers')
        if self._V: print(f'{_t}\t| - Image Stream')
        self.create_sub(
            msgCameraData,
            self.topic,
            self.sub_callback_stream
        )

        if self._V:
            print(
                f'{_t}| - Finished Created:\n{_t}\t' \
                    + repr(self).replace('\n', f'\n\t{_t}')
            )
        else:
            print(f'Created {self}')

    # ===============
    # Get Object Data
    def _get_data(self, short: bool = False) -> _DATA:
        return cast(
            _DATA,
            {
                True: {
                    'node_name': self._node_name,
                    'topic': self.topic,
                },
                False: {
                    'node_name': self._node_name,
                    'topic': self.topic,
                    'data': self._data
                },
            }[short]
        )

    # ==========
    # Main Topic
    @property
    def topic(self) -> str:
        ''' Base topic for getting the image from. '''
        return f'/cameras/{self._topic}'
    
    # =======================
    # Image Stream Subscriber
    def sub_callback_stream(
            self,
            msg: msgCameraData
    ) -> None:
        '''
        Image Stream Subscriber
        -
        Runs whenever data is published to the topic that this `Image_Viewer`
        is subscribed to.

        Parameters
        -
        - msg : `msgCameraData`
            - Contains the data from the image passed through the topic.

        Returns
        -
        None
        '''

        # update data
        self._data = MSG_CameraData.from_msg(msg)
        self._update_stream()

    # ========================
    # Updates the Video Stream
    def _update_stream(self) -> None:
        '''
        Updates the Video Stream
        -
        Updates the OpenCV video stream from the image data saved.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # make sure data exists
        if self._data is None:
            raise RuntimeWarning(
                f'Image_Viewer {self} tried to _update_stream with no valid' \
                    + ' data.'
            )
        
        # convert image array to numpy array
        _i = numpy.array(self._data.image, dtype = numpy.uint8)

        # convert 1D to 3D
        _i = _i.reshape(
            self._data.height, 
            self._data.width, 
            self._data.channels
        )

        # convert colour
        if self._data.channels == 4:
            _i = cv2.cvtColor(_i, cv2.COLOR_BGRA2BGR)
        elif self._data.channels == 3:
            pass # already bgr
        elif self._data.channels == 1:
            _i = cv2.cvtColor(_i, cv2.COLOR_GRAY2BGR)

        # display image
        cv2.imshow(self._node_name, _i)
        if (cv2.waitKey(1) & 0xff) == 27: # 27 = Escape key ASCII code
            raise ConnectionAbortedError


# =============================================================================
# End of File
# =============================================================================
