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
    Client,
    Node,
    Publisher,
    Service,
    Subscription,
    spin_until_future_complete,
    Timer,

    # - sensor_msgs
    msgImage,

    # - .dataflow
    df_wait,
    Signal,

    # - .
    _DATA,
    _TIMER,
    ROS2_obj,
    Services,
    Topics,

    # - .msgs
    MSG_Image,

    # - srvs
    srvCameraData,
    SRV_CameraData,
    SRV_CloseCamera,
    SRV_ListCameras,
    SRV_OpenCamera,
)


# =============================================================================
# Baxter Camera Object
# =============================================================================
class Camera(Node, ROS2_obj):
    '''
    Baxter Camera Object
    -
    Baxter interface object which is able to connect to a single Camera on the
    Baxter robot.

    Topics
    -
    - Servicing:
        - {topic}/image_service

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
            - Defaults to `-1`, which means no verbosity. Any value 0 or
                greater corresponds to verbosity with the specified number of
                tabs used for indentation.
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

        # initialize verbosity loggers
        V: bool = verbose >= 0
        _t: str = '\t' * verbose
        _sub_v: int = {True: verbose + 1, False: -1}[V]
        if V: print(f'{_t}Constructing Camera, topic={topic}')

        # initialize attributes
        if V: print(f'{_t}| - Initializing Attributes')
        _cli_cam_list: Client
        _cam_list: SRV_ListCameras.Response
        self._cli_close: Client
        self._cli_open: Client
        self._data_img: Optional[MSG_Image]
        self._image_display_rt: bool = image_display_rt
        self._node_name: str
        self._number_updates_since_last_print: int = 0
        self._srv_img: Service
        self._sub_img: Subscription
        self._topic: str
        self.update_img: Signal
        self.verbose: int = verbose

        # validate the topic
        if V: print(f'{_t}| - Validating Topic')
        if not topic in Topics.Camera.ALL:
            raise ValueError(
                f'{self.ERR_DIR}.__init__ unable to construct object with ' \
                    + f'invalid topic: topic = {topic}, type = {type(topic)}'
            )
        
        # create main attributes
        if V: print(f'{_t}| - Creating Main Attributes')
        self._node_name = f'Baxter_Camera_{topic}'
        self._topic = topic
        self._data_img = None
        self.update_img = Signal()

        # initialize node
        if V: print(f'{_t}| - Creating Node')
        super().__init__(self._node_name)

        # validate camera can be created
        # if V: print(f'{_t}| - Validating Camera against Camera List Service')
        # _cli_cam_list = self.create_client(
        #     SRV_ListCameras,
        #     'cameras/list'
        # )
        # while not _cli_cam_list.wait_for_service(timeout_sec = 1.0):
        #     if V: print(f'{_t}\t| - Service unavailable, trying again ...')
        # _cam_list = _cli_cam_list.call_async(SRV_ListCameras.Request())
        # spin_until_future_complete(self, _cam_list)
        # if V: print(f'{_t}\t| - Camera List: {repr(_cam_list.cameras)}')
        # if V: print(f'{_t}\t| - Valid Topic: {topic in _cam_list.cameras}')

        # create subscribers
        if V: print(f'{_t}| - Creating Subscribers')
        if V: print(f'{_t}\t| - Creating Image Subscriber')
        self._sub_img = self.create_subscription(
            MSG_Image,
            self.topic_img,
            self.sub_img_callback,
            10
        )

        # create services
        if V: print(f'{_t}| - Creating Services')
        if V: print(f'{_t}\t| - Creating Image Service')
        self._srv_img = self.create_service(
            srvCameraData,
            self.topic_img_srv,
            self.srv_img_callback
        )

        # create publishers
        if V: print(f'{_t}| - Creating Publishers')

        # create timers
        if V: print(f'{_t}| - Creating Timers')
        if print_image_timer > 0:
            if V:
                print(
                    f'{_t}| - Creating Print Image Timer: Time = ' \
                        + f'{print_image_timer}'
                )
            self.create_timer(print_image_timer, self.print_image_data)

        if V: 
            print(f'{_t}| - Created ' + repr(self).replace('\n', '\n\t\t'))

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
                    'verbose': self.verbose,
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
    
    # ===================
    # Image Service Topic
    @property
    def topic_img_srv(self) -> str:
        ''' Topic name of the Image Service for the `Camera`. '''
        return f'{self.topic}/{Services.Camera.IMAGE_DATA}'
    
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

        # parse data
        msg = MSG_Image.from_msg(_msg)

        # update data
        self._data_img = msg

        # notify data update
        self.update_img(self.data_img)
        self._number_updates_since_last_print += 1

        # display image in real-time if required
        if self._image_display_rt:
            self.opencv_image_display()

    # ======================
    # Image Service Callback
    def srv_img_callback(
            self,
            request: srvCameraData.Request,
            response: srvCameraData.Response
    ) -> srvCameraData.Response:
        '''
        Camera Image Service Callback
        -
        Runs whenever a client pings the `Camera` image service topic.

        Parameters
        -
        request : `srvCameraData.Request`
            Camera image data request.
        response : `srvCameraData.Response`
            Camera image data response.

        Returns
        -
        None
        '''

        print(f'Image Service Callback {self.topic}')

        # create response
        if self.data_img is not None:
            response.img_data = self.data_img.data
            response.height = self.data_img.height
            response.width = self.data_img.width
            response.step = self.data_img.step
        else:
            response.img_data = array('B', [])
            response.height = 0
            response.width = 0
            response.step = 0
        # while self.data_img is None:
        #     print(f'\t| - Waiting for initial Camera Data {self.topic}')
        # response.img_data = self.data_img.data
        # response.height = self.data_img.height
        # response.width = self.data_img.width
        # response.step = self.data_img.step
        return response

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
        _t: str = '\t' * self.verbose
        V: bool = self.verbose >= 0
        if V: print(f'{_t}Displaying Image Data for {self.topic}')

        # makes sure data exists
        if self.data_img is None:
            raise RuntimeError(
                f'Camera {self} tried to use opencv_image_display with no' \
                    + ' valid data'
            )

        # convert image list to numpy array
        if V: print(f'{_t}| - Converting Image to NumPy ({self.topic})')
        _img = numpy.array(self.data_img.data, dtype = numpy.uint8)
        if V: print(f'{_t}| - {self.topic} Image 1D: Shape = {_img.shape}')
        if V: print(f'{_t}| - Converting NumPy Array 1D to 3D')
        _img = numpy.reshape(
            _img, 
            (
                self.data_img.height,
                self.data_img.width,
                self.data_img.channels,
            )
        )
        if V: print(f'{_t}| - {self.topic} Image 3D: Shape = {_img.shape}')

        # convert image colour
        if V: print(f'{_t}| - Converting Image Colour ({self.topic})')
        _img = cv2.cvtColor(_img, cv2.COLOR_BGRA2BGR)

        # display image
        if V: print(f'{_t}| - Displaying Image ({self.topic})')
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
# Baxter Camera Client Objects
# =============================================================================
class Client_Camera(Node, ROS2_obj):
    '''
    Baxter Camera Client Object
    -
    Baxter interface object which is a client for a specific Baxter Camera
    topic.
    '''

    ERR_DIR: str = 'baxter_int_ros2.camera.Client_Camera'

    # ===========
    # Constructor
    def __init__(
            self,
            topic: str,
            service: str,
            timer: float = 1.0,
            verbose: int = -1,
            stream_video: bool = False
    ) -> None:
        '''
        Baxter Camera Client Constructor
        -
        Initializes an instance of the Baxter interface object which is a
        client to a specific service of a particular `Camera`.

        Parameters
        -
        - topic : `str`
            - Camera topic to be a client to. Must be in `Topics.Camera.ALL`.
        - service : `str`
            - Camera service to be a client to. Must be in 
                `Services.Camera.ALL`.
        - verbose : `int`
            - Defaults to `-1`, which means no verbosity. Any value 0 or
                greater corresponds to verbosity with the specified number of 
                tabs used for indentation.
        - stream_video : `bool`
            - Defaults to `False`. When `True`, means that the camera data will
                be streamed into a viewable video feed.

        Returns
        -
        None
        '''

        # initialize verbosity loggers
        V: bool = verbose >= 0
        _t: str = '\t' * verbose
        _sub_v: int = {True: verbose + 1, False: -1}[V]
        if V: 
            print(
                f'{_t}Constructing Camera Client, topic = {topic}, service =' \
                    + f' {service}'
            )
        
        # initialize attributes
        if V: print(f'{_t}| - Initializing Variables')
        self._flags: Dict[str, bool]
        self._node_name: str
        self._service: str
        self._topic: str
        self._verbose: int
        self.client: Client
        self.req: srvCameraData.Request

        # validating topic and service
        if V: print(f'{_t}| - Validating Topic and Service')
        if not topic in Topics.Camera.ALL:
            raise ValueError(
                f'{self.ERR_DIR}.__init__ unable to construct object with ' \
                    + f'invalid topic: topic = {repr(topic)}'
            )
        if not service in Services.Camera.ALL:
            raise ValueError(
                f'{self.ERR_DIR}.__init__ unable to construct object with ' \
                    + f'invalid service: service = {repr(service)}'
            )
        
        # create main attributes
        if V: print(f'{_t}| - Creating Main Attributes')
        self._flags = {
            'stream_video': stream_video,
        }
        self._node_name = f'Baxter_Client_Camera_{topic}__{service}'
        self._service = service
        self._topic = topic
        self._verbose = _sub_v

        # initialize node
        if V: print(f'{_t}| - Creating Node')
        super().__init__(self._node_name)

        # create client
        if V: print(f'{_t}| - Creating Client')
        self._client = self.create_client(
            srvCameraData,
            self.topic
        )
        while not self._client.wait_for_service(timeout_sec = 1.0):
            print(f'{_t}\t| - Service not available, waiting for connection')
        self.req = srvCameraData.Request()

        # create timer
        if V: print(f'{_t}| - Create Timer ({timer})')
        self.create_timer(timer, self.timer_callback)

    # =============
    # Service Topic
    @property
    def topic(self) -> str:
        ''' Camera Client Service Topic. '''
        return f'/cameras/{self._topic}/{self._service}'

    # ====================
    # Send Service Request
    def send_request(self) -> srvCameraData.Response:
        '''
        Send Service Request
        -
        Sends a ROS2 request to the `Client_Camera` service.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize verbosity loggers
        V: bool = self._verbose >= 0
        _t: str = '\t' * self._verbose
        if V: print(f'{_t}Send Service Request {self.topic}')

        # create request
        if V: print(f'{_t}| - Create Request {self.topic}')
        # self.req = srvCameraData.Request()

        # get service response
        if V: print(f'{_t}| - Get Response {self.topic}')
        if V: print(f'{_t}\t| - Call Async {self.topic}')
        self.future = self._client.call_async(self.req)
        if V: print(f'{_t}\t| - Spinning until Call Complete {self.topic}')
        spin_until_future_complete(self, self.future)
        if V: print(f'{_t}\t| - Creating Response {self.topic}')
        res: srvCameraData.Response = self.future.result()
        if V: print(f'{_t}\t| - Done {self.topic}')

        # handle response
        if V:
            print(f'{_t}| - Response Data:')
            print(f'{_t}\t| - Width: {res.width}')
            print(f'{_t}\t| - Height: {res.height}')
            print(f'{_t}\t| - Step: {res.step}')

        if V: print(f'{_t}| - Done')

        return res

    # ==============
    # Timer Callback
    def timer_callback(self) -> None:
        '''
        Timer Callback
        -
        Calls the service request each time the timer is reached.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # initialize verbosity loggers
        V: bool = self._verbose >= 0
        _t: str = '\t' * self._verbose
        if V: print(f'{_t}Timer Callback')

        # Run Request/Response
        if V: print(f'{_t}| - Getting Response')
        res: srvCameraData.Response = self.send_request()
        if V: print(f'{_t}\t| - Response: {res}')

        if V: print(f'{_t}| - Done')


# =============================================================================
# End of File
# =============================================================================
