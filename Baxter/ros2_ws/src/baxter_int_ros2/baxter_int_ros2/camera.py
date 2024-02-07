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
    Tuple,
    TYPE_CHECKING,
    Union,

    # - numpy
    numpy,

    # - array
    array,

    # - opencv
    cv2,
    MatLike,

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
            print(f'| - Created {self}')

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
            find_table: bool = False,
            process_table: bool = False,
            table_occu: bool = False,
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
        - find_table : `bool`
            - Whether or not to process the image and identify where the table
                is in the image, as well as drawing a box around it with points
                at each corner and in the centre.
        - process_table : `bool`
            - Whether or not to process the data from the table image and
                identify the objects within the table.
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
            f'ROS2_ImageProcessor_{topic}',
            verbose
        )
        _t = '\t' * self._verbose
        if self._V: print(f'{_t}Constructing Image Processor - topic={topic}')

        # create main attributes
        if self._V: print(f'{_t}| - Creating Main Attributes')
        self._data: Optional[MSG_CameraData] = None
        self._data_table: Optional[MSG_CameraData] = None
        self._find_table: bool = find_table
        self._process_table: bool = process_table
        self._table_occu: bool = table_occu
        self._topic: str = topic
        self._table_pos: Optional[numpy.ndarray] = None
        self.state_changed = Signal()

        # create subscribers
        if self._V: print(f'{_t}| - Creating Subscribers')
        if self._V: print(f'{_t}\t| - Image Processing')
        self.create_sub(
            msgCameraData,
            self.topic_sub,
            self.sub_callback_org
        )
        if self._V: print(f'{_t}\t| - Table Processing')
        self.create_sub(
            msgCameraData,
            self.topic_table,
            self.sub_callback_table
        )

        # create publishers
        if self._V: print(f'{_t}| - Creating Publishers')
        if self._V: print(f'{_t}\t| - Table Processor')
        self._pub_table = self.create_pub(
            msgCameraData,
            self.topic_table
        )
        self._pub_org_gray = self.create_pub(
            msgCameraData,
            f'{self.topic}/gray'
        )
        self._pub_org_blur = self.create_pub(
            msgCameraData,
            f'{self.topic}/blur'
        )
        self._pub_org_edge = self.create_pub(
            msgCameraData,
            f'{self.topic}/edge'
        )
        self._pub_org_edge_blur = self.create_pub(
            msgCameraData,
            f'{self.topic}/edge_blur'
        )
        self._pub_org_contours = self.create_pub(
            msgCameraData,
            f'{self.topic}/contours'
        )
        self._pub_table_edge_blur = self.create_pub(
            msgCameraData,
            f'{self.topic_table}/edge_blur'
        )
        self._pub_table_occ_uint8 = self.create_pub(
            msgCameraData,
            f'{self.topic_table}/occupancy/uint8'
        )
        self._pub_table_occ_bool = self.create_pub(
            msgCameraData,
            f'{self.topic_table}/occupancy/bool'
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
    def sub_callback_org(
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
        if self._find_table:
            self.find_table()

    # ========================================
    # Image Processor Subscriber - Table Image
    def sub_callback_table(
            self,
            msg: msgCameraData
    ) -> None:
        '''
        Image Processor Subscriber - Table Image
        -
        Runs whenever data is published to the Table topic that this 
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
        self._data_table = MSG_CameraData.from_msg(msg)
        if self._process_table:
            self.process_table()

    # ==============
    # Find the Table
    def find_table(self) -> None:
        '''
        Find the Table
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
                f'Image_Processor {self} tried to find_table with no' \
                    + ' valid data.'
            )
        
        # convert image array to numpy array
        img = numpy.array(self._data.image, dtype = numpy.uint8)
        img = img.reshape(
            self._data.height,
            self._data.width,
            self._data.channels
        )
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)

        # get table position if required
        if self._table_pos is None:
            # get edges
            img_gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
            img_blur = cv2.GaussianBlur(img_gray.copy(), (5, 5), 0) # (5,5)
            img_edge = cv2.Canny(img_blur.copy(), 10, 30) # (75, 200) (20, 75)
            img_edge_blur = cv2.GaussianBlur(img_edge.copy(), (7, 7), 0)

            # get contours
            img_contour = img.copy()
            contours = sorted(
                cv2.findContours(
                    img_edge_blur.copy(),
                    cv2.RETR_LIST,
                    cv2.CHAIN_APPROX_SIMPLE
                )[0],
                key = cv2.contourArea,
                reverse = True
            )[:5]

            # find main contour
            doc_contour: MatLike = [] # type: ignore
            doc_points = numpy.array([(0, 0) for _ in range(4)], dtype='float32')
            for i, _c in enumerate(contours):
                approx = cv2.approxPolyDP(
                    _c, 
                    0.02 * cv2.arcLength(_c, True), 
                    True
                )
                for _p in approx:
                    cv2.circle(
                        img_contour,
                        tuple(_p[0]),
                        3,
                        {
                            (True, True): (255, 0, 0),
                            (False, True): (0, 255, 0),
                            (False, False): (255, 255, 0),
                        }[(i<=0, i<=1)],
                        4
                    )
                if len(approx) == 4:
                    doc_contour = approx
                    break
            for i, _c in enumerate(doc_contour):
                _p = tuple(_c[0])
                cv2.circle(img_contour, _p, 6, (0, 0, 255), 8)
                doc_points[i] = _p

            # set table position
            if len(doc_points) == 4:
                self._table_pos = doc_points
            
            # publish camera data
            self._pub_org_gray.publish(
                MSG_CameraData(
                    image = img_gray.flatten().tolist(), # type: ignore
                    width = self._data.width,
                    height = self._data.height,
                    channels = 1,
                    skip_validation = True
                ).create_msg()
            )
            self._pub_org_blur.publish(
                MSG_CameraData(
                    image = img_blur.flatten().tolist(), # type: ignore
                    width = self._data.width,
                    height = self._data.height,
                    channels = 1,
                    skip_validation = True
                ).create_msg()
            )
            self._pub_org_edge.publish(
                MSG_CameraData(
                    image = img_edge.flatten().tolist(), # type: ignore
                    width = self._data.width,
                    height = self._data.height,
                    channels = 1,
                    skip_validation = True
                ).create_msg()
            )
            self._pub_org_edge_blur.publish(
                MSG_CameraData(
                    image = img_edge_blur.flatten().tolist(), # type: ignore
                    width = self._data.width,
                    height = self._data.height,
                    channels = 1,
                    skip_validation = True
                ).create_msg()
            )
            self._pub_org_contours.publish(
                MSG_CameraData(
                    image = img_contour.flatten().tolist(), # type: ignore
                    width = self._data.width,
                    height = self._data.height,
                    channels = 3,
                    skip_validation = True
                ).create_msg()
            )

        # create warped image
        img_warped: Union[None, MatLike] = None
        if self._table_pos is not None:
            img_warped = Image_Processor.PyImageSearch.four_point_transform(
                img.copy(),
                self._table_pos
            )
        
        if img_warped is not None:
            self._pub_table.publish(
                MSG_CameraData(
                    image = img_warped.flatten().tolist(), # type: ignore
                    width = len(img_warped[0]),
                    height = len(img_warped),
                    channels = len(img_warped[0][0]),
                    skip_validation = True
                ).create_msg()
            )

    # =================
    # Process the Table
    def process_table(self) -> None:
        '''
        Process the Table
        -
        Process the table image and identify the objects on the table.

        Parameters
        -
        None

        Returns
        -
        None
        '''

        # make sure data exists
        if self._data_table is None:
            raise RuntimeWarning(
                f'Image_Processor {self} tried to process_table with no' \
                    + ' valid data.'
            )
        
        # convert image array to numpy array
        img = numpy.array(self._data_table.image, dtype = numpy.uint8)
        img = img.reshape(
            self._data_table.height,
            self._data_table.width,
            self._data_table.channels
        )

        # create edge-blur of table data
        img_gray = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray.copy(), (5, 5), 0) # (5,5)
        img_edge = cv2.Canny(img_blur.copy(), 10, 30) # (75, 200) (20, 75)
        img_edge_blur = cv2.GaussianBlur(img_edge.copy(), (7, 7), 0)

        # publish table edge data
        self._pub_table_edge_blur.publish(
            MSG_CameraData(
                image = img_edge_blur.flatten().tolist(), # type: ignore
                width = self._data_table.width,
                height = self._data_table.height,
                channels = 1,
                skip_validation = True
            ).create_msg()
        )

        # create and publish occupancy grid if required
        if self._table_occu:
            _, occ_img_uint8 = Image_Processor.create_occupancy(
                img_edge_blur,
                30,
                40,
                10
            )
            _, occ_img_bool = Image_Processor.create_occupancy(
                img_edge_blur,
                30,
                40,
                10,
                flag_uint8 = False
            )
            self._pub_table_occ_uint8.publish(
                MSG_CameraData(
                    image = occ_img_uint8.flatten().tolist(), # type: ignore
                    width = occ_img_uint8.shape[1],
                    height = occ_img_uint8.shape[0],
                    channels = 1,
                    skip_validation = True
                ).create_msg()
            )
            self._pub_table_occ_bool.publish(
                MSG_CameraData(
                    image = occ_img_bool.flatten().tolist(), # type: ignore
                    width = occ_img_bool.shape[1],
                    height = occ_img_bool.shape[0],
                    channels = 1,
                    skip_validation = True
                ).create_msg()
            )

    # =====================
    # Create Occupancy Grid
    @staticmethod
    def create_occupancy(
            img: numpy.ndarray,
            rows: int,
            cols: int,
            img_blowup: int = 10,
            flag_uint8: bool = True,
            src: str = 'canny'
    ) -> Tuple[numpy.ndarray, numpy.ndarray]:
        '''
        Create Occupancy Grid
        -
        Creates an occupancy grid of the image, with the specified number of
        rows and columns. Can return either a graded or binary grid.

        Parameters
        -
        - img : `numpy.ndarray`
            - Image to create the occupancy grid from.
        - rows : `int`
            - Number of rows to split the image into for the grid.
        - cols : `int`
            - Number of cols to split the image into for the grid.
        - img_blowup : `int`
            - Scalar number of how much to increase the size of the occupancy
                grid for use in viewing on camera.
        - flag_uint8 : `bool`
            - Whether to output a UINT8 or BOOL occupancy grid. Defaults to
                True which means UINT8.
        - src : `str`
            - Image type source. Defaults to `'canny'`, which means the
                image is the output of a canny edge filter.
            - Options:
                - 'canny' : Canny edge filter.

        Returns
        -
        `Tuple[numpy.ndarray, numpy.ndarray]`
            - [0]: Occupancy grid.
            - [1]: Image viewable grid (blown up version of occupancy grid
                which has been converted to image-streaming friendly data).
        '''

        # initialize variables
        _c: int # column number counter
        _c2: int # column number counter 2
        _r: int # row number counter
        _r2: int # row number counter 2
        col_width: int # number of cols per sub-matrix in split image
        img_split: numpy.ndarray # split image
        occ_grid: numpy.ndarray # occupancy grid
        occ_img: numpy.ndarray # occupancy grid image
        row_width: int # number of rows per sub-matrix in split image
        rgb: bool = len(img.shape) == 3 and img.shape[2] == 3

        # get split image
        (
            row_width, 
            col_width, 
            img_split
        ) = Image_Processor.split_img(
            img,
            rows,
            cols
        )

        # initialize occupancy grid + image
        occ_grid = numpy.zeros((rows, cols), numpy.uint8)
        occ_img = numpy.zeros((rows*img_blowup, cols*img_blowup), numpy.uint8)

        # create occupancy grid data
        if src == 'canny':
            for _r in range(rows):
                for _c in range(cols):
                    if flag_uint8:
                        if rgb:
                            occ_grid[_r, _c] = (
                                (img_split[_r, _c].sum()) \
                                / (col_width * row_width * 3) # 3 channels
                            )
                        occ_grid[_r, _c] = (
                            (img_split[_r, _c].sum()) \
                            / (col_width * row_width) # single channel
                        )
                    else:
                        occ_grid[_r, _c] = int(img_split[_r, _c].sum() > 0)
        else:
            raise ValueError(
                f'Image_Processor.create_occupancy invalid src = {src}'
            )
        
        # create occupancy image
        if flag_uint8:
            for _r in range(rows):
                for _c in range(cols):
                    for _r2 in range(img_blowup):
                        for _c2 in range(img_blowup):
                            occ_img[
                                (_r*img_blowup + _r2), 
                                (_c*img_blowup + _c2)
                            ] = occ_grid[_r, _c]
        else:
            for _r in range(rows):
                for _c in range(cols):
                    for _r2 in range(img_blowup):
                        for _c2 in range(img_blowup):
                            occ_img[
                                (_r*img_blowup + _r2), 
                                (_c*img_blowup + _c2)
                            ] = occ_grid[_r, _c] * 255 # (0 or 1) * 255

        return (occ_grid, occ_img)

    # ===========
    # Split Image
    @staticmethod
    def split_img(
        img: numpy.ndarray,
        rows: int,
        cols: int
    ) -> Tuple[int, int, numpy.ndarray]:
        '''
        Split Image
        -
        Splits a parsed image into a segmented version of the original. Adds 2
        dimensions (2D array where each cell in the 2D array contains a
        sub-matrix of the original). Also truncates any excess rows and cols on
        the right/bottom of the image so that the image is the correct size.

        Parameters
        -
        - img : `numpy.ndarray`
            - Image being split.
        - rows : `int`
            - Number of rows to split the image into.
        - cols : `int`
            - Number of columns to split the image into.

        Returns
        -
        `Tuple[int, int, numpy.ndarray]`
            - [0]: Number of rows per sub-matrix.
            - [1]: Number of cols per sub-matrix.
            - [2]: Split Image.
        '''

        # truncate excess rows/cols
        if img.shape[0] % rows > 0:
            img = img[0:(-1*(img.shape[0]%rows)), :]
        if img.shape[1] % cols > 0:
            img = img[:, 0:(-1*(img.shape[1]%cols))]

        # calculate row and col widths
        row_width: int = img.shape[0] // rows
        col_width: int = img.shape[1] // cols

        return (
            row_width,
            col_width,
            numpy.array([
                numpy.array([
                    img[
                        (r*row_width):(((r+1)*row_width)-1),
                        (c*col_width):(((c+1)*col_width)-1)
                    ]
                    for c in range(cols)
                ])
                for r in range(rows)
            ])
        )

    class PyImageSearch():
        ''' Object container for pyimagesearch.com Functions. '''

        @staticmethod
        def order_points(pts: numpy.ndarray) -> numpy.ndarray:
            ''' 
                Taken from: 
                    https://pyimagesearch.com/2014/08/25/4-point-opencv-
                        getperspective-transform-example/
            '''

            # initialzie a list of coordinates that will be ordered
            # such that the first entry in the list is the top-left,
            # the second entry is the top-right, the third is the
            # bottom-right, and the fourth is the bottom-left
            rect = numpy.zeros((4, 2), dtype = "float32")
            # the top-left point will have the smallest sum, whereas
            # the bottom-right point will have the largest sum
            s = pts.sum(axis = 1)
            rect[0] = pts[numpy.argmin(s)]
            rect[2] = pts[numpy.argmax(s)]
            # now, compute the difference between the points, the
            # top-right point will have the smallest difference,
            # whereas the bottom-left will have the largest difference
            diff = numpy.diff(pts, axis = 1)
            rect[1] = pts[numpy.argmin(diff)]
            rect[3] = pts[numpy.argmax(diff)]
            # return the ordered coordinates
            return rect
        
        @staticmethod
        def four_point_transform(
            image: MatLike, 
            pts: numpy.ndarray
        ) -> MatLike:
            ''' 
                Taken from: 
                    https://pyimagesearch.com/2014/08/25/4-point-opencv-
                        getperspective-transform-example/
            '''

            # obtain a consistent order of the points and unpack them
            # individually
            rect = Image_Processor.PyImageSearch.order_points(pts)
            (tl, tr, br, bl) = rect
            # compute the width of the new image, which will be the
            # maximum distance between bottom-right and bottom-left
            # x-coordiates or the top-right and top-left x-coordinates
            widthA = numpy.sqrt(
                ((br[0] - bl[0]) ** 2) \
                + ((br[1] - bl[1]) ** 2)
            )
            widthB = numpy.sqrt(
                ((tr[0] - tl[0]) ** 2) \
                + ((tr[1] - tl[1]) ** 2)
            )
            maxWidth = max(int(widthA), int(widthB))
            # compute the height of the new image, which will be the
            # maximum distance between the top-right and bottom-right
            # y-coordinates or the top-left and bottom-left y-coordinates
            heightA = numpy.sqrt(
                ((tr[0] - br[0]) ** 2) \
                + ((tr[1] - br[1]) ** 2)
            )
            heightB = numpy.sqrt(
                ((tl[0] - bl[0]) ** 2) \
                + ((tl[1] - bl[1]) ** 2)
            )
            maxHeight = max(int(heightA), int(heightB))
            # now that we have the dimensions of the new image, construct
            # the set of destination points to obtain a "birds eye view",
            # (i.e. top-down view) of the image, again specifying points
            # in the top-left, top-right, bottom-right, and bottom-left
            # order
            dst = numpy.array([
                [0, 0],
                [maxWidth - 1, 0],
                [maxWidth - 1, maxHeight - 1],
                [0, maxHeight - 1]], dtype = "float32")
            # compute the perspective transform matrix and then apply it
            M = cv2.getPerspectiveTransform(rect, dst)
            warped = cv2.warpPerspective(image, M, (maxWidth, maxHeight))
            # return the warped image
            return cast(MatLike, warped)


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

        _t: str = '\t' * self._verbose_sub
        if self._V: print(f'{_t}| - {self.topic} Update Stream')

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
