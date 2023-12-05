#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
ROS2 Baxter Message Type Definitions
-
Contains message type definitions used for type-hinting in ROS2.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

from . import (
    # - typing
    _DATA,
    Any,
    cast,
    List,

    # - array
    array,

    # - messages
    MSG_CameraSettings,

    # - baxter_core_msgs
    srvOpenCamera,
    srvCloseCamera,
    srvListCameras,

    # - baxter_int_ros2
    srvCameraData,

    # - .
    ROS2_obj,
    _MSG,
)


# =============================================================================
# ROS2 Services - Generic Parent Class
# =============================================================================


# =============================================================================
# ROS2 Messages
# =============================================================================


# =============================================================================
# Baxter Core Messages - Services
# =============================================================================

# ===========
# CloseCamera
class CloseCamera(srvCloseCamera):
    '''
    baxter_core_msgs - CloseCamera Service
    -
    Contains the service data to close a specific camera on Baxter.

    Request Data
    -
    - name : `string`

    Response Data
    -
    - err : `int32`
    '''

    class Request(srvCloseCamera.Request):
        ''' baxter_core_msgs.srv.CloseCamera Request. '''
        def __init__(
                self,
                name: str
        ) -> None:
            super().__init__()
            self.name = name

    class Response(srvCloseCamera.Response):
        ''' baxter_core_msgs.srv.CloseCamera Response. '''
        def __init__(
                self,
                err: int
        ) -> None:
            super().__init__()
            self.err = err

# ===========
# ListCameras
class ListCameras(srvListCameras):
    '''
    baxter_core_msgs - ListCameras Service
    -
    Contains the service data to list all of the cameras on Baxter.

    Request Data
    -
    None

    Response Data
    -
    - cameras : `array[string]`
    '''

    class Request(srvListCameras.Request):
        ''' baxter_core_msgs.srv.ListCameras Request. '''
        def __init__(self) -> None:
            super().__init__()

    class Response(srvListCameras.Response):
        ''' baxter_core_msgs.srv.ListCameras Response. '''
        def __init__(
                self,
                cameras: array
        ) -> None:
            super().__init__()
            self.cameras = cameras

# ==========
# OpenCamera
class OpenCamera(srvOpenCamera):
    '''
    baxter_core_msgs - OpenCamera Service
    -
    Contains the service data to open a camera on Baxter.

    Request Data
    -
    - name : `string`
    - settings : `MSG_CameraSettings`

    Response Data
    -
    - err : `int32`
    '''

    class Request(srvOpenCamera.Request):
        ''' baxter_core_msgs.srv.OpenCamera Request. '''
        def __init__(
                self,
                name: str,
                settings: MSG_CameraSettings
        ) -> None:
            super().__init__()
            self.name = name
            self.settings = settings

    class Response(srvOpenCamera.Response):
        ''' baxter_core_msgs.srv.OpenCamera Response. '''
        def __init__(
                self,
                err: int
        ) -> None:
            super().__init__()
            self.err = err


# =============================================================================
# Baxter Service Datatypes
# =============================================================================

# ==========
# CameraData
class CameraData(srvCameraData):
    '''
    baxter_int_ros2 - CameraData Service
    -
    Contains the service data from a camera on Baxter.

    Data
    -
    - `Request` : CameraData request data.
    - `Response` : CameraData response data.
    '''

    class Request(srvCameraData.Request):
        '''
        baxter_int_ros2 - CameraData - Request
        -
        Contains the information for a `CameraData` service request.

        Data
        -
        None
        '''

    class Response(srvCameraData.Response):
        '''
        baxter_int_ros2 - CameraData - Response
        -
        Contains the information for a `CameraData` service response.

        Data
        - 
        - img_data : `array[uint8]`
        - height : `uint32`
        - width : `uint32`
        - step : `uint32`
        '''

        def __init__(
                self,
                img_data: array,
                height: int,
                width: int,
                step: int
        ) -> None:
            super().__init__()
            self.img_data = img_data
            self.height = height
            self.width = width
            self.step = step


# =============================================================================
# End of File
# =============================================================================
