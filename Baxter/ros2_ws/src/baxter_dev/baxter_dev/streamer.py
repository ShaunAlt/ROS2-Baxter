#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Baxter Image Streamer
-
When run, creates an Image Streamer which streams the image data from a
specified ROS2 topic.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for OpenCV
import cv2

# used for ros2 python connection
import rclpy # type: ignore

# used for running the main function + parsed arugments
import sys

# used for type-hinting
from typing import (
    TYPE_CHECKING,
)

# baxter interface imports
if TYPE_CHECKING:
    from baxter_int_ros2.baxter_int_ros2 import (
        Image_Viewer,
        Topics,
    )
else:
    from baxter_int_ros2 import (
        Image_Viewer,
        Topics,
    )


# =============================================================================
# Image Streamer Mainloop
# =============================================================================
def main(args: None = None) -> None:
    print('Image Streamer')

    # get parsed arguments
    print('| - Getting Parsed Arguments')
    topic: str = ''
    verbose: int = -1
    if len(sys.argv) >= 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError('Image Streamer was not given topic to stream.')
    if len(sys.argv) >= 3:
        try:
            verbose = int(sys.argv[2])
        except:
            raise RuntimeError(
                'Image Streamer 2nd positional argument "verbose" was not' \
                    + f'given an int value, was given {repr(verbose)}'
            )

    # initialize rclpy    
    print('| - Initializing RCLPY')
    rclpy.init(args = args)

    # create image streamer
    print('| - Creating Image Viewer')
    streamer = Image_Viewer(
        topic = topic,
        verbose = verbose
    )

    # spinning
    print('| - Spinning (Press ESC on streamed window to Close)')
    try:
        rclpy.spin(streamer)
    except ConnectionAbortedError:
        print('\t| - Image Connection Aborted')
    except Exception as e:
        print(f'\t| - UNKNOWN ERROR: {e}')
    
    # end
    print('| - Closing OpenCV Windows')
    cv2.destroyAllWindows()
    print('| - Destroying Streamer Node')
    streamer.destroy_node()
    print('| - Shutting Down')
    rclpy.shutdown()
    print('Done')

if __name__ == '__main__':
    main()


# =============================================================================
# End of File
# =============================================================================
