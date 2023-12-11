#!/usr/bin/env python3

# =============================================================================
# Created by: Shaun Altmann
# =============================================================================
'''
Image Processing
-
Contains object and method definitions required for processing images to
identify objects.
'''
# =============================================================================


# =============================================================================
# Imports
# =============================================================================

# used for image processing
import cv2
from cv2.typing import (
    MatLike,
)

# used for numpy arrays
import numpy

# used for type hinting
from typing import (
    Any,
    List,
    Tuple,
)


# =============================================================================
# Processing Values
# =============================================================================

VALS_EDGE = [
    (75, 200), # 0
    (25, 100), # 1
    (22, 85),  # 2
    (20, 75),  # 3
    (15, 60),  # 4
]
VALS_S = [
    1, # 0
    3, # 1
    5, # 2
    7, # 3
    9, # 4
    11 # 5
]
IMG_STR = 'imgs/Table-Img2.png'


# =============================================================================
# Image Processing Functions
# =============================================================================

gray = lambda _img: cv2.cvtColor(_img.copy(), cv2.COLOR_BGR2GRAY)
blur = lambda _img, _k, _s=0: cv2.GaussianBlur(_img.copy(), (_k, _k), _s)
edge = lambda _img, _min, _max: cv2.Canny(_img.copy(), _min, _max)


# =============================================================================
# Find Contours
# =============================================================================

def contours(_img: MatLike, original_img: MatLike) -> List[MatLike]:
    _circle_radius = 1
    _circle_colour = (0, 0, 255)
    _circle_thickness = 2
    c1 = cv2.findContours(_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[0]
    c2 = cv2.findContours(_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
    c3 = cv2.findContours(_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
    c4 = cv2.findContours(_img.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]

    c1 = c1[0].reshape(-1, 2)
    c2 = c2[0].reshape(-1, 2)
    c3 = sorted(c3, key=cv2.contourArea, reverse=True)[:5]
    c4 = sorted(c4, key=cv2.contourArea)[:5]

    i1 = original_img.copy()
    i2 = original_img.copy()
    i3 = original_img.copy()
    i4 = original_img.copy()

    for (x, y) in c1:
        cv2.circle(
            i1, 
            (x, y), 
            _circle_radius, 
            _circle_colour, 
            _circle_thickness
        )
    for (x, y) in c2:
        cv2.circle(
            i2, 
            (x, y), 
            _circle_radius, 
            _circle_colour, 
            _circle_thickness
        )
    for d in c3:
        peri = cv2.arcLength(d, True)
        approx = cv2.approxPolyDP(d, 0.02*peri, True)
        for d1 in approx:
            color = {
                True: (255, 0, 0),
                False: _circle_colour,
            }[len(approx) == 4]
            cv2.circle(
                i3, 
                tuple(d1[0]), 
                _circle_radius, 
                color, 
                _circle_thickness
            )
    for d in c4:
        peri = cv2.arcLength(d, True)
        approx = cv2.approxPolyDP(d, 0.02*peri, True)
        for d1 in approx:
            color = {
                True: (255, 0, 0),
                False: _circle_colour,
            }[len(approx) == 4]
            cv2.circle(
                i4, 
                tuple(d1[0]), 
                _circle_radius, 
                color, 
                _circle_thickness
            )

    return [i1, i2, i3, i4]


# =============================================================================
# 4-Point Transform - Retreived from 
# https://pyimagesearch.com/2014/08/25/4-point-opencv-getperspective-transform-example/
# =============================================================================
def order_points(pts: Any) -> Any:
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
def four_point_transform(image: Any, pts: Any) -> Any:
	# obtain a consistent order of the points and unpack them
	# individually
	rect = order_points(pts)
	(tl, tr, br, bl) = rect
	# compute the width of the new image, which will be the
	# maximum distance between bottom-right and bottom-left
	# x-coordiates or the top-right and top-left x-coordinates
	widthA = numpy.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
	widthB = numpy.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
	maxWidth = max(int(widthA), int(widthB))
	# compute the height of the new image, which will be the
	# maximum distance between the top-right and bottom-right
	# y-coordinates or the top-left and bottom-left y-coordinates
	heightA = numpy.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
	heightB = numpy.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
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
	return warped


# =============================================================================
# Display Contours from Image
# =============================================================================

def display_image_contours(
        _img: MatLike,
        _blur: int,
        _edge_min: int,
        _edge_max: int
) -> None:
    g = gray(_img)
    _img_g = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    b = blur(g, _blur)
    _img_b = cv2.cvtColor(b, cv2.COLOR_GRAY2BGR)
    e = edge(b, _edge_min, _edge_max)
    _img_e = cv2.cvtColor(e, cv2.COLOR_GRAY2BGR)
    b2 = blur(e, _blur)
    _img_b2 = cv2.cvtColor(b2, cv2.COLOR_GRAY2BGR)
    e2 = edge(b2, _edge_min, _edge_max)
    # _img_e2 = cv2.cvtColor(e2, cv2.COLOR_GRAY2BGR)
    c_s = contours(b2, _img)
    # output_imgs = numpy.vstack([
    #     numpy.hstack([
    #         _img,
    #         cv2.cvtColor(g, cv2.COLOR_GRAY2BGR), 
    #         cv2.cvtColor(b, cv2.COLOR_GRAY2BGR), 
    #         cv2.cvtColor(e, cv2.COLOR_GRAY2BGR)
    #     ]),
    #     numpy.hstack(c_s)
    # ])
    output_imgs = numpy.vstack([
        numpy.hstack([_img_e, _img_b2]),
        numpy.hstack([c_s[2], c_s[3]])
    ])
    cv2.imshow('TL: Blur, TR: Canny, BL: No Chain, BR: Chain', output_imgs)
    while (cv2.waitKey(1) & 0xff) != 27:
        pass
    cv2.destroyAllWindows()


# =============================================================================
# Display Image + Re-arranged Perspective
# =============================================================================

def display_image_document(img: MatLike) -> None:
    # get canny edges
    img_canny: MatLike = blur(edge(blur(gray(img.copy()), 5), 10, 30), 7)

    # get contours
    _contours = sorted(
        cv2.findContours(
            img_canny.copy(),
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )[0],
        key = cv2.contourArea,
        reverse = True
    )[:5]

    # draw main contour
    doc_contour = []
    doc_points = numpy.array([(0, 0) for _ in range(4)], dtype='float32')
    img_contour = img.copy()
    for _c in _contours:
        _approx = cv2.approxPolyDP(
            _c,
            0.02 * cv2.arcLength(_c, True),
            True
        )
        if len(_approx) == 4:
            doc_contour = _approx
            break
    for i, _c in enumerate(doc_contour):
        _p = tuple(_c[0])
        cv2.circle(img_contour, _p, 3, (0, 0, 255), 4)
        doc_points[i] = _p

    # create warped image
    img_warped = four_point_transform(img.copy(), doc_points)
    print(f'Warped Image: {len(img_warped)} x {len(img_warped[0])} x {len(img_warped[0][0])}')
    cv2.imshow('Table', img_warped)
    while (cv2.waitKey(1) & 0xff) != 27:
        pass
    cv2.destroyAllWindows()


# =============================================================================
# End of File
# =============================================================================
