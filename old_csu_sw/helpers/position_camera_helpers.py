import cv2
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as signal
from photutils.centroids import centroid_com, centroid_1dg, centroid_2dg, centroid_quadratic, centroid_sources
from pyueye import ueye
import logging

CAMERA_TYPE_UVC = 0
CAMERA_TYPE_UEYE = 1
CAMERA_TYPE_THORLABS = 2

def open_webcam_uvc(webcam_index=0, width=1280, height=1024):
    if (cv2.VideoCapture(webcam_index).isOpened() == True):
        cv2.VideoCapture(webcam_index).release()
    cap = cv2.VideoCapture(webcam_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    ret, frame = cap.read()
    return cap

def open_webcam_ueye(webcam_index=0, width=None, height=None):
    hCam = ueye.HIDS(webcam_index)             #0: first available camera;  1-254: The camera with the specified camera ID
    sInfo = ueye.SENSORINFO()
    cInfo = ueye.CAMINFO()
    pcImageMemory = ueye.c_mem_p()
    MemID = ueye.int()
    rectAOI = ueye.IS_RECT()
    pitch = ueye.INT()
    nBitsPerPixel = ueye.INT(8)    #24: bits per pixel for color mode; take 8 bits per pixel for monochrome
    channels = 1                    #3: channels for color mode(RGB); take 1 channel for monochrome
    m_nColorMode = ueye.INT()		# Y8/RGB16/RGB24/REG32
    bytes_per_pixel = int(nBitsPerPixel / 8)

    # Starts the driver and establishes the connection to the camera
    nRet = ueye.is_InitCamera(hCam, None)
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_InitCamera ERROR")

    # Reads out the data hard-coded in the non-volatile camera memory and writes it to the data structure that cInfo points to
    nRet = ueye.is_GetCameraInfo(hCam, cInfo)
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_GetCameraInfo ERROR")

    # You can query additional information about the sensor type used in the camera
    nRet = ueye.is_GetSensorInfo(hCam, sInfo)
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_GetSensorInfo ERROR")

    # nRet = ueye.is_ResetToDefault( hCam)
    # if nRet != ueye.IS_SUCCESS:
    #     raise IOError("is_ResetToDefault ERROR")

    # Set display mode to DIB
    nRet = ueye.is_SetDisplayMode(hCam, ueye.IS_SET_DM_DIB) 


    m_nColorMode = ueye.IS_CM_MONO8
    nBitsPerPixel = ueye.INT(8)
    bytes_per_pixel = int(nBitsPerPixel / 8)

    # Can be used to set the size and position of an "area of interest"(AOI) within an image
    nRet = ueye.is_AOI(hCam, ueye.IS_AOI_IMAGE_GET_AOI, rectAOI, ueye.sizeof(rectAOI))
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_AOI ERROR")

    width = rectAOI.s32Width
    height = rectAOI.s32Height

    # Prints out some information about the camera and the sensor
    logging.info("Camera model: {}".format(sInfo.strSensorName.decode('utf-8')))
    logging.info("Camera serial no.: {}".format(cInfo.SerNo.decode('utf-8')))
    logging.info("Maximum image width: {}".format(width))
    logging.info("Maximum image height: {}".format(height))

    # Allocates an image memory for an image having its dimensions defined by width and height and its color depth defined by nBitsPerPixel
    nRet = ueye.is_AllocImageMem(hCam, width, height, nBitsPerPixel, pcImageMemory, MemID)
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_AllocImageMem ERROR")
    else:
        # Makes the specified image memory the active memory
        nRet = ueye.is_SetImageMem(hCam, pcImageMemory, MemID)
        if nRet != ueye.IS_SUCCESS:
            raise IOError("is_SetImageMem ERROR")
        else:
            # Set the desired color mode
            nRet = ueye.is_SetColorMode(hCam, m_nColorMode)



    # Activates the camera's live video mode (free run mode)
    nRet = ueye.is_CaptureVideo(hCam, ueye.IS_DONT_WAIT)
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_CaptureVideo ERROR")

    # Enables the queue mode for existing image memory sequences
    nRet = ueye.is_InquireImageMem(hCam, pcImageMemory, MemID, width, height, nBitsPerPixel, pitch)
    if nRet != ueye.IS_SUCCESS:
        raise IOError("is_InquireImageMem ERROR")
    
    return hCam, width, height, pcImageMemory, MemID, nBitsPerPixel, pitch

def rotate_image(frame, angle):
    if angle == 0:
        return frame
    elif angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    elif angle == 270:
        return cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    else:
        raise ValueError("Angle must be 0, 90, 180, or 270")

def get_frame(cap, bounds=None, rotate_angle=0):
    try:
        if isinstance(cap, cv2.VideoCapture):
            ret, frame = cap.read() # clear buffered images
            ret, frame = cap.read()
            ret, frame = cap.read()
        elif isinstance(cap, tuple):
            hCam, width, height, pcImageMemory, MemID, nBitsPerPixel, pitch = cap
            array = ueye.get_data(pcImageMemory, width, height, nBitsPerPixel, pitch, copy=False)
            frame = np.reshape(array,(height.value, width.value, int(nBitsPerPixel / 8)))
            frame = np.rot90(frame)
        if bounds is not None:
            frame = frame[bounds.y1:bounds.y2, bounds.x1:bounds.x2]
    except Exception as e:
        logging.error(e)
        frame = None
    image = rotate_image(frame, angle=rotate_angle)
    # discard the third dimension of image
    if image.ndim == 3:
        image = image[:,:,0]
    return image

def close_camera(cap):
    if isinstance(cap, cv2.VideoCapture):
        cap.release()
    elif isinstance(cap, tuple):
        hCam, width, height, pcImageMemory, MemID, nBitsPerPixel, pitch = cap
        ueye.is_FreeImageMem(hCam, pcImageMemory, MemID)
        ueye.is_ExitCamera(hCam)

def upscale_frame(frame, upscale_factor=10):
    size = (frame.shape[1] * upscale_factor, frame.shape[0] * upscale_factor)
    return cv2.resize(frame, size, interpolation=cv2.INTER_LANCZOS4)

def get_upscaled_frame(cap, upscale_factor=10):
    frame = get_frame(cap)
    return upscale_frame(frame, upscale_factor=upscale_factor)

def convert_array_to_grayscale(frame):
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def select_image_area(frame, x, y, w, h):
    return frame[y:y+h, x:x+w]

def average_multiple_lines(frame):
    result = np.zeros(frame.shape[1])
    for i in range(frame.shape[0]):
        result = np.add(result, frame[i])
    return result / frame.shape[0]

def correlate_1d_arrays(a, b):
    correlation = signal.correlate(a-a.mean(), b-b.mean(), mode='full')
    # correlation_lags = signal.correlation_lags(a.size, b.size, mode='full')
    print(correlation.shape)
    print(correlation)
    return correlation
    # return correlation

def find_offset_in_um(corr, um_per_pixel):
    return np.argmax(corr) * um_per_pixel - corr.size * um_per_pixel / 2

def gaussian_blur_1d_array(a, sigma=1):
    return signal.convolve(a, signal.windows.gaussian(a.size, sigma), mode='same')

def upscale_and_gaussian_blur_1d_array(a, upscale_factor=10, sigma=1):
    """! Upscales and gaussian blurs a 1D array
    @param a The 1D array to upscale and gaussian blur
    @param upscale_factor The factor to upscale the array by
    @param sigma The sigma value for the gaussian blur
    @return The upscaled and gaussian blurred 1D array
    """
    size = (1, a.shape[0] * upscale_factor)
    a_upscaled = cv2.resize(a, size, interpolation=cv2.INTER_LANCZOS4)
    return a_upscaled[:,0]
    # return gaussian_blur_1d_array(a_upscaled, sigma*upscale_factor)

def get_top_and_bottom_lines(frame, display=False, lines_to_average=200):
    """! Returns the top and bottom lines of the image
    @param frame The image to get the top and bottom lines of
    @param display Whether or not to display the top and bottom lines
    @param lines_to_average The number of lines to average
    @return The top and bottom lines of the image
    """
    
    top_line = average_multiple_lines(select_image_area(convert_array_to_grayscale(frame), 0, 0, frame.shape[1], lines_to_average))
    bottom_line = average_multiple_lines(select_image_area(convert_array_to_grayscale(frame), 0, frame.shape[0]-1, frame.shape[1], lines_to_average))

    top_line_copies = np.tile(top_line, (10, 1))
    bottom_line_copies = np.tile(bottom_line, (10, 1))
    if display:
        f, axarr = plt.subplots(2, 1, constrained_layout=True)
        axarr[0].imshow(top_line_copies)
        axarr[0].set_title('Top Line')
        pos = axarr[1].imshow(bottom_line_copies)
        axarr[1].set_title('Bottom Line')
        f.colorbar(pos, ax = axarr[1], location='bottom', label='Pixel Value', pad=0.5)
        plt.show()
    return top_line, bottom_line

def get_distance_differential_um(top_line, bottom_line, pixels_to_um, upscale_factor=10):
    """! Returns the distance differential in um between the top and bottom lines
    @param top_line The top line of the image
    @param bottom_line The bottom line of the image
    @param pixels_to_um The number of um per pixel
    @return The distance differential in um between the top and bottom lines
    """

    top_line_up = upscale_and_gaussian_blur_1d_array(top_line, upscale_factor=upscale_factor)
    bottom_line_up = upscale_and_gaussian_blur_1d_array(bottom_line, upscale_factor=upscale_factor)
    correlation = correlate_1d_arrays(top_line_up, bottom_line_up)
    return find_offset_in_um(correlation, pixels_to_um)

def get_top_and_bottom_regions_gray(frame, region_height, gap=0, display=False):
    """! Returns the top and bottom regions of the image
    @param frame The image to get the top and bottom regions of
    @param region_height The height of the regions
    @param gap The gap between the regions
    @param display Whether or not to display the top and bottom regions
    @return The top and bottom regions of the image
    """
    top_region = select_image_area(convert_array_to_grayscale(frame), 0, 0, frame.shape[1], region_height)
    bottom_region = select_image_area(convert_array_to_grayscale(frame), 0, frame.shape[0]-region_height-gap, frame.shape[1], region_height)
    if display:
        f, axarr = plt.subplots(2, 1, constrained_layout=True)
        axarr[0].imshow(top_region)
        axarr[0].set_title('Top Region')
        pos = axarr[1].imshow(bottom_region)
        axarr[1].set_title('Bottom Region')
        f.colorbar(pos, ax = axarr[1], location='bottom', label='Pixel Value', pad=0.5)
        plt.show()
    return top_region, bottom_region

def get_top_and_bottom_regions_color(frame, region_height, display=False):
    """! Returns the top and bottom regions of the image
    @param frame The image to get the top and bottom regions of
    @param region_height The height of the regions
    @param gap The gap between the regions
    @param display Whether or not to display the top and bottom regions
    @return The top and bottom regions of the image
    """
    top_region = select_image_area(frame, 0, 0, frame.shape[1], region_height)
    bottom_region = select_image_area(frame, 0, frame.shape[0]-region_height, frame.shape[1], region_height)
    if display:
        f, axarr = plt.subplots(2, 1, constrained_layout=True)
        axarr[0].imshow(top_region)
        axarr[0].set_title('Top Region')
        axarr[1].imshow(bottom_region)
        axarr[1].set_title('Bottom Region')
        plt.show()
    return top_region, bottom_region

def get_n_regions(frame, number_of_regions, display=False):
    """! Returns the n regions of the image
    @param frame The image to get the n regions of
    @param number_of_regions The number of regions
    @param display Whether or not to display the n regions
    @return The n regions of the image
    """
    regions = []
    region_height = int(frame.shape[0]//number_of_regions)
    for i in range(number_of_regions):
        regions.append(select_image_area(frame, 0, i*region_height, frame.shape[1], region_height))
    if display:
        f, axarr = plt.subplots(number_of_regions, 1, constrained_layout=True)
        for i in range(number_of_regions):
            axarr[i].imshow(regions[i])
            axarr[i].set_title('Region ' + str(i))
        plt.show()
    return regions

def cross_correlate_2d_arrays(a, b, greyscale=True):
    """! Cross correlates two 2D arrays
    @param a The first 2D array
    @param b The second 2D array
    @return The cross correlation of the two 2D arrays
    """
    # get rid of the color channels by performing a grayscale transform
    # the type cast into 'float' is to avoid overflows
    if greyscale == False:
        im1_gray = np.sum(a.astype('float'), axis=2)
        im2_gray = np.sum(b.astype('float'), axis=2)
    else:
        im1_gray = a.astype('float')
        im2_gray = b.astype('float')

    # get rid of the averages, otherwise the results are not good
    im1_gray -= np.mean(im1_gray)
    im2_gray -= np.mean(im2_gray)

    # calculate the correlation image; note the flipping of one of the images
    return signal.fftconvolve(im1_gray, im2_gray[::-1,::-1], mode='same')

def get_pixel_shift_2d(a, b, upscale_factor=1):
    """! Returns the pixel shift between two 2D arrays
    @param a The first 2D array
    @param b The second 2D array
    @return The pixel shift between the two 2D arrays (X, Y)
    """
    correlation = cross_correlate_2d_arrays(a, b)
    y, x = np.unravel_index(np.argmax(correlation), correlation.shape)
    return (x - a.shape[1] / 2) / upscale_factor, (y - a.shape[0] / 2) / upscale_factor

def get_pixel_shift_2d_fast(a, b, peak_width=21, upscale_factor=1.0, display=False):
    """! Returns the pixel shift between two 2D arrays
    @param a The first 2D array
    @param b The second 2D array
    @param peak_width The width of the peak to use for centroiding (MUST BE ODD)
    @param upscale_factor The upscale factor to use (should be left at 1 for highest speed)
    @param display Whether or not to display the correlation with centroid targets marked
    @return The pixel shift between the two 2D arrays (X, Y)
    """
    correlation_2d = cross_correlate_2d_arrays(a, b)
    y_init, x_init = np.unravel_index(np.argmax(correlation_2d), correlation_2d.shape)[0:2]
    # result = np.unravel_index(np.argmax(correlation_2d), correlation_2d.shape)
    x, y = centroid_sources(correlation_2d, x_init, y_init, box_size=peak_width,
                        centroid_func=centroid_quadratic)
    if display:
        f, axarr = plt.subplots(3, 1, constrained_layout=True)
        axarr[0].imshow(correlation_2d, origin='lower', interpolation='nearest')
        axarr[0].scatter(x, y, marker='x', color='r')
        axarr[0].set_title('Correlation 2D')
        axarr[1].plot(correlation_2d[y_init, :])
        axarr[1].set_title('Correlation 1D through peak')
        axarr[2].set_title('Correlation 1D through peak zoomed to center')
        axarr[2].plot(correlation_2d[y_init, x_init-peak_width//2:x_init+peak_width//2])
        plt.show()
    return (x[0] - a.shape[1] / 2) / upscale_factor, (y[0] - a.shape[0] / 2) / upscale_factor

def get_pixel_shift_n_regions(reference, regions, peak_width=21, upscale_factor=1.0, display=False):
    """! Returns the pixel shift between a reference image and a list of regions
    @param reference The reference image
    @param regions The list of regions
    @param peak_width The width of the peak to use for centroiding (MUST BE ODD)
    @param upscale_factor The upscale factor to use (should be left at 1 for highest speed)
    @param display Whether or not to display the correlation with centroid targets marked
    @return The pixel shift between the reference image and the list of regions (X, Y)
    """
    pixel_shifts = []
    for region in regions:
        pixel_shifts.append(get_pixel_shift_2d_fast(reference, region, peak_width=peak_width, upscale_factor=upscale_factor, display=display))
    return pixel_shifts