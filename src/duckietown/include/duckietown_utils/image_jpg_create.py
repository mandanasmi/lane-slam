


def d8_compressed_image_from_cv_image(image_cv, same_timestamp_as = None):
    """ 
        Create CompressedIamge from a CV image.
    
        TODO: assumptions on format?
    """
    import cv2
    import numpy as np
    from sensor_msgs.msg import CompressedImage
    compress =  cv2.imencode('.jpg', image_cv)[1]
    jpg_data = np.array(compress).tostring()
    msg = CompressedImage()
    if same_timestamp_as is not None:
        msg.header.stamp = same_timestamp_as.header.stamp
    else:
        from rospy import Time
        msg.header.stamp = Time.now()
    msg.format = "jpeg"
    msg.data = jpg_data
    return msg
