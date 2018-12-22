


def d8_image_zoom_linear(cv_image, ratio=4):
    """ Zooms up by the given ratio """
    import cv2
    H, W, _ = cv_image.shape
    W2 = int(W*ratio)
    H2 = int(H*ratio)
    res = cv2.resize(cv_image, (W2, H2), interpolation=cv2.INTER_NEAREST)
    return res

def d8_image_resize_no_interpolation(cv_image, new_shape):
    """
        new_shape = (H, W)
    """
    import cv2
    H, W = new_shape
    res = cv2.resize(cv_image, (W,H), interpolation=cv2.INTER_NEAREST)
    return res