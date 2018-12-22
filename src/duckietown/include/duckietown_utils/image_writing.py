from collections import OrderedDict
import os

from .exception_utils import check_isinstance
from .file_utils import write_data_to_file
from .image_composition import make_images_grid
from .image_rescaling import d8_image_resize_no_interpolation
from .image_timestamps import add_header_to_image
from .jpg import jpg_from_image_cv, write_jpg_to_file


def write_image_as_jpg(image, filename):
    import numpy as np
    if not isinstance(image, np.ndarray):
        # XXX
        pass
    jpg = jpg_from_image_cv(image)
    write_data_to_file(jpg, filename)
    
def write_jpgs_to_dir(name2image, dirname):
    """ 
        Write a set of images to a directory.
        
        name2image is a dictionary of name -> image 
        
        Images are assumed to be BGR, [H,W,3] uint8.
    """
    check_isinstance(name2image, dict)
    res = OrderedDict(name2image)
    shape = None
    for i, (filename, image) in enumerate(name2image.items()):
        if shape is None:
            shape = image.shape[:2]
        if image.shape[:2] != shape:
            name2image[filename] = d8_image_resize_no_interpolation(image, shape) 
        
    images = []
    for i, (filename, image) in enumerate(res.items()):
        s =  filename
        res[filename] = add_header_to_image(image, s)
        images.append(res[filename])
        
    res['all'] = make_images_grid(images)
        
    for i, (filename, image) in enumerate(res.items()):
        if filename == 'all':
            basename = 'all'
        else:
            basename = ('step%02d-'%i)+filename
        
        fn = os.path.join(dirname,basename+'.jpg')
        write_jpg_to_file(image, fn)
        