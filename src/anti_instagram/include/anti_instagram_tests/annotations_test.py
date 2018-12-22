#!/usr/bin/env python
import copy
import os
import shelve
import cv2
import scipy.io
import yaml

from anti_instagram import logger, wrap_test_main
from anti_instagram.AntiInstagram import ScaleAndShift, calculate_transform
from duckietown_utils.instantiate_utils import instantiate
from duckietown_utils.jpg import (image_clip_255, image_cv_from_jpg_fn,
                                  make_images_grid)
from duckietown_utils.locate_files_impl import locate_files
from duckietown_utils.path_utils import expand_all, get_ros_package_path
from line_detector.line_detector_plot import drawLines
import numpy as np
from duckietown_utils.image_operations import gray2rgb, zoom_image


def merge_comparison_results(comparison_results,overall_results):
    if (comparison_results):
        if (not overall_results):
            r_vals = {}
            for t in comparison_results['r_vals'].keys():
                r_vals[t]=np.array([],'float32')

            g_vals = copy.deepcopy(r_vals)
            b_vals = copy.deepcopy(r_vals)
            h_vals = copy.deepcopy(r_vals)
            s_vals = copy.deepcopy(r_vals)
            v_vals = copy.deepcopy(r_vals)

            overall_results={'average_abs_err':[],'total_pixels':0,'total_error':0,'total_regions':0,
                             'r_vals':r_vals,'g_vals':g_vals,'b_vals':b_vals,'h_vals':h_vals,'s_vals':s_vals,'v_vals':v_vals}

        
        # max_idx,max_type=max(enumerate(comparison_results['r_vals'].keys()), key=operator.itemgetter(1))
        for t in comparison_results['r_vals'].keys():
            overall_results['r_vals'][t]=np.concatenate((overall_results['r_vals'][t],comparison_results['r_vals'][t]),0)
            overall_results['g_vals'][t]=np.concatenate((overall_results['g_vals'][t],comparison_results['g_vals'][t]),0)
            overall_results['b_vals'][t]=np.concatenate((overall_results['b_vals'][t],comparison_results['b_vals'][t]),0)
            overall_results['h_vals'][t]=np.concatenate((overall_results['h_vals'][t],comparison_results['h_vals'][t]),0)
            overall_results['s_vals'][t]=np.concatenate((overall_results['s_vals'][t],comparison_results['s_vals'][t]),0)
            overall_results['v_vals'][t]=np.concatenate((overall_results['v_vals'][t],comparison_results['v_vals'][t]),0)


        overall_results['total_error']=overall_results['total_error']+comparison_results['total_error']
        overall_results['total_pixels']=overall_results['total_pixels']+comparison_results['total_pixels']
        overall_results['total_regions']=overall_results['total_regions']+comparison_results['total_regions']

    return overall_results

def examine_dataset(dirname, out):
    logger.info(dirname)
    dirname = expand_all(dirname)

    jpgs = locate_files(dirname, '*.jpg')
    mats = locate_files(dirname, '*.mat')

    logger.debug('I found %d JPGs and %d .mat.' % (len(jpgs), len(mats)))

    if len(jpgs) == 0:
        msg = 'Not JPGs found in %r.' % dirname
        raise ValueError(msg)

#     if len(mats) == 0:
#         msg = 'Not enough mats.'
#         raise ValueError(msg)

    first_jpg = sorted(jpgs)[0]
    logger.debug('Using jpg %r to learn transformation.' % first_jpg)

    first_jpg_image = image_cv_from_jpg_fn(first_jpg)


    success, health, parameters = calculate_transform(first_jpg_image)

    s = ""
    s += 'success: %s\n' % str(success)
    s += 'health: %s\n' % str(health)
    s += 'parameters: %s\n' % str(parameters)
    w = os.path.join(out, 'learned_transform.txt')
    with open(w, 'w') as f:
        f.write(s)
    logger.info(s)

    transform = ScaleAndShift(**parameters)

    duckietown_package_dir = get_ros_package_path('duckietown')
    config_dir = os.path.join(duckietown_package_dir, 'config/baseline/line_detector/line_detector_node')

    if not os.path.exists(config_dir):
        msg = 'Could not find configuration dir %s' % config_dir
        raise Exception(msg)

    config_dir = expand_all(config_dir)
    configurations = locate_files(config_dir, '*.yaml')

    if not configurations:
        msg = 'Could not find any configuration file in %s.' % config_dir
        raise Exception(msg)
    #logger.info('configurations: %r' % configurations)

    for j in jpgs:
        summaries =[]

        shape = (200, 160)
        interpolation = cv2.INTER_NEAREST

        bn = os.path.splitext(os.path.basename(j))[0]
        fn = os.path.join(out, '%s.all.png' % (bn))

        if os.path.exists(fn):
            logger.debug('Skipping because file exists: %r' % fn)
        else:
            for c in configurations:
                logger.info('Trying %r' % c)
                name = os.path.splitext(os.path.basename(c))[0]
                if name in ['oreo', 'myrtle', 'bad_lighting', '226-night']:
                    continue
                with open(c) as f:
                    stuff = yaml.load(f)

                if not 'detector' in stuff:
                    msg = 'Cannot find "detector" section in %r' % c
                    raise ValueError(msg)

                detector = stuff['detector']
                logger.info(detector)
                if not isinstance(detector, list) and len(detector) == 2:
                    raise ValueError(detector)

                def LineDetectorClass():
                    return instantiate(detector[0], detector[1])

                s = run_detection(transform, j, out, shape=shape,
                                  interpolation=interpolation, name=name,
                                  LineDetectorClass=LineDetectorClass)
                summaries.append(s)

            together = make_images_grid(summaries, cols=1, pad=10, bgcolor=[.5, .5, .5])
            cv2.imwrite(fn, zoom_image(together, 4))

    overall_results=[]
    comparison_results={}
    for m in mats:
        logger.debug(m)
        jpg = os.path.splitext(m)[0] + '.jpg'
        if not os.path.exists(jpg):
            msg = 'JPG %r for mat %r does not exist' % (jpg, m)
            logger.error(msg)
        else:
            frame_results=test_pair(transform, jpg, m, out)
            comparison_results[m]=frame_results
            overall_results=merge_comparison_results(comparison_results[m],overall_results)
            print "comparison_results[m]=frame_results"
            
    print "finished mats: "+dirname
    return overall_results

def run_detection(transform, jpg, out, shape, interpolation,
                  name, LineDetectorClass):
    image = image_cv_from_jpg_fn(jpg)

    image = cv2.resize(image, shape, interpolation=interpolation)


#     bgr = bgr[bgr.shape[0] / 2:, :, :]

    image_detections = line_detection(LineDetectorClass, image)
    transformed = transform(image)

    transformed_clipped = image_clip_255(transformed)
    transformed_detections = line_detection(LineDetectorClass, transformed_clipped)

    if not os.path.exists(out):
        os.makedirs(out)
    bn = os.path.splitext(os.path.basename(jpg))[0]

    def write(postfix, im):
        fn = os.path.join(out, '%s.%s.%s.png' % (bn, name, postfix))
        cv2.imwrite(fn, zoom_image(im, 4))

    together = make_images_grid([image,  # transformed,
                                 merge_masks_res(image_detections),
                                 gray2rgb(image_detections['edges']),
                                 image_detections['annotated'],

                                 transformed_clipped,
                                 merge_masks_res(transformed_detections),
                                 gray2rgb(transformed_detections['edges']),
                                 transformed_detections['annotated'],
                       ],

                                cols=4, pad=35, bgcolor=[1, 1, 1])

    # write the string "name" in the upper left of image together
    cv2.putText(together, name, (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    return together

def merge_masks_res(res):
    return merge_masks(res['area_white'], res['area_red'], res['area_yellow'])

def merge_masks(area_white, area_red, area_yellow):
    B, G, R = 0, 1, 2
    def white(x):
        x = gray2rgb(x)
        return x
    def red(x):
        x = gray2rgb(x)
        x[:,:,R] *= 1
        x[:,:,G] *= 0
        x[:,:,B] *= 0
        return x
    def yellow(x):
        x = gray2rgb(x)
        x[:,:,R] *= 1
        x[:,:,G] *= 1
        x[:,:,B] *= 0
        return x
    h, w = area_white.shape
    orig = [area_white, area_red, area_yellow]
    masks = [white(area_white), red(area_red), yellow(area_yellow)]

    res = np.zeros(shape=masks[0].shape, dtype=np.uint8)

    for i, m in enumerate(masks):
        nz = (orig[i] > 0) * 1.0
        assert nz.shape == (h, w), nz.shape

        for j in [0, 1, 2]:
            res[:,:,j] = (1-nz) * res[:,:,j].copy() + (nz) * m[:,:,j]

    return res


def test_pair(transform, jpg, mat, out):
    """
        jpg = filename
        mat = filename
    """

    data = scipy.io.loadmat(mat)
    regions = data['regions'].flatten()
    max_type=0;
    for r in regions:
        max_type=max(max_type,r['type'][0][0][0][0])
    r_vals={};

    for t in np.arange(max_type):
        r_vals[t+1]=np.array([],'float32')

    g_vals = copy.deepcopy(r_vals);
    b_vals = copy.deepcopy(r_vals);
    h_vals = copy.deepcopy(r_vals);
    s_vals = copy.deepcopy(r_vals);
    v_vals = copy.deepcopy(r_vals);

    result_stats={'average_abs_err':[],'total_pixels':0,'total_error':0,'total_regions':0,
                  'r_vals':r_vals,'g_vals':g_vals,'b_vals':b_vals,'h_vals':h_vals,'s_vals':s_vals,'v_vals':v_vals}
    for r in regions:
        logger.info('region')
        x = r['x'][0][0].flatten()
        y = r['y'][0][0].flatten()
        mask = r['mask'][0][0]
        mask3=cv2.merge([mask,mask,mask])
        print 'x', x
        print 'y', y
        print 'mask shape', mask.shape
        # type in 1- based / matlab-based indices from the list of region types (i.e road, white, 
        # yellow, red, or what ever types were annotated)
        print 'type', r['type'][0][0][0][0]
        # color in [r,g,b] where [r,g,b]are between 0 and 1 
        print 'color', r['color'][0] 
        t=r['type'][0][0][0][0];
        # print 'guy look here'
        region_color=r['color'][0];region_color=region_color[0][0]
        rval=region_color[0]*255.;
        gval=region_color[1]*255.;
        bval=region_color[2]*255.;
        image = image_cv_from_jpg_fn(jpg)
        transformed = transform(image)
        [b2,g2,r2]=cv2.split(transformed)
        thsv=cv2.cvtColor(transformed,cv2.COLOR_BGR2HSV)
        [h2,s2,v2]=cv2.split(thsv)
        r2_=r2[mask.nonzero()];g2_=g2[mask.nonzero()];b2_=b2[mask.nonzero()]
        h2_=h2[mask.nonzero()];s2_=s2[mask.nonzero()];v2_=v2[mask.nonzero()]
        
        result_stats['r_vals'][t]=np.concatenate((result_stats['r_vals'][t],r2_),0)
        result_stats['g_vals'][t]=np.concatenate((result_stats['g_vals'][t],g2_),0)
        result_stats['b_vals'][t]=np.concatenate((result_stats['b_vals'][t],b2_),0)
        result_stats['h_vals'][t]=np.concatenate((result_stats['h_vals'][t],h2_),0)
        result_stats['s_vals'][t]=np.concatenate((result_stats['s_vals'][t],s2_),0)
        result_stats['v_vals'][t]=np.concatenate((result_stats['v_vals'][t],v2_),0)
        absdiff_img=cv2.absdiff(transformed,np.array([bval,gval,rval,0.]))
        masked_diff=cv2.multiply(np.array(absdiff_img,'float32'),np.array(mask3,'float32'))
        num_pixels=cv2.sumElems(mask)[0];
        region_error=cv2.sumElems(cv2.sumElems(masked_diff))[0];
        avg_abs_err=region_error/(num_pixels+1.);
        print 'Average abs. error', avg_abs_err
        result_stats['average_abs_err'].append(avg_abs_err)
        result_stats['total_pixels']=result_stats['total_pixels']+num_pixels
        result_stats['total_error']=result_stats['total_error']+region_error
        result_stats['total_regions']=result_stats['total_regions']+1
        # XXX: to finish
    return result_stats

def line_detection(LineDetectorClass, bgr):
    detector = LineDetectorClass()
    detector.setImage(bgr)
    image_with_lines = bgr.copy()

    # detect lines and normals
    white = detector.detectLines('white')
    yellow = detector.detectLines('yellow')
    red = detector.detectLines('red')

    # draw lines
    drawLines(image_with_lines, white.lines, (0, 0, 0))
    drawLines(image_with_lines, yellow.lines, (255, 0, 0))
    drawLines(image_with_lines, red.lines, (0, 255, 0))

#     elif isinstance(detector, LineDetector2):
#         # detect lines and normals
#         lines_white, normals_white, centers_white, area_white = detector.detectLines2('white')
#         lines_yellow, normals_yellow, centers_yellow, area_yellow = detector.detectLines2('yellow')
#         lines_red, normals_red, centers_red, area_red = detector.detectLines2('red')
#
#         # draw lines
#         drawLines(image_with_lines, lines_white, (0, 0, 0))
#         drawLines(image_with_lines, lines_yellow, (255, 0, 0))
#         drawLines(image_with_lines, lines_red, (0, 255, 0))
#
        # draw normals
        #detector.drawNormals2(centers_white, normals_white, (0, 0, 0))
        #detector.drawNormals2(centers_yellow, normals_yellow, (255, 0, 0))
        #detector.drawNormals2(centers_red, normals_red, (0, 255, 0))

    res = {}
    res['annotated'] = image_with_lines
    res['area_white'] = white.area
    res['area_red'] = red.area
    res['area_yellow'] = yellow.area
    res['edges'] = detector.edges
    return res

#    cv2.imwrite('lines_with_normal.png', detector.getImage())



def anti_instagram_annotations_test(dirname, out_base):
    base = expand_all(dirname)

    if not os.path.exists(base):
        msg = 'Could not find directory %s' % base
        raise Exception(msg)
        
    dirs = locate_files(base, '*.iids1', alsodirs=True)
    directory_results={}
    overall_results=[]

    if not dirs:
        raise ValueError('No IIDS1 directories found')
 
    for d in dirs: 
        out = os.path.join(out_base, os.path.basename(d) + '.v')

        if not os.path.exists(out):
            os.makedirs(out)
        results = examine_dataset(d, out)
        overall_results = merge_comparison_results(results, overall_results)
        directory_results[d] = results
        
    db=shelve.open('tests_results',flag='n')
    db['directory_results'] = directory_results
    db['overall_results'] = overall_results
    db.close()

    logger.info("overall average error: %f" % (overall_results['total_error']/overall_results['total_pixels']))
    logger.info("overall regions checked: %f" % (overall_results['total_regions']))
    for t in overall_results['v_vals'].keys():
        logger.info("region %f: RGB %f,%f,%f, HSV %f,%f,%f"  % 
              (t,
               np.mean(overall_results['r_vals'][t]),
               np.mean(overall_results['g_vals'][t]),
               np.mean(overall_results['b_vals'][t]),
               np.mean(overall_results['h_vals'][t]),
               np.mean(overall_results['s_vals'][t]),
               np.mean(overall_results['v_vals'][t])))

    

if __name__ == '__main__':
    wrap_test_main(anti_instagram_annotations_test)
