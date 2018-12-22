import os
import shutil

from duckietown_utils import logger

from .disk_hierarchy import create_tmpdir

def d8n_make_video_from_bag(bag_filename, topic, out):
    """
       Creates a video out of the topic in the bag.
       
       topic: the topic name (any image-like topic)
       out: an .mp4 file. 
       
       Note that needs a bunch more dependencies to be installed.
       
       Until we fix the dependencies:
        
            sudo pip install SystemCmd==1.2 ros_node_utils==1.0 ConfTools==1.8 QuickApp==1.2.2
        
            sudo apt-get install -y  mplayer mencoder
        
            sudo add-apt-repository ppa:mc3man/trusty-media
            sudo apt-get update
            sudo apt-get install -y ffmpeg 
            
        #gstreamer0.10-ffmpeg
        
    """
    try:
        import procgraph_ros  # @UnusedImport
        from procgraph import pg
    except ImportError:
        raise 
    
    # pg -m procgraph_ros bag2mp4 --bag $bag --topic $topic --out $out
      
    model = 'bag2mp4_fixfps'
#     model = 'bag2mp4'
    tmpdir = create_tmpdir()
    out_tmp = os.path.join(tmpdir, os.path.basename(out))
    logger.debug('Writing temp file to %s' % out_tmp)
    logger.debug('(You can use mplayer to follow along.)')
    pg(model, config=dict(bag=bag_filename, topic=topic, out=out_tmp))
    md = out_tmp + '.metadata.yaml'
    if os.path.exists(md):
        os.unlink(md)

    dn = os.path.dirname(out)
    if not os.path.exists(dn):
        os.makedirs(dn)
    
    shutil.copyfile(out_tmp, out)
    logger.info('Created: %s' % out)

    info = out_tmp + '.info.yaml'
    if os.path.exists(info):
        os.unlink(info)
        
        
        