# 
# 
# class VisualState():
#     """
#         This is the "visual state" that we detected.
#         
#         It contains the original image, the corrected image, the detected lines.
#     
#         This class exists so we can have easy visualization routines
#         and performance estimation.
#     """
#     
#     def __init__(self, original_image, white, yellow, red):
#         self.original_image = original_image
#         self.white = white
#         self.yellow = yellow
#         self.red = red
#         
#         
# 
# def visual_state_from_image(image_cv, line_detector):
#     """ Uses the line detector to construct the Visual State""" 
#     line_detector.setImage(image_cv)
#     white = line_detector.detectLines('white')
#     red = line_detector.detectLines('red')
#     yellow = line_detector.detectLines('yellow')
# 
#     # Visualization
#     vs = VisualState(image_cv, yellow=yellow, red=red, white=white)    
#     return vs