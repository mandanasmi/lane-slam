uint8 state # 0: idle, 1: capturing, 2: processing
float32 capture_progress

uint32[2] cell_size
float32[4] crop_rect_norm

sensor_msgs/CompressedImage variance_map
Vector2D[] candidates

LEDDetectionArray led_all_unfiltered
