clear;
% IMAGES_DIR='images';
IMAGES_DIR='226_night.iids1';
% addpath ./images/

listing = dir([IMAGES_DIR,'/*.jpg']);
file_list = cell(numel(listing), 1);
for i = 1:numel(listing)
    file_list{i} = [IMAGES_DIR,'/', listing(i).name];
end

road_annotation(file_list);

%load('my_saved_file.mat','ud')
