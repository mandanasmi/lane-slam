clear;
addpath ./images/

listing = dir('images/*.jpg');
file_list = cell(numel(listing), 1);
for i = 1:numel(listing)
    file_list{i} = ['images/', listing(i).name];
end

road_annotation(file_list);

%load('my_saved_file.mat','ud')
