file_list={'agirard/frame0000.jpg','agirard/frame0001.jpg','agirard/frame0002.jpg'};
my_classes={};
my_classes{end+1}=struct('name','road','color',[0 0 0]);
my_classes{end+1}=struct('name','duck','color',[1 1 0]);
my_classes{end+1}=struct('name','cone','color',[1 0.5 0]);

road_annotation(file_list,'classes',my_classes);
% load('my_saved_file.mat','ud')