%
% splits annotation files by saving each annotations set from the map under
% XXX.mat, where XXX.jpg was the original filename.
%
% Usage: from the same directory as the annotator, use:
% split_annotation_by_files(joint_filename)
%
% to read in python, use for example:
% In [17]: res=scipy.io.loadmat('frame0000.mat')
% In [18]: reg=res['regions']
% In [19]: r1=reg[0,0]
% In [20]: r1['x']
% Out[20]: 
% array([[ array([[ 273.73076923],
%        [ 374.65384615],
%        [ 428.80769231],
%        [  26.34615385],
%        [ 273.73076923]])]], dtype=object)
% 
%
function split_annotation_by_files(joint_filename)
SAVE_EMPTY_ANNOTATIONS=false;
load(joint_filename,'ud');
close all hidden
annotations=ud.annotations_map;
keys=annotations.keys;
vals=annotations.values;
for i = 1:numel(annotations.keys)
    img_filename=keys{i};
    new_mat_filename=[img_filename(1:(end-4)),'.mat'];
    regions=vals{i};
    if (~isempty(regions)) || SAVE_EMPTY_ANNOTATIONS
    save(new_mat_filename,'regions','img_filename');
    end
end
end