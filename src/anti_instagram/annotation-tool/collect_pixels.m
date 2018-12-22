function res=collect_pixels(files)
res.regions_pixels={};
for i = 1:numel(files)
    try
    mat_file=load(files{i});
    img=imread(mat_file.img_filename); % replace with corrected image
    for j=1:numel(mat_file.regions)
       pixels=[];
       for v=1:3
           img_c=img(:,:,v);
           mask=mat_file.regions{j}.mask;
           pixels(:,v)=img_c(mask);
       end
       type=mat_file.regions{j}.type;
       if (numel(res.regions_pixels)<type)
           res.regions_pixels{type}=[];
       end
       res.regions_pixels{type}=cat(1,res.regions_pixels{type},pixels);
    end
    catch
        
    end
end
res.stdevs=[];
res.means=[];
for i = 1:numel(res.regions_pixels)
    try
        res.stdevs(i,:)=std(res.regions_pixels{i},1);
        res.means(i,:)=mean(res.regions_pixels{i},1);
    catch
    end
end
end