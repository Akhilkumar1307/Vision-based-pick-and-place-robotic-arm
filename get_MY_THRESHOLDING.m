function [output_image]=get_MY_THRESHOLDING(im)
% This function gives the rgb values to be used 
% for thresholding of a MARKER/COLOR

p = impixel(im);    % Lets user select multiple pixels on the marker
[r ,c]=size(p);

red = mean(p(:,1));
grn = mean(p(:,2));
blu = mean(p(:,3));

[r, c, d]=size(im);
output_image=zeros(r,c);
    
% Creating output image based on selected pixels on the MARKER/COLOR.
range = 20;
for i1=1:r
    for i2=1:c
        if( (im(i1,i2,1)>red-range) && (im(i1,i2,1)<red+range) && (im(i1,i2,2)>grn-range) && (im(i1,i2,2)<grn+range) && (im(i1,i2,3)>blu-range) && (im(i1,i2,3)<blu+range) )
            output_image(i1,i2)=1;
        end
    end
end

figure();
imshow(output_image);
end