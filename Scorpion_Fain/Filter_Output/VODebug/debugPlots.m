function [  ] = debugPlots( data, index, dim, titleText, outliers )

points = cellfun(@str2double,data{index}(:,1:2*dim));
indeces = find(outliers==1);
translation = points(:,1:dim)-points(:,dim+1:2*dim);
%translation(indeces,:) = translation(indeces,:)*NaN;
figure
for i=1:dim
    subplot(dim,1,i)
    plot(translation(:,i))
    if i==1
        title(titleText)
    end
end
end