function [DataRms,DataMax,DataMin,DataMean,DataStd,DataDiff] = GetStat(time,data)


DataRms  = sqrt((sum(data.^2))/length(data));
DataMax  = max(abs(data));
DataMin  = min(data);
DataMean = mean(data);
DataStd  = std(data);
DataDiff = diff(data)./diff(time);
DataDiff(end+1,1) = DataDiff(end,1);


end

