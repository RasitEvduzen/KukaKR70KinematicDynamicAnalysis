function [] = PlotRobot(TransPlot,baseGeo,link1Geo,link2Geo,link3Geo,link4Geo,link5Geo,link6Geo,torchGeo,WTorch,WTtransMat)

% Plot Triad
trplot(eye(4),'frame','0','thick',1,'rgb','length',300)
trplot(TransPlot(:,1:4),'frame','1','thick',1,'rgb','length',300);
trplot(TransPlot(:,5:8),'frame','2','thick',1,'rgb','length',300);
trplot(TransPlot(:,9:12),'frame','3','thick',1,'rgb','length',300);
trplot(TransPlot(:,13:16),'frame','4','thick',1,'rgb','length',300);
trplot(TransPlot(:,17:20),'frame','5','thick',1,'rgb','length',300);
trplot(TransPlot(:,21:24),'frame','6','thick',1,'rgb','length',300);

plotLink1 = TransformGeo(link1Geo.Points, TransPlot(:,1:4));
plotLink2 = TransformGeo(link2Geo.Points, TransPlot(:,5:8));
plotLink3 = TransformGeo(link3Geo.Points, TransPlot(:,9:12));
plotLink4 = TransformGeo(link4Geo.Points, TransPlot(:,13:16));
plotLink5 = TransformGeo(link5Geo.Points, TransPlot(:,17:20));
plotLink6 = TransformGeo(link6Geo.Points, TransPlot(:,21:24));

trisurf(baseGeo.ConnectivityList, baseGeo.Points(:, 1), baseGeo.Points(:, 2), baseGeo.Points(:, 3), ...
    FaceColor = [0.5, 0.5, 0.5],EdgeColor = "none");

trisurf(link1Geo.ConnectivityList, plotLink1(:, 1), plotLink1(:, 2), plotLink1(:, 3), ...
   FaceColor = "#EDB120",EdgeColor = "none");

trisurf(link2Geo.ConnectivityList, plotLink2(:, 1), plotLink2(:, 2), plotLink2(:, 3), ...
   FaceColor = "#EDB120",EdgeColor = "none");

trisurf(link3Geo.ConnectivityList, plotLink3(:, 1), plotLink3(:, 2), plotLink3(:, 3), ...
   FaceColor = "#EDB120",EdgeColor = "none");

trisurf(link4Geo.ConnectivityList, plotLink4(:, 1), plotLink4(:, 2), plotLink4(:, 3), ...
   FaceColor = "#EDB120",EdgeColor = "none");

trisurf(link5Geo.ConnectivityList, plotLink5(:, 1), plotLink5(:, 2), plotLink5(:, 3), ...
   FaceColor = "#EDB120",EdgeColor = "none");

trisurf(link6Geo.ConnectivityList, plotLink6(:, 1), plotLink6(:, 2), plotLink6(:, 3), ...
    FaceColor = [0.5, 0.5, 0.5],EdgeColor = "none");

if WTorch == "On"
    % --------------------- Torch Plot  ---------------------%
    trplot(TransPlot(:,21:24)*WTtransMat,'frame','ee','thick',1,'rgb','length',100);
    plotTorch = TransformGeo(torchGeo.Points, TransPlot(:,21:24));
    trisurf(torchGeo.ConnectivityList, plotTorch(:, 1), plotTorch(:, 2), plotTorch(:, 3), ...
        FaceColor = [0, 1, 0.5],EdgeColor = "none");
    % --------------------- Table Plot  ---------------------%
    tableGeo = stlread("table.STL");
    TableTriad = eye(4);
    TableTriad(1:3,4) = TableTriad(1:3,4) + [1430 0 959.85]'; % Table Offset
    plotTable = TransformGeo(tableGeo.Points, TableTriad);
    trisurf(tableGeo.ConnectivityList, plotTable(:, 1), plotTable(:, 2), plotTable(:, 3), ...
        FaceColor = [1, 0, 0],EdgeColor = "none");
    % --------------------- Plate Plot  ---------------------%
    plateGeo = stlread("plate.STL");
    PlateTriad = eye(4);
    PlateTriad(1:3,4) = PlateTriad(1:3,4) + [1430 0 959.85]'; % Table Offset
    plotTable = TransformGeo(plateGeo.Points, PlateTriad);
    trisurf(plateGeo.ConnectivityList, plotTable(:, 1), plotTable(:, 2), plotTable(:, 3), ...
        FaceColor = [1, 1, 1],EdgeColor = "none");
    
end
camlight;
lightangle(250, 15);

end