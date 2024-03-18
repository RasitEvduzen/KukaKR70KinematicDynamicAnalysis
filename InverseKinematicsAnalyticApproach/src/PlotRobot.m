function [] = PlotRobot(TransPlot,baseGeo,link1Geo,link2Geo,link3Geo,link4Geo,link5Geo,link6Geo)

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

base = trisurf(baseGeo.ConnectivityList, baseGeo.Points(:, 1), baseGeo.Points(:, 2), baseGeo.Points(:, 3));
base.FaceColor = [0.5, 0.5, 0.5];
base.EdgeColor = "none";

l1 = trisurf(link1Geo.ConnectivityList, plotLink1(:, 1), plotLink1(:, 2), plotLink1(:, 3));
l1.FaceColor = "#EDB120";
l1.EdgeColor = "none";

l2 = trisurf(link2Geo.ConnectivityList, plotLink2(:, 1), plotLink2(:, 2), plotLink2(:, 3));
l2.FaceColor = "#EDB120";
l2.EdgeColor = "none";

l3 = trisurf(link3Geo.ConnectivityList, plotLink3(:, 1), plotLink3(:, 2), plotLink3(:, 3));
l3.FaceColor = "#EDB120";
l3.EdgeColor = "none";

l4 = trisurf(link4Geo.ConnectivityList, plotLink4(:, 1), plotLink4(:, 2), plotLink4(:, 3));
l4.FaceColor = "#EDB120";
l4.EdgeColor = "none";

l5 = trisurf(link5Geo.ConnectivityList, plotLink5(:, 1), plotLink5(:, 2), plotLink5(:, 3));
l5.FaceColor = "#EDB120";
l5.EdgeColor = "none";

l6 = trisurf(link6Geo.ConnectivityList, plotLink6(:, 1), plotLink6(:, 2), plotLink6(:, 3));
l6.FaceColor = [0.5, 0.5, 0.5];
l6.EdgeColor = "none";

camlight;
lightangle(250, 15);

end