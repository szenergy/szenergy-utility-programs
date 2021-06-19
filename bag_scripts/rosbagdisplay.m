% download a rosbag file e.g. from http://www.sze.hu/~herno/PublicDataAutonomous/

%% clear data
clear

%% choose a measurement
% measurement_database = dir('*.bag');
% [chosenBag,~] = listdlg('PromptString','Select a mat file:','SelectionMode','single', 'ListString', {measurement_database.name}, 'ListSize', [500 200]);
% if ~isempty(chosenBag)
%     bag = rosbag((measurement_database(chosenBag).name));
%     id = measurement_database(chosenBag).name;
% else
%     disp('Nothing is chosen');
%     return;
% end

%% choose a measurement
bag = rosbag('tiny_sample04.bag');

%% List available topics
% bag.AvailableTopics


%% /zed_node/left/image_rect_color/compressed
sel = select(bag, 'Time', [bag.StartTime bag.StartTime+1],'Topic','/zed_node/left/image_rect_color/compressed');
% msgs = readMessages(sel, "DataFormat", "struct"); % https://www.mathworks.com/matlabcentral/answers/523440-error-using-ros-bagselection-deserializemessages
msgs = readMessages(sel);
figure; hold on;
imgMsg = msgs{1}; %% 1st image
rgbImg = readImage(imgMsg);
imshow(rgbImg)


%% /left_os1/os1_cloud_node/points
sel = select(bag, 'Topic','/left_os1/os1_cloud_node/points');
msgs = readMessages(sel);
lidarPtCloudMsg = msgs{1}; %% 1st lidar poincloud
xyz = readXYZ(lidarPtCloudMsg);
% lidarPtCloudMsg.Fields.Name
allfields = readAllFieldNames(lidarPtCloudMsg);
x_ = readField(lidarPtCloudMsg,'x');
y_ = readField(lidarPtCloudMsg,'y');
z_ = readField(lidarPtCloudMsg,'z');
intens_ = readField(lidarPtCloudMsg,'intensity');
refl_ = readField(lidarPtCloudMsg,'reflectivity');
amb_ = readField(lidarPtCloudMsg,'ambient');

%% LIDAR in 2D scatter plot
figure
legend on; hold on;
% scatter3(lidarPtCloudMsg)
% scatter3(x_,y_,z_,10,intens_, 'filled');
scatter3(x_,y_,z_,10, z_, 'filled');
caxis([-1.6 -0.8])
% caxis([min(z_) min(z_)+1.5])
view([-50 70])

% colomap and bar
colormap(jet);
grid on
ax = gca; ax.Clipping = 'off';
colorbar;

%% LIDAR in 2D scatter plot
% figure
% scatter(x_,y_,1,z_, 'filled')
% caxis([-1.6 -0.8]);
% axis equal;
% grid on;
% colormap(jet);
% colorbar;


