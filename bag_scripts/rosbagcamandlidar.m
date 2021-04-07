%% clear data
clear

%% choose a measurement
measurement_database = dir('*.bag');
[chosenBag,~] = listdlg('PromptString','Select a mat file:','SelectionMode','single', 'ListString', {measurement_database.name}, 'ListSize', [500 200]);
if ~isempty(chosenBag)
    bag = rosbag((measurement_database(chosenBag).name));
    id = measurement_database(chosenBag).name;
else
    disp('Nothing is chosen');
    return;
end

%% /gps/current_pose
%{
sel = select(bag, 'Time', [bag.StartTime bag.EndTime],'Topic','/gps/duro/current_pose');
sel_struct = readMessages(sel,'DataFormat','struct');
gps_kvh_x_pos = cellfun(@(m) double(m.Pose.Position.X), sel_struct);
gps_kvh_y_pos = cellfun(@(m) double(m.Pose.Position.Y), sel_struct);
tmp_gps_kvh_q_w = cellfun(@(m) double(m.Pose.Orientation.W), sel_struct);
tmp_gps_kvh_q_x = cellfun(@(m) double(m.Pose.Orientation.X), sel_struct);
tmp_gps_kvh_q_y = cellfun(@(m) double(m.Pose.Orientation.Y), sel_struct);
tmp_gps_kvh_q_z = cellfun(@(m) double(m.Pose.Orientation.Z), sel_struct);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec), sel_struct);
sec = cellfun(@(m) double(m.Header.Stamp.Sec), sel_struct);
gps_kvh_time = sec + nsec / 1000000000;
%}

%% /zed_node/left/image_rect_color/compressed

sel = select(bag, 'Time', [bag.StartTime bag.StartTime+1],'Topic','/zed_node/left/image_rect_color/compressed');
msgs = readMessages(sel);
figure
legend on; hold on;
imgMsg = msgs{1}; %% 1st image
rgbImg = readImage(imgMsg);
imshow(rgbImg)


%% /left_os1/os1_cloud_node/points
sel = select(bag, 'Time', [bag.StartTime bag.StartTime+1],'Topic','/left_os1/os1_cloud_node/points');
msgs = readMessages(sel);
lidarPtCloudMsg = msgs{1}; %% 1st lidar poincloud
xyz = readXYZ(lidarPtCloudMsg);
lidarPtCloudMsg.Fields.Name
allfields = readAllFieldNames(lidarPtCloudMsg);
x_ = readField(lidarPtCloudMsg,'x');
y_ = readField(lidarPtCloudMsg,'y');
z_ = readField(lidarPtCloudMsg,'z');
intens_ = readField(lidarPtCloudMsg,'intensity');
refl_ = readField(lidarPtCloudMsg,'reflectivity');
amb_ = readField(lidarPtCloudMsg,'ambient');

figure
legend on; hold on;
%scatter3(lidarPtCloudMsg)
scatter3(x_,y_,z_,10,intens_, 'filled');
% colomap and bar
colormap(jet);
caxis([1 1000])
colorbar;

%%
%{
figure
legend on; hold on;
plot(gps_kvh_x_pos, 'DisplayName', 'x acceleration', 'MarkerSize', 6); 
plot(gps_kvh_y_pos, 'DisplayName', 'y acceleration', 'MarkerSize', 6); 
%}
