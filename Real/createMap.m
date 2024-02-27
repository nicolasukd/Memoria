rosshutdown;
rosinit('http://192.168.123.161:11311');

go1_odom = rossubscriber('/tf');
fqGet = 1000; %frequency for getting topics
maxRange = 5;


go1_pc_face = rossubscriber('/camera1/point_cloud_face');

tStartOdom = tic;
[odomTF, status1, statusText1] = receive(go1_odom);
[pc_face, status2, statusText2] = receive(go1_pc_face);

map3D = occupancyMap3D(1);



while 1
    [go1_test_msg, status1, statusText1] = receive(go1_odom);
    
    if strcmp(go1_test_msg.Transforms.ChildFrameId, 'trunk')
        odomTF = go1_test_msg;
    end

    tEndOdom = toc(tStartOdom);
    %disp(1/tEndOdom)
    if round(1/tEndOdom) <= fqGet
        [pc_face, status2, statusText2] = receive(go1_pc_face);
        %showdetails(odomTF);
        %showdetails(pc_face);

        
        xPos = odomTF.Transforms.Transform.Translation.X;
        yPos = odomTF.Transforms.Transform.Translation.Y;
        zPos = odomTF.Transforms.Transform.Translation.Z;
    
        xRot = odomTF.Transforms.Transform.Rotation.X;
        yRot = odomTF.Transforms.Transform.Rotation.Y;
        zRot = odomTF.Transforms.Transform.Rotation.Z;
        wRot = odomTF.Transforms.Transform.Rotation.W;
    
        pose = [ xPos yPos zPos wRot xRot yRot zRot];
        points = readXYZ(pc_face);
        pc = pointCloud(pc_face.readXYZ);
        
        pc.Color = pc_face.readRGB;
        
        insertPointCloud(map3D,pose,pc,maxRange);
        show(map3D);


        fprintf('%d HZ\n',round(1/tEndOdom));
        disp('-------------------------------------------------');
        tStartOdom = tic;
    end
end

