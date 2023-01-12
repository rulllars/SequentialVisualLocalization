this_dir = fileparts(mfilename('fullpath'));
CMU_dir = [this_dir '/../../../Data/CMU'];

dvlad_files = [CMU_dir '/DVLAD/dvlad_run%02d.mat']; % precomputed dvlad descriptors for all images
GTfile = [CMU_dir '/FixedMap/ReferencePoses.mat'];
measurement_file = [CMU_dir '/Odometry/measurements_bias01.mat'];

tic;
mapRun = 11;
plotAll = false;
locRuns = [1:10, 12];
slices = 2:25;
nStates = 20000;
[stdPos, stdAng] = motionModelParams;
R = 0.1;
P_rl = 1e-5;

%Load dvlad descriptors and filenames
ref_vlad_fn = sprintf(dvlad_files, mapRun);
load(ref_vlad_fn, 'file2dvlad');
vfn = file2dvlad.keys()';


GT = load(GTfile);

MapPosesGT = cell2mat({GT.gtPose(slices,mapRun).pose}');
MapTime = cell2mat({GT.gtPose(slices,mapRun).time}');

%Create vector of distance traveled so that each image in the reference run
%has a 1-D distance associated to it.
dd = sqrt(sum([0 0 0; diff(MapPosesGT(:,1:3))].^2,2));
tmpfn1 = {GT.images(slices,mapRun).name}';
tmpfn2 = cat(1, tmpfn1{:});
[~, ix] = unique(tmpfn2, 'stable');
ix = sort(ix);
tmpfn3 = tmpfn2(ix);
nix = setdiff(1:numel(dd), ix);
false_duplicate_ix = (dd(nix) ~= 0);
false_duplicate_ix = nix(false_duplicate_ix);
if ~isempty(false_duplicate_ix)
    warning('Not all removed images are duplicates in the distance vector also. Distance diff = %f.', sum(dd(false_duplicate_ix)));
    disp(false_duplicate_ix)
end

x_ref = cumsum(dd(ix));
[x_unique, jx] = unique(x_ref);
pose_ref = MapPosesGT(ix,:);
pose_unique = pose_ref(jx, :);

%sort dvlad descriptors to have the same order as the file name vectors
%above.
[la, ib] = ismember(tmpfn3, vfn);
lb = ismember(vfn, tmpfn3);
tmpfn4 = vfn(ib);
[~, a] = sort(tmpfn3);
[~, b] = sort(tmpfn4);
if ~(all(la) && all(lb) && all(a==b))
    warning('Image file name vectors do not match')
end
dvlads = file2dvlad.values();
refvlads = cell2mat(dvlads(ib));


%Separate vectors for left and right cameras
iCam = contains(tmpfn3, '_c1_')+1;
ref = [];
for i = 1:2
    ref(i).name = tmpfn3(iCam==i);
    ref(i).dist = x_ref(iCam==i);
    ref(i).vlad = refvlads(:,iCam==i);
    [~, uid] = unique(x_ref(iCam==i));
    nuid = setdiff(1:sum(iCam==i), uid);
    assert(numel(nuid) == 0, 'Something is funky with the distance vector or the image file name vector');
end


%Now reference data is all loaded and processed. Time to load the 
%localization data.

measurement = load(measurement_file);
filtertxtfile = fopen([this_dir '/CMU_filter_results.txt'], 'w');
smoothingtxtfile = fopen([this_dir '/CMU_smoothing_results.txt'], 'w');

for locRun = locRuns
    
    system(sprintf('mkdir -p "%s/temp_CMU_run%02d"', this_dir, locRun));
    savefn = sprintf('%s/result_1d_filt_CMU_run%02d.mat', this_dir, locRun);
    if exist(savefn, 'file')
        continue;
    end
    disp(locRun)
    x_max = x_ref(end);

    %vlad descriptors
    loc_vlad_fn = sprintf(dvlad_files, locRun);
    load(loc_vlad_fn, 'file2dvlad');

    %State representation is a histogram/PMF with bins equally distributed
    %along the whole trajectory.
    %A bit wasteful to have same resolution of the state bins everywhere, but
    %it's easy.
    x_state = linspace(-30, 10+x_max, nStates);
    p_state = ones(1, nStates)/nStates;

    [locImTime, lix] = unique(cell2mat({GT.images(slices,locRun).time}'));
    tmpfn5 = {GT.images(slices,locRun).name}';
    tmpfn6 = cat(1, tmpfn5{:});
    locImName = tmpfn6(lix);
    t0 = locImTime(1);
    locImTime = locImTime - t0;
    t = locImTime(1);
    kImg = 0;
    v_DR = measurement.data(locRun).DR.speed;
    t_DR = measurement.data(locRun).DR.time - t0;
    kDR = find(t_DR == t);
    kDR = kDR(1);
    LocPosesFilterEst = zeros(numel(locImTime), 7);
    CamPosesFilterEst = zeros(numel(locImTime), 7);
    LocPosesSmoothEst = zeros(numel(locImTime), 7);
    CamPosesSmoothEst = zeros(numel(locImTime), 7);
    FileNames = cell(numel(locImTime), 1);
    hasGT = ~isempty(GT.gtPose(slices(1),locRun).pose);
    if hasGT
        LocPosesGT = cell2mat({GT.gtPose(slices,locRun).pose}');
        LocPosesGT = LocPosesGT(lix, :);
    else
        LocPosesGT = [];
    end

    if plotAll
        figure(1); clf
    end
    dx = x_max/(nStates-1);
    ss = zeros(numel(locImTime), 1);
    
    while kImg < numel(locImTime)
        %Predict using motion model
        kImg = kImg + 1;
        dt = locImTime(kImg) - t;
        t = locImTime(kImg);
        %integrate speed during the time interval
        s = 0;
        while t_DR(kDR) < t
            dt1 = (t_DR(kDR+1)-t_DR(kDR));
            s = s + dt1*v_DR(kDR, 1);
            kDR = kDR + 1;
        end
        ss(kImg) = s;
        %construct kernel for motion model
        halfKernelSize = max(10, ceil((s+s*stdPos)/dx)+2);
        x_kernel = x_state(1:2*halfKernelSize+1)-x_state(1+halfKernelSize); % linspace(-halfKernelSize*dx,halfKernelSize*dx, 2*halfKernelSize+1);
        kern_motion = normpdf(x_kernel, s, (s+1e-3)*stdPos);
        kern_motion = kern_motion ./ sum(kern_motion);
        %apply kernel
        p_state_pred = conv(p_state, kern_motion, 'same');
        
        %motion model for relocalization to some random other place
        p_state_pred = (1-P_rl)*p_state_pred + P_rl/nStates;
        
        %save filter posterior density from last iteration and prediction
        %density from current iteration along with the motion model between
        %them for later use in the backwards smoothing pass.
        matfn = sprintf('%s/temp_CMU_run%02d/filter_densities_k%05d.mat', this_dir, locRun, kImg);
        save(matfn, 'p_state_pred', 'p_state', 'kern_motion');

        %Update using dvlad descriptor vector
        fn = locImName{kImg};
        vlad = file2dvlad(fn);
        iCam = contains(fn, '_c1_')+1;
        d = sum((ref(iCam).vlad - vlad).^2);
        w = exp(-d/R);
        p_state = p_state_pred .* interp1(ref(iCam).dist, w, x_state, ...
            'pchip', 0.5*(w(1) + w(end)));
        p_state(~isfinite(p_state)) = 0;
        p_state = p_state ./ sum(p_state);

        %Take the MAP filter estimate and save it to the output vector
        [p_max, i_max] = max(p_state);
        x_max = x_state(i_max);
        pose_est = interp1(x_unique, pose_unique, x_max, 'linear', 'extrap');
        LocPosesFilterEst(kImg, :) = pose_est;
        FileNames{kImg} = fn;
        %Multiply by extrinsics
        T_C_V = GT.cams(iCam).P*GT.cams(iCam).P_v2fc;
        T_W_V = [quat2rotm(pose_est(4:7)), pose_est(1:3)'; 0 0 0 1];
        T_C_W = T_C_V / T_W_V;
        CamPosesFilterEst(kImg, :) = [rotm2quat(T_C_W(1:3,1:3)), T_C_W(1:3,4)'];
        fprintf(filtertxtfile, '%s %f %f %f %f %f %f %f\n', fn, CamPosesFilterEst(kImg, :));
        fprintf('Filtering image # %d / %d (%s)\n', kImg, numel(locImTime), fn);
        if plotAll
            plot(x_state, p_state, [x_max; x_max], [0; p_max]);
            axis([x_max-10, x_max+10, 0, 1])
            drawnow;
        end
    end
    
    %for kImg == numel(locImTime), the filter and smoothing densities are
    %identical
    kImg = numel(locImTime);
    LocPosesSmoothEst(kImg, :) = LocPosesFilterEst(kImg, :);
    CamPosesSmoothEst(kImg, :) = CamPosesFilterEst(kImg, :);
    fprintf(smoothingtxtfile, '%s %f %f %f %f %f %f %f\n', fn, CamPosesFilterEst(kImg, :));
    matfn = sprintf('%s/temp_CMU_run%02d/smooth_densities_k%05d.mat', this_dir, locRun, kImg);
    p_state_filter = p_state;
    save(matfn, 'p_state', 'p_state_filter');
    
    %Backwards smoothing pass
    kImg = numel(locImTime) - 1;
    while kImg > 0
        matfn = sprintf('%s/temp_CMU_run%02d/filter_densities_k%05d.mat', this_dir, locRun, kImg+1);
        filter_densities = load(matfn, 'p_state_pred', 'p_state', 'kern_motion');
        p_state_filter = filter_densities.p_state;
        
        p_state_future = p_state ./ filter_densities.p_state_pred;
        kern_motion_back = fliplr(filter_densities.kern_motion);
        p_state_future_pred = conv(p_state_future, kern_motion_back, 'same');
        p_state = p_state_filter .* p_state_future_pred;
        %Normalize 
        p_state = p_state ./ sum(p_state);
        
        %Save back the smoothed density
        matfn = sprintf('%s/temp_CMU_run%02d/smooth_densities_k%05d.mat', this_dir, locRun, kImg);
        save(matfn, 'p_state', 'p_state_filter');
        
        %Take the MAP smoothing estimate and save it to the output vector
        fn = locImName{kImg};
        iCam = contains(fn, '_c1_')+1;
        [p_max, i_max] = max(p_state);
        x_max = x_state(i_max);
        pose_est = interp1(x_unique, pose_unique, x_max, 'linear', 'extrap');
        LocPosesSmoothEst(kImg, :) = pose_est;
        FileNames{kImg} = fn;
        %Multiply by extrinsics
        T_C_V = GT.cams(iCam).P*GT.cams(iCam).P_v2fc;
        T_W_V = [quat2rotm(pose_est(4:7)), pose_est(1:3)'; 0 0 0 1];
        T_C_W = T_C_V / T_W_V;
        CamPosesSmoothEst(kImg, :) = [rotm2quat(T_C_W(1:3,1:3)), T_C_W(1:3,4)'];
        fprintf(smoothingtxtfile, '%s %f %f %f %f %f %f %f\n', fn, CamPosesSmoothEst(kImg, :));
        fprintf('Smoothing image # %d / %d (%s)\n', kImg, numel(locImTime), fn);        

        if plotAll
            plot(x_state, p_state, [x_max; x_max], [0; p_max]);
            axis([x_max-10, x_max+10, 0, 1])
            drawnow;
        end
        kImg = kImg - 1;        
    end
    
    time = locImTime + t0;
    save(savefn, 'LocPosesGT', 'LocPosesFilterEst', 'CamPosesFilterEst', 'LocPosesSmoothEst', 'CamPosesSmoothEst', 'FileNames', 'time');
end
fclose(filtertxtfile);
fclose(smoothingtxtfile);
toc
