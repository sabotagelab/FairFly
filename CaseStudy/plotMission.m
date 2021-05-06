function rob = plotMission(handles)
videomode=0; % record a video
showtrajs=0; %lines for trajs

if(videomode)
    % video stuff
    v = VideoWriter('RA.avi');
    v.FrameRate = 20;
    %v.CompressionRatio = 2;
    v.Quality = 100;
    %v.CompressionRatio = 1;
    open(v);
end

w_opt = handles.myhandle.w_opt;
optParams = handles.myhandle.optParams;
time_print = handles.timeElapsed_data;

[negative_rob, xx,yy,zz] = Mission_Robustness_exact(w_opt,optParams);
rob = -negative_rob;

set(handles.missionRob_data, 'String', sprintf('%.4f',rob));

mar{1} = 'ko';
mar{2} = 'go';
mar{3} = 'ro';
mar{4} = 'bo';
mar{5} = 'co';
mar{6} = 'mo';
mar{7} = 'yo';
mar{8} = 'rp';
mar{9} = 'gp';
mar{10}= 'mp';
mar{11} = 'bp';
mar{12} = 'yp';

col{1} = 'k';
col{2} = 'g';
col{3} = 'r';
col{4} = 'b';
col{5} = 'c';
col{6} = 'm';
col{7} = 'y';
col{8} = 'r';
col{9} = 'g';
col{10}= 'm';
col{11} = 'b';
col{12} = 'y';

p = [];
% Plot Path
for d = 1:optParams.N_drones
    
    hold on;
    if(showtrajs)
        p(d) = plot3(xx(:,d),yy(:,d),zz(:,d),'-.','linewidth',3, 'Color', col{d}, 'DisplayName', strcat('Drone',num2str(d)));
    end
    %     hold all;
end


for t = 1:size(xx,1)
    tic;
    if(exist('gc','var'))
        delete(gc)
    end
    for d = 1:optParams.N_drones
        
        hold on;
        gc(d) = plot3(xx(t,d),yy(t,d),zz(t,d),mar{d},'MarkerSize',20, 'MarkerFaceColor', col{d});
    end
    set(time_print, 'String', sprintf('%.3f',(t-1)*optParams.sampling_time));
    t_ = toc;
    %     pause(0.02);
    pause(optParams.sampling_time - t_);
    
    if(videomode)
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
end

if(videomode)
    for i = 1:ceil(1/optParams.sampling_time) %pause a second
        %         myavi = addframe(myavi,gca);
        frame = getframe(gcf);
        writeVideo(v,frame);
    end
    close(v);
    disp('fin');
end