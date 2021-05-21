function x = plot_redundancy(testnames, legendnames, testp, titlename)
    
    path1 = "/redundancy_util_";
    path3 = ".txt";
    figure('Name', 'Simple plot', 'NumberTitle', 'off');
    subplot(2,1,1)
    for i= 1:length(testnames) 
        A = readmatrix(testnames(i)+path1+testp+path3);
        plot(A(3,:),A(2,:), 'LineWidth',2);
        hold on
    end
    legend(legendnames,'Location', 'northeastoutside','FontSize',7);
    xlabel('Distance [M]');
    ylabel('y [M]');
    xlim([-inf inf]);
    ylim([-inf inf]);
    title('Translation utilization: ' + titlename)
    hold off
    
    subplot(2,1,2)
    for i= 1:length(testnames) 
        A = readmatrix(testnames(i)+path1+testp+path3);
        x = A;
        plot(A(3,:),A(1,:), 'LineWidth',2);
        hold on
    end
    legend(legendnames,'Location', 'northeastoutside','FontSize',7);
    xlabel('Distance [M]');
    ylabel('\beta [rad]');
    xlim([-inf inf]);
    ylim([-inf inf]);
    title('Rotation utilization: ' + titlename)
    hold off
    
    export_fig("MATLABFIGURES/" + titlename,'-transparent','-append', '-native')
end
    