function x = plot_redundancy(testnames, legendnames, testp, titlename)
    
    path1 = "/redundancy_util_";
    path3 = ".txt";
    
    figure
    subplot(2,1,1)
    for i= 1:length(testnames) 
        A = readmatrix(testnames(i)+path1+testp+path3);
        plot(A(3,:),A(2,:));
        hold on
    end
    legend(legendnames);
    xlabel('Distance [M]');
    ylabel('Redundancy Utilization y [M]');
    title('Translation utilization: ' + titlename)
    hold off
    
    subplot(2,1,2)
    for i= 1:length(testnames) 
        A = readmatrix(testnames(i)+path1+testp+path3);
        x = A;
        plot(A(3,:),A(1,:));
        hold on
    end
    legend(legendnames);
    xlabel('Distance [M]');
    ylabel('Redundancy Utilization \beta [rad]');
    title('Rotation utilization: ' + titlename)
    hold off
end
    