
format longG

ori_x=3.0448970794677734;           %distance between origins
ori_y=1.162385940551757805;

%16_39 v2 delta_t=46281319.913-3.229 delta_x=-0.155 delta_y=0.74 delta_a=-17.26
%16_18 v2 delta_t=46281322.461-3.32 delta_x=-0.310 delta_y=0.405 delta_a=-19.8;


action="dist";                   %action select:opt_t opt_a opt_xy dist
f_turt="./log_turtlebot/log_16_39.txt";  %file log turtlebot
f_uwb="log-19-Aug-2024-16_39_19.txt";    %file log uwb 

delta_t=46281319.913-3.229;                       %correction for time angle and xy shift
delta_x=ori_x-0.155;
delta_y=ori_y+0.74;        
delta_a=17.26;

d_opt_span=0.5;                   %dimension of optimization window
a_opt_span=20;
a_opt_center=-17.26;
%%%%%%%%%%%%%%%%%%%%%%%
%%read file turtlebot%%
%%%%%%%%%%%%%%%%%%%%%%%
fprintf("reading from file %s\n",f_turt);   %notify start read
fileID = fopen(f_turt,'r');                 %open the file
tline='1';                                  %init string for reader
index=0;                                    %init index for reader
figure(1);                                  %declare fig.1
turt_data=[];                               %init array samples
hold on;                                    %hold on plotted points
while tline~=-1                             %while there are lines to read
    tline=fgetl(fileID);                    %read one line from the file
                                            %timestamp reading    
    k=strfind(tline,"sec:");                %find position of "sec:"
    if k==5                                 %if in position 5
        news=strsplit(tline,':');           %separate string between ':'
        index=1;                            %set reader index to 1
    elseif k==9                             %if in position 9 
        news=strsplit(tline,':');           %separate the string between ':'
        index=2;                            %set reader index to 1
    end
                                            %pose reading
    k=strfind(tline,"position:");           %find position of "position:"
    if k==5                                 %if in position 5
        index=3;                            %set reader index to 3
    elseif index==3                         %if reader index is 3, after a cycle
        news=strsplit(tline,':');           %separate the string between ':'
        index=4;                            %set reader index to 4
    elseif index==5                         %if reader index is 5
        news=strsplit(tline,':');           %separate the string between ':'
    end
    if index==1                             %index 1: contain second
        t=str2double(news(2));              %save in t the value of sec
        index=0;                            %return index to 0
    elseif index==2                         %index 2: contain nanosecond
        t=t+str2double(news(2))*10^-9;      %add to t nanosecond value
        index=0;                            %return index to 0
    elseif index==4                         %index 4: contain x coordinate
        x=str2double(news(2));              %save in x the x coordinate
        index=5;                            %set reader index to 5
    elseif index==5                         %index 5: contain y coordinate
        y=str2double(news(2));              %save in y the y coordinate
                                            %apply coordinate transform
        
        t=t+delta_t;                        %time translation
        turt_data=[turt_data;t,x,y];        %add point to the data matrix
        %fprintf("t:%2.6f x:%2.6f y:%2.6f\n",t,x,y)
        
        index=0;                            % return index to 0
    end
end
box on;
plot_matrix_x=turt_data(:,2);           %create vector for x coordinate
plot_matrix_y=turt_data(:,3);           %create vector for y coordinate
plot(plot_matrix_x, plot_matrix_y, 'r-')%plot trajectory

%%%%%%%%%%%%%%%%%%%%%%%
%%read file UWB radio%%
%%%%%%%%%%%%%%%%%%%%%%%
fprintf("reading from file %s\n",f_uwb);    %notify start to read 
fileID = fopen(f_uwb,'r');                  %open the log file
tline='1';                                  %inialize reader string
UWB_data=[];                                %initialize data matrix
sample=0;                                   %initialize counter for sample                                     
invalid=0;                                  %initialize counter for invalid reading
while tline~=-1                             %while ther are lines to read
    tline=fgetl(fileID);                    %read a line from the file
    k=strfind(tline,";");                   %find the positions of ';'
    if not(isempty(k))                      %if there are ; simbol in line 
        news=strsplit(tline,';');           %separate line at ';'
        if str2double(news(6))~=0               %if the result form anchor 1 differs from 0
            dub=str2double(cellstr(news));      %convert the array of cell to double
            [dub(2),y]=rotate(dub(2),dub(3),delta_a,0,0);      %rotation
            dub(2)=-dub(2)+delta_x;                       %x translation
            dub(3)=-dub(3)+delta_y;                       %y translation
            UWB_data=[UWB_data;dub(1),dub(2),dub(3)]; %add line to data matrix
            sample=sample+1;                    %count a sample 
        else
            invalid=invalid +1;                 %else count invalid reading
        end
    end
end
fprintf("sample: %d invalid:%d percento of invalid:%2.2f \n",sample, invalid,(invalid/sample)*100)%print statistic

plot_matrix_x=UWB_data(:,2);
plot_matrix_y=UWB_data(:,3);
plot(plot_matrix_x,plot_matrix_y, 'g.', 'MarkerSize', 5)%plot points on trajectory graph
legend({'odom position','UWB position'},'Location','southeast');
xlabel('$x$ [m]','Interpreter','latex');
ylabel('$y$ [m]','Interpreter','latex');
%ax = gca;
%exportgraphics(ax,'q_raw_path.png')
hold off;                                   %terminate graph


[R2,C2]=size(UWB_data);                     %calculate matrix dimension
[R1,C1]=size(turt_data);                    %calculate matrix dimension

if action=="opt_t"                          %optimize time frame
%%%%%%%%%%%%%%%%%%%%%%%%%
%%find time frame shift%%
%%%%%%%%%%%%%%%%%%%%%%%%%
    match=0;                                %initialize variable to match sample
    if delta_t==0                           %on first run, without time correction
        t_shift=abs(UWB_data(1,1)-turt_data(1,1)-(UWB_data(R2,1)-turt_data(R1,1))); %find max t shift
        delta_t=abs(UWB_data(1,1)-turt_data(1,1)); %find t shift between log file
        t_comp=delta_t;                            %define t_comp for first run 
        fprintf("t_shift %2.4f delta_t %2.4f\n",t_shift,delta_t)%show results for constant part
    else
        t_shift=1;                                  %define arbitrar t_shift
        t_comp=0;                                  %define t_comp as 0
    end 
    d_min=5;                                       %define minimum distance value
    t_min=0;                                       %define time correction at min d
    for t_delta = -t_shift*2:t_shift/100:t_shift*2 %between -2 and +2 times t_shift in 400 step
        d_media=0;                                 %init d_media at 0
        for i=1:1:R2                               %for every UWB data point
            t_dist_min=100;                        %define minimum time distance 
            for ii=1:1:R1                          %for every turtlebot data point
                t_dist=abs(UWB_data(i,1)-(turt_data(ii,1)+t_delta+t_comp));%calculate time distance
                if t_dist<t_dist_min               %if temporal distance is minimum
                    t_dist_min=t_dist;             %update minimum
                    match=ii;                      %update match
                end
            end
            d=sqrt((UWB_data(i,2)-turt_data(match,2))^2+(UWB_data(i,3)-turt_data(match,3))^2);%calculate distance between current turt data point and match uwb
            d_media=d_media+d;                     %add d to d_media
        end
        d_media=d_media/R2;                        %calculate d_media
        if d_media<d_min                           %if d_media is minimum
            t_min=t_delta;                         %saved time correction
            d_min=d_media;                         %update min value
        end
    end
    fprintf("delta t correzione %2.3f d_min: %2.6f\n",t_min,d_min) %report result
elseif action=="opt_a"
%%%%%%%%%%%%%%%%%%%%
%%find angle shift%%
%%%%%%%%%%%%%%%%%%%%
    match=0;                                %initialize variable to match sample
    d_min=100;                                       %define minimum distance value
    a_min=0;                                       %define angle correction at min d
    figure(2);                                     %initialize figure 2
    hold on;                                       % hold on plotted points
    for a_delta = a_opt_center-a_opt_span*2:a_opt_span/100:a_opt_center+a_opt_span*2 %between -2 and +2 times a_shift in 400 step
        transform=[];
        for i=1:1:R2                               %for every UWB data point
            t_dist_min=100;                        %define minimum time distance 
            for ii=1:1:R1                          %for every turtlebot data point
                t_dist=abs(UWB_data(i,1)-(turt_data(ii,1)));%calculate time distance
                if t_dist<t_dist_min               %if temporal distance is minimum
                    t_dist_min=t_dist;             %update minimum
                    match=ii;                      %update match
                end
            end                                    
            [a_x,a_y]=rotate(turt_data(match,2),turt_data(match,3),a_delta,delta_x,delta_y);%rotate data point
            transform=[transform;sqrt(abs(a_x-UWB_data(i,2))^2+abs(a_y-UWB_data(i,3))^2)];%add distance to matrix            
        end                                                
        std_xy=std(transform);                     %calculate standard dev. of distance
        plot(a_delta,std_xy,'r.') %plot standard deviation on graph
        if std_xy<d_min           %if standard dev. is minimum
            a_min=a_delta;                         %saved angle correction
            d_min=std_xy;         %update min value
        end
    end    
    hold off;
    fprintf("delta a correzione %2.3f d_min: %2.6f\n",a_min,d_min) %report result
elseif action=="opt_xy"                             %optimize xy shift
%%%%%%%%%%%%%%%%%%%%%%%%
%%find x shift in data%%
%%%%%%%%%%%%%%%%%%%%%%%%
    figure(3);                                                      %open graph 3
    hold on;                                                        %hold on the graph
    minus_x=100;                                                      %init min distance record
    cor_x=0;                                                          %position of min distance
    for xshift=-d_opt_span:d_opt_span/100:d_opt_span                %in 200 point in 2 opt_span
            d_media=0;                                                  %initalize d_media
            for i=1:1:R2                                                %for every data point of UWB
                    t_dist_min=100;                                     %init min t distance record
                    for ii=1:1:R1                                       %for every data point of turt.
                        t_dist=abs(UWB_data(i,1)-(turt_data(ii,1)));    %calculate distance between points
                        if t_dist<t_dist_min                            %if distanche is min
                            t_dist_min=t_dist;                          %update minimum
                            match=ii;                                   %save match index
                        end
                    end                                         
                    d=sqrt((UWB_data(i,2)-turt_data(match,2)-xshift)^2+(UWB_data(i,3)-turt_data(match,3))^2);%calculate distance for best match
                    d_media=d_media+d;                          %add distance to media
            end
            d_media=d_media/R2;                                 %calulate media value
            plot(xshift,d_media,'r^', 'MarkerSize', 0.1, 'MarkerFaceColor', 'r')%plot a x_shift point in graph         
            if d_media<minus_x                                    %if new distance is less than min                                    
                minus_x=d_media;                                  %update minimum
                cor_x=xshift;                                     %save shift value
            end
    end
    
    hold off;                                               %stop hold on graph
%%%%%%%%%%%%%%%%%%%%%%%%
%%find y shift in data%%
%%%%%%%%%%%%%%%%%%%%%%%%    
    figure(4);                                                      %open graph 4
    hold on; 
    minus_y=100;
    cor_y=0;   
    for yshift=-d_opt_span:d_opt_span/100:d_opt_span        %in 200 point in 2 opt_span
            d_media=0;                                                  %initalize d_media
            for i=1:1:R2                                                %for every data point of UWB
                    t_dist_min=100;                                     %init min t distance record
                    for ii=1:1:R1                                       %for every data point of turt.
                        t_dist=abs(UWB_data(i,1)-(turt_data(ii,1)));    %calculate distance between points
                        if t_dist<t_dist_min                            %if distanche is min
                            t_dist_min=t_dist;                          %update minimum
                            match=ii;                                   %save match index
                        end
                    end                                        
                    d=sqrt((UWB_data(i,2)-turt_data(match,2))^2+(UWB_data(i,3)-turt_data(match,3)-yshift)^2);%calculate distance for best match
                    d_media=d_media+d;                          %add distance to media
            end
            d_media=d_media/R2;                                 %calulate media value
            plot(yshift,d_media,'b^', 'MarkerSize', 0.1, 'MarkerFaceColor', 'r')%plot a y_shift point in graph
             if d_media<minus_y                                    %if d_media is less than minimum
                minus_y=d_media;                                  %update minimum value
                cor_y=yshift;                                     %save shift value
            end
     end
    hold off;
    fprintf("minimo x %2.3f correzione x %2.3f\n minimo y %2.3f correzione y %2.3f",minus_x,cor_x,minus_y,cor_y)  %notify result

elseif action=="dist"                               %show statistics of measurments
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%plot distance versus time%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    difference=[];                                  %initialize vector for error distance
    figure(5);                                      %initialize figure 5
    hold on;                                        %hold on plotted points
    d_media=0;                                      %initialize media value
    d_max=0;                                        %initialize max error value
    for i=1:1:R2                                    %for every point of the UWB data
            t_dist_min=100;                         %initialize min time distance variable
            for ii=1:1:R1                           %for every data point of the turtlebot
                t_dist=abs(UWB_data(i,1)-(turt_data(ii,1)));% measure time distance
                if t_dist<t_dist_min                %if time distance is less than minimum
                    t_dist_min=t_dist;              %update the minimum
                    match=ii;                       %record index of the data
                end
            end                                    
            d=sqrt((UWB_data(i,2)-turt_data(match,2))^2+(UWB_data(i,3)-turt_data(match,3))^2);%calculate distance from best match
            dif=[(UWB_data(i,2)-turt_data(match,2)),(UWB_data(i,3)-turt_data(match,3))];
            difference=cat(1,difference,dif);       %save the position error in a matrix
            d_media=d_media+d;                      %add distance do d_media
            plot(UWB_data(i,1)-UWB_data(1,1),d,'b^', 'MarkerSize', 0.1, 'MarkerFaceColor', 'r')%plot a point for every distance value
            if d>d_max                              %if measured distance is greater than max
                d_max=d;                            %update max value
            end
    end
    d_media=d_media/R2;                             %calculate mean value
    fprintf("d_media: %2.4f d_max %2.4f\n",d_media,d_max)%print mean and max value
    box on;                                         %add square around graph
    xlabel('$t$ [s]','Interpreter','latex');        %label x axis
    ylabel('$d$ [m]','Interpreter','latex');        %label y axis
    hold off;                                       %stop hold on graph


    figure(6);                                      %create graph 6
    box on;                                         %add a box around the graph
    hold on;                                        %hold on the point of the graph
    xlabel('$x$ [m]','Interpreter','latex');        %label x axis
    ylabel('$y$ [m]','Interpreter','latex');        %label y axis
    for i=1:1:R2                                    %for every element of difference
        plot(difference(i,1),difference(i,2),'.r')  %plot the differnce component
    end
    
    hold off;                                       %stop hold on grpah points
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%calc. and plot distribution function%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    nbin=100;                                      %define a sutible bin number 
    bins=zeros(1,nbin);                            %initialize bin vector
    for i=1:1:R2                                   %for every UWB data point
        t_dist_min=100;                            %initialize minimum time distance
        for ii=1:1:R1                              %for every turtlebot data point
            t_dist=abs(UWB_data(i,1)-(turt_data(ii,1)));%measure time distance
            if t_dist<t_dist_min                    %if distance is less then minimum
                t_dist_min=t_dist;                  %update minimum
                match=ii;                           %update match index
            end
        end                                        
        d=sqrt((UWB_data(i,2)-turt_data(match,2))^2+(UWB_data(i,3)-turt_data(match,3))^2);%calculate distance from best match
        for b=1:1:nbin                              %for every bin
           if and(d>((b-1)*(d_max/nbin)),d<(b*(d_max/nbin)))%find if the distance falls in a bin
                bins(b)=bins(b)+1;                  %add 1 to the count in the bin
           end
        end
    end

    figure(7);                                     %initialize figure 7
    hold on;                                       %hold on plotted data
    plot_matrix_x= d_max/nbin:d_max/nbin:d_max;     %define x axis values
    plot_matrix_y=bins/R2;                          %define y axis value based on bin
    plot(plot_matrix_x,plot_matrix_y,'r-')          %plot distribution
    %legend('rectangular trajectory','square trajectory');%add legend when comparing experiments
    box on;                                          %add a box around the graph
    xlabel('$d$ [m]','Interpreter','latex');         %label x axis
    ylabel('$probability$','Interpreter','latex');   %label y axis
    
    hold off;                                       %stop hold on graph

    %%%%%%%%%%%%%%%%%%%%%%
    %%calc. and plot cdf%%
    %%%%%%%%%%%%%%%%%%%%%%
    
    cdf=[];                                         %initialize cfd vector
    plot_matrix_x=[];                               %clean plot_matrix_x vector
    for i=1:1:nbin                                  %for every bin
        plot_matrix_x=[plot_matrix_x,(i-0.5)*(d_max/nbin)];%add a value to the x axis
        cdf_bin=0;                                  %init an accumulator variable
        for ii=1:1:i                                %for every index prior to the current
            cdf_bin=cdf_bin+bins(ii);               %add the content of the bin to accumulator
        end
        cdf=[cdf,cdf_bin];                          %add accumulator as a new vector element
    end 
    cdf=cdf/cdf(nbin);                              %normalize the values to the maximum
    cdf=[cdf,1.0];                                  %add one last value, to clean the graph
    plot_matrix_x=[plot_matrix_x,(d_max*1.1)];      %add the corresponding value on x axis
    
    figure(8);                                      %initialize figure 8
    box on;
    hold on;                                        %hold on plotted points
    plot(plot_matrix_x,cdf,'r-','LineWidth',1);     %plot the cdf on graph
    legend({'square trajectory','rectangular trajectory'},'Location','southeast');% add legend when comparing experiments
    xlabel('$d$[m]','Interpreter','latex');         %label x axis
    ylabel('$probability$','Interpreter','latex');  %label y axis
     
    hold off;                                       %stop hold on graph
end

function [x,y]=rotate(in_x,in_y,angle,delta_x,delta_y)%function for rotating points
    au_x=in_x-delta_x;                                %translate points to original origin
    au_y=in_y-delta_y;                              
    out_x=au_x*cosd(angle)-au_y*sind(angle);          %apply rotation
    out_y=au_x*sind(angle)+au_y*cosd(angle);
    x=out_x+delta_x;                                  %translate back to the reference system 
    y=out_y+delta_y;
end
