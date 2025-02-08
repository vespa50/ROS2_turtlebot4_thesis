format longG
i=0;
s="./log_turtlebot/log_17_38.txt";
fprintf("reading from file %s\n",s);
fileID = fopen(s,'r');
tline='1';
index=0;
figure(1);
hold on;
while tline~=-1
    tline=fgetl(fileID);
    k=strfind(tline,"sec:");
    if k==5
        news=strsplit(tline,':');
        index=1;
    elseif k==9
        news=strsplit(tline,':');
        index=2;
    end
    k=strfind(tline,"position:");
    if k==5
        index=3;
    elseif index==3
        news=strsplit(tline,':');
        index=4;
    elseif index==5
        news=strsplit(tline,':');
    end
    if index==1
        t=str2double(news(2));
        index=0;
    elseif index==2
        t=t+str2double(news(2))*10^-9;
        %disp(t);
        index=0;
    elseif index==4
        x=str2double(news(2));
        index=5;
    elseif index==5
        y=str2double(news(2));
        fprintf("t:%2.6f x:%2.6f y:%2.6f\n",t,x,y)
        plot(x, y, 'r^', 'MarkerSize', 0.1, 'MarkerFaceColor', 'r')
        index=0;
    end
end
hold off;


