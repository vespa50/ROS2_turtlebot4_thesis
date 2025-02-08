format longG
s="log-19-Aug-2024-14_45_11.txt";
fprintf("reading from file %s\n",s);
fileID = fopen(s,'r');
tline='1';
data=[];
sample=0;
over=0;
invalid=0;
figure(1);
hold on;
while tline~=-1
    tline=fgetl(fileID);
    k=strfind(tline,";");
    if not(isempty(k))
        news=strsplit(tline,';');
        data=[data;news];
        sample=sample+1;
    end
    fprintf("t:%2.6f x:%2.6f y:%2.6f d_min:%2.6f\n",str2double(news(1)),str2double(news(2)),str2double(news(3)),str2double(news(4)))
    hue=str2double(news(4))/0.02;
    if str2double(news(6))~=0
        if hue<1
            col=[hue,1-hue,0];
        else
            col=[0,0,1];
            over=over+1;
        end
        p=plot(str2double(news(2)),str2double(news(3)), 'w.', 'MarkerSize', 5, 'MarkerFaceColor', col);
        p.Color = col;
        p.Marker = ".";
    else
        invalid=invalid +1;
    end
end
fprintf("sample: %d over:%d percento of over:%2.2f invalid:%d\n",sample,over,(over/sample)*100,invalid)
hold off;

[R,C]=size(data);
max=0;
for i=1:1:R
    if str2double(data(i,4))>max
        max=str2double(data(i,4));
    end
end
fprintf("max d_min %2.4f\n",max)

nbin=100;
vmax=0.1;
bins=zeros(1,nbin);
datapoint=0;
for i=1:1:nbin
    for ii=1:1:R
       if str2double(data(ii,4))<=vmax
           if and(str2double(data(ii,4))>((i-1)*(vmax/nbin)),str2double(data(ii,4))<(i*(vmax/nbin)))
                bins(i)=bins(i)+1;
                datapoint=datapoint+1;
           end
       end
    end
end
fprintf("datapoint %d percent of total data %2.2f %",datapoint,(datapoint/R)*100)
figure(2);
hold on;
x_axis=0.5*(vmax/nbin):(vmax/nbin):vmax;
plot(x_axis,bins, 'b-')
hold off;