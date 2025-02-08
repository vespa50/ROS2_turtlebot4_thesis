format longG                      % define data long format
s="log-19-Aug-2024-17_38_51.txt"; %define input file
fprintf("reading from file %s\n",s);%notify input file
fileID = fopen(s,'r');              %create file ID per input
tline='1';                          %init strang for reader
mat=[];                             %init matrix for each frame
frames=[];                          %init 3D matrix for all frames
first=true;                         %init first indicator
while tline~=-1                     %while there are line to read
    tline=fgetl(fileID);            %read a new line
    k=strfind(tline,",");           %find if separate string at ','
    if not(isempty(k))              %if there are elements
        news=strsplit(tline,',');   %separate string at ','
        [~,C] = size(news);         %get number of new elements
        dub=str2double(cellstr(news));%convert the cell to double 
        mat=[mat;dub(1:C-1)];       %add row to frame matrix
    else                            %if there is not new lines
        if first                    %if first frame
            frames=mat;             %copy matrix to frame
            first=false;            %set not first
        else
            frames=cat(3,frames,mat);%stack new frame on matrix
        end
        mat=[];                     %clean frame matrix
    end
end

[R,C,P]=size(frames);               %determin matrix dimension
[X,Y]=meshgrid(1:1:C,1:1:R);        %create a mesh grid to display
i=1;                                %setup frame counter
surf(X,Y,frames(:,:,1));            %create a first surface
while 1                             %forever do
    view([180 0]);                  %set viewing angle
    surf(X,Y,frames(:,:,i));        %create a surface
    view([180 0]);                  %set viewing angle
    pause (0.1);                    %pause for animation
    i=i+1;                          %update frame counter
    if i>P                          %if end of frames
        i=1;                        %reset counter
    end
end