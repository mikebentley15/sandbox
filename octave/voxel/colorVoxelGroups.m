function [out,centers,masses] = colorVoxelGroups(data,conn)
    %assigns a number to each voxel group in data.
    %COM are the centers of Mass for each voxel group
    %masses are the amount of voxels that belong to each group.
    if nargin == 1
        conn=6;    
    end
    %creates an out array the same size of data that has all distinct
    %groups of entries >0 labeled with different integers
    [sX,sY,sZ]=size(data);
    data2=zeros(sX+2,sY+2,sZ+2);
    data2(2:end-1,2:end-1,2:end-1)=data;
    out=zeros(sX,sY,sZ);
    
    nb=[-1  1  0  0  0  0 -1 -1 -1 -1  0  0  0  0  1  1  1  1  -1 -1 -1 -1  1  1  1  1;...
         0  0 -1  1  0  0 -1 +1  0  0 -1 -1  1  1 -1  1  0  0  -1 -1 +1 +1 -1 -1  1  1;...
         0  0  0  0 -1  1  0  0 -1 +1 -1  1 -1  1  0  0 -1  1  -1 +1 -1 +1 -1  1 -1  1];
    
    group=0;
    centers=[];
    masses=[];
    for x=1:sX
        for y=1:sY
            for z=1:sZ
                if data2(x+1,y+1,z+1)>0 && out(x,y,z) ==0 %found a new block
                   COM=[x,y,z];
                   mass=1;
                   group=group+1;
                   out(x,y,z)=group;
                   [COM,mass] = findN(group,x,y,z,COM,mass);
                   COM=COM./mass;
                   centers=[centers;COM];
                   masses=[masses;mass];
                end
            end
        end
    end
    
    
    function [COM,mass] = findN(g,x,y,z,COM,mass)
        for n=1:conn
            if data2(x+1+nb(1,n),y+1+nb(2,n),z+1+nb(3,n))>0 && out(x+nb(1,n),y+nb(2,n),z+nb(3,n))==0
                out(x+nb(1,n),y+nb(2,n),z+nb(3,n))=g;
                
                [COM,mass] = findN(g,x+nb(1,n),y+nb(2,n),z+nb(3,n),COM+[x+nb(1,n),y+nb(2,n),z+nb(3,n)],mass+1);
            end
        end  
    end
end
