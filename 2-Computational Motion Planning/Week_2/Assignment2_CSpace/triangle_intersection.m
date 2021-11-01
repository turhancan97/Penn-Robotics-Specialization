function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************
    function flag = ss(p_1,p_2,a,b)
        p_1(3) = 0; 
        p_2(3) = 0; 
        a(3) = 0; 
        b(3) = 0;
        cross_1 = cross(b-a, p_1-a);
        cross_2 = cross(b-a, p_2-a);
        if(dot(cross_1,cross_2) >= 0)
            flag = true;
        else
            flag = false;
        end
    end
    
P1(4:5,:) = P1(1:2,:);
P2(4:5,:) = P2(1:2,:);

flag = true;

for i=1:3
    if( ~ss(P1(i,:), P2(1,:), P1(i+1,:), P1(i+2,:) ) && ss( P2(1,:), P2(2,:), P1(i+1,:), P1(i+2,:) ) && ss( P2(2,:), P2(3,:), P1(i+1,:), P1(i+2,:) ) )
        flag = false; return;
    end
end

for i=1:3
    if( ~ss(P2(i,:), P1(1,:), P2(i+1,:), P2(i+2,:) ) && ss( P1(1,:), P1(2,:), P2(i+1,:), P2(i+2,:) ) && ss( P1(2,:), P1(3,:), P2(i+1,:), P2(i+2,:) ) )
        flag = false; return;
    end
end
% *******************************************************************
end