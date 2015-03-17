function run_astar
map = map_create();

astar_execute(map);

function [start,goal,obstacle,clearpath]=map_constants()
start = 7;
goal = 8;
clearpath = 1;
obstacle = 0;

%------------------------------------------------
% map
%------------------------------------------------
function map = map_create()
[S,G,O,C] = map_constants;

map = [ S, C, C, O;
        C, O, C, O;
        C, O, C, O;
        C, O, C, G;
        ];

function [r,c] = map_get_goal_pos(map)
[S,G,O,C] = map_constants;
[r,c] = find(map == G);

function [r,c] = map_get_start_pos(map)
[S,G,O,C] = map_constants;
[r,c] = find(map == S);

%------------------------------------------------
% util
%------------------------------------------------
function index = rc2indx(ROW,COL,r,c)
index = (r-1)*COL+c;

function [r,c] = indx2rc(ROW,COL,i)
r = ceil(i/COL);
c = mod(i,COL);
if( c == 0 )
  c = COL;
end


%------------------------------------------------
% astar
%------------------------------------------------
function astar_execute(map)

[S,G,O,C] = map_constants;
[ROW,COL] = size(map);
[start_r,start_c] = map_get_start_pos(map);
[goal_r,goal_c] = map_get_goal_pos(map);

axis([1 ROW+1 1 COL+1])
grid on;
hold on;
set(gca,'XTick',[1:1:ROW])
set(gca,'YTick',[1:1:COL])

% plot start, goal, obstacles
plot(start_r+0.5,start_c+0.5,'ro');
plot(goal_r+0.5,goal_c+0.5,'go');
for ri = 1:ROW
    for ci = 1:COL
        if(map(ri,ci)==O) % if it is a obstacle draw it
            plot(ri+0.5,ci+0.5,'kx');
        end
    end
end

% compute h values for all nodes, create nodes for the first time
nodes = astar_compute_h(map);

% draw h values
for ni = 1:size(nodes,2)
    s = sprintf('%0.2f',nodes(ni).h);
    text(nodes(ni).r+0.8,nodes(ni).c+0.9,s);
end

% initialize close list with start position
% empty open list
ci=1;
close_list(ci) = rc2indx(ROW,COL,start_r,start_c);  % <---- this is current position
open_list=[];

% find surrounding nodes (max 4), and put them in open list
[current_r,current_c] = indx2rc(ROW,COL,close_list(ci));



% compute g and f values

% find smallest f value









function nodes = astar_compute_h(map)
[S,G,O,C] = map_constants;
[ROW,COL] = size(map);

nodes(ROW*COL) = struct('r',[],'c',[],'h',[],'g',[],'f',[],'parent_r',[],'parent_c',[])

[goal_r,goal_c] = find(map==G);

n = 1;
for r = 1:ROW
  for c = 1:COL
    nodes(n).r = r;
    nodes(n).c = c;
    nodes(n).h = sqrt( abs(goal_r-r)^2 + abs(goal_c-c)^2 );
    n = n+1;
  end
end
