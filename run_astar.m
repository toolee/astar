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
function astar_execute(map);
nodes = astar_compute_h(map);

[S,G,O,C] = map_constants;
[ROW,COL] = size(map);
[start_r,start_c] = map_get_start_pos(map);
[goal_r,goal_c] = map_get_goal_pos(map);

ci=1;
close_list(ci) = rc2indx(ROW,COL,start_r,start_c);

for r = 1:ROW
  for c = 1:COL
  end
end

function nodes = astar_compute_h(map)
[S,G,O,C] = map_constants;
[ROW,COL] = size(map);

nodes(ROW*COL) = struct('r',[],'c',[],'h',[],'g',[],'f',[],'parent',[])

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
