function run_astar

start.r=1;
start.c=1;
final.r=3;
final.c=3;
obs(10).pos.r = 0;
map = make_map(3,3,start,final,0)

compute_h(map);


function [start,goal,obstacle,clearpath]=load_constants
start = 7;
goal = 8;
clearpath = 1;
obstacle = 0;


function map = make_map(max_row, max_col, start_pos, final_pos, obstacles)
[S,G,O,C] = load_constants;

%map = ones(max_row,max_col)*CLEAR;
%map(start_pos.r, start_pos.c) = START;

map = [ S, C, C, O;
        C, O, C, O;
        C, O, C, O;
        C, O, C, G;
        ];

function compute_h(map)
[S,G,O,C] = load_constants;

[ROW,COL] = size(map);

nodes(ROW*COL) = struct('r',[],'c',[],'h',[],'g',[],'f',[],'parent',[])

[goal_r,goal_c] = find(map==G)

n = 1;
for r = 1:ROW
  for c = 1:COL
    nodes(n).r = r;
    nodes(n).c = c;
    nodes(n).h = sqrt( abs(goal_r-r)^2 + abs(goal_c-c)^2 );
    nodes(n)
    n = n+1;
  end
end
