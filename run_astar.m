function run_astar

start.x=1;
start.y=1;
final.x=3;
final.y=3;
make_map(3,3,start,final,0);


function [start,goal,obstacle,clearpath]=load_constants
start = 0;
goal = 1;
obstacle = 2;
clearpath = 3;


function make_map(max_row, max_col, start_pos,final_pos,obstacles)
[START,GOAL,OBS,CLEAR] = load_constants;


world(max_row,max_col) = 0;

% -1 obstacles