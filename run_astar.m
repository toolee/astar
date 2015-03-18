function run_astar
map = map_create();

astar_execute(map);

function [start,goal,obstacle,clearpath]=map_constants()
start = 7;
goal = 8;
clearpath = 1;
obstacle = 0;

%--------------------------------------------------------------------------
% map
%--------------------------------------------------------------------------
function map = map_create()
[S,G,O,C] = map_constants;

map = [ C, C, C, O;
        C, O, C, O;
        C, S, C, O;
        C, O, C, G;
        ];

function [r,c] = map_get_goal_pos(map)
[S,G,O,C] = map_constants;
[r,c] = find(map == G);

function [r,c] = map_get_start_pos(map)
[S,G,O,C] = map_constants;
[r,c] = find(map == S);

%--------------------------------------------------------------------------
% util
%--------------------------------------------------------------------------
function index = rc2indx(ROW,COL,r,c)
index = (r-1)*COL+c;

function [r,c] = indx2rc(ROW,COL,i)
r = ceil(i/COL);
c = mod(i,COL);
if( c == 0 )
  c = COL;
end


%--------------------------------------------------------------------------
% astar
%--------------------------------------------------------------------------
function astar_execute(map)

[S,G,O,C] = map_constants;
[ROW,COL] = size(map);
[start_r,start_c] = map_get_start_pos(map);
[goal_r,goal_c] = map_get_goal_pos(map);

% draw a map base
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

nodes = draw_fgh_value(map,nodes);


% initialize close list with start position
% empty open list
ci=1;  % current open slot in close_list
close_list(ci) = rc2indx(ROW,COL,start_r,start_c);  % <---- this is current position
oi=1;  % current open slot in open_list
open_list=[];

% for loop start here
keep_running = true;
while keep_running


% find surrounding nodes (max 4), and put them in open list
[current_r,current_c] = indx2rc(ROW,COL,close_list(ci));
%      r-1
% c-1       c+1
%      r+1
tmp_r = current_r - 1;
if( tmp_r >= 1 && tmp_r <= ROW )
    % update neighboring nodes' g value if it is smaller
    indx = rc2indx(ROW,COL,tmp_r,current_c);
    nodes(indx) = update_neighbor(nodes(indx),nodes(close_list(ci)));
    % add it to open_list if it is not already
    open_list = list_unique_add(open_list,indx);
end
tmp_c = current_c - 1;
if( tmp_c >= 1 && tmp_c <= COL )
    % update neighboring nodes' g value if it is smaller
    indx = rc2indx(ROW,COL,current_r,tmp_c);
    nodes(indx) = update_neighbor(nodes(indx),nodes(close_list(ci)));
    % add it to open_list if it is not already
    open_list = list_unique_add(open_list,indx);
end
tmp_c = current_c + 1;
if( tmp_c >= 1 && tmp_c <= COL )
    % update neighboring nodes' g value if it is smaller
    indx = rc2indx(ROW,COL,current_r,tmp_c);
    nodes(indx) = update_neighbor(nodes(indx),nodes(close_list(ci)));
    % add it to open_list if it is not already
    open_list = list_unique_add(open_list,indx);
end
tmp_r = current_r + 1;
if( tmp_r >=1 && tmp_r <= ROW )
    % update neighboring nodes' g value if it is smaller
    indx = rc2indx(ROW,COL,tmp_r,current_c);
    nodes(indx) = update_neighbor(nodes(indx),nodes(close_list(ci)));
    % add it to open_list if it is not already
    open_list = list_unique_add(open_list,indx);
end
draw_fgh_value(map,nodes);

% find smallest f value
minv = 999999;
for i = 1:size(open_list)
    if( nodes(open_list(i)).f < minv )
        minv = nodes(open_list(i)).f;
        sm_i = i;
    end
end


end % while keep_running


function neighbor_node = update_neighbor(neighbor_node,current_node)
global INIT_G_VALUE
% if it is first time computing g (-1 value) then update it
if ( neighbor_node.g == INIT_G_VALUE )
    neighbor_node.g = 10;
    neighbor_node.parent_r = current_node.r;
    neighbor_node.parent_c = current_node.c;
    neighbor_node.f = neighbor_node.g + neighbor_node.h;
% or updated g is less also update it
elseif ( current_node.g + 10 < neighbor_node.g )
    neighbor_node.g = current_node.g + 10;
    neighbor_node.parent_r = current_node.r;
    neighbor_node.parent_c = current_node.c;
    neighbor_node.f = neighbor_node.g + neighbor_node.h;
end

function list = list_unique_add(list,value)
if( size(find(list == value)) == 0 )
    i = size(list)+1;
    list(i) = value;
end

function nodes = draw_fgh_value(map,nodes)
[S,G,O,C] = map_constants;
[ROW,COL] = size(map);
global INIT_G_VALUE;
global INIT_F_VALUE;
% draw h values
for ni = 1:size(nodes,2)
    % if it is not an obstacle draw debug info
    if( map(nodes(ni).r,nodes(ni).c) ~= O )
        %------------------------
        % draw f = g + h
        s = sprintf('%0.2f',nodes(ni).h);
        if( nodes(ni).h_hldr == 0 )
            nodes(ni).h_hldr = text(nodes(ni).r+0.8,nodes(ni).c+0.9,s);
        else
            set(nodes(ni).h_hldr,'String',s);
        end
        if( nodes(ni).g ~= INIT_G_VALUE )
            s = sprintf('%0.2f',nodes(ni).g);
            if( nodes(ni).g_hldr == 0 )
                nodes(ni).g_hldr = text(nodes(ni).r+0.45,nodes(ni).c+0.9,s);
            else
                set(nodes(ni).g_hldr,'String',s);
            end
        end
        if( nodes(ni).f ~= INIT_F_VALUE )
            s = sprintf('%0.2f',nodes(ni).f);
            if( nodes(ni).f_hldr == 0 )
                nodes(ni).f_hldr = text(nodes(ni).r+0.1,nodes(ni).c+0.9,s);
            else
                set(nodes(ni).f_hldr,'String',s);
            end
        end
        %----------------------------
        % draw arrow
        update_arrow = false;
        if( 0 < nodes(ni).r+1 && nodes(ni).r+1 == nodes(ni).parent_r && nodes(ni).r+1 < ROW )
            s = '<'; update_arrow = true;
        elseif( 0 < nodes(ni).r-1 && nodes(ni).r-1 == nodes(ni).parent_r && nodes(ni).r-1 < ROW )
            s = '>'; update_arrow = true;
        elseif( 0 < nodes(ni).c+1 && nodes(ni).c+1 == nodes(ni).parent_c && nodes(ni).c+1 < COL )
            s = '\^'; update_arrow = true;
        elseif( 0 < nodes(ni).c-1 && nodes(ni).c-1 == nodes(ni).parent_c && nodes(ni).c-1 < COL )
            s = 'v'; update_arrow = true;
        end
        if( update_arrow )
            if( nodes(ni).arrow_hldr == 0 )
                nodes(ni).arrow_hldr = text(nodes(ni).r+0.6,nodes(ni).c+0.45,s);
            else
                set(nodes(ni).arrow_hldr,'String',s);
            end
        end
    end
end
function nodes = astar_compute_h(map)
[S,G,O,C] = map_constants;
[ROW,COL] = size(map);

nodes(ROW*COL) = struct('r',[],'c',[],'h',[],'g',[],'f',[], ...
    'parent_r',[],'parent_c',[],'h_hldr',[],'g_hldr',[],'f_hldr',[],...
    'arrow_hldr',[])

[goal_r,goal_c] = find(map==G);

global INIT_G_VALUE;
INIT_G_VALUE = 99999;
global INIT_F_VALUE;
INIT_F_VALUE = 99999;

n = 1;
for r = 1:ROW
  for c = 1:COL
          nodes(n).r = r;
          nodes(n).c = c;
          nodes(n).h = sqrt( abs(goal_r-r)^2 + abs(goal_c-c)^2 );
          nodes(n).g = INIT_G_VALUE;
          nodes(n).f = INIT_F_VALUE;
          nodes(n).parent_r = 0;
          nodes(n).parent_c = 0;
          nodes(n).h_hldr = 0;
          nodes(n).g_hldr = 0;
          nodes(n).f_hldr = 0;
          nodes(n).arrow_hldr = 0;
          n = n+1;
  end
end
