<a name="Sokoban AI solver"></a>

## Sokoban Bot ##

Two variations of searches are implemented to search for goal state: Anytime-best-first and Anytime weighted A\* search.
Here are some optimizations on the heuristics that I have done to make the AI smarter and some details on the implementation.

### Hugarian Algoirthm ###

Finding the manhattan distance of each box to the storage with minimum distance is not a good heuristic. This 
is because each storage can contain only 1 box, so need to use hungarian alogirthm to find the optimal state. 
This youtube video offers an simple explanation of hungarian algorithm: https://www.youtube.com/watch?v=cQ5MsiGaDY8

The first 2 steps of preforming row and column reduction are relatively straight-forward to implement, for 
we just need to decrement the matrix the minimum entry in each row/col.

In step 3 and 4, we need to find the minimum number of lines to cover all the zero entries, we can use konig
algorithm, which is detailed here: https://en.wikipedia.org/wiki/Kőnig%27s_theorem_(graph_theory).

We can simply put konig algorithm as follows: If we represent row indices as a set of vertices connected to another
set of vertices representing column matrices, and add an edge for each zero-entry in the bipartite graph,
then we can see that the minimum vertex cover represents the lines we need to cover all zero!

In konig alogrithm, one need to find the maximum matching of the bpg first, then add alternating neighbors to the set.
My implmentation uses a flow network of maximum matching, and bfs to find alternating neighbors.


### Pruning in valid states ###

![corner](https://github.com/winstonww/wwsokobanbot/blob/master/corner.png | width=250)
Some states that will never lead agent to the goal thus should be pruned. 
For instance, when a box is pushed into some corner that is not a storage area, it is impossible to reach a solution 
in this case. Set these states to have max value. When a box is pushed into a "box corner" formed by one or 
more boxes, it is still possible for the box to get out of the "box corner", provided that anybox box that forms to 
"box corner" is not blocked itself. If the box that forms the "box corner" needs the current box to be free in order be
free, this forms a deadlock and is a dead state. Also, even if the box that forms the "box corner" is not blocked,
if it is in a position permanently not reachable by any robots, this is also a dead state.

To find boxes that are permanently cornered and mark the possible boxes that if not reachable by robots will cause 
other boxes to be permanently cornered, we can use DFS. We use BFS to find boxes that are not reachable by robots.

### Caching ### 
It's useful to cache the calculation of the heuristic, especially when none of the boxes moved in a round.
