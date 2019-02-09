import pdb
from collections import deque
import json

# define constants
ROW = 'r'
COL = 'c'

''' some short functions return id of network'''
rid = lambda index: ROW + str(index)
cid = lambda index: COL+ str(index)


class Hungarian:
    def __init__(self, cost_matrix):
        # pad row with zeros if non-square matrix is providied
        while len(cost_matrix) < len(cost_matrix[0]):
            cost_matrix.append( [ 0 for i,_ in enumerate(cost_matrix[0]) ] )
        while len(cost_matrix) > len(cost_matrix[0]):
            for i,_ in enumerate(cost_matrix):
                cost_matrix[i].append(0)
        self.cost_matrix = cost_matrix

    # Use hungarian algorithm to find cumulative minimum distance
    def compute(self):
        # build manhattan distance table 
        d =  [ r[:] for r in self.cost_matrix ]
    
        #  step1: row reduce
        self.row_reduce(d)
    
        #  step2: col reduce
        self.col_reduce(d)
    
        # step3: find minimum number of lines to cover all zeros
        r_cover, c_cover = self.konig(d)
    
        # step4: if minum number of lines != col: find 
        #        mininum in the d matrix and decrement
        while (len(r_cover)+len(c_cover)) < min(len(d),len(d[0])):
            self.shift_zeros(d, r_cover, c_cover)
            r_cover, c_cover = self.konig(d)
        
        # get minimum combination from vertex cover r_cover, cover
        # by now |r_cover|+|c_cover| == dim(R(d)) == dim(C(d))
        M = self.get_spanning_zeros( d )
    
        entries = self.get_entries(M)
        return self.get_total_cost(entries, self.cost_matrix), entries
            
    
    def get_total_cost(self, entries, D): 
        return sum( [ D[i][j] for (i,j) in entries ] )

    def get_spanning_zeros(self,d):
        ''' 
        The function gets the spanning zeros. Again, we can 
        re-use maximum bipartite matching subroutine to find them
        '''
        # return edges in the form of adj list
        R,C = self.create_bipartite_graph(d)
        # max matching in the form of adj list
        return self.max_bipartite_matching(R,C)
    
    def get_entries(self, M):
        ''' Returns the total cost given the matching and cost D matrix'''
        entries = list()
        for i,_ in enumerate(M):
            for j,_  in enumerate(M[i]):
                if M[i][j] == 1: entries.append((i,j))
        return entries
    
    def row_reduce(self,d):
        '''decrement every row by row's min'''
        for i,row in enumerate(d):
            m = min(row)
            for j,_ in enumerate(d[i]):
                d[i][j] -= m 
    
    def col_reduce(self,d):
        '''decrement every col by col's min'''
        for j,_ in enumerate(d[0]):
            m = d[0][j]
            # find mininum
            for i,_ in enumerate(d):
                if d[i][j] < m:
                    m = d[i][j]
            # decrement
            for i,_ in enumerate(d):
                d[i][j] -= m
    
    def create_bipartite_graph(self,d):
        ''' 
        This function creates a bipartite grapg based on d
        using list over dict implementaion for edges for efficiency 
        @return 
            R: list where index are row vertices and value at the index 
            correspondsin to the adjacency list
            C: same as above except idx is col this time
        '''
        R = [ [] for row in d ]
        C = [ [] for col in d[0] ]
        for i,_ in enumerate(R):
            for j,_ in enumerate(C):
                if d[i][j] == 0: 
                    R[i] += [j]
                    C[j] += [i]
        return R, C
    

    class FlowNetwork:
        def __init__(self):
            self.network = {}

        def __getitem__(self,key):
            if key not in self.network: raise IndexError
            return self.network[key]
        
        def __setitem__(self,key, value):
            self.network[key] = value

        def __len__(self):
            return len(self.network)

        def keys(self):
            return self.network.keys()

        def insert(self, u, v, flow, cap):
            if u not in self.network: 
                self.network[u] = {}
            self.network[u][v] = {'flow':flow,'cap':cap}

        # dfs with cycle checking
        def dfs(self, s, t, visited, path):
            # cycle checking
            if visited[s]: return False
            visited[s] = True
            old_path = path[:]
            path.append(s)
            if s == t: return True
            for key,info in self.network[s].items():
                if info['cap'] <= info['flow']: continue
                if self.dfs(key, t, visited, path): return True
            # backtracking
            path[:] = old_path
            return False
        
        def find_augmenting_path(self):
            visited = { k: False for k in self.network.keys() }
            path = []
            if not self.dfs('s','t',visited,path): return False
            return path
        
        def find_residue(self, path):
            min_res = None
            iterpath = iter(path)
            prev = next(iterpath)
            for n in iterpath:
                res = self.network[prev][n]['cap'] - self.network[prev][n]['flow']
                if not min_res or min_res < res: min_res = res
                prev = n
            return min_res
        
        def update(self, path ):
            res = self.find_residue(path)
            iterpath = iter(path)
            prev = next(iterpath)
            for n in iterpath:
                self.network[prev][n]['flow'] += res
                self.network[n][prev]['flow'] -= res
                prev = n
        
        def fill(self):
            '''ford fulkerson'''
            path = self.find_augmenting_path()
            while path:
                #print("path")
                #print(path)
                self.update(path)
                path = self.find_augmenting_path()
    
    
    def create_flow_network(self,R):
        '''construct a (partial) flow network from bipartite graph'''
        network = self.FlowNetwork()
        for r,_ in enumerate(R):
            # foreward flow with cap 1
            network.insert('s',rid(r),0,1)
            # backward flow with cap 0; this should not be incremented
            network.insert(rid(r),'s',0,0)
            for c in R[r]:
                # forward flow
                network.insert(rid(r),cid(c),0,1)
                # for backward return flow with zero capacity
                network.insert(cid(c),rid(r),0,0)
                # connect col nodes to sink
                network.insert(cid(c),'t',0,1)
                network.insert('t',cid(c),0,0)
            network.insert('t',None,0,0)
        return network
    
    # solve using ford fulkerson max flow
    def max_bipartite_matching(self,R,C):
        network = self.create_flow_network(R)
        network.fill()
        M = self.find_matching(R,C,network)
        return M
    
    # reconstruct matching ( both with row/col as key ) from network
    def find_matching(self,R,C,network):
        M = [ [ 0 for c in C ] for r in R ]
        for r,cols in enumerate(R):
            for c in cols:
                if network[rid(r)][cid(c)]['flow'] == 1:
                    M[r][c] = 1
                    break
        return M
    
    def find_unmatch_row_vertices(self,M):
        Z = set()
        for r,col in enumerate(M):
            unmatch = True
            for val in col:
                if val: unmatch = False
            if unmatch: Z.add(rid(r))
        return Z
    
    
    def add_alternating_neighbors(self, Z, R, C, M):
        '''O(|V|^2) complexity'''
        dq = deque()
        for z in list(Z):
            dq.append( (z, False) )
        # bfs solution
        while dq:
            node, match = dq.popleft()
            for n in self.get_neighbors(node,match,R,C,M,Z):
                dq.append( (n, not match) )
                Z.add(n)
    
    def extract_id(self,node):
        '''helper function to extract type (row,col) and idx'''
        return node[0], int(node[1:])
    
    def get_neighbors(self,node,match,R,C,M,Z):
        '''
        A function to yield all valid neightbors to node
        
        @param node: the input node that we want to search neightbors on
        @param nmatch: specifies whether we want this neighbor to be in
        @param nR: the indices of R
        @param nC: column indices
        '''
        t,idx =  self.extract_id(node)
        # extract edges from list R
        if t == ROW:
            for c in R[idx]:
                if self.valid(M,idx,c,match) and cid(c) not in Z:
                    yield(cid(c))
        else:
            for r in C[idx]:
                if self.valid(M,r,idx,match) and rid(r) not in Z:
                    yield(rid(r))
    
    def valid(self,M,r,c,match):
        '''
        A function to check if neighbor is valid
        @param M: A matching between rol vertices and col vertices
        @param r: row idx
        @param c: col idx
        @param match: specifies whether we want this neighbor to be in our matching
        '''
        return (not M[r][c] and not match) or (M[r][c] and match)
    
    def get_vertex_cover(self,R,Z):
        '''
        @param R: list containing the list of col idxs of zeros at row i
        @param C: list containing the list of col idx of zeros at col i
        @param Z: A set of nodes of either type 'rx' or 'colx' in the bp graph
        
        @return U: set containing the row indices
        @return V: set containing the col indices, where U+V is the vertex cover
        '''
        c_cover, not_r_cover = set(), set()
        for z in Z:
            t,idx =  self.extract_id(z)
            if t == ROW: not_r_cover.add(idx)
            if t == COL: c_cover.add(idx)
        # find the set R \ Z
        r_cover = set( range(len(R)) ).difference( not_r_cover )
        return r_cover, c_cover
            
    
    def shift_zeros(self, d, r_cover, c_cover):
        '''
        a function to derement the dist matrix by its minimum entry
        and increment the same value @ r_cover and c_cover intersections.
        Modify distance matrix in-place
        @param: 
        d: 2D list containing distance
        @return: 
        None
        '''
        m = None
        for i,_ in enumerate(d):
            for j,_ in enumerate(d[i]):
                if d[i][j] != 0 and ( not m or m > d[i][j] ):
                    m = d[i][j]
                

        for i,_ in enumerate(d):
            for j,_ in enumerate(d[i]):
                if i in r_cover and j in c_cover: d[i][j] += m
                elif d[i][j] > 0: d[i][j] -= m
        #print('ddd')
        #print(d)
    
    def konig(self, d):
        '''
        Konig Algorithm implementation
        @param:
        distance matrix d
        '''
        # return edges in the form of adj list
        R,C = self.create_bipartite_graph(d)
        # max matching in the form of adj list
        M = self.max_bipartite_matching(R,C)
        # find unmatch vertices set Z (in U )
        Z = self.find_unmatch_row_vertices(M)
        # expand set Z to include
        self.add_alternating_neighbors(Z, R, C, M)
        # get vertex cover from Z
        R_res, C_res = self.get_vertex_cover(R,Z)
        return R_res, C_res


d = [ [ 5, 0, 6, 3, 6, 1],
      [ 7, 1, 0, 0, 3, 1],
      [ 3, 0, 0, 0, 1, 1], 
      [ 3, 0, 0, 2, 1, 1],
      [ 3, 0, 0, 2, 1, 0] ]

D = [ [ 5, 6, 6, 3, 1],
      [ 7, 1, 8, 1, 3],
      [ 3, 6, 7, 6, 1], 
      [ 3, 1, 6, 2, 1],
      [ 1, 6, 6, 2, 1] ]
'''
  = [ [ 0, 1, 0, 0, 0, 1],
      [ 0, 0, 1, 1, 0, 1],
      [ 0, 1, 1, 0, 0, 0], 
      [ 0, 1, 1, 0, 1, 1] ]

R[0] = [1, 5]
R[1] = [2, 3, 5]
'''
# Manhattan
def dist(a,b):
    return sum( [ abs(a[i]-b[i]) for i in [0,1] ] )

def build_dist_table(items1 , items2):
    dtable = [ [ 0 for _ in enumerate(items2) ] for _ in enumerate(items1) ]
    if not items1 or not items2: return None
    for i,b in enumerate(items1):
        for j,s in enumerate(items2):
            dtable[i][j] = dist(b,s)
    return dtable

