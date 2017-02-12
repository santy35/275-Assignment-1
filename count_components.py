import adjacencygraph
import sys
import argparse
from bfs import breadth_first_search
from adjacencygraph import AdjacencyGraph
from adjacencygraph import UndirectedAdjacencyGraph


parser = argparse.ArgumentParser(
    description='Text Frequency Analysis',
    formatter_class=argparse.RawTextHelpFormatter,
    )

parser.add_argument("infile",
    help="file to be sorted, stdin if omitted",
    nargs="?",
    type=argparse.FileType('r'),
    default=sys.stdin
	)

args = parser.parse_args()

def count_components(g):
    ''' Returns the number of connencted components in the graph g

        Args:
            g : the graph containing the connected components

        Returns:
            int: integer stating number of connected components
    '''
    vertices = list(g.vertices())
    paths = []
    count = 0

    while (len(vertices) > 0):
        reached = breadth_first_search(g, vertices[0])
        for vert in range(len(vertices)):
            if vertices[vert] in reached:
                paths.append(vertices[vert])
        vertices = [x for x in vertices if x not in paths]
        count = count + 1
        paths = []
    return count

def read_city_graph(city):
    g = UndirectedAdjacencyGraph()
    temp = []
    for line in args.infile:
        temp = line.split(',')
        if temp[0]=='V':
            g.add_vertex(temp[1]) #if vertex add vertex
        elif temp[0]=='E':
            g.add_edge((temp[1],temp[2])) #if edge add edge
    return g
g = read_city_graph(args.infile)
print(count_components(g))

# number of components in Edmonton graph = 648
