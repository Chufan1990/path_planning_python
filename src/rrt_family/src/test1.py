from path_planning_msgs.msg import Point2D, Vertex, Edge, Edges

if __name__ == "__main__":
    points = set()
    for i in range(100):
        points.add(Edge(Point2D(i,i), Point2D(i+1,i+1)))

    a = Point2D(15,15)
    b = Point2D(16,16)
    c = Edge(a,b)

    for point in points:
        if c == point:
            print("True") 