#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/Vertices.h>
#include <string>
#include <memory>
#include <fstream>
#include <vector>
#include <ctime>
#include <pcl/kdtree/kdtree_flann.h>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <utility>
#include <set>
#include <sstream>
#define Vtx 1853
#include "E:\\Downloads\\happly-master\\happly-master\\happly.h"

int user_data;
using namespace std;
class vertex_data {
    
public:
    //Constructor Class Ref
    vertex_data();

    //Overloader Class Ref
    vertex_data(double, double, double, double, int, int);

    //Destructor Class Ref
    ~vertex_data();

    //Accesory Class Ref
    double getX() const;
    double getY() const;
    double getZ() const;
    double getDist() const;
    int getNeighbor1() const;
    int getNeighbor2() const;

private:
    double x;
    double y;
    double z;
    double dist;
    int neighbor1;
    int neighbor2;
};

class Graph
{
    int V;    // No. of vertices

    // In a weighted graph, we need to store vertex
    // and weight pair for every edge
    list< pair<int, double> >* adj;

public:
    Graph(int V);  // Constructor

    // function to add an edge to graph
    void addEdge(int u, int v, double w);

    // prints shortest path from s
    void shortestPath(int s);
};

// Allocates memory for adjacency list
Graph::Graph(int V)
{
    this->V = V;
    adj = new list< pair<int, double> >[V];
}

void Graph::addEdge(int u, int v, double w)
{
    adj[u].push_back(make_pair(v, w));
    adj[v].push_back(make_pair(u, w));
}

void printPath(int parent[], int j)
{
    // Base Case : If j is source
    if (parent[j] == -1)
        return;
    printPath(parent, parent[j]);
    cout << j << " ";
}

// A utility function to print the constructed distance
// array
void printSolution(int n, int parent[])
{
    int src = 0;
    cout << "Vertex\tPath";
    for (int i = 1; i < Vtx; i++) {
        printf("\n%d -> %d \t%d ", src, i,
            src, "\n");
        printPath(parent, i);
    }
}

// Prints shortest paths from src to all other vertices
void Graph::shortestPath(int src)
{
    // Create a set to store vertices that are being
    // processed
    set< pair<int, int> > setds;

    // Create a vector for distances and initialize all
    // distances as infinite (INF)
    std::vector<double> dist(V, 10000000);

    int parent[Vtx] = { -1 };

    // Insert source itself in Set and initialize its
    // distance as 0.
    setds.insert(make_pair(0, src));
    dist[src] = 0;

    /* Looping till all shortest distance are finalized
       then setds will become empty    */
    while (!setds.empty())
    {
        // The first vertex in Set is the minimum distance
        // vertex, extract it from set.
        pair<int, int> tmp = *(setds.begin());
        setds.erase(setds.begin());

        // vertex label is stored in second of pair (it
        // has to be done this way to keep the vertices
        // sorted distance (distance must be first item
        // in pair)
        int u = tmp.second;
        list< pair<int, double> >::iterator i;
        for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            // Get vertex label and weight of current adjacent
            // of u.
            int v = (*i).first;
            double weight = (*i).second;

            //    If there is shorter path to v through u.
            if (dist[v] > dist[u] + weight)
            {
                /*  If distance of v is not INF then it must be in
                    our set, so removing it and inserting again
                    with updated less distance.
                    Note : We extract only those vertices from Set
                    for which distance is finalized. So for them,
                    we would never reach here.  */
                if (dist[v] != 10000000)
                    setds.erase(setds.find(make_pair(dist[v], v)));

                // Updating distance of v
                dist[v] = dist[u] + weight;
                parent[v] = u;
                setds.insert(make_pair(dist[v], v));
            }
        }
    }

    // Print shortest distances stored in dist[]
    double maxDist = 0;
    int furthestVertex = 0;
    printf("Vertex   Total Distance from Source\n");
    for (int i = 0; i < V; ++i) {
        if (dist[i] > maxDist && dist[i]!= 10000000) {
            maxDist = dist[i];
            furthestVertex = i;
        }
        std::cout << i<< "         " << dist[i] << "\n" << std::endl;
    }
    printSolution(V, parent);
    std::cout << "" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Source Vertex (From Nearest Neighbor): " << src << "\n" << std::endl;
    std::cout << "Furthest Vertex: " << furthestVertex << "        " << "Euclidean Distance: " << maxDist << "\n" << std::endl;
}

vertex_data::vertex_data() {
    x = 0;
    y = 0;
    z = 0;
    dist = 0;
    neighbor1 = 0;
    neighbor2 = 0;
}

vertex_data::vertex_data(double xNew, double yNew, double zNew, double distNew, int neighbor1New, int neighbor2New) {
    x = xNew;
    y = yNew;
    z = zNew;
    dist = distNew;
    neighbor1 = neighbor1New;
    neighbor2 = neighbor2New;
}

vertex_data::~vertex_data() {
}

double vertex_data::getX() const {
    return x;
}

double vertex_data::getY() const {
    return y;
}

double vertex_data::getZ() const {
    return z;
}
double vertex_data::getDist() const {
    return dist;
}
int vertex_data::getNeighbor1() const {
    return neighbor1;
}
int vertex_data::getNeighbor2() const {
    return neighbor2;
}

void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (0.5, 0.2, 1.0);
    pcl::PointXYZ o;
    o.x = 1.23636;
    o.y = 37.4394;
    o.z = -162.792;
    viewer.addSphere (o, 10, "sphere", 0);
    std::cout << "Launched!" << std::endl;
    
}
    
void 
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Viewer Loop Count: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    user_data++;
}

double distance(double x1, double y1,
    double z1, double x2,
    double y2, double z2)
{
    return sqrt(pow(x2 - x1, 2) +
        pow(y2 - y1, 2) +
        pow(z2 - z1, 2) * 1.0);
}

int 
main ()
{
    // declaring the input point clouds
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
        
    //reading in the cloud data
    pcl::PCDReader reader;
    reader.read("referencecloud0.pcd", *cloud); 

    // create the filtering object
    //pcl::voxelgrid<pcl::pclpointcloud2> sor;
    //sor.setinputcloud(cloud);
    //sor.setleafsize(20.0f, 20.0f, 20.0f);
    //sor.filter(*cloud_filtered);

    // writing the downsampled pointcloud
    //pcl::pcdwriter writer;
    //writer.write("referencecloud0_downsampled.pcd", *cloud_filtered,
    //    eigen::vector4f::zero(), eigen::quaternionf::identity(), false);
  
    //reading the downsampled pointcloud
    //pcl::pointcloud<pcl::pointxyz>::ptr cloud1(new pcl::pointcloud<pcl::pointxyz>);
    //pcl::pclpointcloud2 cloud_voxel;
    //pcl::io::loadpcdfile("referencecloud0_downsampled.pcd", cloud_voxel);
    //pcl::frompclpointcloud2(cloud_voxel, *cloud1);

    // normal estimation*
    //pcl::normalestimation<pcl::pointxyz, pcl::normal> n;
    //pcl::pointcloud<pcl::normal>::ptr normals(new pcl::pointcloud<pcl::normal>);
    //pcl::search::kdtree<pcl::pointxyz>::ptr tree(new pcl::search::kdtree<pcl::pointxyz>);
    //tree->setinputcloud(cloud1);
    //n.setinputcloud(cloud1);
    //n.setsearchmethod(tree);
    //n.setksearch(20);
    //n.compute(*normals);
    //* normals should not contain the point normals + surface curvatures

    // concatenate the xyz and normal fields*
    //pcl::pointcloud<pcl::pointnormal>::ptr cloud_with_normals(new pcl::pointcloud<pcl::pointnormal>);
    //pcl::concatenatefields(*cloud1, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // create search tree*
    //pcl::search::kdtree<pcl::pointnormal>::ptr tree2(new pcl::search::kdtree<pcl::pointnormal>);
    //tree2->setinputcloud(cloud_with_normals);

    // initialize objects
    //pcl::greedyprojectiontriangulation<pcl::pointnormal> gp3;
    //pcl::polygonmesh triangles;

    // set the maximum distance between connected points (maximum edge length)
    //gp3.setsearchradius(100);

    // set typical values for the parameters
    //gp3.setmu(100);
    //gp3.setmaximumnearestneighbors(20);
    //gp3.setmaximumsurfaceangle(m_pi / 4); // 45 degrees
    //gp3.setminimumangle(m_pi / 18); // 10 degrees
    //gp3.setmaximumangle(5 * m_pi / 6); // 120 degrees
    //gp3.setnormalconsistency(false);

    // get result
    //gp3.setinputcloud(cloud_with_normals);
    //gp3.setsearchmethod(tree2);
    //gp3.reconstruct(triangles);

    // store mesh
    //pcl::io::saveplyfile("mesh.ply", triangles,10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader2;
    reader2.read("referencecloud0_downsampled.pcd", *cloud2);

    // object to store the centroid coordinates.
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud2, centroid);

    // Declaring Kd Search Tree for Nearest Neighbor Search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    // Declaring centroid as search Point
    pcl::PointXYZ searchPoint;
    searchPoint.x = centroid[0];
    searchPoint.y = centroid[1];
    searchPoint.z = centroid[2];

    // Only want the nearest Neighbor (Variable here for other use cases)
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    std::cout << "K nearest neighbor search at centroid coordinates (" << searchPoint.x
        << " " << searchPoint.y
        << " " << searchPoint.z
        << ") with K=" << K << std::endl;

    // Declare temporary floats.
    float nearestCentroidX = 0;
    float nearestCentroidY = 0;
    float nearestCentroidZ = 0;
    float nearestCentroidDist = 0;

    // Conditional for search Tree
    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size(); ++i) {
            nearestCentroidX = (*cloud2)[pointIdxNKNSearch[i]].x ;
            nearestCentroidY = (*cloud2)[pointIdxNKNSearch[i]].y;
            nearestCentroidZ = (*cloud2)[pointIdxNKNSearch[i]].z;
            nearestCentroidDist = pointNKNSquaredDistance[i];
        }
    }

    // Print the found Neighbor
    std::cout << "Found at: " << nearestCentroidX
        << " " << nearestCentroidY
        << " " << nearestCentroidZ
        << " (squared distance: " << nearestCentroidDist << ")" << std::endl;

    // Construct PLY Reader to read mesh
    happly::PLYData plyIn("mesh.ply");
    
    // Get mesh-style data from the object
    std::vector<std::array<double, 3>> vPos = plyIn.getVertexPositions();
    std::vector<std::vector<size_t>> fInd = plyIn.getFaceIndices<size_t>();


    // Creating vector with custom to store coordinates and dist
    std::vector<vertex_data> coords;

    // Initializing variables
    int iterate = 0;
    int vertexIndex = 0;
    int srcVertexIndex = 0;
    int neighbor1Index = 0;
    int neighbor2Index = 0;
    double neighbor1Weight = 0;
    double neighbor2Weight = 0;

    Graph g(vPos.size());
   
    // Loop through the Face indices 
    while (iterate < fInd.size()) {


        // Declare the first vertex index as starting point
        vertexIndex = fInd[iterate][0];
        
        // Declare neighbors through edge
        neighbor1Index = fInd[iterate][1];
        neighbor2Index = fInd[iterate][2];

        // Create if to see if the vPos[vertexIndex][i] ==  nearestCentroidX, nearestCentroidY, nearestCentroidZ and if so store that vertex index to use for g.shortestPath(x) where
        // x is the srcVertexIndex
        if ((float) vPos[vertexIndex][0] == nearestCentroidX && (float) vPos[vertexIndex][1] == nearestCentroidY && (float) vPos[vertexIndex][2] == nearestCentroidZ) {
            srcVertexIndex = vertexIndex;
        }
        if ((float)vPos[neighbor1Index][0] == nearestCentroidX && (float)vPos[neighbor1Index][1] == nearestCentroidY && (float)vPos[neighbor1Index][2] == nearestCentroidZ) {
            srcVertexIndex = neighbor1Index;
        }
        if ((float)vPos[neighbor2Index][0] == nearestCentroidX && (float)vPos[neighbor2Index][1] == nearestCentroidY && (float)vPos[neighbor2Index][2] == nearestCentroidZ) {
            srcVertexIndex = neighbor2Index;
        }

        neighbor1Weight = distance(vPos[vertexIndex][0], vPos[vertexIndex][1], vPos[vertexIndex][2],
            vPos[neighbor1Index][0], vPos[neighbor1Index][1], vPos[neighbor1Index][2]);

        g.addEdge(vertexIndex, neighbor1Index, neighbor1Weight);

        neighbor2Weight = distance(vPos[vertexIndex][0], vPos[vertexIndex][1], vPos[vertexIndex][2],
            vPos[neighbor2Index][0], vPos[neighbor2Index][1], vPos[neighbor2Index][2]);

        g.addEdge(vertexIndex, neighbor2Index, neighbor2Weight);

        iterate++;
    }
    g.shortestPath(srcVertexIndex);




    //// initializing the viewer
    //pcl::visualization::cloudviewer viewer("cloud viewer");

    ////blocks until the cloud is actually rendered
    //viewer.showcloud(cloud2);

    //viewer.runonvisualizationthreadonce(vieweroneoff);

    ////this will get called once per visualization iteration
    //viewer.runonvisualizationthread(viewerpsycho);
    //while (!viewer.wasstopped())
    //{

    //    user_data++;
    //}

    // to uncomment code use ctrl+k+c or ctrl+k+u
    return 0;

}