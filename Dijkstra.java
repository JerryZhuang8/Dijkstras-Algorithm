import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.io.IOException;
import java.io.FileReader;
import java.io.BufferedReader;
import java.util.ArrayList; 

public class Dijkstra {

  // Keep a fast index to nodes in the map
  private Map<String, Vertex> vertexNames;

  /**
   * Construct an empty Dijkstra with a map. The map's key is the name of a vertex
   * and the map's value is the vertex object.
   */
  public Dijkstra() {
    vertexNames = new HashMap<String, Vertex>();
  }

  /**
   * Adds a vertex to the dijkstra. Throws IllegalArgumentException if two vertices
   * with the same name are added.
   * 
   * @param v
   *          (Vertex) vertex to be added to the dijkstra
   */
  public void addVertex(Vertex v) {
    if (vertexNames.containsKey(v.name))
      throw new IllegalArgumentException("Cannot create new vertex with existing name.");
    vertexNames.put(v.name, v);
  }

  /**
   * Gets a collection of all the vertices in the dijkstra
   * 
   * @return (Collection<Vertex>) collection of all the vertices in the dijkstra
   */
  public Collection<Vertex> getVertices() {
    return vertexNames.values();
  }

  /**
   * Gets the vertex object with the given name
   * 
   * @param name
   *          (String) name of the vertex object requested
   * @return (Vertex) vertex object associated with the name
   */
  public Vertex getVertex(String name) {
    return vertexNames.get(name);
  }

  /**
   * Adds a directed edge from vertex u to vertex v
   * 
   * @param nameU
   *          (String) name of vertex u
   * @param nameV
   *          (String) name of vertex v
   * @param cost
   *          (double) cost of the edge between vertex u and v
   */
  public void addEdge(String nameU, String nameV, Double cost) {
    if (!vertexNames.containsKey(nameU))
      throw new IllegalArgumentException(nameU + " does not exist. Cannot create edge.");
    if (!vertexNames.containsKey(nameV))
      throw new IllegalArgumentException(nameV + " does not exist. Cannot create edge.");
    Vertex sourceVertex = vertexNames.get(nameU);
    Vertex targetVertex = vertexNames.get(nameV);
    Edge newEdge = new Edge(sourceVertex, targetVertex, cost);
    sourceVertex.addEdge(newEdge);
  }

  /**
   * Adds an undirected edge between vertex u and vertex v by adding a directed
   * edge from u to v, then a directed edge from v to u
   * 
   * @param nameU
   *          (String) name of vertex u
   * @param nameV
   *          (String) name of vertex v
   * @param cost
   *          (double) cost of the edge between vertex u and v
   */
  public void addUndirectedEdge(String nameU, String nameV, double cost) {
    addEdge(nameU, nameV, cost);
    addEdge(nameV, nameU, cost);
  }

  // STUDENT CODE STARTS HERE

  /**
   * Computes the euclidean distance between two points as described by their
   * coordinates
   * 
   * @param ux
   *          (double) x coordinate of point u
   * @param uy
   *          (double) y coordinate of point u
   * @param vx
   *          (double) x coordinate of point v
   * @param vy
   *          (double) y coordinate of point v
   * @return (double) distance between the two points
   */
  public double computeEuclideanDistance(double ux, double uy, double vx, double vy) {
        double squaredDistance = Math.pow(vy-uy, 2) + Math.pow(vx-ux, 2); 
        return Math.pow(squaredDistance, 0.5); 
        // TODO
        //return 1.0; // Replace this
  }

  /**
   * Calculates the euclidean distance for all edges in the map using the
   * computeEuclideanCost method.
   */
  public void computeAllEuclideanDistances() {
       Vertex[] array = getVertices().toArray(new Vertex[getVertices().size()]); 
       Vertex temp; 
       double tempDistance; 
       for(int index = 0; index < array.length; index++) { 
          for(int innerIndex = 0; innerIndex < array[index].adjacentEdges.size(); innerIndex++) { 
               temp = array[index].adjacentEdges.get(innerIndex).target; 
               tempDistance = computeEuclideanDistance(temp.x, temp.y, array[index].x, array[index].y); 
               getVertex(array[index].name).adjacentEdges.get(innerIndex).distance = tempDistance; 
               tempDistance = 0; 
               temp = null; 
               
          
          }
       
       }
        // TODO
  }//Basicaly, the following: Make an iterable array of Vertices by converting the Collections to an array. 
    //iterate through the array. Iterate through each element's adjacentEdges, retrieve its target, compute the distance, 
    //and update the distance in the actual hashMap using getVertex (which apparently allows us to modify the hashMap)

  /**
   * Dijkstra's Algorithm. 
   * 
   * @param s
   *          (String) starting city name
   */
    //Description: I placed every vertex into an arraylist. There was an origin, which i located using the parameter. 
    //then, with each iteration, i searched for the elements in the arraylist that were destinations of the origin 
    //variable, and updated them. Then, I found the minimum "distance" from the actual origin
    //since if it was a priorityqueue that would be the element that was "dequeued" next, removed that value,
    //and set the variable for the "origin" to that value and continued updating from there. 
    //this has a really long runtime, but it uses an arraylist as asked for. 
  public void doDijkstra(String s) {
      ArrayList<Vertex> allVertices = new ArrayList<Vertex>(this.getVertices()); 
      for(int index = 0; index < allVertices.size(); index++) { 
          allVertices.get(index).distance = 0; 
          allVertices.get(index).known = false; 
          allVertices.get(index).prev = null; 
      
      }//re-initalizes everything, so that we can run doDijkstra's AGAIN.
      int minIndex = -1; 
      Vertex tempVertex = null;
      int startPoint = 0; 
      Vertex tempSwapper; 
      for(int indexFindOrigin = 0; indexFindOrigin < allVertices.size(); indexFindOrigin++) { 
           if(allVertices.get(indexFindOrigin).name.equals(s)) {
               minIndex = indexFindOrigin; 
               tempVertex = allVertices.get(indexFindOrigin); 
               tempVertex.prev = null; //so that calling prev always works EXCEPT for unconnected graphs and going back to tempVertex.
               break; 
           } 
      }
      allVertices.remove(minIndex); 
      while(allVertices.size() > 0) { 
      minIndex = 0; 
      for(int innerIndex = 0; innerIndex < tempVertex.adjacentEdges.size(); innerIndex++) { //go through current tempVertex edges.
      for(int indexFinder = 0; indexFinder < allVertices.size(); indexFinder++) { //MUST find the edge that corresponds to the current tempVertex edge.    
             
          if(allVertices.get(indexFinder).name.equals(tempVertex.adjacentEdges.get(innerIndex).target.name)) {
                  
                  
                  //no compare to original vertex
                  //detected path. will have to update
                  if(allVertices.get(indexFinder).distance == 0.0) { 
                       allVertices.get(indexFinder).distance = tempVertex.adjacentEdges.get(innerIndex).distance + tempVertex.distance; 
                       allVertices.get(indexFinder).prev = tempVertex; 
                  }
                  else if(tempVertex.adjacentEdges.get(innerIndex).distance + tempVertex.distance < allVertices.get(indexFinder).distance) { 
                       allVertices.get(indexFinder).distance = tempVertex.adjacentEdges.get(innerIndex).distance + tempVertex.distance; 
                       allVertices.get(indexFinder).prev = tempVertex; 
                  }
                  
              
              }
           
           } } //dont we do the get min after all the updates ? //this is supposed to run through the whole array and change the edges. 
          for(int findMin = 0; findMin < allVertices.size(); findMin++) { 
           if(allVertices.get(findMin).distance != 0) { 
              minIndex = findMin;
              break; } }
          startPoint = minIndex; //we cannot have zeroes in the minindex. 
          for(int findMin = startPoint; findMin < allVertices.size(); findMin++) { 
              if(allVertices.get(findMin).distance < allVertices.get(minIndex).distance && allVertices.get(findMin).distance != 0) { 
                   minIndex = findMin; 
              }
          } 
          tempVertex = allVertices.get(minIndex); //ESSENTIALLY, THIS PART IS LIKE THE DEQUEUE. THEN WE UPDATE EVERYTHING RELATED TO IT IN THE NEXT ITERATION. 
          tempVertex.known = true; 
          allVertices.remove(minIndex); //alternatively, consolidate allVertices.get and remove into tempVertex = allvertices.remove(minindex). 
      } }
      //description: First, i convert the collections into an ArrayList. Once again, it is assumed that you can modify
      //the objects from there. 
        // TODO 
  

  /**
   * Returns a list of edges for a path from city s to city t. This will be the
   * shortest path from s to t as prescribed by Dijkstra's algorithm
   * 
   * @param s
   *          (String) starting city name
   * @param t
   *          (String) ending city name
   * @return (List<Edge>) list of edges from s to t
   */
  public List<Edge> getDijkstraPath(String s, String t) {
    doDijkstra(s);
    ArrayList<Edge> cities = new ArrayList<Edge>(); 
     Vertex temp = getVertex(t); 
      double r = 0; 
      while(!temp.name.equals(s)) { 
        if(temp.prev == null) { 
           return new ArrayList<Edge>(); 
        }
        cities.add(0, new Edge(temp.prev, temp, temp.distance-temp.prev.distance)); 
        temp = temp.prev; 
    }
    // TODO
    return cities; // Replace this
  }

  // STUDENT CODE ENDS HERE

  /**
   * Prints out the adjacency list of the dijkstra for debugging
   */
  public void printAdjacencyList() {
    for (String u : vertexNames.keySet()) {
      StringBuilder sb = new StringBuilder();
      sb.append(u);
      sb.append(" -> [ ");
      for (Edge e : vertexNames.get(u).adjacentEdges) {
        sb.append(e.target.name);
        sb.append("(");
        sb.append(e.distance);
        sb.append(") ");
      }
      sb.append("]");
      System.out.println(sb.toString());
    }
  }


  /** 
   * A main method that illustrates how the GUI uses Dijkstra.java to 
   * read a map and represent it as a graph. 
   * You can modify this method to test your code on the command line. 
   */
  public static void main(String[] argv) throws IOException {
    String vertexFile = "cityxy.txt"; 
    String edgeFile = "citypairs.txt";

    Dijkstra dijkstra = new Dijkstra();
    String line;

    // Read in the vertices
    BufferedReader vertexFileBr = new BufferedReader(new FileReader(vertexFile));
    while ((line = vertexFileBr.readLine()) != null) {
      String[] parts = line.split(",");
      if (parts.length != 3) {
        vertexFileBr.close();
        throw new IOException("Invalid line in vertex file " + line);
      }
      String cityname = parts[0];
      int x = Integer.valueOf(parts[1]);
      int y = Integer.valueOf(parts[2]);
      Vertex vertex = new Vertex(cityname, x, y);
      dijkstra.addVertex(vertex);
    }
    vertexFileBr.close();

    BufferedReader edgeFileBr = new BufferedReader(new FileReader(edgeFile));
    while ((line = edgeFileBr.readLine()) != null) {
      String[] parts = line.split(",");
      if (parts.length != 3) {
        edgeFileBr.close();
        throw new IOException("Invalid line in edge file " + line);
      }
      dijkstra.addUndirectedEdge(parts[0], parts[1], Double.parseDouble(parts[2]));
    }
    edgeFileBr.close();

    // Compute distances. 
    // This is what happens when you click on the "Compute All Euclidean Distances" button.
    dijkstra.computeAllEuclideanDistances();
    
    // print out an adjacency list representation of the graph
    dijkstra.printAdjacencyList();

    // This is what happens when you click on the "Draw Dijkstra's Path" button.

    // In the GUI, these are set through the drop-down menus.
    String startCity = "SanFrancisco";
    String endCity = "Boston";

    // Get weighted shortest path between start and end city. 
    List<Edge> path = dijkstra.getDijkstraPath(startCity, endCity);
    
    System.out.print("Shortest path between "+startCity+" and "+endCity+": ");
    System.out.println(path);
  }

}
