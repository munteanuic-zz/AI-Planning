import java.util.*;

//Nodes contants
static int pathLen = 100;
static int pathLen2 = 100;
static int numObstacles = 30;
static int numNodes  = 100;
static int maxNumNodes = 1000;
static Vec3[] nodePos = new Vec3[maxNumNodes];
static Vec3[] nodePos2 = new Vec3[maxNumNodes];
ArrayList<Integer>[] neighbors = new ArrayList[maxNumNodes];  //A list of neighbors can can be reached from a given node
ArrayList<Integer>[] neighbors2 = new ArrayList[maxNumNodes];  
  
//A list of circle obstacles
static int maxNumObstacles = 1000;
Vec3 circlePos[] = new Vec3[maxNumObstacles]; //Circle positions
float circleRad[] = new float[maxNumObstacles];  //Circle radii

int[] costToReach = new int[maxNumNodes];
Boolean[] visited = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent = new int[maxNumNodes]; //A list which stores the best previous node on the optimal path to reach this node

int[] costToReach2 = new int[maxNumNodes];
Boolean[] visited2 = new Boolean[maxNumNodes]; //A list which store if a given node has been visited
int[] parent2 = new int[maxNumNodes];

int closestNode(Vec3 point, Vec3[] nodePos, int numNodes){
  int closestID = -1;
  float minDist = 999999;
  for (int i = 0; i < numNodes; i++){
    float dist = nodePos[i].distanceTo(point);
    if (dist < minDist){
      closestID = i;
      minDist = dist;
    }
  }
  return closestID;
}

void connectNeighbors(Vec3[] centers, float[] radii, int numObstacles, Vec3[] nodePos, int numNodes, ArrayList<Integer>[] neighbors){
  for (int i = 0; i < numNodes; i++){
    neighbors[i] = new ArrayList<Integer>();  //Clear neighbors list
    for (int j = 0; j < numNodes; j++){
      if (i == j) continue; //don't connect to myself 
      Vec3 dir = nodePos[j].minus(nodePos[i]).normalized();
      float distBetween = nodePos[i].distanceTo(nodePos[j]);
      hitInfo circleListCheck = rayCircleListIntersect(centers, radii, numObstacles, nodePos[i], dir, distBetween, agentRad);
      if (!circleListCheck.hit){
        neighbors[i].add(j);
      }
    }
  }
}

void generateRandomNodes(int numNodes, Vec3[] circleCenters, float[] circleRadii){
  nodePos[0] = new Vec3 (agentPosX, agentPosY,0);
  for (int i = 1; i < numNodes; i++){
    Vec3 randPos = new Vec3(random(width),random(height),0);
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    while (insideAnyCircle){
      randPos = new Vec3(random(width),random(height),0);
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    }
    nodePos[i] = randPos;
  }
  nodePos[numNodes] = goalPos;
}

void generateRandomNodes2(int numNodes, Vec3[] circleCenters, float[] circleRadii){
  nodePos2[0] = new Vec3 (agentPosX2, agentPosY2,0);
  for (int i = 1; i < numNodes; i++){
    Vec3 randPos = new Vec3(random(width),random(height),0);
    boolean insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    while (insideAnyCircle){
      randPos = new Vec3(random(width),random(height),0);
      insideAnyCircle = pointInCircleList(circleCenters,circleRadii,numObstacles,randPos);
    }
    nodePos2[i] = randPos;
  }
  nodePos2[numNodes] = goalPos2;
}

void placeRandomObstacles(int numObstacles){
  //Initial obstacle position
  for (int i = 0; i < numObstacles; i++){
    circlePos[i] = new Vec3(random(50,950),random(50,700),0);
    circleRad[i] = (10+40*pow(random(1),3));
    while(circlePos[i].distanceTo(agentPos) <40 || circlePos[i].distanceTo(goalPos) <40 || circlePos[i].distanceTo(agentPos2) <40 || circlePos[i].distanceTo(goalPos2)<40){
      circlePos[i] = new Vec3(random(50,950),random(50,700),0);
      circleRad[i] = (10+40*pow(random(1),3));
    }
  }
  circleRad[0] = 30; //Make the first obstacle big
}

ArrayList<Integer> planPath(Vec3 startPos, Vec3 goalPos, Vec3[] nodePos, int numNodes, int[] costToReach, Boolean[] visited, int[] parent, ArrayList<Integer>[] neighbors){
  ArrayList<Integer> path = new ArrayList();
  int startID = closestNode(startPos, nodePos, numNodes);
  int goalID = closestNode(goalPos, nodePos, numNodes);
  path = runAStar(nodePos, numNodes, startID, goalID, costToReach, visited, parent, neighbors);
  return path;
}

int getMinCost(PriorityQueue<Integer> pQueue) {
  Iterator<Integer> it = pQueue.iterator();
  int indexOfMin = -1;
  int minValue = Integer.MAX_VALUE;;
    while (it.hasNext()) {
        Integer i = it.next();
        if(costToReach[i] < minValue){ 
         indexOfMin = i;
         minValue = costToReach[i]; 
      } 
    }    
    return indexOfMin;  
}

//A* Search
ArrayList<Integer> runAStar(Vec3[] nodePos, int numNodes, int startID, int goalID, int[] costToReach, Boolean[] visited, int[] parent, ArrayList<Integer>[] neighbors){
  // Creating empty priority queue 
  PriorityQueue<Integer> pQueue = new PriorityQueue<Integer>();   
  ArrayList<Integer> path = new ArrayList();  
  //ArrayList<Integer> fringe = new ArrayList();  //New empty fringe
  boolean pathFound = false;  
  for (int i = 0; i < numNodes; i++) { //Clear visit tags and parent pointers
    visited[i] = false;
    parent[i] = -1; //No parent yet
    //adding all of the nodes we see to the queue at the start. It chooses the first node based on the goal position.
    float distToStart = nodePos[i].distanceTo(agentPos);
    Vec3 dir = agentPos.minus(nodePos[i]).normalized();
    hitInfo hit = rayCircleListIntersect(circlePos, circleRad, numObstacles, nodePos[i],dir,distToStart,agentRad);
    if(!hit.hit){
      costToReach[i] = int(distToStart);
      pQueue.add(i);
      visited[i] = true;
    }
  }
  pQueue.add(startID);
  visited[startID] = true;
  int current = 0;
  costToReach[0] = 0;
  while (!pQueue.isEmpty()) {
            current = getMinCost(pQueue);
            pQueue.remove(current);
            for (int i = 0; i < neighbors[current].size(); i++){
              int neighborNode = neighbors[current].get(i);
              if (current == goalID){
                pathFound = true;
                break;
              }
              if (!visited[neighborNode]){
                visited[neighborNode] = true;    
                // calculate distance to neighbor.
                float neighborDistance = nodePos[current].distanceTo(nodePos[neighborNode]);
                costToReach[neighborNode] = (int) (costToReach[current] + neighborDistance);
                // set parent
                parent[neighborNode] = current;
                // enqueue
                pQueue.add(neighborNode);    
              }
             }
}
if (!pathFound){
    path.add(0,-1);
    return path;
}
  int prevNode = parent[goalID];
  path.add(0,goalID);
  while (prevNode >= 0){
    path.add(0,prevNode);
    prevNode = parent[prevNode];
  }
  return path;
}
