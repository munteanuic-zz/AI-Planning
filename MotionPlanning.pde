// CSCI 5611
// AI Planning
// Julie Malyshev & Ioana Munteanu

/* Single  Agent  Navigation
3D  Rendering &  Camera 
Improved  Agent &  Scene Rendering
User  Scenario  Editing
Realtime  User  Interaction
Multiple Agents Planning
Project  Report  &  Video */

import peasy.*;

PeasyCam cam;
PImage bg; //http://a9ragphotography.com/wp-content/uploads/2017/05/grass-background-28.jpg
int size =  10;

boolean paused = true;

PShape spider;
PShape wasp;
PShape spider2;
PShape wasp2;
 
PVector velocity = new PVector(200,40);
Vec3[] path = new Vec3[numNodes]; //A series of collision-free waypoints
Vec3[] path2 = new Vec3[numNodes]; 
int curPathIdx = 0; //Which node in the path we are going towards
int curPathIdx2 = 0;

Vec3 obsPos = new Vec3(440,360,0); //A circular obstacle the path avoids
float obsRad = 180;

//The agent we are controlling
float agentRad = 40;
Vec3 agentPos = new Vec3(220,620,0);
float agentPosX = 220;
float agentPosY = 620;
Vec3 agentVel = new Vec3(200,40,0);

//The agent's goal
Vec3 goalPos = new Vec3(680,300,0);
float goalSpeed = 50;

//Agent 2
float agentRad2 = 40;
Vec3 agentPos2 = new Vec3(300,620,0);
Vec3 agentVel2 = new Vec3(200,40,0);
float agentPosX2 = 300;
float agentPosY2 = 620;

//The agent's goal
Vec3 goalPos2 = new Vec3(200,200,0);
  
void setup(){ 
  size(1040,650,P3D);
  cam = new PeasyCam(this, 400,300,0,800);
  
  bg = loadImage("grass.jpg");
  
  //Initialize the first agent and goal
  wasp = loadShape("Bee.obj");
  spider = loadShape("spider.obj");

  spider.setFill(color(165,42,42));
  spider.rotateX(90);
  spider.scale(0.7);
  
  wasp.setFill(color(165,42,42));
  wasp.rotateX(90);
  wasp.scale(12);
  
  //Initialize the second agent and goal
  
  wasp2 = loadShape("Bee.obj");
  spider2 = loadShape("spider.obj");

  spider2.setFill(color(240,180,0));
  spider2.rotateX(90);
  spider2.scale(0.7);
  
  wasp2.setFill(color(240,180,0));
  wasp2.rotateX(90);
  wasp2.scale(12);

  //Initialize obstacles, nodes, and web
  placeRandomObstacles(numObstacles);
  generateRandomNodes(numNodes, circlePos, circleRad);
  generateRandomNodes2(numNodes, circlePos, circleRad);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, neighbors);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos2, numNodes, neighbors2);

  //Find path1 using A*
  ArrayList<Integer> pathArray = planPath(agentPos, goalPos, nodePos, numNodes, costToReach, visited, parent, neighbors);
  pathLen = pathArray.size();
  if (pathArray.get(0) == -1){
    agentPos = new Vec3(random(width),random(height),0);
    setup();
    curPathIdx = 0;
  }
  else{ for (int i = 0; i < pathArray.size(); i++){
          path[i] = nodePos[pathArray.get(i)];
      }
  }
  
  //Find path2 using A*
  ArrayList<Integer> pathArray2 = planPath(agentPos2, goalPos2, nodePos2, numNodes, costToReach2, visited2, parent2, neighbors2);
  pathLen2 = pathArray2.size();
  if (pathArray2.get(0) == -1){
    agentPos2 = new Vec3(random(width),random(height),0);
    setup();
    curPathIdx2 = 0;
  }
  else{ for (int i = 0; i < pathArray2.size(); i++){
          path2[i] = nodePos2[pathArray2.get(i)];
      }
  }
  
}

int closestObstacle(Vec3 circlePos[], float[] circleRad, Vec3 agentPos, float agentRad){
  float minDist = 999999999;
  int minObs = -1;
  for (int i = 0; i < numObstacles; i++){
      float dist = circlePos[i].distanceTo(agentPos) - circleRad[i] + agentRad;
      if (dist < minDist){
        minDist = dist;
        minObs = i;
      }
  }
  return minObs;  
}

Vec3 computeAgentVel(Vec3 agentPos, Vec3 goalPos, Vec3[] path, int curPathIdx, int pathLen){
  if (agentPos.distanceTo(goalPos) < 20){
    return new Vec3(0,0,0);
  }
  for(int i = curPathIdx; i < pathLen; i++){
    if(!rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos,path[i].minus(agentPos).normalized(),path[i].minus(agentPos).length(),agentRad).hit){
      curPathIdx = i;
    }
    fill(0,0,0);
  }
  if(!rayCircleListIntersect(circlePos, circleRad, numObstacles, agentPos,goalPos.minus(agentPos).normalized(),goalPos.minus(agentPos).length(),agentRad).hit){
      return goalPos.minus(agentPos).normalized().times(100); 
   }
  return path[curPathIdx].minus(agentPos).normalized().times(100);
}

void moveAgent(float dt){
  agentVel = computeAgentVel(agentPos, goalPos, path, curPathIdx, pathLen);
  agentPos.add(agentVel.times(dt));
  agentVel2 = computeAgentVel(agentPos2, goalPos2, path2, curPathIdx2, pathLen2);
  agentPos2.add(agentVel2.times(dt));
}

void draw(){
  lights();
  background(bg);

  // Draw obstacles
  for (int i = 0; i < numObstacles; i++){
    pushMatrix();
    if (i == 0) {
      fill(0,0,237);
      stroke(0,0,237);
    }
    else{
      fill(36,160,237);
      stroke(36,160,237);
    }
    noStroke();
    translate(circlePos[i].x, circlePos[i].y, circlePos[i].z);
    sphere(circleRad[i]);
    popMatrix();
  }
  
  //Update agent if not paused
  if (!paused){
    moveAgent(1.0/frameRate);
  }
  
  //Draw web
  pushMatrix();
  for (int i = 1; i < numNodes;  i++){
    for (int j : neighbors[i]){
      pushMatrix();
      stroke (255, 255, 255);
      line(nodePos[i].x,nodePos[i].y,0,nodePos[j].x,nodePos[j].y,0);
      popMatrix();
    }
  }
  for (int i = 1; i < numNodes;  i++){
    for (int j : neighbors2[i]){
      pushMatrix();
      stroke (255, 255, 255);
      line(nodePos2[i].x,nodePos2[i].y,0,nodePos2[j].x,nodePos2[j].y,0);
      popMatrix();
    }
  }
 popMatrix();
 
  //Draw spider1 and wasp1 
  pushMatrix();
  shape(wasp, goalPos.x, goalPos.y);
  shape(spider, agentPos.x, agentPos.y);
  
  //Draw spider3 and wasp2
  shape(wasp2, goalPos2.x, goalPos2.y);
  shape(spider2, agentPos2.x, agentPos2.y);
  popMatrix();
}

float speed = 10;
void keyPressed(){
  if (key == ' ') paused = !paused;
  if (key == 'r'){
    agentPos = new Vec3(220,620,0);
    curPathIdx = 0;
    agentPos2 = new Vec3(300,620,0);
    curPathIdx2 = 0;
  }
  if (keyCode == RIGHT){
    circlePos[0].x += speed;
  }
  if (keyCode == LEFT){
    circlePos[0].x -= speed;
  }
  if (keyCode == UP){
    circlePos[0].y -= speed;
  }
  if (keyCode == DOWN){
    circlePos[0].y += speed;
  }
  if (key == 'd'){
    agentPos.x += speed;
  }
  if (key == 'a'){
    agentPos.x -= speed;
  }
  if (key == 'w'){
    agentPos.y -= speed;
  }
  if (key == 's'){
    agentPos.y += speed;
  }
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos, numNodes, neighbors);
  connectNeighbors(circlePos, circleRad, numObstacles, nodePos2, numNodes, neighbors2);
}
  
void mousePressed(){
  if(mouseButton == LEFT){
    agentPos2 = new Vec3(mouseX, mouseY, 20);
    nodePos2[0] = new Vec3 (agentPosX2, agentPosY2,0);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos2, numNodes, neighbors2);
    curPathIdx2 = 0;
    path2[0] = new Vec3 (agentPosX2, agentPosY2,0);
    ArrayList<Integer> pathArray2 = planPath(agentPos2, goalPos2, nodePos2, numNodes, costToReach2, visited2, parent2, neighbors2);
    pathLen2 = pathArray2.size();
    if (pathArray2.get(0) == -1){
      agentPos2 = new Vec3(300,620,0);
      curPathIdx2 = 0;
    }
    else{
    for (int i = 0; i < pathArray2.size(); i++){
      path2[i] = nodePos2[pathArray2.get(i)];
    }
    } 
    } else {
    goalPos2 = new Vec3(mouseX, mouseY,0);
    nodePos2[0] = new Vec3 (agentPosX2, agentPosY2,0);
    connectNeighbors(circlePos, circleRad, numObstacles, nodePos2, numNodes, neighbors2);
    curPathIdx2 = 0;
    path2[0] = new Vec3 (agentPosX2, agentPosY2,0);
    ArrayList<Integer> pathArray2 = planPath(agentPos2, goalPos2, nodePos2, numNodes, costToReach2, visited2, parent2, neighbors2);
    pathLen2 = pathArray2.size();
    if (pathArray2.get(0) == -1){
    agentPos2= new Vec3(300,620,0);
    curPathIdx2 = 0;
    }
    else{
    for (int i = 0; i < pathArray2.size(); i++){
    path2[i] = nodePos2[pathArray2.get(i)];
    }
    } 
    }
  }
